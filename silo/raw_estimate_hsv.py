from typing import List, Tuple

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
  QoSDurabilityPolicy,
  QoSHistoryPolicy,
  QoSProfile,
  QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from silo_msgs.msg import Silo, SiloArray
from yolov8_msgs.msg import BoundingBox2D, Detection, DetectionArray


class StateEstimationHSV(Node):
  def __init__(self):
    super().__init__("state_estimation_hsv")

    self.declare_params()
    self.read_params()

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )

    self.silos_state_publisher = self.create_publisher(SiloArray, "state_image", 10)
    self.debug_img_publisher = self.create_publisher(
      Image, "state/dbg_image", qos_profile=image_qos_profile
    )

    # self.detections_subscriber = self.create_subscription(
    #   DetectionArray, "yolo/tracking", self.detections_callback, 10
    # )
    # self.detections_subscriber

    detections_subscriber = message_filters.Subscriber(
      self, DetectionArray, "yolo/tracking"
    )
    img_subscriber = message_filters.Subscriber(
      self, Image, "image_raw", qos_profile=image_qos_profile
    )

    self._synchronizer = message_filters.ApproximateTimeSynchronizer(
      (detections_subscriber, img_subscriber), 10, 0.05, True
    )
    self._synchronizer.registerCallback(self.detections_callback)

    self.bridge = CvBridge()
    self.silos_state_msg = SiloArray()

    ########################################
    if self.team_color == "blue":
      self.opponent_color = "red"
      self.TEAM_REPR = "B"
      self.OPPONENT_REPR = "R"
      self.silo_order_descending = False
    else:
      self.opponent_color = "blue"
      self.TEAM_REPR = "R"
      self.OPPONENT_REPR = "B"
      self.silo_order_descending = True
    ########################################

    self.state = None
    self.silos_num = None
    self.debug_image = Image()
    self.get_logger().info("Silo state estimation node (HSV) started.")

  # def detections_callback(self, detections_msg: DetectionArray):
  def detections_callback(self, detections_msg: DetectionArray, img_msg: Image):
    bgr_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    debug_img = bgr_img.copy()
    hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

    # get mask for team color
    team_mask = self.get_mask(hsv_img, self.team_color)
    # get mask for opponent color
    opponent_mask = self.get_mask(hsv_img, self.opponent_color)

    # preprocess masks
    team_mask = self.preprocess_mask(team_mask)
    opponent_mask = self.preprocess_mask(opponent_mask)

    combined_mask = self.combine_masks(team_mask, opponent_mask)
    colored_mask = self.get_color_mask(bgr_img, combined_mask)

    # filter detections
    silos = self.get_silos(detections_msg.detections)
    silos = self.filter_silos(silos)
    self.silos_num = len(silos)
    if self.silos_num > 5:
      self.get_logger().warn("Too many silos detected")
      return

    # sort silos
    sorted_silos = sorted(
      silos, key=lambda x: x.bbox.center.position.x, reverse=self.silo_order_descending
    )

    # get region of interest of detected silos
    silo_bboxes_xywh = [self.parse_bbox(silo.bbox) for silo in sorted_silos]
    silo_bboxes_xyxy = [self.xywh2xyxy(silo_bbox) for silo_bbox in silo_bboxes_xywh]

    silos_state = []
    # iterate through silos and check rois of individual silos for state estimation
    for i, _ in enumerate(sorted_silos):
      silo_w = silo_bboxes_xywh[i][2] - silo_bboxes_xywh[i][0]
      silo_h = silo_bboxes_xywh[i][3] - silo_bboxes_xywh[i][1]

      y_divisions = [int(y * silo_h) for y in self.y_divisions]

      rois = self.get_rois(silo_bboxes_xyxy[i], y_divisions)

      debug_img = self.draw_rois(colored_mask, rois)

      state = self.estimate_silo_state(hsv_img, rois, team_mask, opponent_mask)

      cv2.putText(
        debug_img,
        f"{state}",
        (
          silo_bboxes_xyxy[i][0] + int(0.1 * silo_w),
          silo_bboxes_xyxy[i][3] - int(0.1 * silo_h),
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        1,
      )
      silos_state.append(state)

    # update state with strings for each silo
    self.update_state(silos_state)
    silos_state_msg = self.get_silo_state_msg(silos_state, silo_bboxes_xyxy)
    # self.display_state()

    self.debug_image = self.bridge.cv2_to_imgmsg(
      debug_img, "bgr8", header=detections_msg.header
    )
    self.debug_img_publisher.publish(self.debug_image)
    # publish the state of silos
    self.silos_state_msg = silos_state_msg
    self.silos_state_publisher.publish(silos_state_msg)

  def get_silos(self, detections: List[Detection]) -> List[Detection]:
    silos = list(filter(lambda detection: detection.class_name == "silo", detections))
    return silos

  def filter_silos(self, silos: List[Detection]) -> List[Detection]:
    filtered_silos = []
    for silo in silos:
      xywh = self.parse_bbox(silo.bbox)
      area = xywh[2] * xywh[3]
      if area > self.__min_silo_area:
        filtered_silos.append(silo)
    return filtered_silos

  def parse_bbox(self, bbox_xywh: BoundingBox2D) -> List[int]:
    """! Parse bbox from BoundingBox2D msg
    @param bbox_xywh a BoundingBox2D msg in format xywh
    @return a tuple of center_x, center_y, width, height
    """
    center_x = int(bbox_xywh.center.position.x)
    center_y = int(bbox_xywh.center.position.y)
    width = int(bbox_xywh.size.x)
    height = int(bbox_xywh.size.y)
    return [center_x, center_y, width, height]

  def xywh2xyxy(self, xywh: List[int]) -> List[int]:
    """Converts bbox xywh format into xyxy format"""
    xyxy = []
    xyxy.append(xywh[0] - int(xywh[2] / 2))
    xyxy.append(xywh[1] - int(xywh[3] / 2))
    xyxy.append(xywh[0] + int(xywh[2] / 2))
    xyxy.append(xywh[1] + int(xywh[3] / 2))
    return xyxy

  def update_state(self, state_repr: List[str]) -> None:
    self.state = state_repr

  def display_state(self) -> None:
    log = ""
    for i, silo in enumerate(self.state):
      log += f"Silo{i+1}: {silo} | "
    self.get_logger().info(log)

  def get_silo_state_msg(
    self, silos_state: List[str], silo_bboxes_xyxy: List[List[int]]
  ) -> SiloArray:
    silo_state_msg = SiloArray()
    for i, state in enumerate(silos_state):
      silo_msg = Silo()
      silo_msg.index = i + 1
      silo_msg.state = state
      silo_msg.xyxy[0] = silo_bboxes_xyxy[i][0]
      silo_msg.xyxy[1] = silo_bboxes_xyxy[i][1]
      silo_msg.xyxy[2] = silo_bboxes_xyxy[i][2]
      silo_msg.xyxy[3] = silo_bboxes_xyxy[i][3]
      silo_state_msg.silos.append(silo_msg)
    return silo_state_msg

  def declare_params(self):
    self.declare_parameter("team_color", "blue")
    self.declare_parameter("width", 921)
    self.declare_parameter("height", 518)
    self.declare_parameter("min_silo_area", 1500)
    self.declare_parameter("y_divisions", [-0.10, 0.20, 0.60, 0.95])

  def read_params(self):
    self.team_color = (
      self.get_parameter("team_color").get_parameter_value().string_value
    )
    self.__image_width = self.get_parameter("width").get_parameter_value().integer_value
    self.__image_height = (
      self.get_parameter("height").get_parameter_value().integer_value
    )
    self.__min_silo_area = (
      self.get_parameter("min_silo_area").get_parameter_value().integer_value
    )
    self.y_divisions = (
      self.get_parameter("y_divisions").get_parameter_value().double_array_value
    )

  def get_mask(self, hsv_frame: cv2.Mat, color: str) -> cv2.Mat:
    match color:
      case "red":
        red1_hsv_low = (0, 120, 50)
        red1_hsv_high = (10, 255, 235)
        red2_hsv_low = (170, 120, 60)
        red2_hsv_high = (180, 255, 220)

        red1_mask = cv2.inRange(hsv_frame, red1_hsv_low, red1_hsv_high)
        red2_mask = cv2.inRange(hsv_frame, red2_hsv_low, red2_hsv_high)
        mask = cv2.bitwise_or(red1_mask, red2_mask)

      case "blue":
        blue1_hsv_low = (80, 130, 30)
        blue1_hsv_high = (110, 170, 90)
        blue2_hsv_low = (100, 100, 50)
        blue2_hsv_high = (115, 230, 230)

        blue1_mask = cv2.inRange(hsv_frame, blue1_hsv_low, blue1_hsv_high)
        blue2_mask = cv2.inRange(hsv_frame, blue2_hsv_low, blue2_hsv_high)
        mask = cv2.bitwise_or(blue1_mask, blue2_mask)

    return mask

  def compute_match_percent(self, hsv_img: cv2.Mat, roi: Tuple, mask: cv2.Mat) -> float:
    x1, y1, x2, y2 = roi
    roi_img = hsv_img[y1:y2, x1:x2]
    roi_mask = mask[y1:y2, x1:x2]

    roi_mask = cv2.bitwise_and(roi_img, roi_img, mask=roi_mask)
    roi_mask = cv2.cvtColor(roi_mask, cv2.COLOR_HSV2BGR)
    roi_mask = cv2.cvtColor(roi_mask, cv2.COLOR_BGR2GRAY)

    match_percent = cv2.countNonZero(roi_mask) / (roi_mask.shape[0] * roi_mask.shape[1])
    return match_percent

  def get_rois(self, bbox_xyxy: List[int], y_divisions: List[int]) -> Tuple[Tuple[int]]:
    roi_1 = (
      bbox_xyxy[0],
      bbox_xyxy[1] + y_divisions[2],
      bbox_xyxy[2],
      bbox_xyxy[1] + y_divisions[3],
    )

    roi_2 = (
      bbox_xyxy[0],
      bbox_xyxy[1] + y_divisions[1],
      bbox_xyxy[2],
      bbox_xyxy[1] + y_divisions[2],
    )

    roi_3 = (
      bbox_xyxy[0],
      max(0, bbox_xyxy[1] + y_divisions[0]),
      bbox_xyxy[2],
      bbox_xyxy[1] + y_divisions[1],
    )

    return roi_1, roi_2, roi_3

  def estimate_silo_state(
    self,
    hsv_img: cv2.Mat,
    rois: Tuple[Tuple[int]],
    team_mask: cv2.Mat,
    opponent_mask: cv2.Mat,
  ) -> None:
    state = ""
    for roi in rois:
      team_match = self.compute_match_percent(hsv_img, roi, team_mask)
      opponent_match = self.compute_match_percent(hsv_img, roi, opponent_mask)

      if team_match > 0.5:
        state += self.TEAM_REPR
      elif opponent_match > 0.5:
        state += self.OPPONENT_REPR
      else:
        break
    return state

  def preprocess_mask(self, mask: cv2.Mat) -> cv2.Mat:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    processed_mask = cv2.dilate(mask, kernel, iterations=2)
    return processed_mask

  def combine_masks(self, mask1: cv2.Mat, mask2: cv2.Mat) -> cv2.Mat:
    combined_mask = cv2.bitwise_or(mask1, mask2)
    return combined_mask

  def get_color_mask(self, img: cv2.Mat, mask: cv2.Mat) -> cv2.Mat:
    colored_mask = cv2.bitwise_and(img, img, mask=mask)
    return colored_mask

  def draw_rois(self, img: cv2.Mat, rois: Tuple[Tuple[int]]) -> cv2.Mat:
    img_copy = img.copy()
    for roi in rois:
      cv2.rectangle(img_copy, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
    return img_copy


def main(args=None):
  rclpy.init(args=args)

  state_estimation_node = StateEstimationHSV()

  rclpy.spin(state_estimation_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  state_estimation_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
