#!/usr/bin/env python3

import os
import socket
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
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
from std_msgs.msg import Bool, Header, UInt8
from std_srvs.srv import Trigger
from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.utils.plotting import Annotator

PORT = 12345


class ImageReceiverNode(Node):
  def __init__(self):
    super().__init__("image_receiver_node")

    self.declare_parameter("use_model", False)
    self.declare_parameter("model", "check_top.pt")
    self.declare_parameter("device", "cuda:0")
    self.declare_parameter("threshold", 0.7)

    self.declare_parameter("top_roi", [0] * 4)  # XYXY format
    self.declare_parameter("match_fraction", 0.50)

    self.declare_parameter("red1_h_low", 0)
    self.declare_parameter("red1_s_low", 100)
    self.declare_parameter("red1_v_low", 40)
    self.declare_parameter("red1_h_high", 15)
    self.declare_parameter("red1_s_high", 255)
    self.declare_parameter("red1_v_high", 235)

    self.declare_parameter("red2_h_low", 165)
    self.declare_parameter("red2_s_low", 115)
    self.declare_parameter("red2_v_low", 65)
    self.declare_parameter("red2_h_high", 185)
    self.declare_parameter("red2_s_high", 255)
    self.declare_parameter("red2_v_high", 210)

    self.declare_parameter("blue1_h_low", 80)
    self.declare_parameter("blue1_s_low", 130)
    self.declare_parameter("blue1_v_low", 30)
    self.declare_parameter("blue1_h_high", 110)
    self.declare_parameter("blue1_s_high", 170)
    self.declare_parameter("blue1_v_high", 90)

    self.declare_parameter("blue2_h_low", 100)
    self.declare_parameter("blue2_s_low", 100)
    self.declare_parameter("blue2_v_low", 50)
    self.declare_parameter("blue2_h_high", 115)
    self.declare_parameter("blue2_s_high", 230)
    self.declare_parameter("blue2_v_high", 230)

    self.red1_h_low = (
      self.get_parameter("red1_h_low").get_parameter_value().integer_value
    )
    self.red1_s_low = (
      self.get_parameter("red1_s_low").get_parameter_value().integer_value
    )
    self.red1_v_low = (
      self.get_parameter("red1_v_low").get_parameter_value().integer_value
    )
    self.red1_h_high = (
      self.get_parameter("red1_h_high").get_parameter_value().integer_value
    )
    self.red1_s_high = (
      self.get_parameter("red1_s_high").get_parameter_value().integer_value
    )
    self.red1_v_high = (
      self.get_parameter("red1_v_high").get_parameter_value().integer_value
    )

    self.red2_h_low = (
      self.get_parameter("red2_h_low").get_parameter_value().integer_value
    )
    self.red2_s_low = (
      self.get_parameter("red2_s_low").get_parameter_value().integer_value
    )
    self.red2_v_low = (
      self.get_parameter("red2_v_low").get_parameter_value().integer_value
    )
    self.red2_h_high = (
      self.get_parameter("red2_h_high").get_parameter_value().integer_value
    )
    self.red2_s_high = (
      self.get_parameter("red2_s_high").get_parameter_value().integer_value
    )
    self.red2_v_high = (
      self.get_parameter("red2_v_high").get_parameter_value().integer_value
    )

    self.blue1_h_low = (
      self.get_parameter("blue1_h_low").get_parameter_value().integer_value
    )
    self.blue1_s_low = (
      self.get_parameter("blue1_s_low").get_parameter_value().integer_value
    )
    self.blue1_v_low = (
      self.get_parameter("blue1_v_low").get_parameter_value().integer_value
    )
    self.blue1_h_high = (
      self.get_parameter("blue1_h_high").get_parameter_value().integer_value
    )
    self.blue1_s_high = (
      self.get_parameter("blue1_s_high").get_parameter_value().integer_value
    )
    self.blue1_v_high = (
      self.get_parameter("blue1_v_high").get_parameter_value().integer_value
    )

    self.blue2_h_low = (
      self.get_parameter("blue1_h_low").get_parameter_value().integer_value
    )
    self.blue2_s_low = (
      self.get_parameter("blue1_s_low").get_parameter_value().integer_value
    )
    self.blue2_v_low = (
      self.get_parameter("blue1_v_low").get_parameter_value().integer_value
    )
    self.blue2_h_high = (
      self.get_parameter("blue1_h_high").get_parameter_value().integer_value
    )
    self.blue2_s_high = (
      self.get_parameter("blue1_s_high").get_parameter_value().integer_value
    )
    self.blue2_v_high = (
      self.get_parameter("blue1_v_high").get_parameter_value().integer_value
    )

    self.top_roi = (
      self.get_parameter("top_roi").get_parameter_value().double_array_value
    )
    self.top_roi = [int(i) for i in self.top_roi]
    self.match_fraction = (
      self.get_parameter("match_fraction").get_parameter_value().double_value
    )
    self.__use_model = self.get_parameter("use_model").get_parameter_value().bool_value

    if self.__use_model:
      self.model = self.get_parameter("model").get_parameter_value().string_value
      self.device = self.get_parameter("device").get_parameter_value().string_value
      self.threshold = (
        self.get_parameter("threshold").get_parameter_value().double_value
      )
      self.yolo = YOLO(self.model)
      self.debug_img_dir = "/home/apil/work/robocon2024/cv/live_capture/close_silo"

    self.srv = self.create_service(
      srv_type=Trigger, srv_name="/is_ball_at_top", callback=self.is_ball_at_top
    )
    self.last_received_img = None

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )

    self.silo_check_subscriber = self.create_subscription(
      UInt8, "/silo_check_request", self.silo_check_callback, 10
    )
    self.silo_check_subscriber
    self.publisher_ = self.create_publisher(
      Image, "image_raw", qos_profile=image_qos_profile
    )
    self.silo_check_publisher = self.create_publisher(Bool, "/silo_check_result", 10)
    self.bridge = CvBridge()

    # Set up socket
    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.server_socket.bind(("0.0.0.0", PORT))
    self.server_socket.listen(1)
    self.get_logger().info(f"Listening on port {PORT}")

    # Run receive loop in constructor
    self.receive_loop()

  def receive_loop(self):
    while True:
      try:
        client_socket, addr = self.server_socket.accept()
        self.get_logger().info(f"Connection from {addr}")

        while True:
          # Receive image size
          size_bytes = client_socket.recv(4)
          if not size_bytes:
            self.get_logger().warn("No size data received, closing connection.")
            break

          size = int.from_bytes(size_bytes, byteorder="big")

          # Receive image data
          img_data = b""
          while len(img_data) < size:
            chunk = client_socket.recv(size - len(img_data))
            if not chunk:
              raise RuntimeError("Socket connection broken")
            img_data += chunk

          if len(img_data) != size:
            self.get_logger().warn(
              f"Incomplete img_data received: {len(img_data)} out of {size} bytes"
            )
            raise Exception("Incomplete img_data received")

          # Decode image
          np_arr = np.frombuffer(img_data, dtype=np.uint8)
          cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

          if cv_image is None:
            self.get_logger().warn("Failed to decode frame.")
            continue

          self.last_received_img = cv_image

          msg_header = Header()
          msg_header.stamp = self.get_clock().now().to_msg()
          msg_header.frame_id = "picam_link_optical"

          # Publish image as ROS message
          ros_image_msg = self.bridge.cv2_to_imgmsg(
            cv_image, encoding="bgr8", header=msg_header
          )
          self.publisher_.publish(ros_image_msg)

      except Exception as e:
        self.get_logger().error(f"Error receiving image: {str(e)}")

  def silo_check_callback(self, msg: UInt8):
    self.get_logger().info(f"Received message: {msg.data}")
    if msg.data != 0xA5:
      return
    response = Bool()
    response.data = False
    if self.last_received_img is None:
      self.get_logger().warn("No image to compare")
      self.silo_check_publisher.publish(response)
      return response

    if self.__use_model:
      result, color = self.query_model()
    else:
      result, color = self.query_in_hsv()

    response.data = result
    self.silo_check_publisher.publish(response)

  def is_ball_at_top(
    self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
    if self.last_received_img is None:
      response.success = False
      response.message = "No image to compare"
      return response

    if self.__use_model:
      result, color = self.query_model()
    else:
      result, color = self.query_in_hsv()

    response.success = result
    if color is None:
      response.message = "Top spot is vacant"
    else:
      response.message = f"{color} is at top"
    return response

  def query_in_hsv(self) -> Tuple[bool, Optional[str]]:
    # Convert image to HSV for color filtering
    hsv_frame = cv2.cvtColor(self.last_received_img, cv2.COLOR_BGR2HSV)

    red_mask = self.get_mask(hsv_frame, "red")
    blue_mask = self.get_mask(hsv_frame, "blue")

    red_mask = self.preprocess_mask(red_mask)
    blue_mask = self.preprocess_mask(blue_mask)

    red_match_percent = self.compute_match_percent(hsv_frame, self.top_roi, red_mask)
    blue_match_percent = self.compute_match_percent(hsv_frame, self.top_roi, blue_mask)

    if (red_match_percent > self.match_fraction) or (
      blue_match_percent > self.match_fraction
    ):
      dominant_color = ""
      if red_match_percent > blue_match_percent:
        dominant_color = "red"
      else:
        dominant_color = "blue"
      return True, dominant_color
    return False, None

  def query_model(self) -> Tuple[bool, Optional[str]]:
    img_copy = self.last_received_img.copy()

    results = self.yolo.predict(
      source=self.last_received_img,
      verbose=False,
      stream=False,
      conf=self.threshold,
      device=self.device,
    )

    annotator = Annotator(img_copy)
    r = results[0].boxes
    for box in r:
      b = box.xyxy[0]
      c = box.cls
      annotator.box_label(b, self.yolo.names[int(c)])
    annotated_img = annotator.result()
    now = time.time()
    cv2.imwrite(os.path.join(self.debug_img_dir, str(now)), annotated_img)

    results: Results = results[0].cpu()

    hypothesis = self.parse_hypothesis(results)
    boxes = self.parse_boxes(results)

    is_on_top, index = self.check_on_top(boxes)
    if index is None:
      return (is_on_top, None)
    return is_on_top, hypothesis[index]["class_name"]

  def parse_hypothesis(self, results: Results) -> List:
    hypothesis_list = []

    for box_data in results.boxes:
      hypothesis = {
        "class_id": int(box_data.cls),
        "class_name": self.yolo.names[int(box_data.cls)],
        "score": float(box_data.conf),
      }
      hypothesis_list.append(hypothesis)

    return hypothesis_list

  def parse_boxes(self, results: Results) -> List:
    boxes_list = []

    for box_data in results.boxes:
      box = box_data.xyxy[0]
      boxes_list.append(box)

    return boxes_list

  def check_on_top(self, boxes: List[List]):
    for i, xyxy in boxes:
      if xyxy[1] <= 50:
        return True, i
    return False, None

  def get_mask(self, hsv_frame: cv2.Mat, color: str) -> cv2.Mat:
    match color:
      case "red":
        red1_hsv_low = np.array([self.red1_h_low, self.red1_s_low, self.red1_v_low])
        red1_hsv_high = np.array([self.red1_h_high, self.red1_s_high, self.red1_v_high])

        red2_hsv_low = np.array([self.red2_h_low, self.red2_s_low, self.red2_v_low])
        red2_hsv_high = np.array([self.red2_h_high, self.red2_s_high, self.red2_v_high])

        red1_mask = cv2.inRange(hsv_frame, red1_hsv_low, red1_hsv_high)
        red2_mask = cv2.inRange(hsv_frame, red2_hsv_low, red2_hsv_high)
        mask = cv2.bitwise_or(red1_mask, red2_mask)

      case "blue":
        blue1_hsv_low = np.array([self.blue1_h_low, self.blue1_s_low, self.blue1_v_low])
        blue1_hsv_high = np.array(
          [self.blue1_h_high, self.blue1_s_high, self.blue1_v_high]
        )

        blue2_hsv_low = np.array([self.blue2_h_low, self.blue2_s_low, self.blue2_v_low])
        blue2_hsv_high = np.array(
          [self.blue2_h_high, self.blue2_s_high, self.blue2_v_high]
        )

        blue1_mask = cv2.inRange(hsv_frame, blue1_hsv_low, blue1_hsv_high)
        mask = blue1_mask
        # blue2_mask = cv2.inRange(hsv_frame, blue2_hsv_low, blue2_hsv_high)
        # mask = cv2.bitwise_or(blue1_mask, blue2_mask)

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

  def preprocess_mask(self, mask: cv2.Mat) -> cv2.Mat:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    processed_mask = cv2.dilate(mask, kernel, iterations=2)
    return processed_mask

  def combine_masks(self, mask1: cv2.Mat, mask2: cv2.Mat) -> cv2.Mat:
    combined_mask = cv2.bitwise_or(mask1, mask2)
    return combined_mask

  def destroy_node(self):
    self.server_socket.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = ImageReceiverNode()
  rclpy.spin(node)  # This is needed to keep the node alive until shutdown
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
