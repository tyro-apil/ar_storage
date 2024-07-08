import rclpy
from rclpy.node import Node
from silo_msgs.msg import Silo, SiloArray
from yolov8_msgs.msg import BoundingBox2D, DetectionArray


def xywh2xyxy(xywh):
  """Converts bbox xywh format into xyxy format"""
  xyxy = []
  xyxy.append(xywh[0] - int(xywh[2] / 2))
  xyxy.append(xywh[1] - int(xywh[3] / 2))
  xyxy.append(xywh[0] + int(xywh[2] / 2))
  xyxy.append(xywh[1] + int(xywh[3] / 2))
  return xyxy


class StateEstimation(Node):
  def __init__(self):
    super().__init__("state_estimation")

    self.declare_parameter("team_color", "blue")
    self.declare_parameter("width", 921)

    self.silos_state_publisher = self.create_publisher(SiloArray, "state_image", 10)
    self.detections_subscriber = self.create_subscription(
      DetectionArray, "yolo/tracking", self.detections_callback, 10
    )
    self.detections_subscriber

    self.silos_state_msg = SiloArray()
    team_color = self.get_parameter("team_color").get_parameter_value().string_value

    ########################################
    # TO BE CHANGED AS PER TEAM COLOR

    self.silo_order_descending = False
    # if team_color == "blue":
    #   self.silo_order_descending = False
    # else:
    #   self.silo_order_descending = True

    ########################################

    self.__image_width = self.get_parameter("width").get_parameter_value().integer_value
    self.__tolerance = 0.05

    self.state = None
    self.silos_num = None
    self.balls_num = None
    self.get_logger().info("Silo state estimation node started.")

  def detections_callback(self, detections_msg: DetectionArray):
    # filter detections
    silos, balls = self.filter_detections(detections_msg.detections)
    self.silos_num = len(silos)
    self.balls_num = len(balls)
    if self.silos_num > 5:
      self.get_logger().warn("Too many silos detected")

    # sort silos
    sorted_silos = sorted(
      silos, key=lambda x: x.bbox.center.position.x, reverse=self.silo_order_descending
    )

    # get region of interest of detected silos
    silo_bboxes_xywh = [self.parse_bbox(silo.bbox) for silo in sorted_silos]
    silo_bboxes_xyxy = [xywh2xyxy(silo_bbox) for silo_bbox in silo_bboxes_xywh]

    state = [[] for _ in range(len(silos))]
    # iterate through detected balls and check if they are inside the silos
    for ball in balls:
      ball_bbox_xywh = self.parse_bbox(ball.bbox)
      ball_bbox_xyxy = xywh2xyxy(ball_bbox_xywh)
      for i, silo_bbox in enumerate(silo_bboxes_xyxy):
        if ball_bbox_xyxy[0] >= max(
          0, silo_bbox[0] - self.__tolerance * self.__image_width
        ) and ball_bbox_xyxy[2] <= min(
          self.__image_width, silo_bbox[2] + self.__tolerance * self.__image_width
        ):
          state[i].append(ball)
          break

    # sort balls inside silo by y_coordinate
    for i in range(len(state)):
      if len(state[i]) > 3:
        self.get_logger().warn(
          f"Too many balls detected in silo-{i+1} i.e. {len(state[i])} balls"
        )
      state[i].sort(key=lambda ball: ball.bbox.center.position.y, reverse=True)

    # stringify the state of silos
    state_repr = self.stringify_state(state)
    silos_state_msg = self.get_silo_state_msg(state_repr, silo_bboxes_xyxy)

    # update state with strings for each silo
    self.update_state(state_repr)
    # self.display_state()

    # publish the state of silos
    self.silos_state_msg = silos_state_msg
    self.silos_state_publisher.publish(silos_state_msg)

  def filter_detections(self, detections):
    silos = list(filter(lambda detection: detection.class_name == "silo", detections))
    balls = list(
      filter(
        lambda detection: detection.class_name != "silo"
        and detection.class_name != "purple",
        detections,
      )
    )
    return silos, balls

  def parse_bbox(self, bbox_xywh: BoundingBox2D):
    """! Parse bbox from BoundingBox2D msg
    @param bbox_xywh a BoundingBox2D msg in format xywh
    @return a tuple of center_x, center_y, width, height
    """
    center_x = int(bbox_xywh.center.position.x)
    center_y = int(bbox_xywh.center.position.y)
    width = int(bbox_xywh.size.x)
    height = int(bbox_xywh.size.y)
    return [center_x, center_y, width, height]

  def stringify_state(self, state):
    state_repr = [None] * self.silos_num
    for i, silo in enumerate(state):
      silo_state = ""
      for ball in silo:
        if ball.class_name == "red":
          silo_state += "R"
        elif ball.class_name == "blue":
          silo_state += "B"
      state_repr[i] = silo_state
    return state_repr

  def update_state(self, state_repr):
    self.state = state_repr

  def display_state(self):
    log = ""
    for i, silo in enumerate(self.state):
      log += f"Silo{i+1}: {silo} | "
    self.get_logger().info(log)

  def get_silo_state_msg(self, silos_state, silo_bboxes_xyxy):
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


def main(args=None):
  rclpy.init(args=args)

  state_estimation_node = StateEstimation()

  rclpy.spin(state_estimation_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  state_estimation_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
