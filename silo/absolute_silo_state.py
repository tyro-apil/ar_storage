import copy
from enum import Enum
from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from silo_msgs.msg import Silo, SiloArray
from std_msgs.msg import UInt8


class RobotState(Enum):
  SEARCHING_BALL = 0
  ALIGNING = 1
  BALL_STORED = 3


class AbsoluteStateEstimation(Node):
  def __init__(self):
    super().__init__("absolute_state_estimation")

    self.declare_parameter("width", 921)
    self.declare_parameter("consistency_threshold", 5)
    self.declare_parameter("silos_state", [""] * 5)
    self.declare_parameter("team_color", "blue")

    self.team_color = (
      self.get_parameter("team_color").get_parameter_value().string_value
    )
    self.TEAM_REPR = "B"
    self.OPPONENT_REPR = "R"
    if self.team_color == "red":
      self.TEAM_REPR, self.OPPONENT_REPR = self.OPPONENT_REPR, self.TEAM_REPR

    self.add_on_set_parameters_callback(self.parameters_change_callback)
    self.create_timer(0.033, self.timer_callback)
    self.silos_absolute_state_publisher = self.create_publisher(
      SiloArray, "state_map", 10
    )
    self.silo_state_image_subscriber = self.create_subscription(
      SiloArray, "state_image", self.silo_state_image_callback, 10
    )
    self.silo_state_image_subscriber
    self.align_info_subscriber = self.create_subscription(
      UInt8, "/aligned_silo", self.aligned_info_callback, 10
    )
    self.robot_state_subscriber = self.create_subscription(
      UInt8, "/robot_state", self.robot_state_callback, 10
    )
    self.robot_state_subscriber
    self.received_state = 0

    ## Mapping of robot state to enum
    self.robot_state_mapping = {
      0: RobotState.SEARCHING_BALL,
      1: RobotState.ALIGNING,
      3: RobotState.BALL_STORED,
    }

    self.robot_state = self.robot_state_mapping[0]
    self.silos_absolute_state_msg = SiloArray()
    self.silos_absolute_state = [
      {"index": i + 1, "state": "", "bbox": [None] * 4} for i in range(5)
    ]
    self.update_silos_absolute_state_msg()
    self.silos_relative_state_received = None
    self.__aligned_silo = 0

    self.__image_width = self.get_parameter("width").get_parameter_value().integer_value
    self.__consistency_threshold = (
      self.get_parameter("consistency_threshold").get_parameter_value().integer_value
    )

    self.x_center_image = self.__image_width / 2
    self.received_msg_consistency_counter = 0

    self.known_state = None
    self.__is_known_state_set = False

    self.get_logger().info("Absolute silo state estimation node started.")

  def parameters_change_callback(self, parameters: List[Parameter]):
    for parameter in parameters:
      if (
        parameter.name == "silos_state"
        and parameter.type_ == Parameter.Type.STRING_ARRAY
      ):
        self.silos_absolute_state = [
          {"index": i + 1, "state": state, "bbox": [None] * 4}
          for i, state in enumerate(parameter.get_parameter_value().string_array_value)
        ]
        self.known_state = copy.deepcopy(self.silos_absolute_state)
        self.__is_known_state_set = True
        self.update_silos_absolute_state_msg()
        return SetParametersResult(successful=True)
    return SetParametersResult(successful=False)

  def robot_state_callback(self, robot_state_msg: UInt8):
    self.received_state = robot_state_msg.data
    self.robot_state = self.robot_state_mapping[self.received_state]

    if self.robot_state == RobotState.BALL_STORED and self.__aligned_silo != 0:
      if len(self.silos_absolute_state[self.__aligned_silo - 1]["state"] < 2):
        self.silos_absolute_state[self.__aligned_silo - 1]["state"] += self.TEAM_REPR
    return

  def timer_callback(self):
    self.silos_absolute_state_publisher.publish(self.silos_absolute_state_msg)
    # self.display_state(self.silos_absolute_state)
    return

  def aligned_info_callback(self, aligned_silo_msg: UInt8):
    self.__aligned_silo = aligned_silo_msg.data
    return

  def silo_state_image_callback(self, silos_detected_state_msg: SiloArray):
    ## parse state from message
    # List of dictionaries
    silos_received_state = self.parse_state(silos_detected_state_msg.silos)

    if self.silos_relative_state_received is None:
      self.silos_relative_state_received = silos_received_state
      return

    ## check for consistency in 5 frames
    if not self.is_state_consistent_across_frames(silos_received_state):
      # self.get_logger().warn("Messages across frames are inconsistent")
      return

    ## check if all 5 states are visible
    # if YES, proceed
    # if NO, yet to implement...
    if len(silos_received_state) > 5 or len(silos_received_state) == 0:
      # self.get_logger().warn(f"{len(silos_received_state)} silos are visible.....")
      return

    if len(silos_received_state) < 5:
      silos_received_state = self.predict_full_state(partial_state=silos_received_state)
      if silos_received_state is None:
        return

    if self.__is_known_state_set or len(silos_received_state) != 5:
      ## Get consistent state from received state
      silos_received_state = self.compute_consistent_state(silos_received_state)

    # ## check consistency for state of each silo state
    # # if additive, update state
    # # if conflicts with previous state, warn and pass
    # if not self.is_consistent_with_previous_state(silos_received_state):
    #   self.get_logger().warn("Received state is inconsistent with previous state")
    #   self.get_logger().warn(f"Received state: {silos_received_state}")
    #   self.get_logger().warn(f"Previous state: {self.silos_absolute_state}")
    #   return

    self.set_silos_absolute_state(silos_received_state)
    self.update_silos_absolute_state_msg()
    return

  def parse_state(self, silos):
    silos_state = []

    for silo in silos:
      silo_state = {}
      silo_state["index"] = silo.index
      silo_state["state"] = silo.state
      silo_state["bbox"] = silo.xyxy
      silos_state.append(silo_state)

    return silos_state

  def is_state_consistent_across_frames(self, silos_received_state):
    previous_received_state = copy.deepcopy(self.silos_relative_state_received)
    self.silos_relative_state_received = silos_received_state
    if len(silos_received_state) != len(previous_received_state):
      self.received_msg_consistency_counter = 0
      return False
    for received, previous in zip(silos_received_state, previous_received_state):
      if received["state"] != previous["state"]:
        self.received_msg_consistency_counter = 0
        return False

    self.received_msg_consistency_counter += 1
    if self.received_msg_consistency_counter == self.__consistency_threshold:
      self.received_msg_consistency_counter = 0
      return True
    return False

  def is_consistent_with_previous_state(self, silos_received_state):
    for silo_received, silo_previous in zip(
      silos_received_state, self.silos_absolute_state
    ):
      if len(silo_received["state"]) < len(silo_previous["state"]):
        return False
      prev_len = len(silo_previous["state"])
      if silo_received["state"][:prev_len] != silo_previous["state"]:
        return False
    return True

  def compute_consistent_state(self, silos_received_state):
    consistent_state = copy.deepcopy(self.silos_absolute_state)
    for silo_received, silo_previous in zip(
      silos_received_state, self.silos_absolute_state
    ):
      # breakpoint()
      if len(silo_received["state"]) < len(silo_previous["state"]):
        self.get_logger().warn(
          f"Silo-{silo_received['index']} -> Previous: {silo_previous['state']} balls | Received: {silo_received['state']}"
        )
        continue
      prev_state_len = len(silo_previous["state"])
      if not silo_received["state"][:prev_state_len] == silo_previous["state"]:
        continue
      consistent_state[silo_received["index"] - 1]["state"] = silo_received["state"]
    return consistent_state

  def predict_full_state(self, partial_state):
    if self.__aligned_silo == 0:
      return None
    aligned_index_relative = self.get_relative_index_aligned_silo(partial_state)
    predicted_state = copy.deepcopy(self.silos_absolute_state)
    for silo in partial_state:
      offset = aligned_index_relative - silo["index"]
      predicted_state[self.__aligned_silo - offset - 1]["state"] = silo["state"]
    # self.display_state(predicted_state)
    return predicted_state

  def get_relative_index_aligned_silo(self, partial_state):
    closest_center_x = 1000
    closest_index = 0
    for silo in partial_state:
      bbox = silo["bbox"]
      center_x = (bbox[0] + bbox[2]) / 2
      if abs(center_x - self.x_center_image) < abs(
        self.x_center_image - closest_center_x
      ):
        closest_center_x = center_x
        closest_index = silo["index"]
    return closest_index

  def set_silos_absolute_state(self, silos_received_state):
    self.silos_absolute_state = copy.deepcopy(silos_received_state)
    return

  def update_silos_absolute_state_msg(self):
    self.silos_absolute_state_msg = SiloArray()
    for silo in self.silos_absolute_state:
      silo_msg = Silo()
      silo_msg.index = silo["index"]
      silo_msg.state = silo["state"]
      self.silos_absolute_state_msg.silos.append(silo_msg)
    return

  def display_state(self, silos_state):
    log = ""
    for silo in silos_state:
      log += f"Silo{silo['index']}: {silo['state']} | "
    self.get_logger().info(log)


def main(args=None):
  rclpy.init(args=args)

  absolute_state_node = AbsoluteStateEstimation()

  rclpy.spin(absolute_state_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  absolute_state_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
