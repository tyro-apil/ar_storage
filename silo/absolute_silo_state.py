from os import close

import rclpy
from rclpy.node import Node
from silo_msgs.msg import Silo, SiloArray
from std_msgs.msg import UInt8


class AbsoluteStateEstimation(Node):
  def __init__(self):
    super().__init__("absolute_state_estimation")

    self.declare_parameter("width", 921)

    self.create_timer(0.033, self.timer_callback)
    self.silos_absolute_state_publisher = self.create_publisher(
      SiloArray, "state_map", 10
    )
    self.silo_state_image_subscriber = self.create_subscription(
      SiloArray, "state_image", self.silo_state_image_callback, 10
    )
    self.silo_state_image_subscriber
    self.align_info_subscriber = self.create_publisher(
      UInt8, "aligned_silo", self.aligned_info_callback, 10
    )

    self.silos_absolute_state_msg = SiloArray()
    self.silos_absolute_state = [
      {"index": i + 1, "state": "", "bbox": [None] * 4} for i in range(5)
    ]
    self.update_silos_absolute_state_msg()
    self.silos_relative_state_received = None
    self.__aligned_silo = 0
    self.__image_width = self.get_parameter("width").get_parameter_value().integer_value
    self.x_center_image = self.__image_width / 2
    self.received_msg_consistency_counter = 0

    self.get_logger().info("Absolute silo state estimation node started.")

  def timer_callback(self):
    self.silos_absolute_state_publisher.publish(self.silos_absolute_state_msg)
    self.display_state()
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
      self.get_logger().warn(f"{len(silos_received_state)} silos are visible.....")
      return
    if self.__aligned_silo == 0:
      return
    if len(silos_received_state) < 5:
      silos_received_state = self.predict_state(partial_state=silos_received_state)

    ## check consistency for state of each silo state
    # if additive, update state
    # if conflicts with previous state, warn and pass
    if not self.is_consistent_with_previous_state(silos_received_state):
      self.get_logger().warn("Received state is inconsistent with previous state")
      self.get_logger().warn(f"Received state: {silos_received_state}")
      self.get_logger().warn(f"Previous state: {self.silos_absolute_state}")
      return

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
    previous_received_state = self.silos_relative_state_received
    self.silos_relative_state_received = silos_received_state
    if len(silos_received_state) != len(previous_received_state):
      self.received_msg_consistency_counter = 0
      return False
    for received, previous in zip(silos_received_state, previous_received_state):
      if received["state"] != previous["state"]:
        self.received_msg_consistency_counter = 0
        return False

    self.received_msg_consistency_counter += 1
    if self.received_msg_consistency_counter == 5:
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

  def predict_state(self, partial_state):
    if self.__aligned_silo == 0:
      return None
    aligned_index_relative = self.get_relative_index_aligned_silo(partial_state)
    predicted_state = self.silos_absolute_state
    for silo in partial_state:
      offset = aligned_index_relative - silo["index"]
      predicted_state[self.__aligned_silo - offset - 1]["state"] = silo["state"]
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
    self.silos_absolute_state = silos_received_state
    return

  def update_silos_absolute_state_msg(self):
    self.silos_absolute_state_msg = SiloArray()
    for silo in self.silos_absolute_state:
      silo_msg = Silo()
      silo_msg.index = silo["index"]
      silo_msg.state = silo["state"]
      self.silos_absolute_state_msg.silos.append(silo_msg)
    return

  def display_state(self):
    log = ""
    for silo_state in self.silos_absolute_state:
      log += f"Silo{silo_state['index']}: {silo_state['state']} | "
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
