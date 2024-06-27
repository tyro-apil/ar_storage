from collections import namedtuple
from tkinter.tix import Tree

import rclpy
from rclpy.node import Node
from silo_msgs.msg import Silo, SiloArray


class AbsoluteStateEstimation(Node):
  def __init__(self):
    super().__init__("absolute_state_estimation")

    self.declare_parameter("team_color", "blue")

    self.create_timer(0.033, self.timer_callback)
    self.silos_absolute_state_publisher = self.create_publisher(
      SiloArray, "state_map", 10
    )
    self.silo_state_image_subscriber = self.create_subscription(
      SiloArray, "state_image", self.silo_state_image_callback, 10
    )
    self.silo_state_image_subscriber

    self.silos_absolute_state_msg = SiloArray()
    self.silos_absolute_state = [
      {
        "index": i + 1,
        "state": "",
      }
      for i in range(5)
    ]
    self.update_silos_absolute_state_msg()
    self.silos_relative_state_received = None
    self.received_msg_consistency_counter = 0

    self.get_logger().info("Absolute silo state estimation node started.")

  def timer_callback(self):
    self.silos_absolute_state_publisher.publish(self.silos_absolute_state_msg)

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
    if len(silos_received_state) != 5:
      self.get_logger().warn(f"{len(silos_received_state)} silos are visible.....")
      return

    ## check consistency for state of each silo state
    # if additive, update state
    # if conflicts with previous state, warn and pass
    if not self.is_consistent_with_previous_state(silos_received_state):
      self.get_logger().warn("Received state is inconsistent with previous state")
      self.get_logger().warn(f"Received state: {silos_received_state}")
      self.get_logger().warn(f"Previous state: {self.silos_absolute_state_previous}")
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

  def update_state(self, state_repr):
    self.state = state_repr

  def display_state(self):
    log = ""
    for i, silo_state in enumerate(self.state):
      log += f"Silo{i+1}: {silo_state} | "
    self.get_logger().info(log)


def main(args=None):
  rclpy.init(args=args)

  state_estimation_node = AbsoluteStateEstimation()

  rclpy.spin(state_estimation_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  state_estimation_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
