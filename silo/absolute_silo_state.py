from collections import defaultdict, deque

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
    self.silos_relative_state_received = deque(maxlen=10)
    self.__consistency_threshold = 5

    self.get_logger().info("Absolute silo state estimation node started.")

  def timer_callback(self):
    self.silos_absolute_state_publisher.publish(self.silos_absolute_state_msg)
    self.display_state()

  def silo_state_image_callback(self, silos_detected_state_msg: SiloArray):
    ## parse state from message
    # List of dictionaries
    silos_received_state = self.parse_state(silos_detected_state_msg.silos)

    self.silos_relative_state_received.append(silos_received_state)

    consistent_state = self.get_consistent_state()
    if consistent_state is None:
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
      silos_state.append(silo_state)

    return silos_state

  def add_state(self, silos_received_state):
    # Convert list of dictionaries to a sorted tuple of tuples
    sorted_state = tuple(
      sorted((silo["index"], silo["state"]) for silo in silos_received_state)
    )
    self.silos_relative_state_received.append(sorted_state)

  def get_consistent_state(self):
    # Use defaultdict to count occurrences of each state
    state_counter = defaultdict(int)

    for state_tuple in self.silos_relative_state_received:
      state_counter[state_tuple] += 1

    # Find the state that meets the threshold
    for state_tuple, count in state_counter.items():
      if count >= self.__consistency_threshold:
        # Convert the tuple back to a list of dictionaries
        return [{"index": idx, "state": st} for idx, st in state_tuple]

    return None

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
