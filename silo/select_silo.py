from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Duration, Time
from silo_msgs.msg import SiloArray
from std_msgs.msg import UInt8MultiArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

"""
Priority List:
1. Team | Opponent or Opponent | Team
2. Team | Team
3. Opponent | Opponent
4. Empty
5. Team
6. Opponent
"""


class SiloSelection(Node):
  def __init__(self):
    node_name = "silo_selection"
    super().__init__("silo_selection")

    # Declare team color as parameter
    self.declare_parameter("team_color", "blue")
    self.declare_parameter("silos_x", [0.0] * 5)
    self.declare_parameter("silo_z_min", 0.0)
    self.declare_parameter("silo_z_max", 0.0)
    self.declare_parameter("silo_y", 0.0)
    self.declare_parameter("silo_radius", 0.0)

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # Timer to publish two best silos
    self.create_timer(0.05, self.timer_callback)
    # Subscribe state of silos w.r.t. map
    self.state_subscriber = self.create_subscription(
      SiloArray, "state_map", self.state_received_callback, 10
    )
    self.state_subscriber

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    # # Subscribe to base_link pose
    # self.baselink_pose_subscriber = self.create_subscription(
    #   Odometry,
    #   "/odometry/filtered",
    #   self.baselink_pose_callback,
    #   qos_profile=qos_profile,
    # )
    # self.baselink_pose_subscriber

    # Publisher of list of 2 best optimal silos
    self.optimal_silos_publisher = self.create_publisher(
      UInt8MultiArray, "/silo_number", 10
    )

    # Get team color as object variable for silo selection
    self.team_color = (
      self.get_parameter("team_color").get_parameter_value().string_value
    )
    self.silos_x = (
      self.get_parameter("silos_x").get_parameter_value().double_array_value
    )
    self.silo_z_min = (
      self.get_parameter("silo_z_min").get_parameter_value().double_value
    )
    self.silo_z_max = (
      self.get_parameter("silo_z_max").get_parameter_value().double_value
    )
    self.silo_y = self.get_parameter("silo_y").get_parameter_value().double_value
    self.silo_radius = (
      self.get_parameter("silo_radius").get_parameter_value().double_value
    )

    # Get x,y of silos
    self.silos_xy = [(x, self.silo_y) for x in self.silos_x]

    # Set character representations
    self.TEAM_REPR = "B"
    self.OPPONENT_REPR = "R"
    if self.team_color == "red":
      self.TEAM_REPR, self.OPPONENT_REPR = self.OPPONENT_REPR, self.TEAM_REPR

      ##############################
      # TO BE CHANGED AS PER TEAM COLOR
      # negate the y coordinate of silos
      self.silos_xy = [(x, self.silo_y) for x in self.silos_x]
      ##############################

    # Initialize optimal silos as zero index
    self.optimal_silos: List[int] = [0] * 2
    self.silo_numbers_msg = UInt8MultiArray()
    self.received_msg = None

    # Initialize list of lists for priority
    self.priority_list = [[] for _ in range(6)]
    self.get_logger().info(f"{node_name} node started")

    # list to indicate full silos state
    self.full_silos_index = []

    # baselink translation w.r.t. map
    self.translation_map2base = None

  def timer_callback(self):
    self.publish_silo_numbers_msg()
    # return

  def publish_silo_numbers_msg(self):
    self.optimal_silos_publisher.publish(self.silo_numbers_msg)
    self.get_logger().info(f"Optimal silos: {self.optimal_silos}")
    return

  def baselink_pose_callback(self, pose_msg: Odometry):
    self.translation_map2base = np.zeros(3)
    self.translation_map2base[0] = pose_msg.pose.pose.position.x
    self.translation_map2base[1] = pose_msg.pose.pose.position.y
    self.translation_map2base[2] = pose_msg.pose.pose.position.z

  def state_received_callback(self, state_msg: SiloArray):
    # if self.translation_map2base is None:
    #   # self.get_logger().info("Waiting for baselink pose")
    #   return

    tf_map2base = self.get_map2base_tf(
      "map", "base_link", Time(seconds=0, nanoseconds=0)
    )
    self.set_baselink_translation(tf_map2base)

    self.received_msg = state_msg

    ## Assign priority to each silo
    # Put index in respective index of priority_list
    self.set_priority_list(state_msg.silos)

    ## Iterate through priority list
    # Select nearest silo index in first non-empty list inaside priority list
    self.update_target()

  def set_priority_list(self, silo_array):
    # reinitalize priority list
    self.priority_list = [[] for _ in range(6)]
    # update priority list
    for silo in silo_array:
      if (
        silo.state == self.TEAM_REPR + self.OPPONENT_REPR
        or silo.state == self.OPPONENT_REPR + self.TEAM_REPR
      ):
        self.priority_list[0].append(silo.index)
      elif silo.state == self.TEAM_REPR * 2:
        self.priority_list[1].append(silo.index)
      elif silo.state == self.OPPONENT_REPR * 2:
        self.priority_list[2].append(silo.index)
      elif silo.state == "":
        self.priority_list[3].append(silo.index)
      elif silo.state == self.TEAM_REPR:
        self.priority_list[4].append(silo.index)
      elif silo.state == self.OPPONENT_REPR:
        self.priority_list[5].append(silo.index)
      else:
        self.full_silos_index.append(silo.index)

  def update_target(self):
    second_priority_list = self.priority_list
    self.optimal_silos = [0] * 2

    ## Get best option from first priority list
    for i, silos in enumerate(self.priority_list):
      if len(silos) == 0:
        continue
      optimal_silo_index = self.get_optimal_silo_index(silos)
      self.optimal_silos[0] = optimal_silo_index
      second_priority_list[i].remove(optimal_silo_index)
      break

    ## Get best option from second priority list
    for i, silos in enumerate(second_priority_list):
      if len(silos) == 0:
        continue
      optimal_silo_index = self.get_optimal_silo_index(silos)
      self.optimal_silos[1] = optimal_silo_index
      break

    ## Update silo_numbers_msg
    self.silo_numbers_msg.data = self.optimal_silos

  def get_optimal_silo_index(self, silo_indexes):
    ## Iterate over silo_indexes
    ## Select silo_index with minimum distance w.r.t. baselink
    silos_distance: List[float] = [
      self.get_distance(silo_index) for silo_index in silo_indexes
    ]
    optimal_silo_index = silo_indexes[silos_distance.index(min(silos_distance))]
    return optimal_silo_index

  def get_distance(self, silo_index):
    del_x = self.silos_xy[silo_index - 1][0] - self.translation_map2base[0]
    del_y = self.silos_xy[silo_index - 1][1] - self.translation_map2base[1]
    return np.sqrt(del_x**2 + del_y**2)

  def get_map2base_tf(
    self, from_frame: str, to_frame: str, time: Time
  ) -> TransformStamped:
    try:
      t = self.tf_buffer.lookup_transform(
        from_frame,
        to_frame,
        time=time,
        timeout=Duration(seconds=0.003),
      )
    except TransformException as ex:
      self.get_logger().info(f"Could not transform {from_frame} to {to_frame}: {ex}")
      return None
    return t

  def set_baselink_translation(self, tf: TransformStamped) -> np.ndarray:
    self.translation_map2base = np.array(
      [
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
      ]
    )
    return


def main(args=None):
  rclpy.init(args=args)

  state_estimation_node = SiloSelection()

  rclpy.spin(state_estimation_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  state_estimation_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
