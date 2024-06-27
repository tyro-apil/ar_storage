#! /usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  silo_config = os.path.join(get_package_share_directory("silo"), "config", "silo.yaml")
  common_config = os.path.join(
    get_package_share_directory("oakd"), "config", "common.yaml"
  )

  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace", default_value="", description="Name of the namespace"
  )

  pose_topic = LaunchConfiguration("pose_topic")
  pose_topic_cmd = DeclareLaunchArgument(
    "pose_topic",
    default_value="/odometry/filtered",
    description="Name of the pose topic of map2base transform",
  )

  tracking_topic = LaunchConfiguration("tracking_topic")
  tracking_topic_cmd = DeclareLaunchArgument(
    "tracking_topic",
    default_value="yolo/tracking",
    description="Name of the tracking topic",
  )

  state_estimation_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="state_estimation_node",
    name="state_estimation_node",
    parameters=[common_config],
    remappings=[
      ("yolo/tracking", tracking_topic),
    ],
  )

  absolute_silo_state_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="absolute_silo_state_node",
    name="absolute_silo_state_node",
  )

  silos_marker_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="silos_marker_node",
    name="silos_marker_node",
    parameters=[common_config, silo_config],
  )

  silo_selection_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="silo_selection_node",
    name="silo_selection_node",
    remappings=[("/odometry/filtered", pose_topic)],
    parameters=[silo_config, common_config],
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(pose_topic_cmd)
  ld.add_action(tracking_topic_cmd)
  ld.add_action(team_color_cmd)

  ld.add_action(state_estimation_node_cmd)
  ld.add_action(absolute_silo_state_node_cmd)
  ld.add_action(silos_marker_node_cmd)
  ld.add_action(silo_selection_node_cmd)

  return ld
