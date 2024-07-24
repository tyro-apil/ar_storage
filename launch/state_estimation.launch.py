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
  camera_info_config = os.path.join(
    get_package_share_directory("silo"), "config", "camera_info.yaml"
  )

  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace",
    default_value="",
    description="Name of the namespace",
  )

  tracking_topic = LaunchConfiguration("tracking_topic")
  tracking_topic_cmd = DeclareLaunchArgument(
    "tracking_topic",
    default_value="yolo/tracking",
  )

  aligned_silo_topic = LaunchConfiguration("aligned_silo_topic")
  aligned_silo_topic_cmd = DeclareLaunchArgument(
    "aligned_silo_topic",
    default_value="/aligned_silo",
  )

  state_estimation_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="state_estimation_node",
    name="state_estimation_node",
    parameters=[common_config, camera_info_config, silo_config],
    remappings=[
      ("yolo/tracking", tracking_topic),
    ],
  )

  state_estimationHSV_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="state_estimation_node_HSV",
    name="state_estimation_node_HSV",
    parameters=[common_config, camera_info_config, silo_config],
    remappings=[
      ("yolo/tracking", tracking_topic),
    ],
  )

  absolute_silo_state_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="absolute_silo_state_node",
    name="absolute_silo_state_node",
    remappings=[
      ("/aligned_silo", aligned_silo_topic),
    ],
    parameters=[camera_info_config],
  )

  silos_marker_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="silos_marker_node",
    name="silos_marker_node",
    parameters=[common_config, silo_config],
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(tracking_topic_cmd)
  ld.add_action(aligned_silo_topic_cmd)

  # ld.add_action(state_estimationHSV_node_cmd)
  ld.add_action(state_estimation_node_cmd)
  ld.add_action(absolute_silo_state_node_cmd)
  ld.add_action(silos_marker_node_cmd)

  return ld
