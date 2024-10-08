#! /usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from cv2 import broadcast
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  capture_config = os.path.join(
    get_package_share_directory("silo"), "config", "capture.yaml"
  )

  #
  # ARGS
  #
  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace", default_value="", description="Name of the namespace"
  )

  input_image_topic = LaunchConfiguration("input_image_topic")
  input_image_topic_cmd = DeclareLaunchArgument(
    "input_image_topic",
    default_value="image_raw",
  )

  debug_image_topic = LaunchConfiguration("debug_image_topic")
  debug_image_topic_cmd = DeclareLaunchArgument(
    "debug_image_topic",
    default_value="dbg_image",
  )

  #
  # NODES
  #
  capture_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="capture_node",
    name="capture_node",
    remappings=[
      ("image_raw", input_image_topic),
      ("dbg_image", debug_image_topic),
    ],
    parameters=[capture_config],
  )

  broadcast_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="broadcast_node",
    name="broadcast_node",
    remappings=[
      ("dbg_image", debug_image_topic),
    ],
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(input_image_topic_cmd)
  ld.add_action(debug_image_topic_cmd)

  ld.add_action(capture_node_cmd)
  ld.add_action(broadcast_node_cmd)
  return ld
