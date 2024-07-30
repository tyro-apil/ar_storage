#! /usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  check_top_config = os.path.join(
    get_package_share_directory("silo"), "config", "check_top.yaml"
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
    description="Name of the input image topic",
  )

  check_top_service = LaunchConfiguration("check_top_service")
  check_top_service_cmd = DeclareLaunchArgument(
    "check_top_service",
    default_value="/is_ball_at_top",
  )

  #
  # NODES
  #
  cam_driver_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="image_receiver_node",
    name="image_receiver_node",
    remappings=[
      ("image_raw", input_image_topic),
      ("/is_ball_at_top", check_top_service)
    ],
    parameters=[check_top_config]
  )

  ld = LaunchDescription()
  # Add arguments
  ld.add_action(namespace_cmd)
  ld.add_action(input_image_topic_cmd)
  ld.add_action(check_top_service_cmd)
  # Add nodes
  ld.add_action(cam_driver_node_cmd)

  return ld
