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
    get_package_share_directory("robot"), "config", "common.yaml"
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

  silo_number_topic = LaunchConfiguration("silo_number_topic")
  silo_number_topic_cmd = DeclareLaunchArgument(
    "silo_number_topic",
    default_value="/silo_number",
  )

  silo_selection_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="silo_selection_node",
    name="silo_selection_node",
    remappings=[
      ("/odometry/filtered", pose_topic),
      ("/silo_number", silo_number_topic),
    ],
    parameters=[silo_config, common_config],
  )

  target_node_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="target_node",
    name="target_node",
    remappings=[
      ("/silo_number", silo_number_topic),
    ],
    parameters=[silo_config, common_config],
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(pose_topic_cmd)
  ld.add_action(silo_number_topic_cmd)

  ld.add_action(silo_selection_node_cmd)
  ld.add_action(target_node_cmd)

  return ld
