#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

  fake_map2base_link_odom_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="fake_map2base_link_node",
    name="fake_map2base_link_node",
    remappings=[("/odometry/filtered", pose_topic)],
  )

  fake_silo_state_cmd = Node(
    package="silo",
    namespace=namespace,
    executable="fake_silo_state_publisher_node",
    name="fake_silo_state_publisher_node",
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(pose_topic_cmd)

  ld.add_action(fake_map2base_link_odom_cmd)
  ld.add_action(fake_silo_state_cmd)

  return ld
