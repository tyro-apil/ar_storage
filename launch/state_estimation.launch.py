#! /usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():  
  state_estimation_node_cmd= Node(
    package='silo',
    executable='state_estimation_node',
    name='state_estimation_node',
  )
  
  ld = LaunchDescription()
  
  ld.add_action(state_estimation_node_cmd)

  return ld