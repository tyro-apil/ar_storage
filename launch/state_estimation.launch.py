#! /usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():  
  team_color = LaunchConfiguration('team_color', default='blue')
  team_color_cmd = DeclareLaunchArgument(
    'team_color',
    default_value='blue',
    description='Team color of the robot (blue or red)'
  )

  state_estimation_node_cmd= Node(
    package='silo',
    executable='state_estimation_node',
    name='state_estimation_node',
    parameters=[{'team_color': team_color}],
  )
  
  ld = LaunchDescription()
  
  ld.add_action(team_color_cmd)
  ld.add_action(state_estimation_node_cmd)

  return ld