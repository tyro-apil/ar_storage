import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  namespace = 'silo'
  input_image_topic = 'image_raw'
  model = 'o_blunder.pt'
  tracker_yaml_path = '/home/apil/main_ws/src/yolov8_ros/yolov8_ros/config/custom_tracker.yaml'
  
  v4l2_camera_params = os.path.join(
    get_package_share_directory('silo'),
    'config',
    'v4l2_params.yaml'
  ) 

  cam_driver = Node(
    package='v4l2_camera',
    executable='v4l2_camera_node',
    name='v4l2_camera',
    output='screen',
    parameters=[v4l2_camera_params],
  )
  cam_driver = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      cam_driver,
    ]
  )
  
  yolov8_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('yolov8_bringup'), 'launch'),
      '/yolov8.launch.py']),
    launch_arguments={
      'namespace': '',                    # By default, the namespace is set to 'yolo'
      'input_image_topic': input_image_topic,
      'model': model,
      'tracker': tracker_yaml_path
    }.items()
    )
  yolov8_bringup = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      yolov8_bringup,
    ]
   )
  
  transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('silo'), 'launch'),
      '/transforms.launch.py'])
    )
  transforms = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      transforms,
    ]
  )

  state_estimation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('silo'), 'launch'),
      '/state_estimation.launch.py'])
    )
  state_estimation = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      state_estimation,
    ]
  )
  

  return LaunchDescription([
    cam_driver,
    transforms,
    yolov8_bringup,
    state_estimation
  ])