import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
  namespace = "/silo"
  team_color = "blue"
  input_image_topic = "image_raw"
  tracking_topic = "yolo/tracking"
  model = "o_blunder.pt"
  tracker = "custom_tracker.yaml"
  baselink_pose_topic = "/odometry/filtered"

  v4l2_camera_params = os.path.join(
    get_package_share_directory("silo"), "config", "v4l2_params.yaml"
  )

  cam_driver = Node(
    package="v4l2_camera",
    namespace=namespace,
    executable="v4l2_camera_node",
    name="v4l2_camera",
    output="screen",
    parameters=[v4l2_camera_params],
  )

  yolov8_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("yolov8_bringup"), "launch"),
        "/yolov8.launch.py",
      ]
    ),
    launch_arguments={
      "namespace": namespace + "/yolo",  # By default, the namespace is set to 'yolo'
      "input_image_topic": namespace + "/" + input_image_topic,
      "model": os.path.join(
        get_package_share_directory("yolov8_ros"), "models", f"{model}"
      ),
      "tracker": os.path.join(
        get_package_share_directory("yolov8_ros"), "config", f"{tracker}"
      ),
    }.items(),
  )

  transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("silo"), "launch"),
        "/transforms.launch.py",
      ]
    ),
    launch_arguments={
      "namespace": namespace,
    }.items(),
  )

  state_estimation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("silo"), "launch"),
        "/state_estimation.launch.py",
      ]
    ),
    launch_arguments={
      "team_color": team_color,
      "namespace": namespace,
      "pose_topic": baselink_pose_topic,
      "tracking_topic": namespace + "/" + tracking_topic,
    }.items(),
  )

  return LaunchDescription([cam_driver, transforms, yolov8_bringup, state_estimation])
