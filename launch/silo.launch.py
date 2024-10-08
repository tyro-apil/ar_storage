import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  namespace = "/silo"
  input_image_topic = "image_raw"
  debug_image_topic = "dbg_image"
  tracking_topic = "yolo/tracking"
  model = "silo_team_red.pt"
  tracker = "custom_tracker.yaml"
  baselink_pose_topic = "/odometry/filtered"
  silo_number_topic = "/silo_number"
  aligned_silo_topic = "/aligned_silo"
  game_over_topic = "/is_game_over"
  infer_on = "cuda:0"
  iou = "0.25"
  check_top_service = "/is_ball_at_top"
  silo_check_request_topic = "/silo_check_request"
  silo_check_result_topic = "/silo_check_result"

  cam_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("silo"), "launch"),
        "/cam_driver.launch.py",
      ]
    ),
    launch_arguments={
      "namespace": namespace,
      "input_image_topic": input_image_topic,
      "check_top_service": check_top_service,
      "silo_check_request_topic": silo_check_request_topic,
      "silo_check_result_topic": silo_check_result_topic,
    }.items(),
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
      "model": os.path.join(get_package_share_directory("robot"), "models", f"{model}"),
      "tracker": os.path.join(
        get_package_share_directory("robot"), "config", f"{tracker}"
      ),
      "device": infer_on,
      "iou": iou,
    }.items(),
  )

  debug = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("silo"), "launch"),
        "/debug.launch.py",
      ]
    ),
    launch_arguments={
      "namespace": namespace,  # By default, the namespace is set to 'yolo'
      "input_image_topic": namespace + "/" + input_image_topic,
      "debug_image_topic": namespace + "/yolo/" + debug_image_topic,
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

  fake_publishers = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("silo"), "launch"),
        "/fake_publishers.launch.py",
      ]
    ),
    launch_arguments={
      "namespace": namespace,
      "pose_topic": baselink_pose_topic,
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
      "namespace": namespace,
      "tracking_topic": namespace + "/" + tracking_topic,
      "aligned_silo_topic": aligned_silo_topic,
    }.items(),
  )

  silo_goal = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(get_package_share_directory("silo"), "launch"),
        "/goal.launch.py",
      ]
    ),
    launch_arguments={
      "namespace": namespace,
      "pose_topic": baselink_pose_topic,
      "silo_number_topic": silo_number_topic,
      "game_over_topic": game_over_topic,
    }.items(),
  )

  ld = LaunchDescription()

  ld.add_action(transforms)
  # ld.add_action(fake_publishers)
  ld.add_action(cam_driver)
  ld.add_action(yolov8_bringup)
  ld.add_action(debug)
  ld.add_action(state_estimation)
  ld.add_action(silo_goal)

  return ld
