import rclpy
from rclpy.node import Node

from silo_msgs.msg import Silo, SiloArray
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import cos

class SiloMatching(Node):
  def __init__(self):
    super().__init__('silo_match')
    self.declare_parameter("team_color", "blue")
    self.declare_parameter("silos_y", [0.0]*5)
    self.declare_parameter("silo_z_min", 0.0)
    self.declare_parameter("silo_z_max", 0.0)
    self.declare_parameter("silo_x", 0.0)
    self.declare_parameter("silo_radius", 0.0)
    self.declare_parameter("team_color", "blue")
    self.declare_parameter("translation", [0.0]*3)  # camera translation
    self.declare_parameter("ypr", [0.0]*3)  # camera orientation
    self.declare_parameter("k", [0.0]*9)  # camera matrix

    self.silos_state_map_publisher = self.create_publisher(
      SiloArray,
      "silo_state_map",
      10
    )
    self.silos_state_subscriber = self.create_subscription(
      SiloArray,
      "silo_state",
      self.silo_state_callback,
      10
    )
    self.silos_state_subscriber
    self.baselink_pose_subscriber = self.create_subscription(
      Odometry,
      "odometry/filtered",
      self.baselink_pose_callback,
      10
    )
    self.baselink_pose_subscriber

    self.translation_map2base = None
    self.quaternion_map2base = None
    self.tf_base2map = None
    self.cam_translation = self.get_parameter("translation").get_parameter_value().double_array_value
    self.cam_ypr = self.get_parameter("ypr").get_parameter_value().double_array_value
    cam_orientation = R.from_euler("ZYX", self.cam_ypr, degrees=True)
    tf_base2cam = np.eye(4)
    tf_base2cam[:3, 3] = self.cam_translation
    tf_base2cam[:3, :3] = cam_orientation.as_matrix()
    self.tf_cam2base = np.linalg.inv(tf_base2cam)

    self.silos_y = self.get_parameter("silos_y").get_parameter_value().double_array_value
    self.silo_z_min = self.get_parameter("silo_z_min").get_parameter_value().double_value 
    self.silo_z_max = self.get_parameter("silo_z_max").get_parameter_value().double_value 
    self.silo_x = self.get_parameter("silo_x").get_parameter_value().double_value 
    self.silo_radius = self.get_parameter("silo_radius").get_parameter_value().double_value

    self.get_logger().info(f"Silo matching node started.")

  def get_silos_y_limits_map(self):
    y_limits = []
    gap = 0.720
    line = 0.030
    silo_1_center = -(gap + line)*2
    silo_1_limits = {
      "min": silo_1_center - self.silo_radius,
      "max": silo_1_center + self.silo_radius
    }
    silo_2_center = -(gap + line)*1
    silo_2_limits = {
      "min": silo_2_center - self.silo_radius,
      "max": silo_2_center + self.silo_radius
    }
    silo_3_center = (gap + line)*0
    silo_3_limits = {
      "min": silo_3_center - self.silo_radius,
      "max": silo_3_center + self.silo_radius
    }
    silo_4_center = (gap + line)*1
    silo_4_limits = {
      "min": silo_4_center - self.silo_radius,
      "max": silo_4_center + self.silo_radius
    }
    silo_5_center = (gap + line)*2
    silo_5_limits = {
      "min": silo_5_center - self.silo_radius,
      "max": silo_5_center + self.silo_radius
    }
    y_limits.append(silo_1_limits)
    y_limits.append(silo_2_limits)
    y_limits.append(silo_3_limits)
    y_limits.append(silo_4_limits)
    y_limits.append(silo_5_limits)
    return y_limits

  def silo_state_callback(self, silos_state_msg: SiloArray):
    # start only if base2map transform is available
    if self.tf_base2map is None:
      return
    
    silos_state_map_msg = SiloArray()
    # iterate through detected silos and assign absolute index to them
    for i, silo in enumerate(silos_state_msg.silos):
      silo_map = silo
      silo_xywh = silo.xywh[0], silo.xywh[1], silo.xywh[2], silo.xywh[3]
      silo_absolute_index = self.get_silo_absolute_index(silo_xywh[:2])
      silo_map.index = silo_absolute_index
      silos_state_map_msg.silos.append(silo_map)

    # publish new state of silos with absolute index
    self.silos_state_map_publisher.publish(silos_state_map_msg)

  def get_silo_absolute_index(self, silo_center):
    silo_center_cam = self.pixel2cam(silo_center)
    silo_center_base = self.cam2base(silo_center_cam)
    silo_center_map = self.base2map(silo_center_base)
    for i, y_limits in enumerate(self.silos_y_limits_map):
      if y_limits["min"] <= silo_center_map[1] <= y_limits["max"]:
        return i+1
    self.get_logger().warn(f"Silo center {silo_center_map} is not in any silo")
    return -1

  def baselink_pose_callback(self, pose_msg: Odometry):
    self.translation_map2base = np.zeros(3)
    self.translation_map2base[0] = pose_msg.pose.pose.position.x
    self.translation_map2base[1] = pose_msg.pose.pose.position.y
    self.translation_map2base[2] = pose_msg.pose.pose.position.z

    self.quaternion_map2base = np.zeros(4)
    self.quaternion_map2base[0] = pose_msg.pose.pose.orientation.x
    self.quaternion_map2base[1] = pose_msg.pose.pose.orientation.y
    self.quaternion_map2base[2] = pose_msg.pose.pose.orientation.z
    self.quaternion_map2base[3] = pose_msg.pose.pose.orientation.w

    tf_map2base = np.eye(4)
    tf_map2base[:3,:3] = R.from_quat(self.quaternion_map2base).as_matrix()
    tf_map2base[:3, 3] = self.translation_map2base

    self.tf_base2map = np.linalg.inv(tf_map2base)

  def pixel2cam(self, pixel_coordinates):
    pixel_coordinates.append(1.0)
    pixel_coordinates_np = np.array(pixel_coordinates).reshape((-1,1))
    cam_coordinates_normalized = self.inv_camera_matrix @ pixel_coordinates_np
    depth = self.get_depth_coordinate()
    cam_coordinates = depth * cam_coordinates_normalized
    return cam_coordinates.reshape(-1)
  
  def cam2base(self, cam_coordinates):
    cam_coordinates.append(1.0)
    cam_coordinates_homogeneous = np.array(cam_coordinates).reshape((-1,1))
    base_coordinates_homogeneous = self.tf_cam2base @ cam_coordinates_homogeneous
    base_coordinates = base_coordinates_homogeneous / base_coordinates_homogeneous[-1]
    return base_coordinates[:-1].reshape(-1)
  
  def base2map(self, base_coordinates):
    base_coordinates.append(1.0)
    base_coordinates_homogeneous = np.array(base_coordinates).reshape((-1,1))
    map_coordinates_homogeneous = self.tf_base2map @ base_coordinates_homogeneous
    map_coordinates = map_coordinates_homogeneous / map_coordinates_homogeneous[-1]
    return map_coordinates[:-1].reshape(-1)
  
  def get_depth_coordinate(self):
    map2silo = -(0.539+1+0.7+0.7+1.225)
    x_depth = map2silo + self.cam_translation[0] + self.translation_map2base[0]
    ypr = R.from_quat(self.quaternion_map2base).as_euler("ZYX")
    try:
      depth_cam = abs(x_depth / cos(ypr[0]))
    except:
      self.get_logger().warn("Division by zero")
      return 0.0
    return depth_cam

  # def display_state(self, silos_state_map):
  #   log = ""
  #   for i, silo in enumerate(self.state):
  #     log += f"Silo{i+1}: {silo} | "
  #   self.get_logger().info(log)

def main(args=None):
  rclpy.init(args=args)

  state_estimation_node = SiloMatching()

  rclpy.spin(state_estimation_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  state_estimation_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()