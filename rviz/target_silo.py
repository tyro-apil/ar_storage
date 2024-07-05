import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class MarkerBroadcaster(Node):
  def __init__(self):
    super().__init__("marker_node")

    self.declare_parameter("team_color", "blue")
    self.declare_parameter("ball_diameter", 0.190)
    self.declare_parameter("silos_x", [0.0] * 5)
    self.declare_parameter("silo_z_min", 0.0)
    self.declare_parameter("silo_z_max", 0.0)
    self.declare_parameter("silo_y", 0.0)
    self.declare_parameter("silo_radius", 0.0)

    self.team_color = (
      self.get_parameter("team_color").get_parameter_value().string_value
    )
    self.ball_diameter = (
      self.get_parameter("ball_diameter").get_parameter_value().double_value
    )
    self.silos_x = (
      self.get_parameter("silos_x").get_parameter_value().double_array_value
    )
    self.silo_z_min = (
      self.get_parameter("silo_z_min").get_parameter_value().double_value
    )
    self.silo_z_max = (
      self.get_parameter("silo_z_max").get_parameter_value().double_value
    )
    self.silo_y = self.get_parameter("silo_y").get_parameter_value().double_value
    self.silo_radius = (
      self.get_parameter("silo_radius").get_parameter_value().double_value
    )

    self.target_silo_subscriber = self.create_subscription(
      UInt8MultiArray, "/silo_number", self.target_received_callback, 10
    )
    self.target_silo_subscriber  # prevent unused variable warning

    self.silos_marker_publisher = self.create_publisher(MarkerArray, "target", 10)
    self.silos_xy = [(x, self.silo_y) for x in self.silos_x]

    # if self.team_color == "red":
    #   self.silos_xy = [(x, self.silo_y) for x in self.silos_x]

    self.get_logger().info("Target silo marker node started")

  def target_received_callback(self, optimal_silos_msg: UInt8MultiArray):
    arrow_markers = MarkerArray()

    for i, silo_number in enumerate(optimal_silos_msg.data):
      arrow = self.create_arrow_marker(priority=i, silo_number=silo_number)

      arrow_markers.markers.append(arrow)

    self.publish_markers(arrow_markers)
    return

  def create_arrow_marker(self, priority, silo_number):
    arrow = Marker()
    arrow.header.frame_id = "map"

    arrow.ns = "silo"
    arrow.id = int(f"{priority}")
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD

    arrow.pose.position.x = self.silos_xy[silo_number - 1][0]
    arrow.pose.position.y = self.silos_xy[silo_number - 1][1] - 1.0
    arrow.pose.position.z = self.silo_z_max / 2

    orientation = R.from_euler("ZYX", [-90.0, 0, 0], degrees=True)
    q_arrow = orientation.as_quat()

    arrow.pose.orientation.x = q_arrow[0]
    arrow.pose.orientation.y = q_arrow[1]
    arrow.pose.orientation.z = q_arrow[2]
    arrow.pose.orientation.w = q_arrow[3]

    arrow.scale.x = 0.5
    arrow.scale.y = 0.1
    arrow.scale.z = 0.1

    if self.team_color == "red":
      arrow.color.r = 0.988
      arrow.color.g = 0.0
      arrow.color.b = 0.0
    elif self.team_color == "blue":
      arrow.color.r = 0.071
      arrow.color.g = 0.310
      arrow.color.b = 0.988

    arrow.color.a = max(0.1, 1.0 - priority * 0.5)

    arrow.lifetime = Duration(seconds=1.0).to_msg()
    return arrow

  def publish_markers(self, marker_array):
    self.silos_marker_publisher.publish(marker_array)
    return


def main(args=None):
  rclpy.init(args=args)

  marker_node = MarkerBroadcaster()

  rclpy.spin(marker_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  marker_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
