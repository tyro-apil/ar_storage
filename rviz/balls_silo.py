import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from silo_msgs.msg import SiloArray
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

    self.__height_offset = 0.2

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

    self.silos_state_subscriber = self.create_subscription(
      SiloArray, "state_map", self.silos_state_callback, 10
    )
    self.silos_state_subscriber  # prevent unused variable warning

    self.silos_marker_publisher = self.create_publisher(MarkerArray, "silos_marker", 10)
    self.silos_xy = [(x, -self.silo_y) for x in self.silos_x]

    # if self.team_color == "red":
    #   self.silos_xy = [(x, self.silo_y) for x in self.silos_x]

    self.get_logger().info("Silo balls marker node started")

  def silos_state_callback(self, silo_state_msg: SiloArray):
    marker_array = MarkerArray()

    for silo in silo_state_msg.silos:
      silos_marker_array = self.create_silo_markers(silo)

      marker_array.markers.extend(silos_marker_array.markers)

    self.publish_markers(marker_array)
    return

  def create_silo_markers(self, silo):
    silos_marker_array = MarkerArray()

    for i, color in enumerate(silo.state):
      marker = self.create_ball_marker(i + 1, color, silo.index)
      silos_marker_array.markers.append(marker)

    return silos_marker_array

  def create_ball_marker(self, position, color, silo_number):
    marker = Marker()
    marker.header.frame_id = "map"

    marker.ns = "silo"
    marker.id = int(f"{silo_number}{position}")
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = self.silos_xy[silo_number - 1][0]
    marker.pose.position.y = self.silos_xy[silo_number - 1][1]

    match position:
      case 1:
        marker.pose.position.z = (
          self.silo_z_max + self.__height_offset + self.ball_diameter / 2
        )
      case 2:
        marker.pose.position.z = (
          self.silo_z_max
          + self.__height_offset
          + self.ball_diameter / 2
          + self.ball_diameter
        )
      case 3:
        marker.pose.position.z = (
          self.silo_z_max
          + self.__height_offset
          + self.ball_diameter / 2
          + 2 * self.ball_diameter
        )

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = self.ball_diameter
    marker.scale.y = self.ball_diameter
    marker.scale.z = self.ball_diameter

    marker_rgb = {"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.8}
    match color:
      ## Red ball rgb(252,0,0)
      case "R":
        marker_rgb["r"] = 0.988
        marker_rgb["g"] = 0.0
        marker_rgb["b"] = 0.0

      ## Blue ball rgb(18,79,252)
      case "B":
        marker_rgb["r"] = 0.071
        marker_rgb["g"] = 0.310
        marker_rgb["b"] = 0.988

    marker.color.r = marker_rgb["r"]
    marker.color.g = marker_rgb["g"]
    marker.color.b = marker_rgb["b"]
    marker.color.a = marker_rgb["a"]

    marker.lifetime = Duration(seconds=0.50).to_msg()
    return marker

  def create_silo_text_marker(self, silo_number):
    text_marker = Marker()
    text_marker.header.frame_id = "map"

    text_marker.ns = "silo"
    text_marker.id = int(f"{-silo_number}")
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD

    text_marker.pose.position.x = self.silos_xy[silo_number - 1][0]
    text_marker.pose.position.y = self.silos_xy[silo_number - 1][1]
    text_marker.pose.position.z = self.silo_z_max + 0.3

    text_marker.pose.orientation.x = 0.0
    text_marker.pose.orientation.y = 0.0
    text_marker.pose.orientation.z = 0.0
    text_marker.pose.orientation.w = 1.0

    text_marker.scale.x = self.ball_diameter
    text_marker.scale.y = self.ball_diameter
    text_marker.scale.z = self.ball_diameter

    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0

    text_marker.lifetime = Duration(seconds=1.0).to_msg()
    return text_marker

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
