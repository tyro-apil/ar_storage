import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TransformPublisher(Node):
  def __init__(self):
    super().__init__("transform_publisher")
    self.publisher_ = self.create_publisher(Odometry, "/odometry/filtered", 10)
    self.timer_ = self.create_timer(1.0, self.publish_transform)

  def publish_transform(self):
    odometry = Odometry()
    odometry.header.stamp = self.get_clock().now().to_msg()
    odometry.header.frame_id = "map"
    odometry.child_frame_id = "base_link"
    odometry.pose.pose.position.x = 0.0  # Set the translation values here
    odometry.pose.pose.position.y = 0.0
    odometry.pose.pose.position.z = 0.0
    odometry.pose.pose.orientation.x = 0.0  # Set the rotation values here
    odometry.pose.pose.orientation.y = 0.0
    odometry.pose.pose.orientation.z = 0.0
    odometry.pose.pose.orientation.w = 1.0

    self.publisher_.publish(odometry)


def main(args=None):
  rclpy.init(args=args)
  transform_publisher = TransformPublisher()
  rclpy.spin(transform_publisher)
  transform_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
