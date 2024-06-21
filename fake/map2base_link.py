import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


class TransformPublisher(Node):
  def __init__(self):
    super().__init__("transform_publisher")
    self.publisher_ = self.create_publisher(TransformStamped, "tf", 10)
    self.timer_ = self.create_timer(1.0, self.publish_transform)

  def publish_transform(self):
    transform = TransformStamped()
    transform.header.stamp = self.get_clock().now().to_msg()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = 0.0  # Set the translation values here
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0  # Set the rotation values here
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    self.publisher_.publish(transform)


def main(args=None):
  rclpy.init(args=args)
  transform_publisher = TransformPublisher()
  rclpy.spin(transform_publisher)
  transform_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
