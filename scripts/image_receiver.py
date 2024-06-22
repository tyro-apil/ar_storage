#!/usr/bin/env python3

import socket

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

PORT = 12345


class ImageReceiverNode(Node):
  def __init__(self):
    super().__init__("image_receiver_node")
    self.publisher_ = self.create_publisher(Image, "image_raw", 10)
    self.bridge = CvBridge()

    # Set up socket
    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.server_socket.bind(("0.0.0.0", PORT))
    self.server_socket.listen(1)
    self.get_logger().info(f"Listening on port {PORT}")

    # Run receive loop in constructor
    self.receive_loop()

  def receive_loop(self):
    try:
      client_socket, addr = self.server_socket.accept()
      self.get_logger().info(f"Connection from {addr}")

      while True:
        # Receive image size
        size_bytes = client_socket.recv(4)
        size = int.from_bytes(size_bytes, byteorder="big")

        # Receive image data
        img_data = b""
        while len(img_data) < size:
          chunk = client_socket.recv(size - len(img_data))
          if not chunk:
            raise RuntimeError("Socket connection broken")
          img_data += chunk

        # Decode image
        np_arr = np.frombuffer(img_data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Publish image as ROS message
        ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(ros_image_msg)

    except Exception as e:
      self.get_logger().error(f"Error receiving image: {str(e)}")

  def destroy_node(self):
    self.server_socket.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = ImageReceiverNode()
  rclpy.spin(node)  # This is needed to keep the node alive until shutdown
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
