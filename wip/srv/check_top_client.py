import threading

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger


class CheckTopClient(Node):
  def __init__(self):
    super().__init__("check_top_client")
    self.cli = self.create_client(Trigger, "/is_ball_on_top")

    while not self.cli.wait_for_service(timeout_sec=0.5):
      self.get_logger().info("service not available, waiting again...")

    self.future: Future = None

  def send_request(self):
    if self.future is not None and not self.future.done():
      self.future.cancel()  # Cancel the future. The callback will be called with Future.result == None.

      self.get_logger().info(
        "Service Future canceled. The Node took too long to process the service call."
      )

    req = Trigger.Request()
    self.future = self.cli.call_async(req)
    self.future.add_done_callback(self.process_response)

    # Start a timer for the timeout
    self.timeout_thread = threading.Timer(1.0, self.check_timeout)
    self.timeout_thread.start()

  def process_response(self, future: Future):
    if self.timeout_thread.is_alive():
      self.timeout_thread.cancel()

    response: Trigger.Response = future.result()
    if response is None:
      self.get_logger().info("Response is None")
      return
    self.get_logger().info(f"Success: {response.success}")
    self.get_logger().info(f"Response: {response.message}")

  def check_timeout(self):
    if self.future and not self.future.done():
      self.future.cancel()
      self.get_logger().info("Service call timed out.")


def main(args=None):
  try:
    rclpy.init(args=args)

    check_top_client = CheckTopClient()

    rclpy.spin(check_top_client)

  except KeyboardInterrupt:
    pass

  except Exception as e:
    print(e)


if __name__ == "__main__":
  main()
