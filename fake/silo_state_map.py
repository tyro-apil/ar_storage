import rclpy
from rclpy.node import Node
from silo_msgs.msg import Silo, SiloArray


class FakeSiloStatePublisher(Node):
  def __init__(self):
    super().__init__("silo_state_publisher")
    self.silo_state_publisher = self.create_publisher(SiloArray, "state_map", 10)
    timer_period = 0.05  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.states = ["", "", "", "", ""]
    self.silo_msg = SiloArray()

  def timer_callback(self):
    silo1 = Silo()
    silo1.id = 1
    silo1.state = self.states[0]

    silo2 = Silo()
    silo2.id = 2
    silo2.state = self.states[1]

    silo3 = Silo()
    silo3.id = 3
    silo3.state = self.states[2]

    silo4 = Silo()
    silo4.id = 4
    silo4.state = self.states[3]

    silo5 = Silo()
    silo5.id = 5
    silo5.state = self.states[4]

    self.silo_msg.silos = [silo1, silo2, silo3, silo4, silo5]
    self.silo_state_publisher.publish(self.silo_msg)


def main(args=None):
  rclpy.init(args=args)

  fake_silo_state_publisher = FakeSiloStatePublisher()

  rclpy.spin(fake_silo_state_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  fake_silo_state_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
