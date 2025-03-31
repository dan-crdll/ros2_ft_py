import rclpy
from rclpy.node import Node

class MySecondNode(Node):
    def __init__(self):
        super().__init__("my_second_node")

        self.get_logger().info("My Second Node Created!")

        timer_period = 2.0  # seconds
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Timer callback executed {self.counter} times!")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MySecondNode()
    rclpy.spin(node)
    rclpy.shutdown()