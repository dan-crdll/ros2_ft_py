import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__("my_first_node")

        self.get_logger().info("My First Node Created!")

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.shutdown()