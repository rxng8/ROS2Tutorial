#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Xin chào! My node has been started')
        
        # Create a timer that calls timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0
        
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer callback called: {self.counter} times')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 