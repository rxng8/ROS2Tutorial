#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher object
        # Arguments: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer for publishing every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for message numbering
        self.i = 0

    def timer_callback(self):
        # Create a message
        msg = String()
        msg.data = f'Xin chào thế giới: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log that we published
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    minimal_publisher = MinimalPublisher()

    try:
        # Spin the node to execute callbacks
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()