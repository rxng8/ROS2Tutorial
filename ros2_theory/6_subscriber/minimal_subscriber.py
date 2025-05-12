#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscription
        # Arguments: message type, topic name, callback function, queue size
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Counter for received messages
        self.count = 0

        self.get_logger().info('Subscriber node started, waiting for messages...')

    def listener_callback(self, msg):
        self.count += 1
        self.get_logger().info(f'Received message #{self.count}: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Spin the node to execute callbacks
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()