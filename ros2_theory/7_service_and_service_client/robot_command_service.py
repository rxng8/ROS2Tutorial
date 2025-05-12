#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Using a built-in service type for simplicity
from std_msgs.msg import String

import sys
import threading
import time


class RobotCommandService(Node):
    """Service that processes robot commands"""
    def __init__(self):
        super().__init__('robot_command_service')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,  # Service type
            'execute_command',  # Service name
            self.execute_command_callback  # Callback function
        )

        # Create a publisher to report command execution status
        self.status_publisher = self.create_publisher(
            String,
            'command_status',
            10
        )

        self.get_logger().info('Robot Command Service started. Ready to accept commands.')

    def execute_command_callback(self, request, response):
        """Process the command request and return a response"""
        # In this example, we interpret the two integers as command codes:
        # - First integer (a): Command type (1=move, 2=rotate, 3=stop)
        # - Second integer (b): Command value (distance, angle, etc.)

        command_type = request.a
        command_value = request.b

        self.get_logger().info(f'Received command: Type={command_type}, Value={command_value}')

        # Simulate command execution
        if command_type == 1:  # Move command
            status_msg = String()
            status_msg.data = f'Di chuyển robot {command_value} đơn vị'
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f'Moving robot {command_value} units')
            time.sleep(1.0)  # Simulate execution time

        elif command_type == 2:  # Rotate command
            status_msg = String()
            status_msg.data = f'Xoay robot {command_value} độ'
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f'Rotating robot {command_value} degrees')
            time.sleep(1.5)  # Simulate execution time

        elif command_type == 3:  # Stop command
            status_msg = String()
            status_msg.data = 'Dừng robot'
            self.status_publisher.publish(status_msg)
            self.get_logger().info('Stopping robot')
            time.sleep(0.5)  # Simulate execution time

        else:
            status_msg = String()
            status_msg.data = f'Lệnh không hợp lệ: {command_type}'
            self.status_publisher.publish(status_msg)
            self.get_logger().warn(f'Invalid command type: {command_type}')
            response.sum = -1  # Error code
            return response

        # Return success code (in a real system, this would be more meaningful)
        response.sum = command_type * 1000 + command_value
        return response


class RobotCommandClient(Node):
    """Client that sends commands to the robot service"""
    def __init__(self):
        super().__init__('robot_command_client')

        # Create a client
        self.client = self.create_client(
            AddTwoInts,  # Service type
            'execute_command'  # Service name
        )

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Subscribe to command status updates
        self.status_subscription = self.create_subscription(
            String,
            'command_status',
            self.status_callback,
            10
        )

        self.get_logger().info('Robot Command Client ready')

    def status_callback(self, msg):
        """Process the status messages"""
        self.get_logger().info(f'Status update: {msg.data}')

    def send_command(self, command_type, command_value):
        """Send a command to the service"""
        # Create request
        request = AddTwoInts.Request()
        request.a = command_type
        request.b = command_value

        # Send the request
        self.get_logger().info(f'Sending command: Type={command_type}, Value={command_value}')
        future = self.client.call_async(request)

        # Add a callback for when the service call is complete
        future.add_done_callback(self.command_response_callback)

    def command_response_callback(self, future):
        """Process the response from the service"""
        try:
            response = future.result()
            self.get_logger().info(f'Response received: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def service_main(args=None):
    """Main function for running just the service"""
    rclpy.init(args=args)

    service = RobotCommandService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()


def client_main(args=None):
    """Main function for running just the client with demo commands"""
    rclpy.init(args=args)

    client = RobotCommandClient()

    # Spin in a separate thread so we can send commands
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Send a few demo commands
        commands = [
            (1, 100),   # Move 100 units
            (2, 90),    # Rotate 90 degrees
            (3, 0),     # Stop
            (1, 50),    # Move 50 units
            # (99, 0)     # Invalid command
        ]

        for cmd_type, cmd_value in commands:
            client.send_command(cmd_type, cmd_value)
            time.sleep(2.0)  # Wait a bit between commands

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()


def main(args=None):
    """Main function that runs both service and client"""
    if len(sys.argv) < 2:
        print('Usage: robot_command_service.py [service|client]')
        return

    # Run either the service or client based on command-line argument
    if sys.argv[1] == 'service':
        service_main(args)
    elif sys.argv[1] == 'client':
        client_main(args)
    else:
        print('Invalid argument. Use "service" or "client"')


if __name__ == '__main__':
    main()