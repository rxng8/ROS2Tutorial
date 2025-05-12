# ROS2 Nodes in Python

This tutorial explains how to create and work with ROS2 nodes in Python.

## Prerequisites
- ROS2 installed (see [Installation Guide](../1_installation/))
- Basic knowledge of Python and ROS2 packages (see [Python Package Creation](../2_ros2_node_cli/))

## Understanding ROS2 Nodes

A node in ROS2 is a process that performs computation. Nodes are designed to operate at a fine-grained scale, with a system typically composed of many nodes. The advantages of this approach include:

- Reduced code complexity
- Fault tolerance (crashes in one node don't affect others)
- Reusability and modularity
- Language-independent implementation

## Creating a Basic Node

Below is a template for creating a basic ROS2 node in Python:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        # The node name is crucial for ROS2 communication
        super().__init__('my_basic_node')
        self.get_logger().info('Xin chào! Node has been created')


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the node
    node = MyNode()
    
    try:
        # Keep the node running until it's explicitly stopped
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle clean shutdown on Ctrl+C
        pass
    finally:
        # Clean up and destroy the node
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Node Components

### Node Lifecycle
1. Initialization - `rclpy.init()`
2. Node creation - `Node()` constructor
3. Execution - `rclpy.spin()`
4. Shutdown - `node.destroy_node()` and `rclpy.shutdown()`

### Key Node Features

#### 1. Logging

ROS2 nodes come with built-in logging capabilities:

```python
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warning('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

#### 2. Parameters

Nodes can have parameters configured at runtime:

```python
# Declare a parameter with a default value
self.declare_parameter('my_param', 'default_value')

# Get parameter value
my_param = self.get_parameter('my_param').value

# Handle parameter changes
self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'my_param':
            self.get_logger().info(f'Parameter changed to: {param.value}')
    return SetParametersResult(successful=True)
```

#### 3. Timers

Create periodic callbacks:

```python
# Create a timer with a period of 1 second
self.timer = self.create_timer(1.0, self.timer_callback)

def timer_callback(self):
    self.get_logger().info('Timer triggered!')
```

## Example Node with Multiple Features

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class AdvancedNode(Node):
    def __init__(self):
        super().__init__('advanced_node')
        self.get_logger().info('Xin chào! Advanced node started')

        # Declare parameters
        self.declare_parameter('interval', 1.0)
        self.interval = self.get_parameter('interval').value

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Create a timer
        self.counter = 0
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'interval':
                self.get_logger().info(f'Updating timer interval to: {param.value}')
                self.interval = param.value
                # Recreate the timer with the new interval
                self.timer.timer_period_ns = int(self.interval * 1e9)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Counter: {self.counter}, Interval: {self.interval}')


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Node

Save the code in a file within your ROS2 package, update the `setup.py` entry points, then build and run:

```bash
# Build the package
colcon build --packages-select my_package

# Source the setup files
source install/setup.bash

# Run the node
ros2 run my_package my_node_executable
```

## Node Commands

List running nodes:
```bash
ros2 node list
```

Get information about a node:
```bash
ros2 node info /node_name
```

## Next Steps
Move on to understanding [ROS2 Topics](../4_topic/) for node communication. 