# ROS2 Robotics Tutorial

## Overview
This repository contains a comprehensive guide and examples for learning Robot Operating System 2 (ROS2). The tutorial is organized in a step-by-step manner, starting from installation to more advanced topics.

## Repository Structure

### ros2_theory
A complete learning path for ROS2 fundamentals:

1. [Installation](./ros2_theory/1_installation/) - Setting up ROS2
2. [ROS2 Node CLI](./ros2_theory/2_ros2_node_cli/) - Command line interface for ROS2 nodes
3. [Python Package](./ros2_theory/3_python_package/) - Creating Python packages for ROS2
4. [ROS2 Node Python](./ros2_theory/4_ros2_node_python/) - Implementing ROS2 nodes in Python
5. [Topics](./ros2_theory/5_topic/) - Understanding ROS2 topics
6. [Publisher](./ros2_theory/6_publisher/) - Creating publishers in ROS2
7. [Subscriber](./ros2_theory/7_subscriber/) - Implementing subscribers
8. [Publisher-Subscriber Example](./ros2_theory/8_publisher_subscriber_example/) - Complete example of pub-sub pattern
9. [Service and Service Client](./ros2_theory/9_service_and_service_client/) - Working with ROS2 services
10. [URDF and Launch](./ros2_theory/10_URDF_and_launch/) - Creating robot descriptions and launch files

### px100_demo
Practical examples using the PincherX 100 robot arm with ROS2.

## Getting Started
1. Start with the [Installation guide](./ros2_theory/1_installation/)
2. Follow the numbered tutorials in order
3. Experiment with the practical examples in the px100_demo directory

## Example

```python
# Ví dụ về nút ROS2 đơn giản
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Xin chào từ nút ROS2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Installation

```bash

# Setup python environment
bash setup-venv.sh

# Source the environment to run python scripts
source .all

# Source the environment to run ros2 commands
source .ros

```

