# ROS2 Publishers

This tutorial explains how to create and implement publishers in ROS2, which allow nodes to send messages on topics.

## Prerequisites
- ROS2 installed (see [Installation Guide](../1_installation/))
- Understanding of ROS2 nodes and topics (see [ROS2 Nodes in Python](../4_ros2_node_python/) and [Topics](../5_topic/))

## Publishers in ROS2

A publisher is an object that allows a node to send messages on a ROS2 topic. When creating a publisher, you need to:

1. Specify the message type
2. Specify the topic name 
3. Set the QoS settings (optional)

## Basic Publisher Example

```python
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
    rclpy.spin(minimal_publisher)

    # Clean up
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Publishing Different Message Types

ROS2 has many predefined message types that you can use:

### Publishing Int32
```python
from std_msgs.msg import Int32

self.publisher_ = self.create_publisher(Int32, 'counter', 10)

# When publishing:
msg = Int32()
msg.data = 42
self.publisher_.publish(msg)
```

### Publishing Twist (for robot movement)
```python
from geometry_msgs.msg import Twist

self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

# When publishing:
msg = Twist()
msg.linear.x = 0.5  # Move forward at 0.5 m/s
msg.angular.z = 0.2  # Turn at 0.2 rad/s
self.publisher_.publish(msg)
```

## Quality of Service (QoS)

You can customize the publisher's QoS settings:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Create a custom QoS profile
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(String, 'topic', qos_profile=qos_profile)
```

## Advanced Example: Natural Language Robot Control

In the `sim_pub_sub_llm.py` file, we provide an advanced example of publishers in action. This script:

1. Uses a ROS2 publisher to send joint trajectory commands to a robot
2. Accepts natural language instructions like "move left" or "open gripper"
3. Uses the ChatGPT API to translate these commands into robot joint movements

Key publisher implementation:

```python
# Publishers for arm and gripper control
self.arm_pub = self.create_publisher(
    JointTrajectory,
    '/px100/arm_controller/joint_trajectory',
    10
)

self.gripper_pub = self.create_publisher(
    JointTrajectory,
    '/px100/gripper_controller/joint_trajectory',
    10
)
```

### Using the Natural Language Control Script

To run the script:

1. Install the OpenAI Python package:
   ```bash
   pip install openai pygame
   ```

2. Set your OpenAI API key:
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

3. Run the script:
   ```bash
   python3 sim_pub_sub_llm.py
   ```

4. Type natural language commands like:
   - "move the waist left"
   - "open the gripper"
   - "close the gripper"
   - "move the elbow up"
   - "return to home position"

The script translates these commands using ChatGPT and publishes the appropriate trajectory commands to the robot.

## Best Practices for Publishers

1. **Choose appropriate QoS settings** - For critical data, use RELIABLE delivery
2. **Use descriptive topic names** - Use namespaces and clear naming
3. **Check subscriber connections** - Use `get_subscription_count()` before expensive computations
4. **Understand message lifecycles** - Be aware of when message data is copied/referenced
5. **Rate limiting** - Use timers to control publishing frequency
6. **Error handling** - Handle publishing failures gracefully

## Next Steps
Move on to implementing a [Subscriber](../7_subscriber/) to receive messages from topics. 