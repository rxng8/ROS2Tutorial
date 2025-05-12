# ROS2 Topics

This tutorial explains the concept of topics in ROS2 and how they facilitate communication between nodes.

## Prerequisites
- ROS2 installed (see [Installation Guide](../1_installation/))
- Basic understanding of ROS2 nodes (see [ROS2 Nodes in Python](../4_ros2_node_python/))

## What are Topics?

Topics are named buses over which nodes exchange messages. They implement a publish/subscribe communication model, allowing multiple publishers and subscribers to connect.

![Topic Model](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherSingleSubscriber.gif)

## Key Characteristics of Topics

- **Many-to-many** - Multiple publishers and subscribers can use the same topic
- **Anonymous** - Publishers don't know which nodes are subscribing, and subscribers don't know which nodes are publishing
- **Asynchronous** - Publishers and subscribers operate independently
- **Unidirectional** - Data flows from publishers to subscribers 
- **Typed** - Each topic is strongly associated with a specific message type

## Message Types

Topics transmit messages of a specific type. ROS2 comes with standard message types in packages like:
- `std_msgs` - Simple data types (Bool, String, Int32, etc.)
- `geometry_msgs` - Geometric primitives (Point, Vector3, Pose, etc.)
- `sensor_msgs` - Sensor data (Image, LaserScan, Imu, etc.)

## Topic Commands

### List all active topics
```bash
ros2 topic list
```

### Show information about a topic
```bash
ros2 topic info /topic_name
```

### Echo (print) messages on a topic
```bash
ros2 topic echo /topic_name
```

### Show the message type of a topic
```bash
ros2 topic type /topic_name
```

### Show the interfaces (messages) available
```bash
ros2 interface list
```

### Show details of a message type
```bash
ros2 interface show std_msgs/msg/String
```

### Publish a message to a topic from the command line
```bash
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Xin chào từ command line'"
```

### Monitor topic rate
```bash
ros2 topic hz /topic_name
```

## Examples

### Example 1: Viewing TurtleSim Topics

Start TurtleSim:
```bash
ros2 run turtlesim turtlesim_node
```

List topics:
```bash
ros2 topic list
```

Output:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Check topic info:
```bash
ros2 topic info /turtle1/cmd_vel
```

### Example 2: Publishing to TurtleSim

Make the turtle move by publishing to the velocity topic:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Make the turtle move in a circle continuously:
```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

## Topic QoS (Quality of Service)

ROS2 introduces QoS profiles for topics, allowing fine-grained control over message delivery:

- **Reliability**: Best effort vs. reliable delivery
- **History**: How many old messages to keep
- **Durability**: Whether to store messages for late-joining subscribers
- **Deadline**: Maximum expected period between messages
- **Lifespan**: How long messages are valid

Example setting QoS:
```bash
ros2 topic pub --qos-reliability reliable --qos-durability transient_local /topic_name std_msgs/msg/String "data: 'message'"
```

## Creating Custom Message Types

While not covered in this tutorial, ROS2 allows creating custom message types in your packages. These define the data structure for your specific application needs.

## Next Steps
Move on to implementing a [Publisher](../6_publisher/) to send messages on topics.