# ROS2 Node Command Line Interface

This guide explains the basic commands to interact with ROS2 nodes through the command line interface.

## Prerequisites
- ROS2 installed (see [Installation Guide](../1_installation/))
- Terminal with ROS2 environment sourced

## Basic ROS2 Node Commands

### List all active nodes
```bash
ros2 node list
```

### Get information about a node
```bash
ros2 node info /node_name
```

### Run a node from a package
```bash
ros2 run package_name executable_name
```

## Examples

### Run the built-in turtlesim node
```bash
ros2 run turtlesim turtlesim_node
```

### Run the turtle teleop control
In another terminal:
```bash
ros2 run turtlesim turtle_teleop_key
```

### List running nodes
```bash
ros2 node list
```
Output should include:
```
/teleop_turtle
/turtlesim
```

### Get information about the turtlesim node
```bash
ros2 node info /turtlesim
```

## Node Namespaces and Remapping

### Run a node with a custom name
```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

### Run a node in a namespace
```bash
ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/my_namespace
```

## Useful Commands

### Run a node with parameter settings
```bash
ros2 run package_name node_executable --ros-args -p param_name:=param_value
```

### Run a node using a launch file
```bash
ros2 launch package_name launch_file.py
```

## Practice Exercises

1. Start turtlesim and explore its nodes
2. Run multiple turtlesim instances with different namespaces
3. Use remapping to change node names
4. View node information for different nodes

## Next Steps
Move on to creating a [Python Package](../3_python_package/) for ROS2. 