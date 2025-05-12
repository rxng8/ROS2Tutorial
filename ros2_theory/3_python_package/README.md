# Creating a Python Package for ROS2

This tutorial explains how to create a basic Python package in ROS2.

## Prerequisites
- ROS2 installed (see [Installation Guide](../1_installation/))
- A workspace directory

## Structure of a ROS2 Python Package

```
my_package/
├── my_package/
│   ├── __init__.py
│   └── my_node.py
├── resource/
│   └── my_package
├── package.xml
├── setup.cfg
└── setup.py
```

## Step 1: Create the Package Structure

```bash
# Go to your workspace src directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package using ros2 cli
ros2 pkg create --build-type ament_python my_package

# Alternatively, create the structure manually
mkdir -p my_package/my_package
touch my_package/my_package/__init__.py
mkdir -p my_package/resource
touch my_package/resource/my_package
```

## Step 2: Configure Package Files

### `package.xml`
Define package metadata and dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Example ROS2 Python package</description>
  <maintainer email="user@email.com">user</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `setup.py`
Configure how Python will build and install the package:

```python
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    description='Example ROS2 Python package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
        ],
    },
)
```

## Step 3: Create Your Node

Create a Python file in the `my_package` inner directory:

```python
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
```

## Step 4: Build and Run

Build your package:

```bash
# From the workspace root directory
cd ~/ros2_ws
colcon build --packages-select my_package
```

Source the install setup:

```bash
source install/setup.bash
```

Run your node:

```bash
ros2 run my_package my_node
```

## Expected Output

When running the node, you should see output like:

```
[INFO] [my_node]: Xin chào! My node has been started
[INFO] [my_node]: Timer callback called: 1 times
[INFO] [my_node]: Timer callback called: 2 times
[INFO] [my_node]: Timer callback called: 3 times
...
```

## Next Steps
Move on to understanding [ROS2 Nodes in Python](../4_ros2_node_python/) in more detail. 