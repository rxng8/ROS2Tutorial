#!/usr/bin/env python3

import pathlib
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Path to the urdf file
    # urdf_path = os.path.join(os.getcwd(), 'ros2_theory/8_URDF_and_launch/simple_robot.urdf')
    urdf_path = str((pathlib.Path(__file__).parent / 'simple_robot.urdf').absolute())

    # Load the URDF as a parameter
    robot_description = {'robot_description': open(urdf_path).read()}

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # RViz configuration
    # rviz_config_path = os.path.join(os.getcwd(), 'ros2_theory/8_URDF_and_launch/robot.rviz')
    rviz_config_path = str((pathlib.Path(__file__).parent / 'robot.rviz').absolute())
    if not os.path.exists(rviz_config_path):
        rviz_config_path = ''  # Use default configuration if file doesn't exist

    # Nodes to launch
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
    )

    # Launch Description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])