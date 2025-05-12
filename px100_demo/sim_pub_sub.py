#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pygame
import numpy as np
import threading
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PX100SimControl(Node):
    def __init__(self):
        super().__init__('px100_sim_control')

        # Initialize joint state tracking
        self.joint_states = {}
        self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'gripper']
        self.current_joint = 'waist'  # Default joint to control

        # Define joint limits (in radians)
        self.joint_limits = {
            'waist':       [-1.8, 1.8],
            'shoulder':    [-1.4, 1.8],
            'elbow':       [-1.7, 1.7],
            'wrist_angle': [-1.8, 1.8],
            'gripper':     [0.0, 0.037]  # Gripper width limits in m
        }

        # Define home position
        self.home_position = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist_angle': 0.0,
            'gripper': 0.037  # Open position
        }

        # Thread lock for data access
        self.lock = threading.Lock()

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/px100/joint_states',
            self.joint_state_callback,
            10
        )

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

        # Movement increment (radians)
        self.increment = 0.1

        self.get_logger().info('PX100 Simulation Control initialized')

    def joint_state_callback(self, msg):
        """Process incoming joint state messages"""
        with self.lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.joint_states[name] = msg.position[i]

    def get_joint_position(self, joint_name):
        """Get current position of the specified joint"""
        with self.lock:
            if joint_name in self.joint_states:
                return self.joint_states[joint_name]
            else:
                return self.home_position.get(joint_name, 0.0)

    def get_all_joint_positions(self):
        """Get positions of all joints"""
        with self.lock:
            return dict(self.joint_states)

    def move_arm_joint(self, joint_name, delta):
        """Move a single arm joint by delta amount"""
        # Skip if joint doesn't exist or is gripper
        if joint_name not in self.joint_names or joint_name == 'gripper':
            return

        # Get current position and calculate target
        current_pos = self.get_joint_position(joint_name)
        target_pos = current_pos + delta

        # Apply joint limits
        if joint_name in self.joint_limits:
            min_val, max_val = self.joint_limits[joint_name]
            target_pos = max(min_val, min(max_val, target_pos))

        # Create message
        traj = JointTrajectory()
        traj.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']

        # Get current positions for all joints
        point = JointTrajectoryPoint()
        point.positions = [
            target_pos if name == joint_name else self.get_joint_position(name)
            for name in traj.joint_names
        ]

        # Set duration for movement
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000  # 0.5 seconds

        traj.points = [point]

        # Publish
        self.arm_pub.publish(traj)
        self.get_logger().info(f"Moving {joint_name} to {target_pos:.2f}")

    def move_gripper(self, open_gripper=True):
        """Open or close the gripper"""
        traj = JointTrajectory()
        traj.joint_names = ['left_finger', 'right_finger']

        point = JointTrajectoryPoint()

        # For PX100 simulation, 0.037 is fully open, 0.0 is closed
        position = 0.037 if open_gripper else 0.00

        point.positions = [position, position]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000  # 0.5 seconds

        traj.points = [point]

        # Publish
        self.gripper_pub.publish(traj)
        state = "open" if open_gripper else "closed"
        self.get_logger().info(f"Gripper {state}")

    def go_to_home_position(self):
        """Move all joints to home position"""
        traj = JointTrajectory()
        traj.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']

        point = JointTrajectoryPoint()
        point.positions = [self.home_position[name] for name in traj.joint_names]
        point.time_from_start.sec = 1  # Slower movement
        point.time_from_start.nanosec = 0

        traj.points = [point]

        # Publish
        self.arm_pub.publish(traj)
        self.get_logger().info("Moving to home position")

        # Open gripper
        self.move_gripper(open_gripper=True)

    def change_joint(self, direction="next"):
        """Change the current joint being controlled"""
        # Get arm joints only (exclude gripper)
        arm_joints = [j for j in self.joint_names if j != 'gripper']

        if self.current_joint not in arm_joints:
            self.current_joint = arm_joints[0]
            return

        current_idx = arm_joints.index(self.current_joint)

        if direction == "next":
            next_idx = (current_idx + 1) % len(arm_joints)
        else:  # previous
            next_idx = (current_idx - 1) % len(arm_joints)

        self.current_joint = arm_joints[next_idx]
        self.get_logger().info(f"Switched to control: {self.current_joint}")


class PyGameInterface:
    def __init__(self, ros_node):
        # Store the ROS node
        self.ros_node = ros_node

        # Initialize pygame
        pygame.init()

        # Screen dimensions
        self.width, self.height = 800, 600

        # Create window
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("PX100 Simulation Control")

        # Font for text
        self.font_large = pygame.font.SysFont('Arial', 36, bold=True)
        self.font_medium = pygame.font.SysFont('Arial', 28)
        self.font_small = pygame.font.SysFont('Arial', 20)

        # Enable key repeat for smoother control
        pygame.key.set_repeat(200, 150)

        # Color definitions
        self.colors = {
            'background': (30, 30, 40),
            'title': (200, 200, 255),
            'text': (255, 255, 255),
            'highlight': (255, 255, 0),
            'info': (100, 255, 100),
            'warning': (255, 100, 100),
            'joint_active': (100, 200, 255),
            'joint_normal': (150, 150, 150)
        }

        self.gripper_state = "open"  # Track gripper state

    def render_ui(self):
        """Render the UI elements on the screen"""
        # Clear the screen
        self.screen.fill(self.colors['background'])

        # Title
        title = self.font_large.render("PX100 Simulation Control", True, self.colors['title'])
        self.screen.blit(title, (self.width//2 - title.get_width()//2, 20))

        # Get current joint positions
        joint_states = self.ros_node.get_all_joint_positions()

        # Draw joint positions
        if joint_states:
            y_pos = 100

            # Add header
            header = self.font_medium.render("Joint Positions:", True, self.colors['info'])
            self.screen.blit(header, (50, y_pos))
            y_pos += 50

            # Display each joint position
            for joint_name in ['waist', 'shoulder', 'elbow', 'wrist_angle']:
                if joint_name in joint_states:
                    # Highlight active joint
                    color = self.colors['joint_active'] if joint_name == self.ros_node.current_joint else self.colors['joint_normal']

                    # Format joint name and position
                    joint_text = f"{joint_name}: {joint_states[joint_name]:.3f}"

                    # Check if at limit
                    if joint_name in self.ros_node.joint_limits:
                        min_val, max_val = self.ros_node.joint_limits[joint_name]
                        if abs(joint_states[joint_name] - min_val) < 0.01:
                            joint_text += " [MIN]"
                            color = self.colors['warning']
                        elif abs(joint_states[joint_name] - max_val) < 0.01:
                            joint_text += " [MAX]"
                            color = self.colors['warning']

                    text = self.font_medium.render(joint_text, True, color)
                    self.screen.blit(text, (80, y_pos))
                    y_pos += 40

            # Add gripper state
            gripper_text = f"Gripper: {self.gripper_state}"
            gripper_render = self.font_medium.render(gripper_text, True, self.colors['info'])
            self.screen.blit(gripper_render, (80, y_pos))
            y_pos += 60

            # Add current mode indicator
            mode_text = f"Controlling: {self.ros_node.current_joint}"
            mode_render = self.font_large.render(mode_text, True, self.colors['highlight'])
            self.screen.blit(mode_render, (50, y_pos))

        # Instructions
        instructions = [
            "Controls:",
            "LEFT/RIGHT: Change joint",
            "UP/DOWN: Move joint",
            "SPACE: Toggle gripper",
            "ENTER: Go to home position",
            "ESC: Quit"
        ]

        y_pos = 400
        for instruction in instructions:
            text = self.font_small.render(instruction, True, self.colors['text'])
            self.screen.blit(text, (50, y_pos))
            y_pos += 30

    def run(self):
        """Main loop for the pygame interface"""
        print("Starting interactive control. Press ESC to exit.")

        clock = pygame.time.Clock()
        running = True

        last_move_time = 0
        move_delay = 0.15  # Seconds between moves

        # Wait a moment for joint states to be received
        time.sleep(1.0)

        try:
            while running:
                # Process pygame events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False
                        elif event.key == pygame.K_LEFT:
                            self.ros_node.change_joint("previous")
                        elif event.key == pygame.K_RIGHT:
                            self.ros_node.change_joint("next")
                        elif event.key == pygame.K_SPACE:
                            # Toggle gripper
                            if self.gripper_state == "open":
                                self.ros_node.move_gripper(open_gripper=False)
                                self.gripper_state = "closed"
                            else:
                                self.ros_node.move_gripper(open_gripper=True)
                                self.gripper_state = "open"
                        elif event.key == pygame.K_RETURN:  # Enter key
                            # Go to home position
                            self.ros_node.go_to_home_position()
                            self.gripper_state = "open"

                # Handle continuous key presses for joint movement
                current_time = time.time()
                if current_time - last_move_time > move_delay:
                    keys = pygame.key.get_pressed()
                    current_joint = self.ros_node.current_joint

                    if current_joint != "gripper":
                        increment = self.ros_node.increment

                        # UP/DOWN keys to move the joint
                        if keys[pygame.K_UP]:
                            self.ros_node.move_arm_joint(current_joint, increment)
                            last_move_time = current_time
                        elif keys[pygame.K_DOWN]:
                            self.ros_node.move_arm_joint(current_joint, -increment)
                            last_move_time = current_time

                # Render the UI
                self.render_ui()

                # Update the display
                pygame.display.flip()

                # Cap the frame rate
                clock.tick(30)

        finally:
            # Clean up
            print("Shutting down...")
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the ROS node
        node = PX100SimControl()

        # Create a separate thread for ROS spinning
        thread = threading.Thread(target=lambda: rclpy.spin(node))
        thread.daemon = True
        thread.start()

        # Create and run the pygame interface
        gui = PyGameInterface(node)
        gui.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Shutdown ROS
        rclpy.shutdown()
        print("Control terminated.")


if __name__ == '__main__':
    main()


