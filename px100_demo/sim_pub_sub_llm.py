#!/usr/bin/env python3

import pathlib
import rclpy
from rclpy.node import Node
import pygame
import numpy as np
import threading
import time
import openai
import os
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from dotenv import dotenv_values

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


class ChatGPTControl:
    def __init__(self, api_key=None):
        """Initialize ChatGPT API control"""
        # Set OpenAI API key either from parameter or environment variable
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            print("WARNING: OpenAI API key not found. Please set OPENAI_API_KEY environment variable.")

        # Set up OpenAI client
        if self.api_key:
            openai.api_key = self.api_key
            print("ChatGPT API initialized successfully.")

        # System prompt to instruct ChatGPT about its role
        self.system_prompt = """
        You are a robot control assistant that translates natural language commands into precise robot joint controls.
        
        IMPORTANT: You MUST ONLY respond with a valid JSON object and NOTHING else. No explanations, no code blocks, no Markdown formatting.
        
        Available joints and their ranges:
        - waist: -1.8 to 1.8 radians (rotation left/right)
        - shoulder: -1.4 to 1.8 radians (up/down)
        - elbow: -1.7 to 1.7 radians (bend/extend)
        - wrist_angle: -1.8 to 1.8 radians (rotation)
        - gripper: open or closed
        
        Instructions for specific commands:
        
        1. For joint movements:
           For "move waist left" use:
           {"action": "move_joint", "joint": "waist", "delta": -0.2}
           For "move shoulder up" use:
           {"action": "move_joint", "joint": "shoulder", "delta": 0.2}
           
           NOTE: Negative delta moves left/down, positive delta moves right/up
           Use appropriate delta values between 0.1 and 0.3 radians
        
        2. For gripper control:
           For "open gripper" use:
           {"action": "gripper", "state": "open"}
           For "close gripper" use:
           {"action": "gripper", "state": "closed"}
        
        3. For home position:
           For "go home" or "reset position" etc. use:
           {"action": "home"}
        
        ALWAYS interpret the user's intent and map it to one of these three command structures.
        NEVER include any text outside of the JSON object.
        DO NOT include backticks (```) or any other formatting.
        """

        # Conversation history for context
        self.conversation_history = [
            {"role": "system", "content": self.system_prompt}
        ]

    def process_command(self, user_input):
        """Process a natural language command through ChatGPT"""
        if not self.api_key:
            return {"error": "API key not set"}

        try:
            # Add user input to conversation history
            self.conversation_history.append({"role": "user", "content": user_input})

            # Call ChatGPT API
            response = openai.chat.completions.create(
                # model="gpt-3.5-turbo",
                model="gpt-4o-mini",
                messages=self.conversation_history,
                temperature=0.2,  # Lower temperature for more deterministic responses
                max_tokens=150,
                response_format={"type": "json_object"}  # Force JSON response format
            )

            # Extract the response text
            completions = openai.chat.completions.list()
            first_id = completions[0].id
            first_completion: str = openai.chat.completions.retrieve(completion_id=first_id)
            # print(first_completion)

            assistant_response = first_completion.strip()
            # assistant_response = response.choices[0].message.content.strip()
            self.conversation_history.append({"role": "assistant", "content": assistant_response})

            # Limit conversation history to last 10 exchanges to prevent token limits
            if len(self.conversation_history) > 10:
                # Keep system prompt and last 4 exchanges
                self.conversation_history = [self.conversation_history[0]] + self.conversation_history[-8:]

            # Return the response as a string (the caller will convert to JSON)
            return assistant_response

        except Exception as e:
            print(f"Error calling ChatGPT API: {e}")
            return {"error": str(e)}


class LLMInterface:
    def __init__(self, ros_node):
        # Store the ROS node
        self.ros_node = ros_node

        # Initialize ChatGPT controller
        config = dotenv_values(pathlib.Path(__file__).parent.parent / ".env")
        openai_api_key = config.get('OPENAI_API_KEY')
        print(f"OpenAI API key: {openai_api_key}")
        self.chatgpt = ChatGPTControl(api_key=openai_api_key)

        # Initialize pygame
        pygame.init()

        # Screen dimensions
        self.width, self.height = 800, 600

        # Create window
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("PX100 ChatGPT Control")

        # Font for text
        self.font_large = pygame.font.SysFont('Arial', 36, bold=True)
        self.font_medium = pygame.font.SysFont('Arial', 28)
        self.font_small = pygame.font.SysFont('Arial', 20)

        # Color definitions
        self.colors = {
            'background': (30, 30, 40),
            'title': (200, 200, 255),
            'text': (255, 255, 255),
            'highlight': (255, 255, 0),
            'info': (100, 255, 100),
            'warning': (255, 100, 100),
            'input_bg': (50, 50, 60),
            'input_text': (255, 255, 255),
            'command_bg': (40, 60, 40),
            'response_bg': (60, 40, 70)
        }

        # Input text properties
        self.input_text = ""
        self.input_active = True
        self.input_rect = pygame.Rect(50, 450, 700, 40)

        # Command history
        self.command_history = []
        self.responses = []
        self.max_history = 5

        # Current state
        self.gripper_state = "open"

        print("Natural language robot control initialized. Type your commands in the input box.")
        print("Example commands: 'move left', 'close the gripper', 'move elbow up', 'go to home position'")

    def render_ui(self):
        """Render the UI elements on the screen"""
        # Clear the screen
        self.screen.fill(self.colors['background'])

        # Title
        title = self.font_large.render("PX100 Natural Language Control", True, self.colors['title'])
        self.screen.blit(title, (self.width//2 - title.get_width()//2, 20))

        # Get current joint positions
        joint_states = self.ros_node.get_all_joint_positions()

        # Draw joint positions
        if joint_states:
            y_pos = 80

            # Add header
            header = self.font_medium.render("Joint Positions:", True, self.colors['info'])
            self.screen.blit(header, (50, y_pos))
            y_pos += 40

            # Display each joint position
            for joint_name in ['waist', 'shoulder', 'elbow', 'wrist_angle']:
                if joint_name in joint_states:
                    joint_text = f"{joint_name}: {joint_states[joint_name]:.3f}"
                    text = self.font_small.render(joint_text, True, self.colors['text'])
                    self.screen.blit(text, (80, y_pos))
                    y_pos += 30

            # Add gripper state
            gripper_text = f"Gripper: {self.gripper_state}"
            gripper_render = self.font_small.render(gripper_text, True, self.colors['info'])
            self.screen.blit(gripper_render, (80, y_pos))
            y_pos += 50

            # Command history header
            history_header = self.font_medium.render("Command History:", True, self.colors['highlight'])
            self.screen.blit(history_header, (50, y_pos))
            y_pos += 40

            # Display command history
            for i, (cmd, resp) in enumerate(zip(self.command_history, self.responses)):
                if i >= self.max_history:
                    break

                # User command
                cmd_bg = pygame.Rect(60, y_pos, self.width - 120, 25)
                pygame.draw.rect(self.screen, self.colors['command_bg'], cmd_bg)
                cmd_text = self.font_small.render(f"You: {cmd}", True, self.colors['text'])
                self.screen.blit(cmd_text, (70, y_pos))
                y_pos += 30

                # Response
                resp_bg = pygame.Rect(60, y_pos, self.width - 120, 25)
                pygame.draw.rect(self.screen, self.colors['response_bg'], resp_bg)
                resp_text = self.font_small.render(f"Robot: {resp}", True, self.colors['info'])
                self.screen.blit(resp_text, (70, y_pos))
                y_pos += 40

        # Input box
        pygame.draw.rect(self.screen, self.colors['input_bg'], self.input_rect)

        # Input text with cursor blink
        input_surface = self.font_medium.render(self.input_text, True, self.colors['input_text'])
        self.screen.blit(input_surface, (self.input_rect.x + 10, self.input_rect.y + 5))

        # Input prompt
        prompt_text = self.font_small.render("Enter command:", True, self.colors['highlight'])
        self.screen.blit(prompt_text, (50, 420))

        # Instructions
        instructions = [
            "Press ENTER to send command",
            "ESC to quit"
        ]

        y_pos = 510
        for instruction in instructions:
            text = self.font_small.render(instruction, True, self.colors['text'])
            self.screen.blit(text, (50, y_pos))
            y_pos += 30

    def process_and_execute_command(self, command):
        """Send command to ChatGPT and execute the response"""
        # Send to ChatGPT
        response = self.chatgpt.process_command(command)

        # Add to history
        self.command_history.insert(0, command)

        # Parse response
        try:
            # Parse the response as JSON directly since we're using response_format=json_object
            import json

            if isinstance(response, str):
                response_data = json.loads(response)
            else:
                response_data = response

            # Execute the command
            action_response = self.execute_robot_command(response_data)
            self.responses.insert(0, action_response)

            # Limit history
            if len(self.command_history) > self.max_history:
                self.command_history.pop()
                self.responses.pop()

        except Exception as e:
            print(f"Error processing ChatGPT response: {e}")
            print(f"Raw response: {response}")
            self.responses.insert(0, f"Error: {str(e)}")

    def execute_robot_command(self, command_data):
        """Execute a robot command based on the parsed ChatGPT response"""
        try:
            action = command_data.get('action')

            if action == 'move_joint':
                joint = command_data.get('joint')
                delta = float(command_data.get('delta', 0.0))

                if joint in self.ros_node.joint_names:
                    self.ros_node.move_arm_joint(joint, delta)
                    return f"Moving {joint} by {delta:.2f} radians"
                else:
                    return f"Unknown joint: {joint}"

            elif action == 'gripper':
                state = command_data.get('state')

                if state == 'open':
                    self.ros_node.move_gripper(open_gripper=True)
                    self.gripper_state = "open"
                    return "Opening gripper"
                elif state == 'closed' or state == 'close':
                    self.ros_node.move_gripper(open_gripper=False)
                    self.gripper_state = "closed"
                    return "Closing gripper"
                else:
                    return f"Unknown gripper state: {state}"

            elif action == 'home':
                self.ros_node.go_to_home_position()
                self.gripper_state = "open"
                return "Moving to home position"

            else:
                return f"Unknown action: {action}"

        except Exception as e:
            print(f"Error executing command: {e}")
            return f"Error executing command: {str(e)}"

    def run(self):
        """Main loop for the interface"""
        print("Starting natural language control. Type commands and press ENTER to send.")
        print("Press ESC to exit.")

        clock = pygame.time.Clock()
        running = True

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
                            
                        elif event.key == pygame.K_RETURN and self.input_text:
                            # Send the command
                            command = self.input_text
                            self.input_text = ""
                            
                            # Process in a separate thread to avoid blocking the UI
                            threading.Thread(
                                target=self.process_and_execute_command,
                                args=(command,),
                                daemon=True
                            ).start()
                            
                        elif event.key == pygame.K_BACKSPACE:
                            self.input_text = self.input_text[:-1]
                            
                        else:
                            # Add character to input text
                            if event.unicode.isprintable():
                                self.input_text += event.unicode

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

        # Create and run the LLM interface
        gui = LLMInterface(node)
        gui.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Shutdown ROS
        rclpy.shutdown()
        print("Control terminated.")


if __name__ == '__main__':
    main()