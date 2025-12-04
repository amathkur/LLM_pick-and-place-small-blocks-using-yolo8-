#!/usr/bin/env python3
"""
Voice and Text Controller Node - Controls robot with voice and text commands
Commands: "move left", "move right", "move up", "move down", "go home", "pick bottle"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import threading
import sys

try:
    import speech_recognition as sr
    SPEECH_AVAILABLE = True
except ImportError:
    SPEECH_AVAILABLE = False
    print("speech_recognition not available. Install with: pip install SpeechRecognition pyaudio")


class VoiceTextController(Node):
    def __init__(self):
        super().__init__('voice_text_controller')
        
        # MoveIt Action Client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup!')
        
        # Current position (relative movements)
        self.current_x = 0.2
        self.current_y = 0.0
        self.current_z = 0.2
        
        # Movement increment (meters)
        self.step_size = 0.05
        
        # Voice recognition
        if SPEECH_AVAILABLE:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            self.get_logger().info('Voice recognition ready!')
        else:
            self.get_logger().warn('Voice recognition not available')
        
        self.get_logger().info('=== Voice & Text Controller Ready ===')
        self.get_logger().info('Commands: left, right, up, down, forward, back, home')
        self.get_logger().info('Type command or say it out loud!')
        
    def send_move_command(self, x, y, z, description):
        """Send movement command to MoveIt"""
        goal_msg = MoveGroup.Goal()
        
        # Set planning group
        goal_msg.request.group_name = 'magician_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Set goal pose
        goal_msg.request.goal_constraints.append(self._create_position_constraint(x, y, z))
        
        self.get_logger().info(f'Moving {description}: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
        
    def _create_position_constraint(self, x, y, z):
        """Create position constraint for goal"""
        from moveit_msgs.msg import Constraints, PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        
        constraint = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'wrist_pitch'
        
        # Target region (small sphere around target)
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.01]  # 1cm tolerance
        
        position_constraint.constraint_region.primitives.append(region)
        
        # Target pose
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0
        
        position_constraint.constraint_region.primitive_poses.append(target_pose)
        position_constraint.weight = 1.0
        
        constraint.position_constraints.append(position_constraint)
        return constraint
    
    def _goal_response_callback(self, future):
        """Handle action goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
            
        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        self.get_logger().info(f'Movement complete! Error code: {result.error_code.val}')
        
    def process_command(self, command):
        """Process voice or text command"""
        command = command.lower().strip()
        self.get_logger().info(f'Processing command: "{command}"')
        
        if 'left' in command:
            self.current_y += self.step_size
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'LEFT')
            
        elif 'right' in command:
            self.current_y -= self.step_size
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'RIGHT')
            
        elif 'up' in command:
            self.current_z += self.step_size
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'UP')
            
        elif 'down' in command:
            self.current_z -= self.step_size
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'DOWN')
            
        elif 'forward' in command:
            self.current_x += self.step_size
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'FORWARD')
            
        elif 'back' in command or 'backward' in command:
            self.current_x -= self.step_size
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'BACK')
            
        elif 'home' in command:
            self.current_x = 0.2
            self.current_y = 0.0
            self.current_z = 0.2
            self.send_move_command(self.current_x, self.current_y, self.current_z, 'HOME')
            
        elif 'exit' in command or 'quit' in command:
            self.get_logger().info('Exiting...')
            rclpy.shutdown()
            
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')
            self.get_logger().info('Valid commands: left, right, up, down, forward, back, home')
    
    def listen_for_voice(self):
        """Listen for voice commands (blocking)"""
        if not SPEECH_AVAILABLE:
            self.get_logger().error('Voice recognition not available!')
            return
            
        with self.microphone as source:
            self.get_logger().info('Listening for voice command...')
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            
            try:
                audio = self.recognizer.listen(source, timeout=5.0)
                command = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Heard: "{command}"')
                self.process_command(command)
            except sr.WaitTimeoutError:
                self.get_logger().warn('No speech detected')
            except sr.UnknownValueError:
                self.get_logger().warn('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
    
    def text_input_loop(self):
        """Text command input loop (blocking)"""
        while rclpy.ok():
            try:
                command = input('\nEnter command (left/right/up/down/forward/back/home): ')
                self.process_command(command)
            except EOFError:
                break
            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)
    controller = VoiceTextController()
    
    # Spin in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*60)
    print("VOICE & TEXT ROBOT CONTROLLER")
    print("="*60)
    print("Commands:")
    print("  - 'left' / 'right'   : Move left/right")
    print("  - 'up' / 'down'      : Move up/down")
    print("  - 'forward' / 'back' : Move forward/backward")
    print("  - 'home'             : Return to home position")
    print("  - 'exit'             : Quit")
    print("="*60)
    
    mode = input("Choose mode - (1) Text commands, (2) Voice commands: ").strip()
    
    if mode == '2' and SPEECH_AVAILABLE:
        print("\n🎤 VOICE MODE - Speak commands clearly")
        print("Press Ctrl+C to exit\n")
        try:
            while rclpy.ok():
                controller.listen_for_voice()
        except KeyboardInterrupt:
            pass
    else:
        print("\n⌨️  TEXT MODE - Type commands")
        print("Press Ctrl+C to exit\n")
        controller.text_input_loop()
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
