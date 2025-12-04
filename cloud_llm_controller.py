#!/usr/bin/env python3
"""
Cloud LLM Controller - Natural Language Robot Control using Anthropic Claude
Subscribes to voice/text commands and controls the robot via MoveIt
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene, RobotState
import json
import requests
import time
import re
import math
import threading
import tkinter as tk
from pymoveit2 import MoveIt2
import argparse
import numpy as np
import cv2
import tf2_py
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer


class GeminiDesktopCommander:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Gemini Desktop Commander")
        self.root.geometry("600x400")

        # GUI elements
        self.command_entry = tk.Entry(self.root, width=60, font=("Arial", 12))
        self.send_button = tk.Button(self.root, text="Send Command", command=self.send_command, font=("Arial", 12), bg="lightblue")
        self.voice_button = tk.Button(self.root, text="Voice Command", command=self.voice_command, font=("Arial", 12), bg="lightgreen")
        self.status_label = tk.Label(self.root, text="Status: Ready", font=("Arial", 10), fg="green")
        self.response_text = tk.Text(self.root, height=15, width=60, font=("Arial", 10), wrap=tk.WORD)
        self.clear_button = tk.Button(self.root, text="Clear History", command=self.clear_history, font=("Arial", 10))
        
        # End effector selection
        self.end_effector_var = tk.StringVar(value="suction")
        self.end_effector_frame = tk.Frame(self.root)
        self.end_effector_label = tk.Label(self.end_effector_frame, text="End Effector:", font=("Arial", 10))
        self.gripper_radio = tk.Radiobutton(self.end_effector_frame, text="Gripper", variable=self.end_effector_var, value="gripper", command=lambda: self.node.update_end_effector("gripper"))
        self.suction_radio = tk.Radiobutton(self.end_effector_frame, text="Suction", variable=self.end_effector_var, value="suction", command=lambda: self.node.update_end_effector("suction"))

        # Layout
        tk.Label(self.root, text="Enter Natural Language Command:", font=("Arial", 12)).pack(pady=5)
        self.command_entry.pack(pady=5)
        
        # End effector selection
        self.end_effector_frame.pack(pady=5)
        self.end_effector_label.pack(side=tk.LEFT)
        self.gripper_radio.pack(side=tk.LEFT, padx=10)
        self.suction_radio.pack(side=tk.LEFT, padx=10)
        
        self.send_button.pack(pady=5)
        self.voice_button.pack(pady=5)
        self.status_label.pack(pady=5)
        tk.Label(self.root, text="Response History:", font=("Arial", 12)).pack(pady=5)
        self.response_text.pack(pady=5)
        self.clear_button.pack(pady=5)

    def send_command(self):
        command = self.command_entry.get().strip()
        if command:
            self.update_status("Processing...")
            self.command_entry.delete(0, tk.END)
            # Publish to /llm_command
            msg = String()
            msg.data = command
            self.node.command_pub.publish(msg)
            self.update_response(f"Sent: {command}")

    def voice_command(self):
        try:
            import speech_recognition as sr
            r = sr.Recognizer()
            with sr.Microphone() as source:
                self.update_status("Listening... (speak now)")
                self.root.update()
                audio = r.listen(source, timeout=5)
                self.update_status("Recognizing...")
                self.root.update()
                text = r.recognize_google(audio)
                self.command_entry.delete(0, tk.END)
                self.command_entry.insert(0, text)
                self.update_status("Voice recognized - executing command...")
                self.root.update()
                # Automatically execute the recognized command
                self.send_command()
        except sr.UnknownValueError:
            self.update_status("Could not understand audio")
        except sr.RequestError as e:
            self.update_status(f"Speech recognition error: {e}")
        except Exception as e:
            self.update_status(f"Voice error: {e}")

    def update_status(self, status):
        self.status_label.config(text=f"Status: {status}")

    def update_response(self, response):
        self.response_text.insert(tk.END, f"{response}\n")
        self.response_text.see(tk.END)

    def clear_history(self):
        self.response_text.delete(1.0, tk.END)

    def run(self):
        self.root.mainloop()


class CloudLLMController(Node):
    def __init__(self):
        super().__init__('cloud_llm_controller')

        # Gemini API configuration
        self.api_key = "AIzaSyDxZyXZsJu3EYfMZqIN1nQfUf7a0K7wlw8"  # Gemini API key
        self.api_url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"
        self.model = "gemini-2.0-flash"

        # Initialize MoveIt2 (optional - will use direct joint control if not available)
        try:
            from pymoveit2 import MoveIt2
            from pymoveit2.robots import magician_lite
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=magician_lite.joint_names(),
                base_link_name="base_link",
                end_effector_name="gripper",
                group_name="magician_arm",
            )
            self.moveit_available = True
            self.get_logger().info('MoveIt2 initialized successfully')
        except ImportError as e:
            self.get_logger().warn(f'MoveIt2 not available: {e}. Using direct joint control only.')
            self.moveit2 = None
            self.moveit_available = False

        # Publishers
        self.response_pub = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        self.command_pub = self.create_publisher(
            String,
            '/llm_command',
            10
        )
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/set_joint_positions',
            10
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/llm_command',
            self.command_callback,
            10
        )
        self.response_sub = self.create_subscription(
            String,
            '/llm_response',
            self.response_callback,
            10
        )

        # Subscriber for YOLO target pose
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/yolo/target_pose',
            self.target_pose_callback,
            10
        )

        # Subscriber for YOLO detections
        self.yolo_detection_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.yolo_callback,
            10
        )

        # Store latest target pose
        self.latest_target_pose = None

        # Store latest YOLO detections
        self.latest_detections = []

        # End effector configuration
        self.end_effector = "suction"  # Default to suction, will be updated from GUI

        # GUI reference (initialized later in main)
        self.gui = None

        # TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Maze workspace boundaries based on calibration
        self.maze_workspace = {
            'x_min': 165.0,  # Left boundary (mm from base)
            'x_max': 365.0,  # Right boundary
            'y_min': -100.0, # Front boundary  
            'y_max': 100.0,  # Back boundary
            'z_min': 0.0,    # Table height
            'z_max': 200.0   # Maximum reach height
        }

        # Maze calibration points (4x4 or 5x5 area)
        self.CALIB_IMG_PTS_PX_FULL = np.array([
            [152, 29],  # point_1
            [499, 49],  # point_2
            [481, 388],  # point_3
            [142, 376],  # point_4
        ], dtype=np.float32)

        self.CALIB_ROBOT_PTS_MM = np.array([
            [165.00, -100.00],  # point_1 BL
            [365.00, -100.00],  # point_2 BR
            [365.00, 100.00],  # point_3 TR
            [165.00, 100.00],  # point_4 TL
        ], dtype=np.float32)

        # Workspace boundaries (corners in base_link frame)
        # Define rectangular workspace area where robot can safely operate
        self.workspace_corners = {
            'x_min': self.maze_workspace['x_min'] / 1000.0,  # Convert mm to meters
            'x_max': self.maze_workspace['x_max'] / 1000.0,
            'y_min': self.maze_workspace['y_min'] / 1000.0,
            'y_max': self.maze_workspace['y_max'] / 1000.0,
            'z_min': self.maze_workspace['z_min'] / 1000.0,
            'z_max': self.maze_workspace['z_max'] / 1000.0
        }
        
        # Load workspace boundaries from parameters if available
        self.declare_parameter('workspace.x_min', self.workspace_corners['x_min'])
        self.declare_parameter('workspace.x_max', self.workspace_corners['x_max'])
        self.declare_parameter('workspace.y_min', self.workspace_corners['y_min'])
        self.declare_parameter('workspace.y_max', self.workspace_corners['y_max'])
        self.declare_parameter('workspace.z_min', self.workspace_corners['z_min'])
        self.declare_parameter('workspace.z_max', self.workspace_corners['z_max'])
        
        # Load camera calibration parameters
        self.declare_parameter('camera.calibrated', False)
        self.declare_parameter('camera.fx', 600.0)  # Focal length x
        self.declare_parameter('camera.fy', 600.0)  # Focal length y
        self.declare_parameter('camera.cx', 320.0)  # Principal point x
        self.declare_parameter('camera.cy', 240.0)  # Principal point y
        self.declare_parameter('camera.k1', 0.0)    # Radial distortion
        self.declare_parameter('camera.k2', 0.0)    # Radial distortion
        self.declare_parameter('camera.p1', 0.0)    # Tangential distortion
        self.declare_parameter('camera.p2', 0.0)    # Tangential distortion
        
        # Load parameters
        self.load_workspace_parameters()
        self.load_camera_parameters()

    def load_workspace_parameters(self):
        """Load workspace boundary parameters from ROS parameter server"""
        try:
            # Load workspace boundaries (in base_link frame)
            self.workspace_boundaries = {
                'x_min': self.get_parameter('workspace.x_min').get_parameter_value().double_value,
                'x_max': self.get_parameter('workspace.x_max').get_parameter_value().double_value,
                'y_min': self.get_parameter('workspace.y_min').get_parameter_value().double_value,
                'y_max': self.get_parameter('workspace.y_max').get_parameter_value().double_value,
                'z_min': self.get_parameter('workspace.z_min').get_parameter_value().double_value,
                'z_max': self.get_parameter('workspace.z_max').get_parameter_value().double_value
            }
            self.get_logger().info(f'Loaded workspace boundaries: {self.workspace_boundaries}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load workspace parameters, using defaults: {e}')
            # Default workspace boundaries (safe defaults)
            self.workspace_boundaries = {
                'x_min': -0.3, 'x_max': 0.3,
                'y_min': -0.3, 'y_max': 0.3,
                'z_min': 0.0, 'z_max': 0.4
            }

    def load_camera_parameters(self):
        """Load camera calibration parameters from ROS parameter server"""
        try:
            # Load camera calibration status
            self.camera_calibrated = self.get_parameter('camera.calibrated').get_parameter_value().bool_value
            
            if self.camera_calibrated:
                # Load camera intrinsics
                self.camera_matrix = np.array([
                    [self.get_parameter('camera.fx').get_parameter_value().double_value, 0, self.get_parameter('camera.cx').get_parameter_value().double_value],
                    [0, self.get_parameter('camera.fy').get_parameter_value().double_value, self.get_parameter('camera.cy').get_parameter_value().double_value],
                    [0, 0, 1]
                ])
                
                # Load distortion coefficients
                self.dist_coeffs = np.array([
                    self.get_parameter('camera.k1').get_parameter_value().double_value,
                    self.get_parameter('camera.k2').get_parameter_value().double_value,
                    self.get_parameter('camera.p1').get_parameter_value().double_value,
                    self.get_parameter('camera.p2').get_parameter_value().double_value
                ])
                
                # Load extrinsic parameters (camera to base transformation)
                # For now, store as identity - will be updated during calibration
                self.camera_to_base_rotation = np.eye(3)
                self.camera_to_base_translation = np.zeros(3)
                
                self.get_logger().info('Loaded camera calibration parameters')
                self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
                self.get_logger().info(f'Distortion coefficients: {self.dist_coeffs}')
            else:
                # Perform calibration using provided points
                self.perform_maze_calibration()
                
        except Exception as e:
            self.get_logger().warn(f'Failed to load camera parameters, performing calibration: {e}')
            self.perform_maze_calibration()

    def perform_maze_calibration(self):
        """Perform camera calibration using maze corner points"""
        try:
            self.get_logger().info('Performing maze camera calibration...')
            
            # Use the provided calibration points
            image_points = self.CALIB_IMG_PTS_PX_FULL
            robot_points = self.CALIB_ROBOT_PTS_MM
            
            # Convert robot points from mm to meters for consistency
            robot_points_m = robot_points / 1000.0
            
            # Calculate homography matrix to transform from pixel coordinates to robot coordinates
            self.homography_matrix, _ = cv2.findHomography(image_points, robot_points_m)
            
            if self.homography_matrix is not None:
                self.camera_calibrated = True
                self.get_logger().info('Maze camera calibration successful!')
                self.get_logger().info(f'Homography matrix:\n{self.homography_matrix}')
                
                # Set default camera intrinsics (will be refined if needed)
                self.camera_matrix = np.array([
                    [600.0, 0, 320.0],
                    [0, 600.0, 240.0],
                    [0, 0, 1]
                ])
                self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0])
                
            else:
                self.get_logger().error('Failed to compute homography matrix')
                self.camera_calibrated = False
                
        except Exception as e:
            self.get_logger().error(f'Error during maze calibration: {e}')
            self.camera_calibrated = False

    def is_point_in_workspace(self, x, y, z):
        """Check if a point is within the defined workspace boundaries"""
        return (self.workspace_boundaries['x_min'] <= x <= self.workspace_boundaries['x_max'] and
                self.workspace_boundaries['y_min'] <= y <= self.workspace_boundaries['y_max'] and
                self.workspace_boundaries['z_min'] <= z <= self.workspace_boundaries['z_max'])

    def calibrate_camera_extrinsics(self, object_points, image_points):
        """Calibrate camera extrinsics using PnP with known object points"""
        try:
            # object_points: 3D points in world/base frame
            # image_points: corresponding 2D points in image
            
            success, rotation_vector, translation_vector = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coeffs
            )
            
            if success:
                # Convert rotation vector to rotation matrix
                self.camera_to_base_rotation, _ = cv2.Rodrigues(rotation_vector)
                self.camera_to_base_translation = translation_vector.flatten()
                
                self.camera_calibrated = True
                self.get_logger().info('Camera extrinsic calibration successful')
                self.get_logger().info(f'Rotation matrix:\n{self.camera_to_base_rotation}')
                self.get_logger().info(f'Translation vector: {self.camera_to_base_translation}')
                
                return True
            else:
                self.get_logger().error('Camera extrinsic calibration failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error during camera calibration: {e}')
            return False

    def pixel_to_robot_coords(self, u, v):
        """Convert pixel coordinates to robot coordinates using homography"""
        try:
            if not self.camera_calibrated or self.homography_matrix is None:
                self.get_logger().warn('Camera not calibrated, cannot convert pixel to robot coordinates')
                return None
            
            # Add homogeneous coordinate
            pixel_point = np.array([[u, v, 1]], dtype=np.float32).T
            
            # Apply homography transformation
            robot_coords = self.homography_matrix @ pixel_point
            
            # Convert from homogeneous coordinates
            x = robot_coords[0, 0] / robot_coords[2, 0]
            y = robot_coords[1, 0] / robot_coords[2, 0]
            
            # Return robot coordinates in meters (X, Y in base frame)
            return np.array([x, y])
            
        except Exception as e:
            self.get_logger().error(f'Error converting pixel to robot coordinates: {e}')
            return None

    def camera_to_base_transform_point(self, camera_point):
        """Transform a 3D point from camera frame to base frame"""
        try:
            # Apply rotation and translation
            base_point = self.camera_to_base_rotation @ camera_point + self.camera_to_base_translation
            return base_point
            
        except Exception as e:
            self.get_logger().error(f'Error transforming point: {e}')
            return None

    def update_end_effector(self, effector_type):
        """Update the end effector type (gripper or suction)"""
        self.end_effector = effector_type
        self.get_logger().info(f'End effector updated to: {effector_type}')

    def target_pose_callback(self, msg):
        """Store latest target pose from YOLO"""
        self.latest_target_pose = msg
        self.get_logger().debug(f'Updated target pose: {msg.pose.position}')

    def yolo_callback(self, msg):
        """Store latest YOLO detections"""
        detection_str = msg.data
        # Parse detection string: "small red cube: conf=0.85, size=5.2x5.2cm"
        try:
            parts = detection_str.split(':')
            obj_desc = parts[0].strip()
            conf_part = parts[1].split(',')[0].replace('conf=', '').strip()
            confidence = float(conf_part)
            
            # Parse object description: "small red cube"
            desc_parts = obj_desc.split()
            if len(desc_parts) >= 3:
                size = desc_parts[0]
                color = desc_parts[1]
                obj_type = desc_parts[2]
                
                # Try to parse optional position (pos=x,y,z) in the trailing part
                x_pos = None
                y_pos = None
                z_pos = None
                trailing = parts[1]
                if 'pos=' in trailing:
                    try:
                        # pos=0.123,0.045,0.500
                        pos_part = trailing.split('pos=')[-1].split()[0].strip().strip(',')
                        coords = pos_part.split(',')
                        if len(coords) >= 3:
                            x_pos = float(coords[0])
                            y_pos = float(coords[1])
                            z_pos = float(coords[2])
                    except Exception:
                        pass

                detection = {
                    'size': size,
                    'color': color,
                    'type': obj_type,
                    'confidence': confidence,
                    'description': obj_desc,
                    'position': {'x': x_pos, 'y': y_pos, 'z': z_pos} if x_pos is not None else None
                }
                self.latest_detections.append(detection)
                # Keep only recent detections
                if len(self.latest_detections) > 10:
                    self.latest_detections.pop(0)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse YOLO detection: {detection_str}, error: {e}')

    def response_callback(self, msg):
        """Handle response messages for GUI update"""
        if self.gui:
            self.gui.update_response(f"Response: {msg.data}")
            self.gui.update_status("Ready")

    def command_callback(self, msg):
        """Process incoming natural language commands"""
        command = msg.data.strip()
        self.get_logger().info(f'Received command: {command}')
        if self.gui:
            self.gui.update_status("Processing Gemini API...")

        try:
            # Interpret command using Gemini
            robot_command = self.interpret_command_with_gemini(command)

            if robot_command:
                if self.gui:
                    self.gui.update_status("Planning motion...")
                # Execute the robot command
                success = self.execute_robot_command(robot_command)
                response = f"Executed: {robot_command}" if success else f"Failed to execute: {robot_command}"
                if self.gui:
                    self.gui.update_status("Completed" if success else "Failed")
            else:
                response = "Could not understand command"
                if self.gui:
                    self.gui.update_status("Command not understood")

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            response = f"Error: {str(e)}"
            if self.gui:
                self.gui.update_status("Error")

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

    def interpret_command_with_gemini(self, command):
        """Use Gemini to interpret natural language into robot commands"""

        system_prompt = """
        You are a robot control assistant. Convert natural language commands into specific robot actions.
        The robot uses YOLO vision to detect objects like cubes and cylinders in various colors and sizes.
        Available actions:
        - move_to_pose: Move to a specific pose (x, y, z, roll, pitch, yaw) in meters and radians
        - pick_object: Pick up an object at position (x, y, z) - uses YOLO to find objects
        - place_object: Place object at position (x, y, z)
        - home: Move to home position (all joints at 0)
        - ready: Move to ready position

        Commands can be relative movements:
        - "move/go up/down/left/right X cm" → move_to_pose with relative coordinates
        - "pick large/small/medium red/blue/green/yellow cube/cylinder" → pick_object using YOLO detection

        Respond with JSON format:
        {"action": "action_name", "parameters": {"param1": value1, ...}}

        Examples:
        User: "go home"
        Response: {"action": "home", "parameters": {}}

        User: "go to ready"
        Response: {"action": "ready", "parameters": {}}

        User: "move up 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.01}}

        User: "go up 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.01}}

        User: "move up 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.02}}

        User: "go up 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.02}}

        User: "move up 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.03}}

        User: "go up 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.03}}

        User: "move up 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.04}}

        User: "go up 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.04}}

        User: "move up 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.05}}

        User: "go up 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": 0.05}}

        User: "move down 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.01}}

        User: "go down 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.01}}

        User: "move down 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.02}}

        User: "go down 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.02}}

        User: "move down 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.03}}

        User: "go down 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.03}}

        User: "move down 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.04}}

        User: "go down 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.04}}

        User: "move down 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.05}}

        User: "go down 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.0, "z": -0.05}}

        User: "move left 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.01, "z": 0.0}}

        User: "go left 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.01, "z": 0.0}}

        User: "move left 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.02, "z": 0.0}}

        User: "go left 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.02, "z": 0.0}}

        User: "move left 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.03, "z": 0.0}}

        User: "go left 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.03, "z": 0.0}}

        User: "move left 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.04, "z": 0.0}}

        User: "go left 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.04, "z": 0.0}}

        User: "move left 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.05, "z": 0.0}}

        User: "go left 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": 0.05, "z": 0.0}}

        User: "move right 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.01, "z": 0.0}}

        User: "go right 1 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.01, "z": 0.0}}

        User: "move right 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.02, "z": 0.0}}

        User: "go right 2 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.02, "z": 0.0}}

        User: "move right 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.03, "z": 0.0}}

        User: "go right 3 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.03, "z": 0.0}}

        User: "move right 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.04, "z": 0.0}}

        User: "go right 4 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.04, "z": 0.0}}

        User: "move right 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.05, "z": 0.0}}

        User: "go right 5 cm"
        Response: {"action": "move_to_pose", "parameters": {"x": 0.0, "y": -0.05, "z": 0.0}}

        User: "pick large cube"
        Response: {"action": "pick_object", "parameters": {"size": "large", "color": "red", "type": "cube"}}

        User: "pick small cube"
        Response: {"action": "pick_object", "parameters": {"size": "small", "color": "red", "type": "cube"}}

        User: "pick medium cube"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "red", "type": "cube"}}

        User: "pick large cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "large", "color": "red", "type": "cylinder"}}

        User: "pick small cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "small", "color": "red", "type": "cylinder"}}

        User: "pick medium cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "red", "type": "cylinder"}}

        User: "pick red cube"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "red", "type": "cube"}}

        User: "pick blue cube"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "blue", "type": "cube"}}

        User: "pick green cube"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "green", "type": "cube"}}

        User: "pick yellow cube"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "yellow", "type": "cube"}}

        User: "pick red cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "red", "type": "cylinder"}}

        User: "pick blue cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "blue", "type": "cylinder"}}

        User: "pick green cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "green", "type": "cylinder"}}

        User: "pick yellow cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "yellow", "type": "cylinder"}}

        User: "pick large red cube"
        Response: {"action": "pick_object", "parameters": {"size": "large", "color": "red", "type": "cube"}}

        User: "pick small blue cube"
        Response: {"action": "pick_object", "parameters": {"size": "small", "color": "blue", "type": "cube"}}

        User: "pick medium green cube"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "green", "type": "cube"}}

        User: "pick large red cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "large", "color": "red", "type": "cylinder"}}

        User: "pick small blue cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "small", "color": "blue", "type": "cylinder"}}

        User: "pick medium green cylinder"
        Response: {"action": "pick_object", "parameters": {"size": "medium", "color": "green", "type": "cylinder"}}
        """

        full_prompt = system_prompt + "\n\nUser: " + command

        payload = {
            "contents": [
                {
                    "parts": [
                        {"text": full_prompt}
                    ]
                }
            ]
        }

        headers = {
            "Content-Type": "application/json"
        }

        url = f"{self.api_url}?key={self.api_key}"

        # Retry logic for API calls
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = requests.post(url, json=payload, headers=headers, timeout=10)
                response.raise_for_status()

                result = response.json()
                content = result['candidates'][0]['content']['parts'][0]['text']

                # Parse JSON response
                try:
                    parsed = json.loads(content.strip())
                    return parsed
                except json.JSONDecodeError:
                    self.get_logger().error(f'Failed to parse Gemini response as JSON: {content}')
                    return None

            except requests.exceptions.HTTPError as e:
                if response.status_code == 429:
                    wait_time = 2 ** attempt  # Exponential backoff
                    self.get_logger().warn(f'Gemini API rate limited (429). Retrying in {wait_time} seconds...')
                    time.sleep(wait_time)
                    continue
                else:
                    self.get_logger().error(f'Gemini API request failed: {str(e)}')
                    return None
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f'Gemini API request failed: {str(e)}')
                return None

        self.get_logger().error('Max retries exceeded for Gemini API')
        return None

    def execute_robot_command(self, robot_command):
        """Execute the interpreted robot command using MoveIt2"""
        try:
            action = robot_command.get('action')
            params = robot_command.get('parameters', {})

            self.get_logger().info(f'Executing action: {action} with params: {params}')

            if action == 'move_to_pose':
                return self.move_to_pose(params)
            elif action == 'pick_object':
                return self.pick_object(params)
            elif action == 'place_object':
                return self.place_object(params)
            elif action == 'home':
                return self.move_home()
            elif action == 'ready':
                return self.move_ready()
            else:
                self.get_logger().error(f'Unknown action: {action}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing robot command: {str(e)}')
            return False

    def move_to_pose(self, params):
        """Move to a specific pose"""
        if not self.moveit_available:
            # Use direct joint control for relative movements
            self.get_logger().info(f'Using direct joint control for pose: {params}')
            # For relative movements, adjust joints based on direction
            x = float(params.get('x', 0.0))
            y = float(params.get('y', 0.0))
            z = float(params.get('z', 0.0))
            
            # Simple joint adjustments for basic movements
            # This is a rough approximation - in reality would need IK
            joint_positions = [0.0, 0.0, 0.0, 0.0]  # Start from current (assuming home)
            
            if z > 0:  # Move up
                joint_positions[1] -= z * 2.0  # Joint2 negative for up (assuming positive is down)
            elif z < 0:  # Move down
                joint_positions[1] -= z * 2.0  # z negative, so -z positive for down
            
            if y > 0:  # Move left
                joint_positions[0] -= y * 2.0  # Joint1 negative for left (assuming positive is right)
            elif y < 0:  # Move right
                joint_positions[0] -= y * 2.0  # y negative, so -y positive for right
            
            # Publish direct joint command
            msg = Float64MultiArray()
            msg.data = joint_positions
            self.joint_cmd_pub.publish(msg)
            
            self.get_logger().info(f'Direct joint move command sent: {joint_positions}')
            return True
        
        # Original MoveIt code
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(params.get('x', 0.0))
        pose.pose.position.y = float(params.get('y', 0.0))
        pose.pose.position.z = float(params.get('z', 0.0))

        # Check if target pose is within workspace boundaries
        if not self.is_point_in_workspace(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z):
            self.get_logger().error(f'Target pose ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}) is outside workspace boundaries!')
            self.get_logger().error(f'Workspace: x=[{self.workspace_boundaries["x_min"]:.3f}, {self.workspace_boundaries["x_max"]:.3f}], '
                                  f'y=[{self.workspace_boundaries["y_min"]:.3f}, {self.workspace_boundaries["y_max"]:.3f}], '
                                  f'z=[{self.workspace_boundaries["z_min"]:.3f}, {self.workspace_boundaries["z_max"]:.3f}]')
            return False
        
        self.get_logger().info('Target pose is within workspace boundaries')
        roll = float(params.get('roll', 0.0))
        pitch = float(params.get('pitch', 0.0))
        yaw = float(params.get('yaw', 0.0))

        # Simple quaternion calculation
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        self.get_logger().info(f'Moving to pose: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}')

        # Use MoveIt2 to plan and execute
        plan_result = self.moveit2.move_to_pose(pose)
        self.get_logger().info(f'Plan result: {plan_result}')
        if plan_result:
            self.get_logger().info(f'Error code: {plan_result.error_code.val}')
        if plan_result and hasattr(plan_result, 'error_code') and plan_result.error_code.val == 1:
            self.moveit2.wait_until_executed()
            self.get_logger().info('Pose move execution completed')
            return True
        else:
            self.get_logger().warn(f'Planning failed! Error code: {plan_result.error_code.val if plan_result else "Unknown"}')
            return False

    def pick_object(self, params):
        """Pick up an object using precise YOLO detections and depth estimation"""
        if not self.moveit_available:
            # Use direct joint control with precise YOLO positioning
            self.get_logger().info(f'Using direct joint control to pick: {params} with end effector: {self.end_effector}')
            
            # Map object types to YOLO classes (sphere -> cube since both are similar)
            object_type_mapping = {
                'ball': 'cube',
                'cube': 'cube',
                'cylinder': 'cylinder',
                'block': 'cube'
            }
            
            target_size = params.get('size', 'medium')
            target_color = params.get('color', 'red')
            target_type = params.get('type', None)  # Optional type parameter
            
            # Map the requested type to YOLO class
            yolo_class = object_type_mapping.get(target_type, target_type)
            self.get_logger().info(f'Mapping object type "{target_type}" to YOLO class "{yolo_class}"')
            
            # Try to find matching YOLO detection
            best_match = None
            best_conf = 0.0
            
            for det in self.latest_detections:
                # Check size and color match
                size_match = det['size'] == target_size
                color_match = det['color'] == target_color
                
                # If type is specified, check it too
                type_match = True
                if target_type:
                    yolo_class = object_type_mapping.get(target_type, target_type)
                    type_match = det['type'] == yolo_class
                
                # If all conditions match, consider this detection
                if size_match and color_match and type_match and det['confidence'] > best_conf:
                    best_match = det
                    best_conf = det['confidence']
            
            # If no exact match and no type specified, try to find any object with matching size and color
            if not best_match and not target_type:
                for det in self.latest_detections:
                    if (det['size'] == target_size and 
                        det['color'] == target_color and 
                        det['confidence'] > best_conf):
                        best_match = det
                        best_conf = det['confidence']
            
            if target_type:
                yolo_class = object_type_mapping.get(target_type, target_type)
                self.get_logger().info(f'Mapping object type "{target_type}" to YOLO class "{yolo_class}"')
            else:
                yolo_class = "any"
                self.get_logger().info(f'Looking for {target_size} {target_color} object (any type)')
            
            if best_match and self.latest_target_pose:
                self.get_logger().info(f'Found YOLO detection: {best_match}')

                # Get object position from YOLO target pose (now in base_link frame)
                base_pose = self.latest_target_pose
                
                # Extract position in base frame (no transform needed)
                obj_x = base_pose.pose.position.x
                obj_y = base_pose.pose.position.y
                obj_z = base_pose.pose.position.z
                
                self.get_logger().info(f'Object position in base frame: x={obj_x:.3f}, y={obj_y:.3f}, z={obj_z:.3f}')

                # Objects are on the table, so set Z to table height
                pick_z = 0.03  # 3cm above table for grasping
                pick_x = obj_x
                pick_y = obj_y
                
                # Check if pick position is within workspace boundaries
                if not self.is_point_in_workspace(pick_x, pick_y, pick_z):
                    self.get_logger().error(f'Pick position ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f}) is outside workspace boundaries!')
                    self.get_logger().error(f'Workspace: x=[{self.workspace_boundaries["x_min"]:.3f}, {self.workspace_boundaries["x_max"]:.3f}], '
                                          f'y=[{self.workspace_boundaries["y_min"]:.3f}, {self.workspace_boundaries["y_max"]:.3f}], '
                                          f'z=[{self.workspace_boundaries["z_min"]:.3f}, {self.workspace_boundaries["z_max"]:.3f}]')
                    return False
                
                self.get_logger().info('Pick position is within workspace boundaries')
                # Calculate joint angles to reach the target position in base frame

                # Target position in base frame
                target_x = pick_x  # Forward/backward from base
                target_y = pick_y  # Left/right from base
                target_z = pick_z  # Height from base

                self.get_logger().info(f'Calculating IK for target: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')

                # DoBot link lengths (approximate)
                l1 = 0.135  # Base to joint2
                l2 = 0.147  # Joint2 to joint3
                l3 = 0.06   # Joint3 to end effector

                # Calculate distance in XY plane from base
                r = math.sqrt(target_x**2 + target_y**2)
                self.get_logger().info(f'Distance in XY plane: r={r:.3f}')

                # Joint1 (base rotation) - atan2 gives angle from positive X axis
                joint1 = math.atan2(target_y, target_x)
                self.get_logger().info(f'Joint1 (base): {joint1:.3f} rad ({math.degrees(joint1):.1f}°)')

                # For joint2 and joint3, use geometric IK
                # Effective distance from base to target in XY plane (subtract end effector length)
                r_eff = max(0.01, r - l3)  # Avoid negative values
                self.get_logger().info(f'Effective XY distance: r_eff={r_eff:.3f}')

                # Effective height from base (subtract shoulder height)
                z_eff = target_z - l1
                self.get_logger().info(f'Effective height: z_eff={z_eff:.3f}')

                # Distance from joint2 to target point
                d = math.sqrt(r_eff**2 + z_eff**2)
                self.get_logger().info(f'Distance from shoulder to target: d={d:.3f}')

                # Check if target is reachable
                if d > (l2 + l3) or d < abs(l2 - l3):
                    self.get_logger().warn(f'Target may not be reachable: d={d:.3f}, arm reach={l2 + l3:.3f}')
                else:
                    self.get_logger().info(f'Target is within reach')

                # Joint3 (elbow) - use cosine rule
                cos_joint3 = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
                cos_joint3 = max(-1.0, min(1.0, cos_joint3))  # Clamp to valid range
                joint3 = math.acos(cos_joint3)
                self.get_logger().info(f'Joint3 (elbow): {joint3:.3f} rad ({math.degrees(joint3):.1f}°)')

                # Joint2 (shoulder) - elevation angle
                # atan2 gives the angle from horizontal to the target
                alpha = math.atan2(z_eff, r_eff)
                # asin gives the correction for the forearm length
                beta = math.asin((l3 * math.sin(joint3)) / d)
                joint2 = alpha - beta
                self.get_logger().info(f'Joint2 (shoulder): {joint2:.3f} rad ({math.degrees(joint2):.1f}°) - alpha={math.degrees(alpha):.1f}°, beta={math.degrees(beta):.1f}°')

                # Joint4 (wrist) - keep fixed at 0.0 for suction cup alignment
                joint4 = 0.0

                # Convert to joint positions array
                joint_positions = [joint1, joint2, joint3, joint4]

                # Ensure joints stay within limits
                joint_limits = [(-1.57, 1.57), (-0.5, 1.0), (-1.0, 1.0), (-1.0, 1.0)]  # Approximate limits
                for i in range(4):
                    min_val, max_val = joint_limits[i]
                    joint_positions[i] = max(min_val, min(max_val, joint_positions[i]))

                self.get_logger().info(f'Final joint positions: {[f"{p:.3f}" for p in joint_positions]}')
                
                # First move to approach position (above object)
                approach_positions = joint_positions.copy()
                approach_positions[2] -= 0.2  # Move elbow up for approach (joint3 negative = up)
                
                self.get_logger().info(f'Moving to approach position: {[f"{p:.2f}" for p in approach_positions]}')
                msg = Float64MultiArray()
                msg.data = approach_positions
                self.joint_cmd_pub.publish(msg)
                time.sleep(3)  # Wait for movement
                
                # Then move down to pick position
                self.get_logger().info(f'Moving to pick position: {[f"{p:.2f}" for p in joint_positions]}')
                msg = Float64MultiArray()
                msg.data = joint_positions
                self.joint_cmd_pub.publish(msg)
                time.sleep(2)  # Wait for movement
                
                # Activate gripper/suction
                self.activate_end_effector()
                time.sleep(0.5)  # Wait for grasping
                
                # Move back up slightly after grasping
                lift_positions = joint_positions.copy()
                lift_positions[2] -= 0.1  # Lift elbow up (joint3 negative = up)
                
                self.get_logger().info(f'Lifting object: {[f"{p:.2f}" for p in lift_positions]}')
                msg = Float64MultiArray()
                msg.data = lift_positions
                self.joint_cmd_pub.publish(msg)
                
                return True
            else:
                self.get_logger().warn(f'No YOLO detection or target pose found for {target_size} {target_color} {yolo_class} (mapped from {target_type})')
                return False
        
        # Original MoveIt code (if available)
        # Move to approach position
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "base_link"
        approach_pose.header.stamp = self.get_clock().now().to_msg()
        approach_pose.pose.position.x = float(params.get('x', 0.0))
        approach_pose.pose.position.y = float(params.get('y', 0.0))
        approach_pose.pose.position.z = float(params.get('z', 0.0)) + 0.1  # 10cm above
        approach_pose.pose.orientation.w = 1.0

        self.moveit2.move_to_pose(approach_pose)
        self.moveit2.wait_until_executed()

        # Move down to pick
        pick_pose = approach_pose
        pick_pose.pose.position.z = float(params.get('z', 0.0))

        self.moveit2.move_to_pose(pick_pose)
        self.moveit2.wait_until_executed()

        # TODO: Add gripper control here

        return True

    def activate_end_effector(self):
        """Activate the selected end effector (gripper or suction)"""
        if self.end_effector == "gripper":
            self.get_logger().info("Activating gripper")
            # TODO: Add gripper activation command
            # For now, just log
        elif self.end_effector == "suction":
            self.get_logger().info("Activating suction cup")
            # TODO: Add suction activation command
            # For now, just log
        else:
            self.get_logger().warn(f"Unknown end effector: {self.end_effector}")

    def deactivate_end_effector(self):
        """Deactivate the selected end effector"""
        if self.end_effector == "gripper":
            self.get_logger().info("Deactivating gripper")
            # TODO: Add gripper deactivation command
        elif self.end_effector == "suction":
            self.get_logger().info("Deactivating suction cup")
            # TODO: Add suction deactivation command
        else:
            self.get_logger().warn(f"Unknown end effector: {self.end_effector}")

    def place_object(self, params):
        """Place object at specified location"""
        if not self.moveit_available:
            # Use direct joint control
            self.get_logger().info(f'Using direct joint control to place at: {params}')
            x = float(params.get('x', 0.0))
            y = float(params.get('y', 0.0))
            z = float(params.get('z', 0.0))
            
            # Check if place position is within workspace boundaries
            if not self.is_point_in_workspace(x, y, z):
                self.get_logger().error(f'Place position ({x:.3f}, {y:.3f}, {z:.3f}) is outside workspace boundaries!')
                self.get_logger().error(f'Workspace: x=[{self.workspace_boundaries["x_min"]:.3f}, {self.workspace_boundaries["x_max"]:.3f}], '
                                      f'y=[{self.workspace_boundaries["y_min"]:.3f}, {self.workspace_boundaries["y_max"]:.3f}], '
                                      f'z=[{self.workspace_boundaries["z_min"]:.3f}, {self.workspace_boundaries["z_max"]:.3f}]')
                return False
            
            self.get_logger().info('Place position is within workspace boundaries')
            
            # Use simplified inverse kinematics for DoBot
            target_x = x
            target_y = y  
            target_z = z
            
            # DoBot link lengths (approximate)
            l1 = 0.135  # Base to joint2
            l2 = 0.147  # Joint2 to joint3
            l3 = 0.06   # Joint3 to end effector
            
            # Calculate distance in XY plane from base
            r = math.sqrt(target_x**2 + target_y**2)
            
            # Joint1 (base rotation)
            joint1 = math.atan2(target_y, target_x)
            
            # Distance from base to target in XY plane
            r_eff = r - l3
            
            # Height from base
            z_eff = target_z - l1
            
            # Distance from joint2 to target
            d = math.sqrt(r_eff**2 + z_eff**2)
            
            # Joint3 (elbow)
            cos_joint3 = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
            cos_joint3 = max(-1.0, min(1.0, cos_joint3))
            joint3 = math.acos(cos_joint3)
            
            # Joint2 (shoulder)
            joint2 = math.atan2(z_eff, r_eff) - math.asin((l3 * math.sin(joint3)) / d)
            
            # Joint4 (wrist) - keep fixed at 0.0 for suction cup alignment
            joint4 = 0.0
            
            joint_positions = [joint1, joint2, joint3, joint4]
            joint_positions = [max(-3.14, min(3.14, pos)) for pos in joint_positions]
            
            # Move to approach position (above place location)
            approach_positions = joint_positions.copy()
            approach_positions[2] -= 0.2  # Move elbow up for approach
            
            self.get_logger().info(f'Moving to place approach position: {[f"{p:.2f}" for p in approach_positions]}')
            msg = Float64MultiArray()
            msg.data = approach_positions
            self.joint_cmd_pub.publish(msg)
            time.sleep(3)  # Wait for movement
            
            # Move down to place position
            self.get_logger().info(f'Moving to place position: {[f"{p:.2f}" for p in joint_positions]}')
            msg = Float64MultiArray()
            msg.data = joint_positions
            self.joint_cmd_pub.publish(msg)
            time.sleep(2)  # Wait for movement
            
            # Deactivate end effector to release object
            self.deactivate_end_effector()
            time.sleep(0.5)
            
            # Move up slightly after placing
            lift_positions = joint_positions.copy()
            lift_positions[2] -= 0.2  # Lift elbow up
            
            self.get_logger().info(f'Moving up after placement: {[f"{p:.2f}" for p in lift_positions]}')
            msg = Float64MultiArray()
            msg.data = lift_positions
            self.joint_cmd_pub.publish(msg)
            
            return True
        
        # Original MoveIt code
        # Move to place position
        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.header.stamp = self.get_clock().now().to_msg()
        place_pose.pose.position.x = float(params.get('x', 0.0))
        place_pose.pose.position.y = float(params.get('y', 0.0))
        place_pose.pose.position.z = float(params.get('z', 0.0)) + 0.05  # 5cm above
        place_pose.pose.orientation.w = 1.0

        self.moveit2.move_to_pose(place_pose)
        self.moveit2.wait_until_executed()

        # TODO: Add gripper release here

        return True

    def move_home(self):
        """Move to home position (direct joint control for testing)"""
        # Home position: all joints at 0 degrees
        joint_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Publish direct joint command
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_cmd_pub.publish(msg)
        
        self.get_logger().info('Home move command sent (direct)')
        return True

    def move_ready(self):
        """Move to ready position (direct joint control for testing)"""
        # Ready position from SRDF - keep joint4 at 0.0 for suction cup alignment
        joint_positions = [0.0, 0.5, -1.0, 0.0]
        
        # Publish direct joint command
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_cmd_pub.publish(msg)
        
        self.get_logger().info('Ready move command sent (direct)')
        return True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=unknown)
    node = CloudLLMController()
    
    if not parsed_args.no_gui:
        node.gui = GeminiDesktopCommander(node)
        node.get_logger().info('GUI enabled: True')
        node.get_logger().info('Desktop Commander: True')
    else:
        node.get_logger().info('GUI enabled: False')
        node.get_logger().info('Desktop Commander: False')

    # Run ROS spin in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()

    try:
        if not parsed_args.no_gui:
            # Run GUI in main thread
            try:
                node.gui.run()
            except Exception as gui_error:
                node.get_logger().warn(f'GUI failed to start: {gui_error}. Running without GUI.')
                # Fall back to no-GUI mode
                while rclpy.ok():
                    time.sleep(1)
        else:
            # Run without GUI, wait for ROS
            while rclpy.ok():
                time.sleep(1)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Cloud LLM Controller')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()