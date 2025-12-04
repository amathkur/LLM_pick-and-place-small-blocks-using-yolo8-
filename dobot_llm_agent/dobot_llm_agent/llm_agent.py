#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import cv2
import numpy as np
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import os
import json
import threading

# Voice recognition
try:
    import speech_recognition as sr
    VOICE_AVAILABLE = True
except ImportError:
    VOICE_AVAILABLE = False
    print("speech_recognition not available. Install with: pip install SpeechRecognition pyaudio")

# MCP server integration
try:
    from fastmcp import FastMCP
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False
    print("MCP/fastmcp not available. Install with: pip install fastmcp")


class SimpleDobotController:
    """Simple controller that publishes commands to the cloud LLM controller"""
    def __init__(self, node):
        self.node = node
        
    def move_to(self, x_mm, y_mm, z_mm, r_deg):
        """Move to position in millimeters and degrees"""
        command = f"move to {x_mm/1000:.3f} {y_mm/1000:.3f} {z_mm/1000:.3f}"
        msg = String()
        msg.data = command
        self.node.publisher.publish(msg)
        return True
        
    def pick(self, x_mm, y_mm, z_mm, r_deg):
        """Pick object at position - use YOLO detection instead of direct positioning"""
        # Instead of direct positioning, use object detection
        # This will be called with specific object info from command parsing
        command = f"pick at {x_mm/1000:.3f} {y_mm/1000:.3f} {z_mm/1000:.3f}"
        msg = String()
        msg.data = command
        self.node.publisher.publish(msg)
        return True
        
    def place(self, x_mm, y_mm, z_mm, r_deg):
        """Place object at position"""
        command = f"place at {x_mm/1000:.3f} {y_mm/1000:.3f} {z_mm/1000:.3f}"
        msg = String()
        msg.data = command
        self.node.publisher.publish(msg)
        return True


class LLMAgent(Node):
    def __init__(self):
        super().__init__('llm_agent')
        
        # ROS subscriptions
        self.subscription = self.create_subscription(
            String,
            'llm_command',
            self.command_callback,
            10)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.yolo_subscription = self.create_subscription(
            MarkerArray,
            '/yolo/detection_markers',
            self.yolo_callback,
            10)
        
        # ROS publishers
        self.publisher = self.create_publisher(String, '/robot_command', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        self.latest_image = None
        self.detected_objects = []

        # Configure Gemini
        api_key = os.getenv('GEMINI_API_KEY', 'AIzaSyAVQFGMggydYZCkJGwN9_TgFxzezDt3LAw')
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-1.5-flash')

        # Voice recognition setup
        if VOICE_AVAILABLE:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            self.get_logger().info('✅ Voice recognition ready!')
        else:
            self.get_logger().warn('❌ Voice recognition not available')

        # MCP server setup
        if MCP_AVAILABLE:
            self.mcp = FastMCP("dobot-llm-agent")
            self.setup_mcp_tools()
            self.robot_controller = SimpleDobotController(self)
            self.get_logger().info('✅ Robot controller initialized')
        else:
            self.get_logger().warn('❌ MCP server not available')
            self.robot_controller = SimpleDobotController(self)

        self.get_logger().info('🎯 LLM Agent initialized with Gemini, Voice, and MCP integration')
        self.get_logger().info('📝 Available commands: voice/text + object manipulation')
        
        # Start voice listening thread
        if VOICE_AVAILABLE:
            self.voice_thread = threading.Thread(target=self.voice_listening_loop, daemon=True)
            self.voice_thread.start()

    def setup_mcp_tools(self):
        """Setup MCP tools for robot control"""
        @self.mcp.tool(description="Move robot to specified position (x, y, z in meters, r in degrees)")
        def move_to_position(x: float, y: float, z: float, r: float = 0.0):
            if self.robot_controller:
                try:
                    success = self.robot_controller.move_to(x * 1000, y * 1000, z * 1000, r)  # Convert to mm
                    return {"success": success, "message": f"Moved to ({x}, {y}, {z}, {r})"}
                except Exception as e:
                    return {"success": False, "error": str(e)}
            return {"error": "Robot controller not available"}

        @self.mcp.tool(description="Pick up object at specified location")
        def pick_object(x: float, y: float, z: float, r: float = 0.0):
            if self.robot_controller:
                try:
                    success = self.robot_controller.pick(x * 1000, y * 1000, z * 1000, r)
                    return {"success": success, "message": f"Picked at ({x}, {y}, {z})"}
                except Exception as e:
                    return {"success": False, "error": str(e)}
            return {"error": "Robot controller not available"}

        @self.mcp.tool(description="Place object at specified location")
        def place_object(x: float, y: float, z: float, r: float = 0.0):
            if self.robot_controller:
                try:
                    success = self.robot_controller.place(x * 1000, y * 1000, z * 1000, r)
                    return {"success": success, "message": f"Placed at ({x}, {y}, {z})"}
                except Exception as e:
                    return {"success": False, "error": str(e)}
            return {"error": "Robot controller not available"}

    def voice_listening_loop(self):
        """Continuous voice listening loop"""
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                    self.get_logger().info('🎤 Listening for voice command...')
                    audio = self.recognizer.listen(source, timeout=3.0)
                    
                    try:
                        command = self.recognizer.recognize_google(audio)
                        self.get_logger().info(f'🎤 Heard: "{command}"')
                        self.process_voice_command(command)
                    except sr.UnknownValueError:
                        pass  # No speech detected
                    except sr.RequestError as e:
                        self.get_logger().error(f'Voice recognition error: {e}')
                        
            except sr.WaitTimeoutError:
                pass  # Timeout, continue listening
            except Exception as e:
                self.get_logger().error(f'Voice listening error: {e}')
                self.get_logger().info('⏸️  Voice recognition paused for 5 seconds...')
                import time
                time.sleep(5)

    def process_voice_command(self, command):
        """Process voice command and convert to text command"""
        # Create a String message and publish to llm_command topic
        msg = String()
        msg.data = command
        self.subscription.callback(msg)  # Directly call the callback

    def command_callback(self, msg):
        """Process text commands from ROS topic"""
        command = msg.data.strip()
        self.get_logger().info(f'📝 Processing command: "{command}"')
        self.process_command(command)

    def process_command(self, command):
        """Process natural language command using Gemini and MCP tools"""
        try:
            # Parse simple pick commands directly
            command_lower = command.lower().strip()
            
            # Check for "pick size color" pattern
            if command_lower.startswith('pick '):
                parts = command_lower.split()
                if len(parts) >= 3:
                    size = parts[1]  # small, medium, large
                    color = parts[2]  # red, green, blue, yellow, etc.
                    
                    # Find matching detected object
                    target_object = self.find_object_by_size_color(size, color)
                    if target_object:
                        self.get_logger().info(f'Found matching object: {target_object["name"]} at {target_object["position"]}')
                        # Execute pick command for the found object
                        pos = target_object['position']
                        self.execute_pick_command(pos['x'], pos['y'], pos['z'])
                        return
                    else:
                        self.get_logger().warn(f'No {size} {color} object found. Available objects: {[obj["name"] for obj in self.detected_objects]}')
                        self.publisher.publish(String(data=f"ERROR: No {size} {color} object detected"))
                        return
            
            # Check for "pick and place in box" pattern
            if 'pick' in command_lower and 'place' in command_lower and 'box' in command_lower:
                # Parse pick and place in box command
                self.handle_pick_and_place_in_box(command_lower)
                return
            
            # Check for "place in right/left" pattern
            if command_lower.startswith('place') and ('right' in command_lower or 'left' in command_lower):
                self.handle_place_in_position(command_lower)
                return
            
            # For other commands, use Gemini
            context = self.build_context()
            
            # Use Gemini to parse the command
            prompt = f"""
You are a robot control assistant. Based on the following context, interpret the user's command and generate appropriate robot actions.

CONTEXT:
{context}

USER COMMAND: "{command}"

INSTRUCTIONS:
1. Analyze the command and identify the intended action (move, pick, place, detect, etc.)
2. If objects are mentioned, match them to detected objects in the context
3. Generate a structured response with the action type and parameters
4. Use MCP tools when appropriate for robot control

RESPONSE FORMAT:
{{
    "action": "move|pick|place|detect|describe",
    "target": "object_name or position",
    "position": [x, y, z, r] if applicable,
    "confidence": 0.0-1.0,
    "reasoning": "brief explanation"
}}
"""
            
            response = self.model.generate_content(prompt)
            result = json.loads(response.text.strip())
            
            self.get_logger().info(f'🤖 Parsed action: {result}')
            
            # Execute the action using MCP tools
            self.execute_action(result)
            
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')
            self.publisher.publish(String(data=f"ERROR: {str(e)}"))

    def build_context(self):
        """Build context string from detected objects and robot state"""
        context = "DETECTED OBJECTS:\n"
        if self.detected_objects:
            for obj in self.detected_objects:
                context += f"- {obj['name']} at position ({obj['position'][0]:.3f}, {obj['position'][1]:.3f}, {obj['position'][2]:.3f})\n"
        else:
            context += "- No objects currently detected\n"
        
        context += "\nROBOT CAPABILITIES:\n"
        context += "- Move to positions (x, y, z in meters, r in degrees)\n"
        context += "- Pick up objects\n"
        context += "- Place objects\n"
        context += "- Detect objects with camera\n"
        
        return context

    def execute_action(self, action_data):
        """Execute parsed action using robot controller"""
        try:
            action = action_data.get('action', '')
            target = action_data.get('target', '')
            position = action_data.get('position', [0, 0, 0, 0])
            
            if action == 'move':
                if self.robot_controller:
                    result = self.robot_controller.move_to(*position)
                    self.publisher.publish(String(data=f"MOVED: {result}"))
                else:
                    self.publisher.publish(String(data=f"SIMULATED_MOVE: {position}"))
                    
            elif action == 'pick':
                if self.robot_controller:
                    result = self.robot_controller.pick(*position)
                    self.publisher.publish(String(data=f"PICKED: {result}"))
                else:
                    self.publisher.publish(String(data=f"SIMULATED_PICK: {position}"))
                    
            elif action == 'place':
                if self.robot_controller:
                    result = self.robot_controller.place(*position)
                    self.publisher.publish(String(data=f"PLACED: {result}"))
                else:
                    self.publisher.publish(String(data=f"SIMULATED_PLACE: {position}"))
                    
            elif action == 'detect':
                self.publisher.publish(String(data=f"DETECTING: Scanning for objects"))
                # Trigger YOLO detection
                self.publisher.publish(String(data="DETECT"))
                
            elif action == 'describe':
                context = self.build_context()
                self.publisher.publish(String(data=f"CONTEXT: {context}"))
                
            else:
                self.publisher.publish(String(data=f"UNKNOWN_ACTION: {action}"))
                
        except Exception as e:
            self.get_logger().error(f'Action execution error: {e}')
            self.publisher.publish(String(data=f"EXECUTION_ERROR: {str(e)}"))

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().debug('Received camera image')

    def yolo_callback(self, msg):
        # Process YOLO marker array
        self.detected_objects = []
        
        # Process markers in pairs (sphere + text markers)
        sphere_markers = {}
        text_markers = {}
        
        for marker in msg.markers:
            if marker.type == 2:  # Marker.SPHERE
                sphere_markers[marker.id] = marker
            elif marker.type == 9:  # Marker.TEXT_VIEW_FACING
                # Text markers have IDs offset by 1000
                original_id = marker.id - 1000
                text_markers[original_id] = marker
        
        # Combine sphere and text markers
        for marker_id, sphere_marker in sphere_markers.items():
            text_marker = text_markers.get(marker_id)
            
            # Safely get text from text marker
            name = f'object_{marker_id}'
            if text_marker and hasattr(text_marker, 'text'):
                name = text_marker.text
            
            obj = {
                'name': name,
                'position': [sphere_marker.pose.position.x, sphere_marker.pose.position.y, sphere_marker.pose.position.z],
                'color': self.get_marker_color(sphere_marker),
                'confidence': 1.0  # Default confidence since not stored in marker
            }
            self.detected_objects.append(obj)
        
        self.get_logger().info(f'Updated detected objects: {len(self.detected_objects)} objects')

    def find_object_by_size_color(self, size, color):
        """Find detected object matching size and color"""
        for obj in self.detected_objects:
            obj_name = obj.get('name', '').lower()
            if size.lower() in obj_name and color.lower() in obj_name:
                return obj
        return None
    
    def execute_pick_command(self, x, y, z):
        """Execute pick command at specific position"""
        try:
            # Convert meters to millimeters for robot controller
            x_mm, y_mm, z_mm = x * 1000, y * 1000, z * 1000
            
            if self.robot_controller:
                result = self.robot_controller.pick(x_mm, y_mm, z_mm, 0)
                self.publisher.publish(String(data=f"PICKED: {result} at ({x:.3f}, {y:.3f}, {z:.3f})"))
            else:
                self.publisher.publish(String(data=f"SIMULATED_PICK: at ({x:.3f}, {y:.3f}, {z:.3f})"))
                
        except Exception as e:
            self.get_logger().error(f'Pick execution error: {e}')
            self.publisher.publish(String(data=f"PICK_ERROR: {str(e)}"))

    def handle_pick_and_place_in_box(self, command):
        """Handle 'pick and place in box' commands"""
        try:
            # Parse the command to extract object description
            # Example: "pick small yellow cube and place in box"
            command_lower = command.lower()
            
            # Find the black box first
            black_box = self.find_black_box()
            if not black_box:
                self.publisher.publish(String(data="ERROR: No black box detected for placement"))
                return
            
            # Extract object description from command
            # Look for pattern like "pick [size] [color] [shape]"
            import re
            match = re.search(r'pick\s+(\w+)\s+(\w+)\s+(\w+)', command_lower)
            if match:
                size, color, shape = match.groups()
                
                # Find the object to pick
                target_object = self.find_object_by_size_color_shape(size, color, shape)
                if target_object:
                    self.get_logger().info(f'Found object to pick: {target_object["name"]} at {target_object["position"]}')
                    
                    # Execute pick command
                    pick_pos = target_object['position']
                    self.execute_pick_command(pick_pos['x'], pick_pos['y'], pick_pos['z'])
                    
                    # Wait a moment then place in box
                    import time
                    time.sleep(2)  # Simple delay
                    
                    # Place in box position
                    box_pos = black_box['position']
                    # Offset slightly above the box
                    place_pos = {
                        'x': box_pos['x'],
                        'y': box_pos['y'], 
                        'z': box_pos['z'] + 0.05  # 5cm above box
                    }
                    self.execute_place_command(place_pos['x'], place_pos['y'], place_pos['z'])
                    
                else:
                    self.publisher.publish(String(data=f"ERROR: No {size} {color} {shape} object found"))
            else:
                self.publisher.publish(String(data="ERROR: Could not parse pick command"))
                
        except Exception as e:
            self.get_logger().error(f'Pick and place error: {e}')
            self.publisher.publish(String(data=f"PICK_PLACE_ERROR: {str(e)}"))

    def handle_place_in_position(self, command):
        """Handle 'place in right/left' commands"""
        try:
            command_lower = command.lower()
            
            # Define positions for right and left
            # These are relative to the robot's base frame
            if 'right' in command_lower:
                # Right side position (positive Y in base_link frame)
                place_pos = {'x': 0.2, 'y': 0.2, 'z': 0.1}
            elif 'left' in command_lower:
                # Left side position (negative Y in base_link frame)  
                place_pos = {'x': 0.2, 'y': -0.2, 'z': 0.1}
            else:
                self.publisher.publish(String(data="ERROR: Could not determine placement position"))
                return
            
            self.execute_place_command(place_pos['x'], place_pos['y'], place_pos['z'])
            
        except Exception as e:
            self.get_logger().error(f'Place in position error: {e}')
            self.publisher.publish(String(data=f"PLACE_POSITION_ERROR: {str(e)}"))

    def execute_place_command(self, x, y, z):
        """Execute place command at specific position"""
        try:
            # Convert meters to millimeters for robot controller
            x_mm, y_mm, z_mm = x * 1000, y * 1000, z * 1000
            
            if self.robot_controller:
                result = self.robot_controller.place(x_mm, y_mm, z_mm, 0)
                self.publisher.publish(String(data=f"PLACED: {result} at ({x:.3f}, {y:.3f}, {z:.3f})"))
            else:
                self.publisher.publish(String(data=f"SIMULATED_PLACE: at ({x:.3f}, {y:.3f}, {z:.3f})"))
                
        except Exception as e:
            self.get_logger().error(f'Place execution error: {e}')
            self.publisher.publish(String(data=f"PLACE_ERROR: {str(e)}"))

    def find_black_box(self):
        """Find the black box for placement"""
        for obj in self.detected_objects:
            obj_name = obj.get('name', '').lower()
            if 'black' in obj_name and 'box' in obj_name:
                return obj
        return None

    def find_object_by_size_color_shape(self, size, color, shape):
        """Find detected object matching size, color, and shape"""
        for obj in self.detected_objects:
            obj_name = obj.get('name', '').lower()
            if (size.lower() in obj_name and 
                color.lower() in obj_name and 
                shape.lower() in obj_name):
                return obj
        return None

def main(args=None):
    rclpy.init(args=args)
    llm_agent = LLMAgent()
    rclpy.spin(llm_agent)
    llm_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()