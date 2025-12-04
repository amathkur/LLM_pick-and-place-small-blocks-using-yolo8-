#!/usr/bin/env python3
"""
Integrated Pick-Place System with Voice/Text, Depth, LLM, and Visual Feedback
Features:
- Voice and text command processing with LLM
- Depth-based 3D object localization
- Visual markers in RViz showing detected objects
- Camera-suction offset compensation (50mm)
- Automatic home position after task completion
- Real-time feedback
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import numpy as np
import threading
import json
import time
import tf2_ros
from rclpy.duration import Duration

# Voice recognition (optional)
try:
    import speech_recognition as sr
    SPEECH_AVAILABLE = True
except ImportError:
    SPEECH_AVAILABLE = False
    print("⚠️  Speech recognition not available. Install: pip install SpeechRecognition pyaudio")

# LLM integration (optional)
try:
    import requests
    LLM_AVAILABLE = True
except ImportError:
    LLM_AVAILABLE = False


class IntegratedPickPlaceSystem(Node):
    def __init__(self):
        super().__init__('integrated_pick_place_system')
        
        # ============ PARAMETERS ============
        self.declare_parameter('camera_offset_x', 0.050)  # 50mm camera offset from suction (105-55=50mm)
        self.declare_parameter('home_position', [0.2, 0.0, 0.25])
        self.declare_parameter('llm_enabled', False)
        self.declare_parameter('llm_api_url', 'http://localhost:11434/api/generate')  # Ollama default
        
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.home_pos = self.get_parameter('home_position').value
        self.llm_enabled = self.get_parameter('llm_enabled').value
        self.llm_api = self.get_parameter('llm_api_url').value
        
        # ============ ROS INTERFACES ============
        # Subscribers
        self.detection_sub = self.create_subscription(
            String, '/yolo/detections', self.detection_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects_markers', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        self.command_pub = self.create_publisher(String, '/robot/command', 10)
        
        # MoveIt Action Client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('⏳ Waiting for MoveGroup action server...')
        # Don't block - continue initialization
        
        # TF buffer / listener for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ============ STATE VARIABLES ============
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None
        self.rgb_image = None
        self.current_detections = []
        self.current_position = list(self.home_pos)
        self.is_busy = False
        self.task_history = []
        
        # Voice recognition
        if SPEECH_AVAILABLE:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            self.get_logger().info('🎤 Voice recognition ready')
        
        # ============ STARTUP ============
        self.get_logger().info('=' * 70)
        self.get_logger().info('🤖 INTEGRATED PICK-PLACE SYSTEM INITIALIZED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'📏 Camera offset: {self.camera_offset_x*1000:.0f}mm')
        self.get_logger().info(f'🏠 Home position: {self.home_pos}')
        self.get_logger().info(f'🧠 LLM enabled: {self.llm_enabled}')
        self.get_logger().info(f'🎤 Voice enabled: {SPEECH_AVAILABLE}')
        self.get_logger().info('=' * 70)
        
    # ============ CAMERA CALLBACKS ============
    
    def camera_info_callback(self, msg):
        """Store camera calibration parameters for depth-to-3D conversion"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'📷 Camera calibration received: {msg.width}x{msg.height}')
            self.fx = msg.k[0]  # Focal length X
            self.fy = msg.k[4]  # Focal length Y
            self.cx = msg.k[2]  # Principal point X
            self.cy = msg.k[5]  # Principal point Y
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def detection_callback(self, msg):
        """Process YOLO detections and create 3D positions with depth"""
        try:
            detections = json.loads(msg.data)
            self.current_detections = []
            markers = MarkerArray()
            
            for idx, det in enumerate(detections):
                # Parse detection: "small red cube at (x, y, conf)"
                obj_class = det.get('class', 'unknown')
                bbox = det.get('bbox', [0, 0, 0, 0])  # [x_min, y_min, x_max, y_max]
                confidence = det.get('confidence', 0.0)
                
                # Calculate bbox center in image coordinates
                img_x = int((bbox[0] + bbox[2]) / 2)
                img_y = int((bbox[1] + bbox[3]) / 2)
                
                # Get 3D position from depth (in camera frame)
                position_3d = self.get_3d_position(img_x, img_y)

                if position_3d:
                    camera_x, camera_y, camera_z = position_3d

                    # Build a PoseStamped in camera frame
                    from geometry_msgs.msg import PoseStamped
                    pose_cam = PoseStamped()
                    pose_cam.header.frame_id = 'camera_link'
                    pose_cam.header.stamp = self.get_clock().now().to_msg()
                    pose_cam.pose.position.x = camera_x
                    pose_cam.pose.position.y = camera_y
                    pose_cam.pose.position.z = camera_z
                    pose_cam.pose.orientation.w = 1.0

                    # Transform camera pose to base_link so markers align with robot base
                    try:
                        pose_base = self.tf_buffer.transform(pose_cam, 'base_link', timeout=Duration(seconds=1.0))
                        base_x = pose_base.pose.position.x
                        base_y = pose_base.pose.position.y
                        base_z = pose_base.pose.position.z
                    except Exception as e:
                        # If TF fails, fall back to using camera-frame coordinates (assume alignment)
                        self.get_logger().warn(f'TF transform failed: {e} - publishing in camera frame')
                        base_x, base_y, base_z = camera_x, camera_y, camera_z

                    # Apply camera-suction offset compensation in base frame (camera X ahead of suction)
                    suction_x = base_x - self.camera_offset_x
                    suction_y = base_y
                    suction_z = base_z

                    detection_info = {
                        'class': obj_class,
                        'confidence': confidence,
                        'camera_pos': (camera_x, camera_y, camera_z),
                        'camera_pos_base': (base_x, base_y, base_z),
                        'suction_pos': (suction_x, suction_y, suction_z),
                        'bbox': bbox
                    }
                    self.current_detections.append(detection_info)

                    # Create visual marker for RViz in base_link frame
                    marker = self.create_marker(idx, obj_class, base_x, base_y, base_z, confidence)
                    markers.markers.append(marker)

                    # Also create a marker for suction target position in base_link
                    suction_marker = self.create_suction_marker(idx, suction_x, suction_y, suction_z)
                    markers.markers.append(suction_marker)
            
            # Publish markers to RViz
            if markers.markers:
                self.marker_pub.publish(markers)
                self.get_logger().info(f'📍 Visualized {len(self.current_detections)} objects in RViz')
                
        except Exception as e:
            self.get_logger().error(f'Detection processing error: {e}')
    
    def get_3d_position(self, img_x, img_y):
        """Convert 2D image coordinates + depth to 3D position in camera frame"""
        if self.depth_image is None or self.camera_info is None:
            return None
        
        try:
            # Get depth value at pixel location
            depth = self.depth_image[img_y, img_x]
            
            if depth <= 0 or np.isnan(depth) or np.isinf(depth):
                # Invalid depth, use default
                depth = 0.3  # 30cm default
                self.get_logger().warn(f'Invalid depth at ({img_x},{img_y}), using default: {depth}m')
            
            # Convert to meters if needed
            if depth > 100:  # Likely in millimeters
                depth = depth / 1000.0
            
            # Convert pixel coordinates to 3D using camera intrinsics
            # X = (u - cx) * Z / fx
            # Y = (v - cy) * Z / fy
            # Z = depth
            x = (img_x - self.cx) * depth / self.fx
            y = (img_y - self.cy) * depth / self.fy
            z = depth
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().error(f'3D position calculation error: {e}')
            return None
    
    def create_marker(self, marker_id, obj_class, x, y, z, confidence):
        """Create visualization marker for detected object (camera position)"""
        marker = Marker()
        marker.header.frame_id = 'camera_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detected_objects'
        marker.id = marker_id * 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (where camera sees it)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color based on object class
        colors = {
            'cube': (1.0, 0.0, 0.0, 0.8),      # Red
            'sphere': (0.0, 1.0, 0.0, 0.8),    # Green
            'cylinder': (0.0, 0.0, 1.0, 0.8),  # Blue
            'pyramid': (1.0, 1.0, 0.0, 0.8),   # Yellow
        }
        color = colors.get(obj_class, (1.0, 1.0, 1.0, 0.8))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        
        marker.lifetime.sec = 2  # Disappear after 2 seconds
        
        return marker
    
    def create_suction_marker(self, marker_id, x, y, z):
        """Create visualization marker for suction target position"""
        marker = Marker()
        marker.header.frame_id = 'camera_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'suction_targets'
        marker.id = marker_id * 2 + 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position (where suction will move)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.03
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        
        # Color - Purple for suction target
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        
        marker.lifetime.sec = 2
        
        return marker
    
    # ============ COMMAND PROCESSING ============
    
    def process_command(self, command_text):
        """Process text or voice command, optionally using LLM"""
        command_text = command_text.lower().strip()
        self.get_logger().info(f'📝 Processing command: "{command_text}"')
        
        # Use LLM to parse command if enabled
        if self.llm_enabled and LLM_AVAILABLE:
            parsed = self.parse_command_with_llm(command_text)
        else:
            parsed = self.parse_command_simple(command_text)
        
        if not parsed:
            self.get_logger().warn(f'❌ Could not understand command: "{command_text}"')
            return False
        
        # Execute action
        return self.execute_action(parsed)
    
    def parse_command_simple(self, text):
        """Simple rule-based command parsing"""
        text = text.lower()
        
        # Go home
        if 'home' in text or 'reset' in text:
            return {'action': 'go_home'}
        
        # Pick commands
        if 'pick' in text:
            # Extract object type
            objects = ['cube', 'sphere', 'cylinder', 'pyramid', 'bottle', 'cup']
            for obj in objects:
                if obj in text:
                    # Extract placement if specified
                    placement = 'box_a'  # default
                    if 'box a' in text or 'boxa' in text:
                        placement = 'box_a'
                    elif 'box b' in text or 'boxb' in text:
                        placement = 'box_b'
                    elif 'bin' in text:
                        placement = 'bin'
                    
                    return {
                        'action': 'pick_and_place',
                        'object': obj,
                        'placement': placement
                    }
        
        # Movement commands
        movements = {
            'left': {'action': 'move', 'direction': 'left'},
            'right': {'action': 'move', 'direction': 'right'},
            'up': {'action': 'move', 'direction': 'up'},
            'down': {'action': 'move', 'direction': 'down'},
            'forward': {'action': 'move', 'direction': 'forward'},
            'back': {'action': 'move', 'direction': 'back'},
        }
        
        for keyword, result in movements.items():
            if keyword in text:
                return result
        
        return None
    
    def parse_command_with_llm(self, text):
        """Use LLM to parse natural language command"""
        try:
            prompt = f"""Parse this robot command into JSON format. Return ONLY valid JSON.
            
Command: "{text}"

Return format:
{{"action": "pick_and_place" | "move" | "go_home", "object": "cube|sphere|cylinder|pyramid", "placement": "box_a|box_b|bin", "direction": "left|right|up|down|forward|back"}}

Examples:
"pick the red cube" -> {{"action": "pick_and_place", "object": "cube", "placement": "box_a"}}
"move left" -> {{"action": "move", "direction": "left"}}
"go home" -> {{"action": "go_home"}}

JSON:"""
            
            response = requests.post(
                self.llm_api,
                json={
                    'model': 'llama2',
                    'prompt': prompt,
                    'stream': False,
                    'temperature': 0.1
                },
                timeout=5.0
            )
            
            if response.status_code == 200:
                result = response.json()
                llm_output = result.get('response', '')
                # Extract JSON from response
                import re
                json_match = re.search(r'\{.*\}', llm_output)
                if json_match:
                    return json.loads(json_match.group())
            
        except Exception as e:
            self.get_logger().warn(f'LLM parsing failed: {e}, using simple parser')
        
        return self.parse_command_simple(text)
    
    # ============ ACTION EXECUTION ============
    
    def execute_action(self, parsed_command):
        """Execute parsed command"""
        if self.is_busy:
            self.get_logger().warn('⏳ Robot is busy, please wait...')
            return False
        
        action = parsed_command.get('action')
        
        if action == 'go_home':
            return self.go_home()
        
        elif action == 'move':
            direction = parsed_command.get('direction')
            return self.move_relative(direction)
        
        elif action == 'pick_and_place':
            obj_type = parsed_command.get('object')
            placement = parsed_command.get('placement', 'box_a')
            return self.pick_and_place(obj_type, placement)
        
        return False
    
    def go_home(self):
        """Move robot to home position"""
        self.get_logger().info('🏠 Going to home position...')
        self.is_busy = True
        
        try:
            x, y, z = self.home_pos
            success = self.move_to_position(x, y, z, 'HOME')
            self.current_position = list(self.home_pos)
            self.status_pub.publish(String(data='at_home'))
            return success
        finally:
            self.is_busy = False
    
    def move_relative(self, direction, step=0.05):
        """Move robot in specified direction"""
        self.get_logger().info(f'↔️ Moving {direction}...')
        self.is_busy = True
        
        try:
            x, y, z = self.current_position
            
            if direction == 'left':
                y += step
            elif direction == 'right':
                y -= step
            elif direction == 'up':
                z += step
            elif direction == 'down':
                z -= step
            elif direction == 'forward':
                x += step
            elif direction == 'back':
                x -= step
            
            success = self.move_to_position(x, y, z, direction.upper())
            if success:
                self.current_position = [x, y, z]
            return success
        finally:
            self.is_busy = False
    
    def pick_and_place(self, object_type, placement):
        """Pick specified object and place it"""
        self.get_logger().info(f'🎯 Pick {object_type} and place in {placement}')
        self.is_busy = True
        
        try:
            # Find object in current detections
            target_det = None
            for det in self.current_detections:
                if object_type in det['class'].lower():
                    target_det = det
                    break
            
            if not target_det:
                self.get_logger().error(f'❌ Object "{object_type}" not found in view!')
                return False
            
            # Get suction position (already offset-compensated)
            suction_x, suction_y, suction_z = target_det['suction_pos']
            
            self.get_logger().info(f'📸 Camera sees at: ({target_det["camera_pos"][0]:.3f}, {target_det["camera_pos"][1]:.3f}, {target_det["camera_pos"][2]:.3f})')
            self.get_logger().info(f'🔧 Suction moves to: ({suction_x:.3f}, {suction_y:.3f}, {suction_z:.3f})')
            self.get_logger().info(f'   ⚙️ Compensated {self.camera_offset_x*1000:.0f}mm camera offset')
            
            # Move to pick position
            if not self.move_to_position(suction_x, suction_y, suction_z, 'PICK'):
                return False
            
            # Simulate gripper close
            time.sleep(0.5)
            self.get_logger().info('🤏 Closing gripper...')
            
            # Move up
            if not self.move_to_position(suction_x, suction_y, suction_z + 0.1, 'LIFT'):
                return False
            
            # Move to placement location
            place_positions = {
                'box_a': (0.3, 0.15, 0.15),
                'box_b': (0.3, -0.15, 0.15),
                'bin': (0.35, 0.0, 0.1)
            }
            place_x, place_y, place_z = place_positions.get(placement, place_positions['box_a'])
            
            if not self.move_to_position(place_x, place_y, place_z, 'PLACE'):
                return False
            
            # Simulate gripper open
            time.sleep(0.5)
            self.get_logger().info('✋ Opening gripper...')
            
            # Record task
            self.task_history.append({
                'object': object_type,
                'placement': placement,
                'timestamp': time.time()
            })
            
            self.get_logger().info(f'✅ Task complete! Picked {object_type} → {placement}')
            
            # Go home automatically
            time.sleep(1.0)
            return self.go_home()
            
        except Exception as e:
            self.get_logger().error(f'❌ Pick and place failed: {e}')
            return False
        finally:
            self.is_busy = False
    
    def move_to_position(self, x, y, z, description):
        """Send movement command to MoveIt (stub for now)"""
        self.get_logger().info(f'🚀 Moving to: ({x:.3f}, {y:.3f}, {z:.3f}) [{description}]')
        
        # Publish target pose for visualization
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)
        
        # Simulate movement (replace with actual MoveIt call)
        time.sleep(1.0)
        
        self.status_pub.publish(String(data=f'moved_to_{description}'))
        return True
    
    # ============ VOICE INPUT ============
    
    def listen_for_voice(self):
        """Listen for voice command"""
        if not SPEECH_AVAILABLE:
            self.get_logger().error('Voice recognition not available!')
            return None
        
        with self.microphone as source:
            self.get_logger().info('🎤 Listening...')
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            
            try:
                audio = self.recognizer.listen(source, timeout=5.0)
                command = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'🔊 Heard: "{command}"')
                return command
            except sr.WaitTimeoutError:
                self.get_logger().warn('⏱️ No speech detected')
            except sr.UnknownValueError:
                self.get_logger().warn('❓ Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'❌ Speech recognition error: {e}')
        
        return None


def main(args=None):
    rclpy.init(args=args)
    system = IntegratedPickPlaceSystem()
    
    # Spin in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(system,), daemon=True)
    spin_thread.start()
    
    print("\n" + "=" * 70)
    print("🤖 INTEGRATED PICK-PLACE SYSTEM")
    print("=" * 70)
    print("Commands:")
    print("  • 'pick [object]' - Pick object (cube/sphere/cylinder/pyramid)")
    print("  • 'pick [object] and place in box a/b' - Pick and place")
    print("  • 'move left/right/up/down/forward/back' - Move robot")
    print("  • 'go home' - Return to home position")
    print("  • 'exit' / 'quit' - Exit system")
    print("=" * 70)
    print("\n💡 Objects detected by YOLO will appear as colored markers in RViz")
    print("   • Sphere = detected object position (camera view)")
    print("   • Arrow = target suction position (offset-compensated)")
    print("=" * 70)
    
    mode = input("\nChoose mode - (1) Text, (2) Voice: ").strip()
    
    try:
        while rclpy.ok():
            if mode == '2' and SPEECH_AVAILABLE:
                command = system.listen_for_voice()
                if command:
                    system.process_command(command)
            else:
                command = input('\n💬 Command: ')
                if command.lower() in ['exit', 'quit']:
                    break
                system.process_command(command)
    except KeyboardInterrupt:
        pass
    
    print("\n👋 Shutting down...")
    system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
