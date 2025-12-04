#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import logging
from glob import glob
import time
import math
import os
import json
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Try to import pydobot libraries
try:
    import pydobot as pydobot
    logging.info("Imported as 'pydobot' ✅")
except Exception:
    try:
        import pydobot2 as pydobot
        logging.info("Imported as 'pydobot2' ✅")
    except Exception:
        pydobot = None
        logging.info("No Dobot Python driver found; will run in dry-run mode")

def find_ports():
    """Find available serial ports for Dobot connection"""
    candidates = []
    candidates += glob('/dev/ttyACM*')
    candidates += glob('/dev/ttyUSB*')
    candidates += glob('/dev/serial/by-id/*')
    # remove duplicates while preserving order
    seen = set()
    out = []
    for p in candidates:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out

class YOLOv9DobotController(Node):
    def __init__(self):
        super().__init__('yolov9_dobot_controller')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # YOLOv9 model
        self.model = None
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # YOLOv9 model path - UPDATE THIS PATH with your downloaded weights
        self.model_path = '/home/abdulhamid/dobot_rviz_ws/models/yolov9-c.pt'  # <-- UPDATED

        # Camera
        self.cap = None
        self.camera_index = 0  # Default USB camera

        # Dobot connection
        self.dobot = None
        self.dobot_connected = False

        # Control parameters
        self.target_class = "bottle"  # Object to pick up
        self.workspace_bounds = {
            'x': [-0.3, 0.3],  # meters
            'y': [-0.3, 0.3],
            'z': [0.0, 0.4]
        }

        # Home position
        self.home_position = (0.2, 0.0, 0.1, 0.0)  # x, y, z, rotation

        # ROS2 publishers and subscribers
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('YOLOv9 Dobot Controller initialized')

        # Initialize components
        self.initialize_yolo()
        self.initialize_camera()
        self.initialize_dobot()

    def initialize_yolo(self):
        """Initialize YOLOv9 model"""
        try:
            # Load YOLOv9 model (you'll need to download the model weights)
            if not os.path.exists(self.model_path):
                self.get_logger().warning(f'YOLOv9 weights not found at {self.model_path}')
                self.get_logger().info('Download weights from: https://github.com/WongKinYiu/yolov9/releases')
                self.get_logger().info('Then update model_path in this script')
                return

            self.model = torch.hub.load('WongKinYiu/yolov9', 'custom', self.model_path, trust_repo=True)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info(f'YOLOv9 model loaded successfully from {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv9 model: {e}')
            self.get_logger().info('Running without YOLOv9 - camera feed only')

    def initialize_camera(self):
        """Initialize USB camera"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera {self.camera_index}')
                return

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            self.get_logger().info(f'Camera {self.camera_index} initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')

    def initialize_dobot(self):
        """Initialize Dobot connection"""
        if pydobot is None:
            self.get_logger().warning('No pydobot library available - running in simulation mode')
            return

        ports = find_ports()
        if not ports:
            self.get_logger().error('No serial ports found for Dobot')
            return

        for port in ports:
            try:
                self.get_logger().info(f'Trying to connect to Dobot on port: {port}')
                self.dobot = pydobot.Dobot(port=port, verbose=False)
                self.dobot_connected = True
                self.get_logger().info(f'Successfully connected to Dobot on {port}')

                # Move to home position
                self.move_to_home()
                break
            except Exception as e:
                self.get_logger().warning(f'Failed to connect on {port}: {e}')
                continue

        if not self.dobot_connected:
            self.get_logger().error('Failed to connect to Dobot on any available port')

    def move_to_home(self):
        """Move robot to home position"""
        if not self.dobot_connected:
            self.get_logger().info('Simulation: Moving to home position')
            return

        try:
            x, y, z, r = self.home_position
            self.dobot.move_to(x, y, z, r, wait=True)
            self.get_logger().info('Moved to home position')
        except Exception as e:
            self.get_logger().error(f'Failed to move to home position: {e}')

    def detect_objects(self, frame):
        """Detect objects using YOLOv9"""
        if self.model is None:
            return []

        try:
            # Preprocess frame
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (640, 640))
            img = img.astype(np.float32) / 255.0
            img = np.transpose(img, (2, 0, 1))
            img = torch.from_numpy(img).unsqueeze(0).to(self.device)

            # Run inference
            with torch.no_grad():
                results = self.model(img)

            # Process results
            detections = []
            if hasattr(results, 'xyxy'):
                for *xyxy, conf, cls in results.xyxy[0]:
                    if conf > 0.5:  # Confidence threshold
                        x1, y1, x2, y2 = map(int, xyxy)
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        detections.append({
                            'bbox': (x1, y1, x2, y2),
                            'center': (center_x, center_y),
                            'confidence': conf.item(),
                            'class': int(cls.item())
                        })

            return detections
        except Exception as e:
            self.get_logger().error(f'Object detection failed: {e}')
            return []

    def pixel_to_world(self, pixel_x, pixel_y, depth=0.3):
        """Convert pixel coordinates to world coordinates using calibration"""
        try:
            # Load calibration data
            calib_file = '/home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/camera_calibration.json'
            if os.path.exists(calib_file):
                with open(calib_file, 'r') as f:
                    calib = json.load(f)

                # Use calibrated camera matrix
                camera_matrix = np.array(calib['camera_matrix'])
                fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
                cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
            else:
                # Default camera intrinsics
                fx, fy = 600, 600
                cx, cy = 320, 240
        except:
            # Fallback to default values
            fx, fy = 600, 600
            cx, cy = 320, 240

        # Convert to camera coordinates
        x_camera = (pixel_x - cx) * depth / fx
        y_camera = (pixel_y - cy) * depth / fy
        z_camera = depth

        # Simple transformation (camera at origin looking down)
        # In a real setup, you'd use the full camera-to-robot transformation matrix
        x_world = x_camera
        y_world = -y_camera  # Flip Y axis
        z_world = z_camera

        return x_world, y_world, z_world

    def pick_object(self, x, y, z):
        """Pick up object at specified coordinates"""
        if not self.dobot_connected:
            self.get_logger().info(f'Simulation: Picking object at ({x:.3f}, {y:.3f}, {z:.3f})')
            return

        try:
            # Move above object
            self.dobot.move_to(x, y, z + 0.1, 0.0, wait=True)

            # Move down to object
            self.dobot.move_to(x, y, z, 0.0, wait=True)

            # Close gripper (assuming suction cup or gripper)
            # self.dobot.gripper_close()  # Uncomment if you have a gripper

            # Move up
            self.dobot.move_to(x, y, z + 0.1, 0.0, wait=True)

            # Move to drop location
            self.dobot.move_to(0.2, 0.2, 0.1, 0.0, wait=True)

            # Open gripper
            # self.dobot.gripper_open()  # Uncomment if you have a gripper

            self.get_logger().info(f'Successfully picked object at ({x:.3f}, {y:.3f}, {z:.3f})')

        except Exception as e:
            self.get_logger().error(f'Failed to pick object: {e}')

    def control_loop(self):
        """Main control loop"""
        if self.cap is None or not self.cap.isOpened():
            return

        # Capture frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # Publish camera image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

        # Detect objects
        detections = self.detect_objects(frame)

        # Draw detections on frame
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Check if target object detected
            if self.model and hasattr(self.model, 'names'):
                class_name = self.model.names[detection['class']]
                if class_name == self.target_class:
                    center_x, center_y = detection['center']

                    # Convert to world coordinates
                    x_world, y_world, z_world = self.pixel_to_world(center_x, center_y)

                    # Check if within workspace bounds
                    if (self.workspace_bounds['x'][0] <= x_world <= self.workspace_bounds['x'][1] and
                        self.workspace_bounds['y'][0] <= y_world <= self.workspace_bounds['y'][1] and
                        self.workspace_bounds['z'][0] <= z_world <= self.workspace_bounds['z'][1]):

                        self.get_logger().info(f'Target {self.target_class} detected at world coordinates: ({x_world:.3f}, {y_world:.3f}, {z_world:.3f})')

                        # Pick up the object
                        self.pick_object(x_world, y_world, z_world)

                        # Return to home
                        self.move_to_home()

        # Display frame (optional - for debugging)
        cv2.imshow('YOLOv9 Detection', frame)
        cv2.waitKey(1)

    def __del__(self):
        """Cleanup"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

        if self.dobot_connected and self.dobot:
            try:
                self.move_to_home()
                self.dobot.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)

    # Setup logging
    logging.basicConfig(level=logging.INFO)

    node = YOLOv9DobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()