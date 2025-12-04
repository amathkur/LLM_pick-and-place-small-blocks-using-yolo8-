#!/usr/bin/env python3
"""
Camera Publisher Node - Publishes webcam feed as ROS2 Image messages
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Parameters
        self.declare_parameter('camera_index', 0)  # USB camera at /dev/video0
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        camera_index = self.get_parameter('camera_index').value
        frame_rate = self.get_parameter('frame_rate').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        if not self.cap.isOpened():
            self.get_logger().warn(f'Failed to open camera {camera_index}, will publish test pattern')
            self.use_test_pattern = True
            self.cap = None
        else:
            self.use_test_pattern = False
            self.get_logger().info(f'Camera opened successfully: {self.width}x{self.height} @ {frame_rate}Hz')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Timer for publishing
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Camera info (simplified calibration)
        self.camera_info_msg = self.create_camera_info(self.width, self.height)
        
    def create_camera_info(self, width, height):
        """Create simplified camera info message"""
        msg = CameraInfo()
        msg.header.frame_id = self.camera_frame
        msg.width = width
        msg.height = height
        
        # Simplified camera matrix (assumes reasonable webcam parameters)
        fx = fy = float(width)  # Focal length approximation
        cx = float(width / 2.0)
        cy = float(height / 2.0)
        
        # Camera intrinsic matrix K (3x3 flattened to 9 elements)
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        
        # Distortion parameters (5 elements)
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Projection matrix P (3x4 flattened to 12 elements)
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return msg
    
    def timer_callback(self):
        """Capture and publish camera frame"""
        if self.use_test_pattern:
            # Create test pattern
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # Add text
            cv2.putText(frame, 'No Camera - Test Pattern', (100, self.height//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Failed to capture frame')
                return
        
        # Create timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.camera_frame
        self.image_pub.publish(img_msg)
        
        # Publish camera info
        self.camera_info_msg.header.stamp = timestamp
        self.camera_info_pub.publish(self.camera_info_msg)
    
    def __del__(self):
        """Release camera on shutdown"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
