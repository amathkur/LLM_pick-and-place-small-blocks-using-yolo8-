#!/usr/bin/env python3
"""
View what the ROS2 camera is publishing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('📹 Camera Viewer Started - Viewing /camera/image_raw')
        self.get_logger().info('   Press Q in the window to quit')
        self.frame_count = 0
        
    def image_callback(self, msg):
        self.frame_count += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add frame info
            h, w = cv_image.shape[:2]
            text = f"Frame: {self.frame_count} | Size: {w}x{h}"
            cv2.putText(cv_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('ROS2 Camera Feed', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    viewer = CameraViewer()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(viewer, timeout_sec=0.1)
            if cv2.getWindowProperty('ROS2 Camera Feed', cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
