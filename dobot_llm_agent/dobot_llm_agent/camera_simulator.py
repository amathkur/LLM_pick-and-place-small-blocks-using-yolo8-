#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create a simple image with some colored blocks
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw some blocks
        cv2.rectangle(image, (100, 100), (150, 150), (255, 0, 0), -1)  # Blue block
        cv2.rectangle(image, (200, 200), (250, 250), (0, 255, 0), -1)  # Green block
        cv2.rectangle(image, (300, 300), (350, 350), (0, 0, 255), -1)  # Red block
        
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info('Published simulated camera image')

def main(args=None):
    rclpy.init(args=args)
    camera_simulator = CameraSimulator()
    rclpy.spin(camera_simulator)
    camera_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()