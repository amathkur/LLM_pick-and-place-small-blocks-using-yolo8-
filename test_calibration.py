#!/usr/bin/env python3
"""
Test script to verify camera calibration and coordinate transformation
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')

        # Camera calibration parameters (same as in controller)
        self.CALIB_IMG_PTS_PX_FULL = np.array([
            [152, 29],  # point_1
            [499, 49],  # point_2
            [481, 388],  # point_3
            [142, 376],  # point_4
        ], dtype=np.float32)

        self.CALIB_ROBOT_PTS_MM = np.array([
            [150.00, -100.00],  # point_1
            [350.00, -100.00],  # point_2
            [350.00, 100.00],  # point_3
            [150.00, 100.00],  # point_4
        ], dtype=np.float32)

        # Perform calibration
        self.perform_calibration()

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info('Calibration tester started')

    def perform_calibration(self):
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
                self.get_logger().info('Maze camera calibration successful!')
                self.get_logger().info(f'Homography matrix:\n{self.homography_matrix}')

                # Test some known points
                test_points_px = np.array([
                    [152, 29],    # Should be [0.15, -0.1]
                    [499, 49],    # Should be [0.35, -0.1]
                    [481, 388],   # Should be [0.35, 0.1]
                    [142, 376],   # Should be [0.15, 0.1]
                    [320, 240],   # Center point
                ], dtype=np.float32)

                self.get_logger().info('Testing coordinate transformation:')
                for i, pt in enumerate(test_points_px):
                    robot_coords = self.pixel_to_robot_coords(pt[0], pt[1])
                    if robot_coords is not None:
                        self.get_logger().info(f'Pixel ({pt[0]:.0f}, {pt[1]:.0f}) -> Robot ({robot_coords[0]:.3f}, {robot_coords[1]:.3f}) m')
                    else:
                        self.get_logger().error(f'Failed to transform pixel ({pt[0]}, {pt[1]})')

            else:
                self.get_logger().error('Failed to compute homography matrix')

        except Exception as e:
            self.get_logger().error(f'Error during calibration: {e}')

    def pixel_to_robot_coords(self, u, v):
        """Convert pixel coordinates to robot coordinates using homography"""
        try:
            if self.homography_matrix is None:
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

    def image_callback(self, msg):
        """Process camera images and show calibration points"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Draw calibration points on the image
            for i, pt in enumerate(self.CALIB_IMG_PTS_PX_FULL):
                cv2.circle(cv_image, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
                cv2.putText(cv_image, f'P{i+1}', (int(pt[0])+10, int(pt[1])-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the image
            cv2.imshow('Calibration Test', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()