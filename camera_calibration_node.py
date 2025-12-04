#!/usr/bin/env python3
"""
Camera Calibration Node using Charuco Board
Detects Charuco corners and performs camera calibration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')

        # Charuco board parameters
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.board = cv2.aruco.CharucoBoard((5, 7), 0.04, 0.02, self.dictionary)  # 5x7 board, square size 4cm, marker size 2cm
        self.detector_params = cv2.aruco.DetectorParameters()

        # Calibration data storage
        self.all_corners = []
        self.all_ids = []
        self.all_img_points = []
        self.all_obj_points = []

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibrated = False

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Calibration parameters
        self.min_frames = 10  # Minimum frames for calibration
        self.frame_count = 0

        self.get_logger().info('Camera Calibration Node started')
        self.get_logger().info('Charuco board: 5x7, square size: 4cm, marker size: 2cm')
        self.get_logger().info('Collecting calibration data...')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect Charuco corners
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.detector_params)

            if ids is not None and len(ids) > 0:
                # Interpolate Charuco corners
                ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, cv_image, self.board
                )

                if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) >= 4:
                    # Draw detected corners
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                    cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)

                    # Add to calibration data
                    self.all_corners.append(charuco_corners)
                    self.all_ids.append(charuco_ids)
                    self.all_img_points.append(charuco_corners)
                    self.all_obj_points.append(self.board.getChessboardCorners()[charuco_ids.flatten()])

                    self.frame_count += 1
                    self.get_logger().info(f'Frame {self.frame_count}: Detected {len(charuco_ids)} Charuco corners')

                    # Check if we have enough data for calibration
                    if self.frame_count >= self.min_frames and not self.calibrated:
                        self.perform_calibration()

            # Display image (optional, for debugging)
            cv2.imshow('Camera Calibration', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def perform_calibration(self):
        try:
            self.get_logger().info('Performing camera calibration...')

            # Prepare data for calibration
            obj_points = []
            img_points = []

            for i in range(len(self.all_obj_points)):
                obj_points.append(self.all_obj_points[i])
                img_points.append(self.all_img_points[i])

            # Get image size from last frame
            h, w = cv_image.shape[:2]

            # Perform calibration
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, (w, h), None, None
            )

            if ret:
                self.camera_matrix = camera_matrix
                self.dist_coeffs = dist_coeffs
                self.calibrated = True

                self.get_logger().info('Camera calibration successful!')
                self.get_logger().info(f'Camera Matrix:\n{camera_matrix}')
                self.get_logger().info(f'Distortion Coefficients: {dist_coeffs.flatten()}')

                # Save calibration data
                self.save_calibration()

            else:
                self.get_logger().error('Camera calibration failed')

        except Exception as e:
            self.get_logger().error(f'Error during calibration: {str(e)}')

    def save_calibration(self):
        """Save calibration parameters to YAML file"""
        try:
            calibration_data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'distortion_coefficients': self.dist_coeffs.flatten().tolist(),
                'calibration_date': str(self.get_clock().now())
            }

            filename = os.path.expanduser('~/dobot_rviz_ws/camera_calibration.yaml')
            with open(filename, 'w') as f:
                yaml.dump(calibration_data, f)

            self.get_logger().info(f'Calibration saved to {filename}')

        except Exception as e:
            self.get_logger().error(f'Error saving calibration: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Camera Calibration Node')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()