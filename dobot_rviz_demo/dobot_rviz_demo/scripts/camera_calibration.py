#!/usr/bin/env python3

"""
Camera-Robot Calibration Script
Helps calibrate the transformation between camera pixel coordinates and robot world coordinates.
"""

import cv2
import numpy as np
import json
import os

def calibrate_camera_robot():
    """Interactive calibration between camera and robot coordinates"""

    # Camera parameters (adjust these based on your setup)
    camera_matrix = np.array([
        [600, 0, 320],   # fx, 0, cx
        [0, 600, 240],   # 0, fy, cy
        [0, 0, 1]        # 0, 0, 1
    ])

    # Camera position relative to robot base (meters)
    camera_position = np.array([0.0, 0.0, 0.5])  # x, y, z
    camera_rotation = np.array([0.0, 0.0, 0.0])  # rx, ry, rz (radians)

    # Known calibration points (pixel -> world coordinates)
    calibration_points = [
        # Format: (pixel_x, pixel_y, world_x, world_y, world_z)
        (320, 240, 0.0, 0.0, 0.2),    # Center point
        (400, 240, 0.1, 0.0, 0.2),    # Right point
        (240, 240, -0.1, 0.0, 0.2),   # Left point
        (320, 160, 0.0, 0.1, 0.2),    # Top point
        (320, 320, 0.0, -0.1, 0.2),   # Bottom point
    ]

    calibration_data = {
        'camera_matrix': camera_matrix.tolist(),
        'camera_position': camera_position.tolist(),
        'camera_rotation': camera_rotation.tolist(),
        'calibration_points': calibration_points,
        'notes': 'Default calibration - adjust for your setup'
    }

    # Save calibration data
    calibration_file = '/home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/camera_calibration.json'
    with open(calibration_file, 'w') as f:
        json.dump(calibration_data, f, indent=2)

    print(f"Calibration data saved to: {calibration_file}")
    print("\nCalibration points:")
    for i, (px, py, wx, wy, wz) in enumerate(calibration_points):
        print(f"Point {i+1}: Pixel({px},{py}) -> World({wx:.3f},{wy:.3f},{wz:.3f})")

    return calibration_data

def test_camera_calibration():
    """Test camera calibration with live feed"""

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    print("Testing camera calibration...")
    print("Press 'c' to capture calibration point")
    print("Press 'q' to quit")

    calibration_points = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Draw crosshair at center
        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)

        # Display captured points
        for i, (x, y) in enumerate(calibration_points):
            cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"{i+1}", (x+10, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow('Camera Calibration', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            # Capture point at center
            calibration_points.append((w//2, h//2))
            print(f"Captured point {len(calibration_points)} at ({w//2}, {h//2})")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    print(f"Captured {len(calibration_points)} calibration points")

if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        test_camera_calibration()
    else:
        calibrate_camera_robot()