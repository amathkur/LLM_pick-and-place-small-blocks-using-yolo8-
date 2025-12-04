#!/usr/bin/env python3
"""
Calibration Verification Script for DoBot Camera-Robot Alignment

This script helps verify that the camera calibration matches the real robot workspace.
Move the real robot to the corner positions and check if the camera view aligns with RViz.

Calibration Points:
- BL (Bottom-Left):  X=165mm, Y=-100mm  → Pixel: [152, 29]
- BR (Bottom-Right): X=365mm, Y=-100mm  → Pixel: [499, 49]
- TR (Top-Right):    X=365mm, Y=100mm   → Pixel: [481, 388]
- TL (Top-Left):     X=165mm, Y=100mm   → Pixel: [142, 376]

Usage:
1. Run this script
2. Move real robot to each corner position
3. Check if camera view matches expected pixel coordinates
4. Verify RViz shows robot at correct position relative to camera workspace grid
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import time
import math

class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__('calibration_verifier')

        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/set_joint_positions',
            10
        )

        # Calibration points in robot coordinates (mm)
        self.calibration_points = {
            'BL': {'x': 165.0, 'y': -100.0, 'name': 'Bottom-Left'},
            'BR': {'x': 365.0, 'y': -100.0, 'name': 'Bottom-Right'},
            'TR': {'x': 365.0, 'y': 100.0, 'name': 'Top-Right'},
            'TL': {'x': 165.0, 'y': 100.0, 'name': 'Top-Left'}
        }

        # Corresponding pixel coordinates
        self.pixel_coords = {
            'BL': [152, 29],
            'BR': [499, 49],
            'TR': [481, 388],
            'TL': [142, 376]
        }

        # DoBot link lengths (approximate)
        self.l1 = 0.135  # Base to joint2
        self.l2 = 0.147  # Joint2 to joint3
        self.l3 = 0.06   # Joint3 to end effector

        self.get_logger().info('Calibration Verifier initialized')
        self.print_instructions()

    def print_instructions(self):
        """Print calibration verification instructions"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('DoBot Camera-Robot Calibration Verification')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('Move the REAL robot to each corner position and verify:')
        self.get_logger().info('1. Camera view shows object at expected pixel coordinates')
        self.get_logger().info('2. RViz shows robot aligned with camera workspace grid')
        self.get_logger().info('')
        self.get_logger().info('Calibration Points:')
        for key, point in self.calibration_points.items():
            pixels = self.pixel_coords[key]
            self.get_logger().info(f'  {key} ({point["name"]}): Robot({point["x"]:4.0f}mm, {point["y"]:4.0f}mm) → Camera({pixels[0]:3d}, {pixels[1]:3d})px')
        self.get_logger().info('')
        self.get_logger().info('Commands:')
        self.get_logger().info('  move BL  - Move to Bottom-Left position')
        self.get_logger().info('  move BR  - Move to Bottom-Right position')
        self.get_logger().info('  move TR  - Move to Top-Right position')
        self.get_logger().info('  move TL  - Move to Top-Left position')
        self.get_logger().info('  home     - Move to home position')
        self.get_logger().info('  quit     - Exit')
        self.get_logger().info('=' * 60)

    def calculate_ik(self, x_mm, y_mm, z_mm=30.0):
        """Calculate inverse kinematics for DoBot"""
        # Convert mm to meters
        target_x = x_mm / 1000.0
        target_y = y_mm / 1000.0
        target_z = z_mm / 1000.0

        # Transform for rotated base_link (90° around X)
        std_x = target_x
        std_y = target_z
        std_z = -target_y

        # Calculate distance in XY plane from base
        r = math.sqrt(std_x**2 + std_y**2)

        # Joint1 (base rotation) - atan2 gives angle from positive X axis
        joint1 = math.atan2(std_y, std_x)

        # For joint2 and joint3, use geometric IK
        # Effective distance from base to target in XY plane (subtract end effector length)
        r_eff = max(0.01, r - self.l3)  # Avoid negative values

        # Effective height from base (subtract shoulder height)
        z_eff = std_z - self.l1

        # Distance from joint2 to target point
        d = math.sqrt(r_eff**2 + z_eff**2)

        # Joint3 (elbow) - use cosine rule
        cos_joint3 = (d**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_joint3 = max(-1.0, min(1.0, cos_joint3))  # Clamp to valid range
        joint3 = math.acos(cos_joint3)

        # Joint2 (shoulder) - elevation angle
        alpha = math.atan2(z_eff, r_eff)
        beta = math.asin((self.l3 * math.sin(joint3)) / d)
        joint2 = alpha - beta

        # Joint4 (wrist) - set to 90° for suction cup down
        joint4 = 1.57

        # Convert to joint positions array
        joint_positions = [joint1, joint2, joint3, joint4]

        # Ensure joints stay within limits
        joint_limits = [(-1.57, 1.57), (-0.5, 1.0), (-1.0, 1.0), (-1.0, 1.0)]  # Approximate limits
        for i in range(4):
            min_val, max_val = joint_limits[i]
            joint_positions[i] = max(min_val, min(max_val, joint_positions[i]))

        return joint_positions

    def move_to_position(self, position_key):
        """Move robot to specified calibration position"""
        if position_key not in self.calibration_points:
            self.get_logger().error(f'Unknown position: {position_key}')
            return

        point = self.calibration_points[position_key]
        pixels = self.pixel_coords[position_key]

        self.get_logger().info(f'Moving to {position_key} ({point["name"]}): Robot({point["x"]:4.0f}mm, {point["y"]:4.0f}mm) → Camera({pixels[0]:3d}, {pixels[1]:3d})px')

        # Calculate joint positions
        joint_positions = self.calculate_ik(point['x'], point['y'])

        self.get_logger().info(f'Joint positions: {[f"{p:.3f}" for p in joint_positions]}')

        # Publish joint command
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_cmd_pub.publish(msg)

        self.get_logger().info(f'Command sent. Check camera view at pixel ({pixels[0]}, {pixels[1]})')

    def move_home(self):
        """Move to home position"""
        self.get_logger().info('Moving to home position')
        joint_positions = [0.0, 0.0, 0.0, 0.0]

        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    verifier = CalibrationVerifier()

    try:
        while rclpy.ok():
            cmd = input("Enter command (move BL/BR/TR/TL, home, quit): ").strip().lower()

            if cmd == 'quit':
                break
            elif cmd == 'home':
                verifier.move_home()
            elif cmd.startswith('move '):
                position = cmd.split()[1].upper()
                verifier.move_to_position(position)
            else:
                print("Invalid command. Use: move BL/BR/TR/TL, home, quit")

            time.sleep(0.1)  # Small delay

    except KeyboardInterrupt:
        pass
    finally:
        verifier.get_logger().info('Shutting down calibration verifier')
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()