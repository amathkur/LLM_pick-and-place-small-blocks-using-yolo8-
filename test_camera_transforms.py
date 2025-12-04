#!/usr/bin/env python3
"""
Test script to verify camera and suction cup transforms are correct.
Run this after launching the robot system.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import math

class TransformTester(Node):
    def __init__(self):
        super().__init__('transform_tester')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Give TF time to fill
        self.get_logger().info("Waiting for transforms to be available...")
        self.create_timer(2.0, self.test_transforms)
        
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def test_transforms(self):
        """Test all important transforms"""
        
        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("CAMERA AND SUCTION CUP TRANSFORM VERIFICATION")
        self.get_logger().info("="*80)
        
        # Test 1: wrist_pitch to suction_cup
        try:
            trans = self.tf_buffer.lookup_transform('wrist_pitch', 'suction_cup', rclpy.time.Time())
            t = trans.transform.translation
            r = trans.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(r.x, r.y, r.z, r.w)
            
            self.get_logger().info("\n[TEST 1] wrist_pitch → suction_cup")
            self.get_logger().info(f"  Translation: X={t.x:.4f}m, Y={t.y:.4f}m, Z={t.z:.4f}m")
            self.get_logger().info(f"  Expected:    X=0.0350m, Y=0.0000m, Z=0.0000m")
            self.get_logger().info(f"  Rotation:    Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
            self.get_logger().info(f"  Expected:    Roll=0.0°, Pitch=0.0°, Yaw=0.0°")
            
            # Check if correct
            if abs(t.x - 0.035) < 0.001 and abs(t.y) < 0.001 and abs(t.z) < 0.001:
                self.get_logger().info("  ✅ PASS: Suction cup at correct position (35mm forward)")
            else:
                self.get_logger().error("  ❌ FAIL: Suction cup position incorrect!")
                
        except Exception as e:
            self.get_logger().error(f"  ❌ FAIL: Cannot get transform: {e}")
        
        # Test 2: suction_cup to camera_link
        try:
            trans = self.tf_buffer.lookup_transform('suction_cup', 'camera_link', rclpy.time.Time())
            t = trans.transform.translation
            r = trans.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(r.x, r.y, r.z, r.w)
            
            self.get_logger().info("\n[TEST 2] suction_cup → camera_link")
            self.get_logger().info(f"  Translation: X={t.x:.4f}m, Y={t.y:.4f}m, Z={t.z:.4f}m")
            self.get_logger().info(f"  Expected:    X=0.0250m, Y=0.0000m, Z=0.0000m")
            self.get_logger().info(f"  Rotation:    Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
            self.get_logger().info(f"  Expected:    Roll=0.0°, Pitch=-90.0°, Yaw=0.0°")
            
            # Check if correct
            pos_ok = abs(t.x - 0.025) < 0.001 and abs(t.y) < 0.001 and abs(t.z) < 0.001
            rot_ok = abs(pitch + 90.0) < 1.0  # Allow 1 degree tolerance
            
            if pos_ok and rot_ok:
                self.get_logger().info("  ✅ PASS: Camera at correct position (25mm forward, pointing down)")
            else:
                if not pos_ok:
                    self.get_logger().error("  ❌ FAIL: Camera position incorrect!")
                if not rot_ok:
                    self.get_logger().error("  ❌ FAIL: Camera rotation incorrect (should point down)!")
                    
        except Exception as e:
            self.get_logger().error(f"  ❌ FAIL: Cannot get transform: {e}")
        
        # Test 3: wrist_pitch to camera_link (total)
        try:
            trans = self.tf_buffer.lookup_transform('wrist_pitch', 'camera_link', rclpy.time.Time())
            t = trans.transform.translation
            r = trans.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(r.x, r.y, r.z, r.w)
            
            self.get_logger().info("\n[TEST 3] wrist_pitch → camera_link (TOTAL)")
            self.get_logger().info(f"  Translation: X={t.x:.4f}m, Y={t.y:.4f}m, Z={t.z:.4f}m")
            self.get_logger().info(f"  Expected:    X=0.0600m, Y=0.0000m, Z=0.0000m")
            self.get_logger().info(f"  Total distance from wrist_pitch: {math.sqrt(t.x**2 + t.y**2 + t.z**2):.4f}m")
            self.get_logger().info(f"  Expected: 0.0600m (35mm suction + 25mm camera)")
            
            if abs(t.x - 0.060) < 0.001:
                self.get_logger().info("  ✅ PASS: Total offset correct (60mm)")
            else:
                self.get_logger().error("  ❌ FAIL: Total offset incorrect!")
                
        except Exception as e:
            self.get_logger().error(f"  ❌ FAIL: Cannot get transform: {e}")
        
        # Test 4: base_footprint to camera_link (when robot at home)
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', 'camera_link', rclpy.time.Time())
            t = trans.transform.translation
            
            self.get_logger().info("\n[TEST 4] base_footprint → camera_link (AT CURRENT POSE)")
            self.get_logger().info(f"  Camera position in base frame:")
            self.get_logger().info(f"    X={t.x:.4f}m (forward from base)")
            self.get_logger().info(f"    Y={t.y:.4f}m (left/right from base)")
            self.get_logger().info(f"    Z={t.z:.4f}m (height above base)")
            self.get_logger().info(f"  NOTE: These values change as robot moves")
            
        except Exception as e:
            self.get_logger().error(f"  ❌ FAIL: Cannot get transform: {e}")
        
        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("TEST COMPLETE")
        self.get_logger().info("="*80 + "\n")
        
        # Shut down after tests
        self.get_logger().info("Shutting down tester node...")
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    tester = TransformTester()
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
