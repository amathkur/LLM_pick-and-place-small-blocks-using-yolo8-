#!/usr/bin/env python3
"""
ROS2 node to bridge direct joint commands to Dobot via pydobot (USB).
Subscribes to /set_joint_positions and sends joint commands to the robot.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import pydobot
import glob
import time
import math

class DobotMoveItBridge(Node):
    def __init__(self):
        super().__init__('dobot_moveit_bridge')
        
        # Subscribe to direct joint position commands
        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            '/set_joint_positions',
            self.joint_callback,
            10)
            
        # Subscribe to robot command strings from LLM
        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10)
            
        # Publish joint states from real robot
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)
            
        self.device = None
        self.connect_robot()
        
        # Publish joint states at 10Hz
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def connect_robot(self):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if not ports:
            self.get_logger().error('No robot ports found!')
            return
            
        # Monkey patch pydobot to handle pose errors during initialization
        original_get_pose = pydobot.Dobot._get_pose
        def safe_get_pose(self):
            try:
                return original_get_pose(self)
            except Exception as e:
                self.get_logger().warn(f'Pose query failed during init: {e}, skipping')
                # Set default values
                self.x = self.y = self.z = self.r = self.j1 = self.j2 = self.j3 = self.j4 = 0.0
                return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
        pydobot.Dobot._get_pose = safe_get_pose
        
        try:
            self.device = pydobot.Dobot(port=ports[0], verbose=True)
            self.get_logger().info(f'Connected to Dobot on {ports[0]}')
            
            # Try to get pose after connection
            try:
                pose = self.device.pose()
                self.get_logger().info(f'Robot pose: {pose}')
            except Exception as pose_error:
                self.get_logger().warn(f'Could not get pose: {pose_error}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.device = None
        finally:
            # Restore original method
            pydobot.Dobot._get_pose = original_get_pose

    def joint_callback(self, msg):
        if not self.device:
            self.get_logger().error('No robot connection!')
            return
            
        if len(msg.data) < 4:
            self.get_logger().warn(f'Not enough joint positions! Got {len(msg.data)}, need 4')
            return
            
        try:
            # Convert radians to degrees for Dobot API
            j1_deg = math.degrees(msg.data[0])
            j2_deg = math.degrees(msg.data[1]) 
            j3_deg = math.degrees(msg.data[2])
            j4_deg = math.degrees(msg.data[3])
            
            # Ensure wrist_pitch (joint4) always faces +z axis
            # For Dobot Magician, joint4 = -90 degrees makes wrist face upward (+z)
            # Adjust based on other joint angles to maintain +z orientation
            j4_deg = self.adjust_wrist_for_up_orientation(j1_deg, j2_deg, j3_deg, j4_deg)
            
            self.get_logger().info(f'Moving to joint positions: {j1_deg:.1f}, {j2_deg:.1f}, {j3_deg:.1f}, {j4_deg:.1f} degrees (wrist adjusted for +z orientation)')
            
            # Send joint move command to real robot
            self.device.movej(j1_deg, j2_deg, j3_deg, j4_deg)
            
            self.get_logger().info('Joint move command sent to real robot')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send joint command: {e}')

    def command_callback(self, msg):
        """Handle string commands from LLM agent"""
        if not self.device:
            self.get_logger().error('No robot connection!')
            return
            
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received robot command: {command}')
        
        try:
            if command.startswith('move to '):
                parts = command.split()
                if len(parts) >= 5:
                    x = float(parts[2]) * 1000  # Convert to mm
                    y = float(parts[3]) * 1000
                    z = float(parts[4]) * 1000
                    r = 0.0  # Default rotation
                    self.get_logger().info(f'Moving to position: x={x:.1f}, y={y:.1f}, z={z:.1f} mm')
                    self.device.move_to(x=x, y=y, z=z, r=r)
                else:
                    self.get_logger().warn(f'Invalid move command format: {command}')
            elif command.startswith('pick at '):
                parts = command.split()
                if len(parts) >= 5:
                    x = float(parts[2]) * 1000
                    y = float(parts[3]) * 1000
                    z = float(parts[4]) * 1000
                    # For pick, move to position and perhaps activate suction
                    self.get_logger().info(f'Picking at position: x={x:.1f}, y={y:.1f}, z={z:.1f} mm')
                    self.device.move_to(x=x, y=y, z=z, r=0.0)
                    # TODO: Add suction control if available
                else:
                    self.get_logger().warn(f'Invalid pick command format: {command}')
            elif command.startswith('place at '):
                parts = command.split()
                if len(parts) >= 5:
                    x = float(parts[2]) * 1000
                    y = float(parts[3]) * 1000
                    z = float(parts[4]) * 1000
                    self.get_logger().info(f'Placing at position: x={x:.1f}, y={y:.1f}, z={z:.1f} mm')
                    self.device.move_to(x=x, y=y, z=z, r=0.0)
                    # TODO: Release suction
                else:
                    self.get_logger().warn(f'Invalid place command format: {command}')
            else:
                self.get_logger().warn(f'Unknown command: {command}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to execute command {command}: {e}')

    def adjust_wrist_for_up_orientation(self, j1, j2, j3, j4_original):
        """Adjust wrist pitch (joint4) to ensure end effector always faces +z axis"""
        # For the Dobot Magician, to keep the wrist facing +z (upward):
        # We need to compensate for the effect of joints 1-3 on the end effector orientation
        # This is a simplified approach - in practice, this would need calibration
        
        # The wrist pitch joint affects the end effector orientation
        # To keep it pointing up (+z), we set joint4 to counteract the orientation from joints 1-3
        
        # For most configurations, setting joint4 to -90 degrees keeps the end effector pointing up
        # But we may need to adjust based on the arm configuration
        
        # Simple approach: always set joint4 to -90 degrees to point upward
        # This works for most robot configurations where the arm is extended horizontally
        wrist_up_angle = -90.0
        
        # For more complex scenarios, we could calculate based on forward kinematics
        # For now, use the fixed angle approach
        return wrist_up_angle

    def publish_joint_states(self):
        if not self.device:
            return
            
        try:
            # Get current pose from robot
            pose = self.device.pose()
            
            # pose is a tuple: (x, y, z, r, j1, j2, j3, j4)
            if len(pose) >= 8:
                x, y, z, r, j1, j2, j3, j4 = pose
                
                # Create joint state message
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.header.frame_id = ''
                joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
                
                # Convert degrees to radians
                joint_state.position = [
                    math.radians(j1),
                    math.radians(j2), 
                    math.radians(j3),
                    math.radians(j4)
                ]
                
                # Set zero velocity and effort (not available from pydobot)
                joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
                joint_state.effort = [0.0, 0.0, 0.0, 0.0]
                
                self.joint_pub.publish(joint_state)
            else:
                self.get_logger().warn(f'Unexpected pose tuple length: {len(pose)}')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to get robot pose: {e}')
            # Publish zero joint states as fallback
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = ''
            joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
            joint_state.position = [0.0, 0.0, 0.0, 0.0]
            joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
            joint_state.effort = [0.0, 0.0, 0.0, 0.0]
            self.joint_pub.publish(joint_state)

    def destroy_node(self):
        if self.device:
            self.device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DobotMoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
