#!/usr/bin/env python3
"""
Real Dobot Robot Serial Interface
Connects to robot via /dev/ttyACM0 and bridges to ROS2
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import struct
import time
import threading

class DobotSerialInterface(Node):
    def __init__(self):
        super().__init__('dobot_serial_interface')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'✅ Connected to Dobot on {port}')
            self.connected = True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect: {e}')
            self.connected = False
            return
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            JointState,
            '/joint_command',
            self.joint_command_callback,
            10
        )
        
        # State
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        
        # Start read thread
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
        
        # Status timer
        self.timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('🤖 Dobot Serial Interface Ready')
    
    def joint_command_callback(self, msg):
        """Send joint commands to robot"""
        if not self.connected:
            return
        
        try:
            # Send move command (simplified protocol)
            # Format: CMD + 4 floats (joint angles in degrees)
            cmd = b'M'  # Move command
            data = struct.pack('4f', *msg.position[:4])
            self.serial_port.write(cmd + data)
            self.get_logger().info(f'Sent: {list(msg.position[:4])}')
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
    
    def read_loop(self):
        """Read joint states from robot"""
        while rclpy.ok() and self.connected:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    
                    # Parse joint positions
                    if line.startswith('J:'):
                        parts = line[2:].split(',')
                        if len(parts) >= 4:
                            self.current_joints = [float(x) for x in parts[:4]]
            except Exception as e:
                pass
            
            time.sleep(0.01)
    
    def publish_status(self):
        """Publish current joint states"""
        if not self.connected:
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = self.current_joints
        self.joint_pub.publish(msg)
        
        status = String()
        status.data = f'Connected: joints={self.current_joints}'
        self.status_pub.publish(status)

def main():
    rclpy.init()
    node = DobotSerialInterface()
    
    if node.connected:
        rclpy.spin(node)
    else:
        node.get_logger().error('Not connected - exiting')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
