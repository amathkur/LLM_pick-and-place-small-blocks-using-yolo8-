#!/usr/bin/env python3
"""
Dobot CAN Interface for USB-CAN communication
Supports MG400 robot via CAN bus
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import can
import struct

class DobotCANInterface(Node):
    def __init__(self):
        super().__init__('dobot_can_interface')
        
        # Declare parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('device_id', 1)
        
        can_interface = self.get_parameter('can_interface').value
        bitrate = self.get_parameter('can_bitrate').value
        self.device_id = self.get_parameter('device_id').value
        
        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(
                channel=can_interface,
                bustype='socketcan',
                bitrate=bitrate
            )
            self.get_logger().info(f'✅ CAN interface initialized: {can_interface} @ {bitrate} bps')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize CAN: {e}')
            self.get_logger().info('Run: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up')
            self.bus = None
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_command', self.joint_command_callback, 10)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # State
        self.current_joints = [0.0] * 6
        
        # Timer for reading CAN messages
        if self.bus:
            self.create_timer(0.01, self.read_can_messages)  # 100Hz
            self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        
        self.get_logger().info('🤖 Dobot CAN Interface Ready!')
    
    def joint_command_callback(self, msg):
        """Send joint commands via CAN"""
        if not self.bus:
            self.get_logger().warn('CAN bus not initialized')
            return
        
        if len(msg.position) < 6:
            self.get_logger().warn('Need 6 joint positions')
            return
        
        self.get_logger().info(f'Moving to: {[f"{j:.3f}" for j in msg.position]}')
        
        # Send joint positions via CAN
        # MG400 CAN Protocol: 0x200 + joint_id for position commands
        for i, pos in enumerate(msg.position[:6]):
            self.send_joint_position(i, pos)
    
    def send_joint_position(self, joint_id, position_rad):
        """Send joint position command via CAN"""
        try:
            # CAN message format for MG400:
            # ID: 0x200 + joint_id
            # Data: position in degrees (float32) + velocity (float32)
            position_deg = position_rad * 180.0 / 3.14159
            
            # Pack as 2 floats: position, velocity
            data = struct.pack('<ff', position_deg, 50.0)  # 50 deg/s velocity
            
            msg = can.Message(
                arbitration_id=0x200 + joint_id,
                data=data,
                is_extended_id=False
            )
            
            self.bus.send(msg)
            self.get_logger().debug(f'Sent joint {joint_id}: {position_deg:.2f}°')
            
        except Exception as e:
            self.get_logger().error(f'CAN send error: {e}')
    
    def read_can_messages(self):
        """Read feedback from CAN bus"""
        if not self.bus:
            return
        
        try:
            msg = self.bus.recv(timeout=0.001)
            if msg:
                self.process_can_message(msg)
        except Exception as e:
            if 'Timeout' not in str(e):
                self.get_logger().debug(f'CAN read: {e}')
    
    def process_can_message(self, msg):
        """Process received CAN message"""
        # MG400 feedback format:
        # ID: 0x300 + joint_id for position feedback
        if 0x300 <= msg.arbitration_id < 0x306:
            joint_id = msg.arbitration_id - 0x300
            if len(msg.data) >= 4:
                position_deg = struct.unpack('<f', msg.data[:4])[0]
                position_rad = position_deg * 3.14159 / 180.0
                self.current_joints[joint_id] = position_rad
    
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = self.current_joints
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DobotCANInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.bus:
            node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
