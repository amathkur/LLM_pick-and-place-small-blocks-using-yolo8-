#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        
        self.publisher = self.create_publisher(Float64MultiArray, '/set_joint_positions', 10)
        
        # Current joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('Joint Controller Started!')
        self.get_logger().info('Commands:')
        self.get_logger().info('  set <j1> <j2> <j3> <j4>  - Set joint positions (radians)')
        self.get_logger().info('  add <j1> <j2> <j3> <j4>  - Add to current positions')
        self.get_logger().info('  home                     - Go to home position')
        self.get_logger().info('  zero                     - Go to zero position')
        self.get_logger().info('  help                     - Show this help')
        self.get_logger().info('  quit                     - Exit')
        self.get_logger().info('')
        self.get_logger().info('Example: set 0.5 0.2 -0.3 0.1')
        
        self.run_interactive()

    def set_joints(self, positions):
        """Set joint positions"""
        if len(positions) != 4:
            self.get_logger().error('Need exactly 4 joint positions')
            return
        
        self.joint_positions = positions
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Set joints to: {[f"{p:.3f}" for p in self.joint_positions]}')

    def add_joints(self, deltas):
        """Add to current joint positions"""
        if len(deltas) != 4:
            self.get_logger().error('Need exactly 4 joint deltas')
            return
        
        self.joint_positions = [p + d for p, d in zip(self.joint_positions, deltas)]
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Added to joints: {[f"{d:.3f}" for d in deltas]}')
        self.get_logger().info(f'New positions: {[f"{p:.3f}" for p in self.joint_positions]}')

    def go_home(self):
        """Go to home position"""
        self.set_joints([0.0, 0.0, 0.0, 0.0])

    def go_zero(self):
        """Go to zero position (same as home)"""
        self.go_home()

    def run_interactive(self):
        """Run interactive command loop"""
        while rclpy.ok():
            try:
                cmd = input('joint_control> ').strip().lower()
                
                if not cmd:
                    continue
                    
                parts = cmd.split()
                command = parts[0]
                
                if command == 'quit' or command == 'exit':
                    break
                elif command == 'help':
                    self.show_help()
                elif command == 'home':
                    self.go_home()
                elif command == 'zero':
                    self.go_zero()
                elif command == 'set':
                    if len(parts) != 5:
                        print('Usage: set <j1> <j2> <j3> <j4>')
                        continue
                    try:
                        positions = [float(p) for p in parts[1:]]
                        self.set_joints(positions)
                    except ValueError:
                        print('Invalid numbers')
                elif command == 'add':
                    if len(parts) != 5:
                        print('Usage: add <j1> <j2> <j3> <j4>')
                        continue
                    try:
                        deltas = [float(p) for p in parts[1:]]
                        self.add_joints(deltas)
                    except ValueError:
                        print('Invalid numbers')
                else:
                    print(f'Unknown command: {command}')
                    print('Type "help" for available commands')
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        self.get_logger().info('Joint controller exiting...')

    def show_help(self):
        """Show help message"""
        print('\nJoint Controller Commands:')
        print('  set <j1> <j2> <j3> <j4>  - Set joint positions (radians)')
        print('  add <j1> <j2> <j3> <j4>  - Add to current positions')
        print('  home                     - Go to home position (0,0,0,0)')
        print('  zero                     - Go to zero position (same as home)')
        print('  help                     - Show this help')
        print('  quit                     - Exit')
        print('')
        print('Joint ranges (approximate):')
        print('  J1 (base): -3.14 to 3.14 radians')
        print('  J2 (shoulder): -1.57 to 1.57 radians')
        print('  J3 (elbow): -1.57 to 1.57 radians')
        print('  J4 (wrist): -3.14 to 3.14 radians')
        print('')
        print('Example: set 0.5 0.2 -0.3 0.1')

def main():
    rclpy.init()
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()