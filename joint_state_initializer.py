#!/usr/bin/env python3
"""
Simple Joint State Initializer - Publishes initial joint states without interfering with GUI
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStateInitializer(Node):
    def __init__(self):
        super().__init__('joint_state_initializer')
        
        # Create publisher
        self.publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        self.get_logger().info('Joint State Initializer Started')
        
        # Publish initial joint states once
        self.publish_initial_states()
        
        # Create a timer to occasionally republish (low frequency)
        self.timer = self.create_timer(10.0, self.publish_initial_states)  # Every 10 seconds
    
    def publish_initial_states(self):
        """Publish initial joint states at zero position"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = [0.0, 0.0, 0.0, 0.0]  # Start at zero
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        
        self.publisher.publish(msg)
        self.get_logger().info('Published initial joint states')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()