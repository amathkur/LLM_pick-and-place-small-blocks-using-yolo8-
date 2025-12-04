#!/usr/bin/env python3
"""
Fake Joint State Publisher - Publishes joint states for MoveIt planning
Subscribes to planned trajectories and updates joint positions
"""

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import time


class FakeJointPublisher(Node):
    def __init__(self):
        super().__init__('fake_joint_publisher')
        
        # Joint positions (start at zero position for visibility)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # Start at zero position
        self.target_positions = [0.0, 0.0, 0.0, 0.0]  # Target starts at zero
        
        # Smooth interpolation
        self.interpolation_speed = 0.05  # Much slower interpolation for smoother movement
        
        # Create publisher with larger queue
        self.publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        # Subscribe to planned trajectories from MoveIt display
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/move_group/display_planned_path',
            self.trajectory_callback,
            10
        )
        
        # Subscriber for direct joint position commands (our custom interface)
        self.joint_command_sub = self.create_subscription(
            Float64MultiArray,
            '/set_joint_positions',
            self.joint_command_callback,
            10
        )
        
        # Create action server for trajectory execution
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'magician_arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback
        )
        
        # Create action server for trajectory execution
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'magician_arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback
        )
        
        self.get_logger().info('Action server created for trajectory execution')
        self.get_logger().info('='*60)
        self.get_logger().info('📡 Fake Joint State Publisher Started')
        self.get_logger().info(f'Publishing to: /joint_states at 50Hz')
        self.get_logger().info(f'Subscribing to: /move_group/display_planned_path')
        self.get_logger().info(f'Joint names: joint1, joint2, joint3, joint4')
        self.get_logger().info(f'Joint positions: {self.joint_positions}')
        self.get_logger().info('='*60)
        
        # Create timer to publish joint states at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)
    
    def execute_trajectory_callback(self, goal_handle):
        """Execute trajectory by updating target positions"""
        self.get_logger().info('Received trajectory goal')
        self.get_logger().info('Executing trajectory...')
        
        trajectory = goal_handle.request.trajectory
        if trajectory.points:
            # Get the final point
            final_point = trajectory.points[-1]
            self.target_positions = list(final_point.positions)
            # Ensure we have positions for all joints
            while len(self.target_positions) < len(self.joint_positions):
                self.target_positions.append(0.0)
            self.target_positions = self.target_positions[:len(self.joint_positions)]
            self.get_logger().info(f'🎯 Executing to: {[f"{p:.2f}" for p in self.target_positions]}')
        
        # Accept the goal
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result
    
    def trajectory_callback(self, msg):
        """Update target positions when new trajectory is received"""
        if msg.points:
            # Get the last point in the trajectory (final position)
            last_point = msg.points[-1]
            self.target_positions = list(last_point.positions)
            # Ensure we have positions for all joints
            while len(self.target_positions) < len(self.joint_positions):
                self.target_positions.append(0.0)
            self.target_positions = self.target_positions[:len(self.joint_positions)]
            self.get_logger().info(f'📬 New target: {[f"{p:.2f}" for p in self.target_positions]}')
    
    def joint_command_callback(self, msg):
        """Directly set joint positions"""
        if len(msg.data) >= 4:
            self.target_positions = list(msg.data[:4])
            self.get_logger().info(f'🔧 Direct joint command: {[f"{p:.2f}" for p in self.target_positions]}')
        else:
            self.get_logger().warn('Invalid joint command: need at least 4 values')
    
    def publish_joint_states(self):
        # Interpolate toward target for smooth motion (but slowly)
        for i in range(len(self.joint_positions)):
            if i < len(self.target_positions):
                diff = self.target_positions[i] - self.joint_positions[i]
                if abs(diff) > 0.001:  # Only update if significant difference
                    self.joint_positions[i] += diff * self.interpolation_speed
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_positions)
        msg.effort = [0.0] * len(self.joint_positions)
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointPublisher()
    
    # Log every 5 seconds
    last_log_time = time.time()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Periodic status log
            current_time = time.time()
            if current_time - last_log_time > 5.0:
                node.get_logger().info(f'📊 Current: [{", ".join([f"{p:.2f}" for p in node.joint_positions])}]')
                last_log_time = current_time
                
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fake joint publisher...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
