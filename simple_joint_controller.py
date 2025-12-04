#!/usr/bin/env python3
"""
Simple Joint Controller - Tests joint movements directly
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class SimpleJointController(Node):
    def __init__(self):
        super().__init__('simple_joint_controller')
        
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for MoveGroup...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected!')
        
    def send_joint_goal(self, joint_values):
        """Send joint space goal"""
        goal_msg = MoveGroup.Goal()
        
        goal_msg.request.group_name = 'magician_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Use current state as start
        goal_msg.request.start_state.is_diff = True
        
        # Create joint constraints
        constraints = Constraints()
        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        for i, (name, value) in enumerate(zip(joint_names, joint_values)):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Sending joint goal: {joint_values}')
        
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
            
        self.get_logger().info('Goal accepted, planning...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('✅ SUCCESS!')
            return True
        else:
            self.get_logger().error(f'❌ FAILED: error code {result.error_code.val}')
            return False


def main():
    rclpy.init()
    controller = SimpleJointController()
    
    print('\n🤖 TESTING JOINT MOVEMENTS')
    print('='*50)
    
    # Test 1: Small movement from home (all zeros)
    print('\n1️⃣ Test: Move joint2 (shoulder) slightly')
    controller.send_joint_goal([0.0, 0.2, 0.0, 0.0])
    
    input('\nPress Enter for next test...')
    
    # Test 2: Move joint3 (elbow)
    print('\n2️⃣ Test: Move joint3 (elbow)')
    controller.send_joint_goal([0.0, 0.2, 0.3, 0.0])
    
    input('\nPress Enter for next test...')
    
    # Test 3: Return to home
    print('\n3️⃣ Test: Return to home')
    controller.send_joint_goal([0.0, 0.0, 0.0, 0.0])
    
    print('\n✅ All tests complete!')
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
