#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateToTrajectory(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')

        # Subscribe to joint_states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Publisher for joint trajectory
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        self.get_logger().info('Joint State to Trajectory node initialized')

    def joint_state_callback(self, msg):
        # Create joint trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = []
        
        # Map joint states to trajectory positions
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                point.positions.append(msg.position[index])
            else:
                point.positions.append(0.0)  # Default position
        
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds
        
        trajectory.points = [point]
        
        self.trajectory_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()