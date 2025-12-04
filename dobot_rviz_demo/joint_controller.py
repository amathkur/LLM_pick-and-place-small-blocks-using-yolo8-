#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Action server for FollowJointTrajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback
        )

        # Publisher for joint trajectory commands to Gazebo
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Subscriber to joint states for manual control
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Joint names for Dobot Magician (matching MoveIt config)
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5'
        ]

        # Store last joint positions to detect changes
        self.last_positions = [0.0] * len(self.joint_names)
        self.position_threshold = 0.01  # radians

        self.get_logger().info('Joint Controller initialized with action server and joint state subscriber')

    def joint_state_callback(self, msg):
        """Convert joint state updates to trajectory commands for Gazebo"""
        if len(msg.position) != len(self.joint_names):
            return

        # Check if positions have changed significantly
        positions_changed = False
        for i, pos in enumerate(msg.position):
            if abs(pos - self.last_positions[i]) > self.position_threshold:
                positions_changed = True
                break

        if positions_changed:
            # Create trajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = list(msg.position)
            point.velocities = [0.0] * len(msg.position)
            point.accelerations = [0.0] * len(msg.position)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 100000000  # 0.1 seconds

            trajectory.points.append(point)

            # Publish to Gazebo
            self.joint_cmd_pub.publish(trajectory)

            # Update last positions
            self.last_positions = list(msg.position)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing trajectory goal...')

        trajectory = goal_handle.request.trajectory

        # Convert to JointTrajectory message for Gazebo
        gazebo_trajectory = JointTrajectory()
        gazebo_trajectory.joint_names = self.joint_names

        for point in trajectory.points:
            gazebo_point = JointTrajectoryPoint()
            gazebo_point.positions = point.positions
            gazebo_point.velocities = point.velocities if point.velocities else [0.0] * len(point.positions)
            gazebo_point.accelerations = point.accelerations if point.accelerations else [0.0] * len(point.positions)
            gazebo_point.time_from_start = point.time_from_start
            gazebo_trajectory.points.append(gazebo_point)

        # Publish the trajectory
        self.joint_cmd_pub.publish(gazebo_trajectory)

        # Mark as succeeded immediately (simplified)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()