#!/usr/bin/env python3
"""
MoveIt Interface Demo Script
This script demonstrates how to use MoveIt's Python interface to:
1. Plan trajectories to goal positions
2. Execute trajectories (visualization only without real controllers)
3. Get current robot state
4. Plan to named targets
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
import sys

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
except ImportError:
    print("MoveItPy not available. Using moveit_commander instead...")
    import moveit_commander

class MoveItInterfaceDemo(Node):
    def __init__(self):
        super().__init__('moveit_interface_demo')
        self.get_logger().info("Initializing MoveIt Interface Demo...")
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Create robot commander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Create move group for the robot arm
        self.group_name = "magician_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # Display trajectory publisher for RViz
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory,
            '/move_group/display_planned_path',
            10
        )
        
        self.get_logger().info(f"Planning group: {self.group_name}")
        self.get_logger().info(f"Reference frame: {self.move_group.get_planning_frame()}")
        self.get_logger().info(f"End effector link: {self.move_group.get_end_effector_link()}")
        self.get_logger().info(f"Available Planning Groups: {self.robot.get_group_names()}")
        
    def print_robot_state(self):
        """Print current robot state information"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("CURRENT ROBOT STATE")
        self.get_logger().info("=" * 60)
        
        current_joints = self.move_group.get_current_joint_values()
        self.get_logger().info(f"Current Joint Values: {[f'{j:.3f}' for j in current_joints]}")
        
        current_pose = self.move_group.get_current_pose().pose
        self.get_logger().info(f"Current End Effector Position:")
        self.get_logger().info(f"  x: {current_pose.position.x:.3f}")
        self.get_logger().info(f"  y: {current_pose.position.y:.3f}")
        self.get_logger().info(f"  z: {current_pose.position.z:.3f}")
        self.get_logger().info("=" * 60)
    
    def plan_to_joint_goal(self, joint_values):
        """Plan to a specific joint configuration"""
        self.get_logger().info(f"Planning to joint goal: {[f'{j:.3f}' for j in joint_values]}")
        
        self.move_group.set_joint_value_target(joint_values)
        
        success, plan, planning_time, error_code = self.move_group.plan()
        
        if success:
            self.get_logger().info(f"✓ Planning succeeded! Time: {planning_time:.2f}s")
            return plan
        else:
            self.get_logger().error(f"✗ Planning failed! Error code: {error_code}")
            return None
    
    def plan_to_pose_goal(self, pose):
        """Plan to a specific end effector pose"""
        self.get_logger().info("Planning to pose goal...")
        self.get_logger().info(f"  Target position: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        self.move_group.set_pose_target(pose)
        
        success, plan, planning_time, error_code = self.move_group.plan()
        
        if success:
            self.get_logger().info(f"✓ Planning succeeded! Time: {planning_time:.2f}s")
            return plan
        else:
            self.get_logger().error(f"✗ Planning failed! Error code: {error_code}")
            return None
    
    def execute_plan(self, plan):
        """
        Execute a planned trajectory
        Note: Without real controllers, this will only update the visualization
        """
        self.get_logger().info("Executing plan (visualization only)...")
        
        # Display the trajectory in RViz
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        
        # Attempt execution (will fail gracefully without controllers)
        success = self.move_group.execute(plan, wait=True)
        
        if success:
            self.get_logger().info("✓ Execution complete (visualization)")
        else:
            self.get_logger().warn("⚠ Execution failed (no controllers available - visualization only)")
        
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        return success
    
    def run_demo(self):
        """Run the complete demonstration"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("MOVEIT INTERFACE DEMONSTRATION")
        self.get_logger().info("=" * 60 + "\n")
        
        # 1. Print current state
        self.print_robot_state()
        
        input("\nPress Enter to plan to first joint configuration...")
        
        # 2. Plan to joint goal 1
        self.get_logger().info("\n[DEMO 1] Planning to Joint Configuration 1")
        joint_goal_1 = [0.0, 0.5, -0.5, 0.3, 0.0]
        plan = self.plan_to_joint_goal(joint_goal_1)
        
        if plan:
            input("Press Enter to visualize the trajectory...")
            self.execute_plan(plan)
        
        input("\nPress Enter to plan to second joint configuration...")
        
        # 3. Plan to joint goal 2
        self.get_logger().info("\n[DEMO 2] Planning to Joint Configuration 2")
        joint_goal_2 = [0.5, 0.0, 0.0, 0.0, 0.5]
        plan = self.plan_to_joint_goal(joint_goal_2)
        
        if plan:
            input("Press Enter to visualize the trajectory...")
            self.execute_plan(plan)
        
        input("\nPress Enter to plan to a Cartesian pose...")
        
        # 4. Plan to Cartesian pose
        self.get_logger().info("\n[DEMO 3] Planning to Cartesian Pose")
        pose_goal = Pose()
        pose_goal.position.x = 0.2
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.3
        pose_goal.orientation.w = 1.0
        
        plan = self.plan_to_pose_goal(pose_goal)
        
        if plan:
            input("Press Enter to visualize the trajectory...")
            self.execute_plan(plan)
        
        # 5. Print final state
        self.print_robot_state()
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("DEMONSTRATION COMPLETE")
        self.get_logger().info("=" * 60)
        self.get_logger().info("\nNote: Execution only updates visualization.")
        self.get_logger().info("To execute on real hardware, configure ros2_control.")
        self.get_logger().info("=" * 60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = MoveItInterfaceDemo()
        demo.run_demo()
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
