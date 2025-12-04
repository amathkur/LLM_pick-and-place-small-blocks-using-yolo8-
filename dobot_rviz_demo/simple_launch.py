#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_service import LaunchService

def generate_launch_description():
    urdf_path = '/home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/dobot_rviz_demo/launch/rviz_gazebo_control/magician_lite_control.urdf'
    
    return LaunchDescription([
        # Static TF
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
                 '--frame-id', 'world', '--child-frame-id', 'base_footprint',
                 '--x', '0.0', '--y', '0.0', '--z', '0.0',
                 '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0'],
            output='screen'
        ),
        
        # Robot State Publisher
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                 '--ros-args', '-p', f'robot_description:={open(urdf_path).read()}'],
            output='screen'
        ),
        
        # RViz
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', 
                 '-d', '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
