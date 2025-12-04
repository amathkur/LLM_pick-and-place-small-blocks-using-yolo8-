#!/usr/bin/env python3
"""
Complete Integrated System Launch File
Combines: USB Camera + YOLO Detection + LLM Controller + MoveIt + RViz + Real Robot
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='2',
        description='Camera device index (0=laptop, 2=USB)'
    )
    
    use_real_robot_arg = DeclareLaunchArgument(
        'use_real_robot',
        default_value='true',
        description='Connect to real robot via serial'
    )
    
    # Get workspace path
    ws_path = os.path.expanduser('~/dobot_rviz_ws')
    
    # Camera Publisher Node
    camera_node = Node(
        package='python3',
        executable='python3',
        name='camera_publisher',
        output='log',
        arguments=[
            os.path.join(ws_path, 'camera_publisher.py'),
            '--camera_index', LaunchConfiguration('camera_index')
        ]
    )
    
    # YOLO Detection Node
    yolo_node = Node(
        package='python3',
        executable='python3',
        name='yolo_detection_node',
        output='log',
        arguments=[os.path.join(ws_path, 'yolo_detection_node.py')]
    )
    
    # LLM Controller Node
    llm_node = Node(
        package='python3',
        executable='python3',
        name='intelligent_llm_controller',
        output='log',
        arguments=[os.path.join(ws_path, 'dobot_llm_project/intelligent_llm_controller.py')]
    )
    
    # Real Robot Serial Interface
    robot_node = Node(
        package='python3',
        executable='python3',
        name='dobot_serial_interface',
        output='log',
        arguments=[os.path.join(ws_path, 'dobot_serial_interface.py')],
        condition=LaunchConfiguration('use_real_robot')
    )
    
    # MoveIt Launch (as subprocess since it's a launch file)
    moveit_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            os.path.join(ws_path, 'launch/moveit_demo.launch.py')
        ],
        output='log'
    )
    
    return LaunchDescription([
        camera_index_arg,
        use_real_robot_arg,
        camera_node,
        yolo_node,
        llm_node,
        robot_node,
        moveit_launch
    ])

if __name__ == '__main__':
    generate_launch_description()
