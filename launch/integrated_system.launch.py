#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Arguments
    use_real_robot_arg = DeclareLaunchArgument(
        'use_real_robot',
        default_value='false',
        description='Use real robot via USB-CAN (true) or simulation (false)'
    )
    
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='2',
        description='USB camera device index'
    )
    
    use_real_robot = LaunchConfiguration('use_real_robot')
    camera_index = LaunchConfiguration('camera_index')
    
    # Paths
    ws_path = os.path.expanduser('~/dobot_rviz_ws')
    urdf_file = os.path.join(ws_path, 'urdf', 'mg400_robot.urdf')
    rviz_config = os.path.join(ws_path, 'rviz', 'mg400_config.rviz')
    
    return LaunchDescription([
        use_real_robot_arg,
        camera_index_arg,
        
        # Robot State Publisher (always needed)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        
        # Static TF - Robot Base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base',
            arguments=['0', '0', '0.12', '0', '0', '0', 'world', 'base_link']
        ),
        
        # Static TF - Camera on wrist
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=['0.12', '0', '0', '0', '0', '0', 'wrist_pitch', 'camera_link']
        ),
        
        # MoveIt MoveGroup
        ExecuteProcess(
            cmd=['ros2', 'run', 'moveit_py', 'move_group_node',
                 '--ros-args', '-p', 'use_sim_time:=false'],
            output='screen',
            condition=UnlessCondition(use_real_robot)
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # Camera Publisher
        ExecuteProcess(
            cmd=['python3', os.path.join(ws_path, 'camera_publisher.py'),
                 '--ros-args', '-p', f'camera_index:={camera_index}'],
            output='screen'
        ),
        
        # YOLO Detection Node
        ExecuteProcess(
            cmd=['python3', os.path.join(ws_path, 'yolo_detection_node.py')],
            output='screen'
        ),
        
        # Intelligent LLM Controller
        ExecuteProcess(
            cmd=['python3', os.path.join(ws_path, 'dobot_llm_project', 'intelligent_llm_controller.py')],
            output='screen'
        ),
        
        # Fake Joint Publisher (simulation only)
        ExecuteProcess(
            cmd=['python3', os.path.join(ws_path, 'fake_joint_publisher.py')],
            output='screen',
            condition=UnlessCondition(use_real_robot)
        ),
        
        # Real Robot CAN Interface (real robot only)
        ExecuteProcess(
            cmd=['python3', os.path.join(ws_path, 'dobot_can_interface.py')],
            output='screen',
            condition=IfCondition(use_real_robot)
        ),
    ])
