#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mode     = LaunchConfiguration('mode')
    lift_z   = LaunchConfiguration('lift_z')
    use_rviz = LaunchConfiguration('use_rviz')

    dd = get_package_share_directory('dobot_description')
    urdf_path = os.path.join(dd, 'urdf', 'dobot_magician_4dof.urdf')
    robot_description = Command([TextSubstitution(text='cat '), urdf_path])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('lift_z', default_value='0.15'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0', lift_z, '0','0','0','world','base_link'], output='screen'),

        # keep a steady pose; your FK/IK overrides any time
        Node(condition=IfCondition(LaunchConfiguration('mode')),
             package='dobot_rviz_demo', executable='fk_to_rviz.py', name='fk_hold_pose',
             arguments=['--thetas','0','20','10','0','--seconds','1e9','--topic','/joint_states',
                        '--urdf', urdf_path],
             output='screen'),

        Node(condition=IfCondition(use_rviz), package='rviz2', executable='rviz2', output='screen'),
    ])
