#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Read files
    with open('/home/abdulhamid/dobot_rviz_ws/install/magician_description/share/magician_description/urdf/magician_lite_gazebo_control.urdf', 'r') as f:
        urdf = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_footprint']
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam'
        ),
        Node(
            package='dobot_llm_agent',
            executable='llm_agent',
            name='llm_agent'
        ),
        Node(
            package='dobot_llm_agent',
            executable='robot_controller',
            name='robot_controller'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/abdulhamid/dobot_rviz_ws/display.rviz']
        )
    ])