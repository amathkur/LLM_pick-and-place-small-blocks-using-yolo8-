#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    lift_z   = LaunchConfiguration('lift_z')

    dd = get_package_share_directory('dobot_description')

    # Try Xacro first; fall back to URDF
    # Common names in magician_ros2:
    candidates = [
        os.path.join(dd, 'urdf', 'dobot_magician_4dof.urdf.xacro'),
        os.path.join(dd, 'urdf', 'dobot_magician.urdf.xacro'),
        os.path.join(dd, 'urdf', 'magician.urdf.xacro'),
        os.path.join(dd, 'urdf', 'dobot_magician_4dof.urdf'),
        os.path.join(dd, 'urdf', 'dobot_magician.urdf'),
        os.path.join(dd, 'urdf', 'magician.urdf'),
    ]

    robot_description = None
    for p in candidates:
        if os.path.exists(p):
            if p.endswith('.xacro'):
                # Adjust Xacro args to your needs; DOF and tool are common in this repo
                robot_description = Command([TextSubstitution(text='xacro '), p,
                                             TextSubstitution(text=' DOF:=4 tool:=none')])
            else:
                robot_description = Command([TextSubstitution(text='cat '), p])
            break

    if robot_description is None:
        raise RuntimeError("Could not find a URDF or Xacro in dobot_description/urdf")

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('lift_z',   default_value='0.15'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0', lift_z, '0','0','0', 'world', 'base_link'], output='screen'),

        Node(condition=IfCondition(use_rviz),
             package='rviz2', executable='rviz2', output='screen'),
    ])
