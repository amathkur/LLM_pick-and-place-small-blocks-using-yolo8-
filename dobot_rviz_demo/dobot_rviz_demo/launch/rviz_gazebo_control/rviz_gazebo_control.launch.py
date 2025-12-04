#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    # Direct paths
    rviz_config_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # MoveIt move_group
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('magician_moveit_config'), 'launch', 'moveit.launch.py')
            )
        ),

        Node(condition=IfCondition(use_rviz), package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config_path], 
             output='screen'),
    ])