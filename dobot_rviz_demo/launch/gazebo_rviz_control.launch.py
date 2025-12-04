#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gz = LaunchConfiguration('use_gz', default='true')

    # Direct paths
    urdf_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/urdf/magician_lite_gazebo.urdf'
    rviz_config_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'
    robot_description = open(urdf_path).read()

    gz = ExecuteProcess(condition=IfCondition(use_gz),
                        cmd=['ign','gazebo','-r','-v','3','empty.sdf'],
                        output='screen')

    gz_spawn = Node(condition=IfCondition(use_gz),
                    package='ros_gz_sim', executable='create',
                    arguments=['-name','dobot_magician','-topic','/robot_description','-x','0','-y','0','-z','0'],
                    output='screen')

    # Joint state bridge from Gazebo to ROS
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/model/dobot_magician/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'],
        remappings=[('/world/empty/model/dobot_magician/joint_state', '/joint_states_from_gazebo')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gz', default_value='true'),

        # Robot State Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),

        # Static transform from world to base_footprint
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0.15','0','0','0','world','base_footprint'], output='screen'),

        # Joint State Publisher GUI for manual control
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),

        # Joint Controller to bridge GUI commands to Gazebo
        ExecuteProcess(
            cmd=['python3', '/home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/joint_controller.py'],
            output='screen'
        ),

        # Gazebo
        gz, gz_spawn,

        # RViz
        Node(condition=IfCondition(use_rviz), package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config_path], output='screen'),
    ])
