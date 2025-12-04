#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_gz   = LaunchConfiguration('use_gz')
    world    = LaunchConfiguration('world')
    lift_z   = LaunchConfiguration('lift_z')

    # Direct paths
    urdf_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/urdf/magician_lite.urdf'
    rviz_config_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'
    robot_description = open(urdf_path).read()

    gz = ExecuteProcess(condition=IfCondition(use_gz),
                        cmd=['ign','gazebo','-r','-v','3','empty.sdf'],
                        output='screen')

    gz_spawn = Node(condition=IfCondition(use_gz),
                    package='ros_gz_sim', executable='create',
                    arguments=['-name','dobot_magician','-topic','/robot_description','-x','0','-y','0','-z','0'],
                    output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gz',   default_value='true'),
        DeclareLaunchArgument('world',    default_value=''),
        DeclareLaunchArgument('lift_z',   default_value='0.15'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
             output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0', lift_z, '0','0','0','world','base_footprint'], output='screen'),



        # Comment out fk_to_rviz for now
        # Node(package='dobot_fk_ik_tools', executable='fk_to_rviz.py', name='fk_hold_pose',
        #      arguments=['--thetas','0','20','10','0','--seconds','1e9','--topic','/joint_states',
        #                 '--urdf', '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/urdf/magician_lite.urdf'],
        #      output='screen'),

        Node(condition=IfCondition(use_rviz), package='rviz2', executable='rviz2', 
             arguments=['-d', rviz_config_path], output='screen'),

        gz, gz_spawn,
    ])
