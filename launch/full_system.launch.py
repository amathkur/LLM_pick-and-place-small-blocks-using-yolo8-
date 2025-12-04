#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths
    try:
        urdf_path = os.path.join(get_package_share_directory('magician_ros2'), 'dobot_description', 'urdf', 'magician_lite.urdf')
    except Exception:
        urdf_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/urdf/magician_lite.urdf'

    try:
        rviz_config = os.path.join(get_package_share_directory('magician_ros2'), 'dobot_description', 'rviz', 'display.rviz')
    except Exception:
        rviz_config = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'

    try:
        srdf_path = os.path.join(get_package_share_directory('magician_moveit_config'), 'config', 'magician.srdf')
    except Exception:
        srdf_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_moveit_config/config/magician.srdf'

    with open(urdf_path, 'r') as f:
        urdf = f.read()

    with open(srdf_path, 'r') as f:
        srdf = f.read()

    # Joint state initializer (provides initial joint states without interfering with GUI)
    joint_initializer = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/joint_state_initializer.py'],
        output='screen',
        shell=False
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf}],
        output='screen'
    )

    # Static transforms
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.110', '0.0', '0.0', '0.0', 'world', 'base_footprint'],
        output='screen'
    )

    static_tf_suction = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.055', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'wrist_pitch', 'suction_cup'],
        output='screen'
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.120', '0.0', '-0.000', '1.57', '3.14159', '0.0', 'wrist_pitch', 'camera_link'],
        output='screen'
    )

    # Camera publisher
    camera_pub = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/camera_publisher.py', '--ros-args', '-p', 'camera_index:=2'],
        output='screen',
        shell=False
    )

    # YOLO detection node
    yolo_detector = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/yolo_detection_node.py'],
        output='screen',
        shell=False
    )

    # Joint State Publisher GUI for interactive joint control
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': urdf}],
        output='screen'
    )

    # Demonstration recorder
    demo_recorder = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/demonstration_recorder.py'],
        output='screen',
        shell=False
    )

    # RViz with MoveIt interactive markers
    rviz = TimerAction(
        period=5.0,  # Start RViz after other components
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{
                    'robot_description': urdf,
                    'robot_description_semantic': srdf,
                }],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        joint_initializer,
        robot_state_pub,
        static_tf_base,
        static_tf_suction,
        static_tf_camera,
        camera_pub,
        yolo_detector,
        joint_state_gui,
        demo_recorder,
        rviz
    ])