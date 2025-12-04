#!/usr/bin/env python3

import os
import yaml
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    """Load YAML with fallback to source directory"""
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
    except Exception:
        # Fallback to source directory
        ws_path = os.path.expanduser('~/dobot_rviz_ws/src')
        absolute_file_path = os.path.join(ws_path, package_name, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Warning: Could not load {file_path}: {e}")
        return {}

def generate_launch_description():

    # Camera publisher node: publishes camera images for YOLO and RViz
    camera_pub = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/camera_publisher.py', '--ros-args', '-p', 'camera_index:=2'],
        output='screen',
        shell=False
    )
    # Dobot MoveIt bridge node (real robot connection)
    dobot_bridge = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/dobot_moveit_bridge.py'],
        output='screen',
        shell=False
    )
    # Read files (prefer installed package share; fallback to source tree)
    try:
        urdf_path = os.path.join(get_package_share_directory('magician_ros2'), 'dobot_description', 'urdf', 'magician_lite.urdf')
    except Exception:
        urdf_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/urdf/magician_lite.urdf'

    try:
        srdf_path = os.path.join(get_package_share_directory('magician_moveit_config'), 'config', 'magician.srdf')
    except Exception:
        srdf_path = '/home/abdulhamid/dobot_rviz_ws/src/magician_moveit_config/config/magician.srdf'

    try:
        rviz_config = os.path.join(get_package_share_directory('magician_ros2'), 'dobot_description', 'rviz', 'display.rviz')
    except Exception:
        rviz_config = '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'
    
    # Load YAML files (with fallback to empty dict)
    kinematics_yaml = load_yaml('magician_moveit_config', 'config/kinematics.yaml') or {}
    ompl_planning_yaml = load_yaml('magician_moveit_config', 'config/ompl_planning.yaml') or {}
    joint_limits_yaml = load_yaml('magician_moveit_config', 'config/joint_limits.yaml') or {}

    # Override kinematics settings for better performance
    kinematics_yaml['kinematics_solver_timeout'] = 1.0
    kinematics_yaml['kinematics_solver_search_resolution'] = 0.005
    kinematics_yaml['kinematics_solver_attempts'] = 10

    # Unwrap kinematics parameters if wrapped
    if kinematics_yaml and 'ros__parameters' in kinematics_yaml:
        kinematics_yaml = kinematics_yaml['ros__parameters']

    with open(urdf_path, 'r') as f:
        urdf = f.read()

    with open(srdf_path, 'r') as f:
        srdf = f.read()

    # Trajectory execution parameters (disabled for simulation - only planning)
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'allow_trajectory_execution': True,  # Enable execution for simulation
        'execution_duration_monitoring': False,
    }

    # Controller configuration for simulation
    controller_config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['magician_arm_controller'],
            'magician_arm_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['joint1', 'joint2', 'joint3', 'joint4'],
                'action_ns': 'follow_joint_trajectory',
            }
        }
    }

    # MoveIt parameters for MoveGroup
    moveit_params = {
        'robot_description': urdf,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': kinematics_yaml,
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            'jiggle_fraction': 0.05,
        },
        'ompl': ompl_planning_yaml,
        'robot_description_planning': joint_limits_yaml,
    }
    
    # Add controller and trajectory execution params
    moveit_params.update(controller_config)
    moveit_params.update(trajectory_execution)
    
    # RViz MoveIt plugin parameters (to enable interactive markers)
    rviz_moveit_params = {
        'robot_description': urdf,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'planning_pipelines': ['move_group'],
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
        },
        'ompl': ompl_planning_yaml,
    }
    
    # Add kinematics directly for interactive markers (already unwrapped above)
    if kinematics_yaml:
        rviz_moveit_params.update(kinematics_yaml)

    # Define all nodes
    fake_joint_pub = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/fake_joint_publisher.py'],
        output='screen',
        shell=False
    )
    
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf}],
        output='screen'
    )
    
    
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.110', '0.0', '0.0', '0.0', 'world', 'base_footprint'],  # 110mm above world
        output='screen'
    )
    
    # MoveGroup - delayed start to ensure joint states are publishing
    
    # RViz - delayed start after MoveGroup
    rviz = TimerAction(
        period=4.0,  # Wait for MoveGroup to initialize
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[rviz_moveit_params],
                output='screen'
            )
        ]
    )
    

    # LLM Controller: uses Cloud LLM (OpenAI GPT) for command interpretation
    # Prompts for end effector, then accepts text commands to control robot in RViz and real hardware
    # REMOVED FROM LAUNCH - Run separately with: python3.10 cloud_llm_controller.py
    # llm_controller = ExecuteProcess(
    #     cmd=['python3.10', os.path.join(os.path.expanduser('~'), 'dobot_rviz_ws', 'cloud_llm_controller.py')],
    #     output='screen',
    #     shell=False
    # )
    
    yolo_node = ExecuteProcess(
        cmd=['python3.10', '/home/abdulhamid/dobot_rviz_ws/yolo_detection_node.py', '--ros-args', '-p', 'confidence_threshold:=0.3', '-p', 'target_class:=cube', '-p', 'publish_annotated_image:=true'],
        output='screen',
        shell=False
    )
    
    # Suction cup is 55mm (0.055m) forward from wrist_pitch along X axis
    static_tf_suction = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.055', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'wrist_pitch', 'suction_cup'],
        output='screen'
    )
    
    # Camera is 120mm forward, 100mm down from wrist_pitch, looking straight down
    # Rotation: roll=0, pitch=180° (around Y), yaw=0
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.120', '0.0', '-0.000', '1.57', '3.14159', '0.0', 'wrist_pitch', 'camera_link'],
        output='screen'
    )


    return LaunchDescription([
        # Start fake joint publisher FIRST
        fake_joint_pub,
        # Start robot state publisher and TF
        robot_state_pub,
        static_tf_base,
        static_tf_suction,  # Suction cup transform
        static_tf_camera,   # Camera transform
        # Start camera publisher for YOLO and RViz
        camera_pub,
        # Start YOLO node (subscribes to camera)
        yolo_node,
        # Start Dobot bridge node (real robot connection)
        dobot_bridge,
        # Start RViz (visualization) before LLM controller
        rviz,
        # LLM components removed - focus on YOLO detection only
    ])