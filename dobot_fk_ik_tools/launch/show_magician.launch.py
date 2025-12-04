#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os, glob

def pick_robot_file(dd_root):
    # try xacro first, then urdf; search model/, urdf/, and root of package
    for ext in ('.urdf.xacro', '.xacro', '.urdf'):
        for sub in ('model','urdf',''):
            hits = glob.glob(os.path.join(dd_root, sub, f'*{ext}'))
            if hits:
                return hits[0]
    return None

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    lift_z   = LaunchConfiguration('lift_z')

    dd = get_package_share_directory('dobot_description')
    robot_file = pick_robot_file(dd)
    if robot_file is None:
        raise RuntimeError("Could not find a URDF/Xacro in dobot_description (looked in model/ and urdf/)")

    # If it's xacro, expand; otherwise cat
    is_xacro = robot_file.endswith('.xacro')
    cmd = ['xacro ', robot_file, ' lift_z:=', lift_z] if is_xacro else \
          [TextSubstitution(text='cat '), TextSubstitution(text=robot_file)]

    robot_desc = ParameterValue(Command(cmd), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('lift_z',   default_value='0.15'),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}],
             output='screen'),

        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0','0', lift_z, '0','0','0','world','base_link'],
             output='screen'),

        Node(condition=IfCondition(use_rviz),
             package='rviz2',
             executable='rviz2',
             output='screen'),
    ])
