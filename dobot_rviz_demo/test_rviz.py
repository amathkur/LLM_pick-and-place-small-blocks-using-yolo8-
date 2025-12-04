#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time
import os

def main():
    # Start robot_state_publisher
    rsp_proc = subprocess.Popen([
        'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher'
    ], env=dict(os.environ, PYTHONPATH='/opt/ros/humble/lib/python3.10/site-packages'))
    
    time.sleep(1)
    
    # Set robot description parameter
    with open('/home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/dobot_rviz_demo/launch/rviz_gazebo_control/magician_lite_control.urdf', 'r') as f:
        urdf_content = f.read()
    
    subprocess.run(['ros2', 'param', 'set', '/robot_state_publisher', 'robot_description', urdf_content])
    
    # Start RViz
    rviz_proc = subprocess.Popen([
        'ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/abdulhamid/dobot_rviz_ws/src/magician_ros2/dobot_description/rviz/display.rviz'
    ])
    
    try:
        rviz_proc.wait()
    except KeyboardInterrupt:
        rviz_proc.terminate()
        rsp_proc.terminate()

if __name__ == '__main__':
    main()
