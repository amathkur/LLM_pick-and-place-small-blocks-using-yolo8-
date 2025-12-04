#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

class ParamSetter(Node):
    def __init__(self):
        super().__init__('param_setter')

        # Read SRDF
        with open('/home/abdulhamid/dobot_rviz_ws/install/magician_moveit_config/share/magician_moveit_config/config/magician.srdf', 'r') as f:
            srdf_content = f.read()

        # Set the parameter
        self.declare_parameter('robot_description_semantic', srdf_content)
        self.get_logger().info('Set robot_description_semantic parameter')

        # Keep the node alive
        while rclpy.ok():
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ParamSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()