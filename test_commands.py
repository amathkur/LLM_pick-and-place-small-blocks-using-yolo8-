#!/usr/bin/env python3
"""
Test script to send commands to the LLM controller
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CommandTester(Node):
    def __init__(self):
        super().__init__('command_tester')

        # Publisher for LLM commands
        self.command_pub = self.create_publisher(
            String,
            '/llm_command',
            10
        )

        # Subscriber for LLM responses
        self.response_sub = self.create_subscription(
            String,
            '/llm_response',
            self.response_callback,
            10
        )

        self.get_logger().info('Command tester started')

        # Send test commands
        self.send_test_commands()

    def send_test_commands(self):
        """Send test commands to the LLM controller"""
        commands = [
            "go home",
            "move up 5 cm",
            "move down 5 cm",
            "move left 5 cm",
            "move right 5 cm"
        ]

        for cmd in commands:
            self.get_logger().info(f'Sending command: {cmd}')
            msg = String()
            msg.data = cmd
            self.command_pub.publish(msg)
            time.sleep(3)  # Wait for response

    def response_callback(self, msg):
        """Handle responses from LLM controller"""
        self.get_logger().info(f'Response: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()