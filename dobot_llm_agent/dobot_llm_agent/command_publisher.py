#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(String, 'voice_command', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.commands = [
            "Pick up the small blue block and place it in the box on the right.",
            "Pick up the large green block and place it in the box on the left.",
            "Pick up a small block and place it on top of the large block.",
            "Pick up the small blue block, rotate by 90 degrees in z, and place it on large red block.",
            "Pick a small yellow block and place it to the right of the red block."
        ]
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.commands):
            msg = String()
            msg.data = self.commands[self.index]
            self.publisher.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()
    rclpy.spin(command_publisher)
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()