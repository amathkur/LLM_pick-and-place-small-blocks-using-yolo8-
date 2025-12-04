#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(String, '/demonstration_input', 10)
        
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_command.py <command>")
        print("Commands: start, pick, place, save, stop, train")
        return
    
    command = sys.argv[1]
    
    rclpy.init()
    node = CommandPublisher()
    
    node.send_command(command)
    
    # Give time for the message to be sent
    import time
    time.sleep(0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()