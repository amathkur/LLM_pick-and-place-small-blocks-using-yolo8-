#!/usr/bin/env python3
import rclpy,math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dobot_wrapper import Dobot

class JP(Node):
    def __init__(self):
        super().__init__('dobot_joint_pub')
        self.declare_parameter('port','/dev/ttyACM0')
        port=self.get_parameter('port').get_parameter_value().string_value
        self.dev=Dobot(port=port)
        self.names=['joint1','joint2','joint3','joint4']
        self.pub=self.create_publisher(JointState,'joint_states',10)
        self.create_timer(0.05,self.tick)
    def tick(self):
        _,j=self.dev.get_pose_and_joints()
        q=[math.radians(j.x),math.radians(j.y),math.radians(j.z),math.radians(j.r)]
        msg=JointState(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.names; msg.position=q; self.pub.publish(msg)

def main():
    rclpy.init(); node=JP()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
