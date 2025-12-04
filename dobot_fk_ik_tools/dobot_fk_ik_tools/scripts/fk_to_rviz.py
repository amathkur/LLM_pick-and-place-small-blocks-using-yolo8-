#!/usr/bin/env python3
import argparse, time, os, xml.etree.ElementTree as ET
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
def deg2rad(d): return d*3.141592653589793/180.0
def movable_joints_from_urdf(path:str)->List[str]:
    try:
        root=ET.parse(path).getroot()
        return [j.get('name') for j in root.findall('joint') if j.get('type') in ('revolute','continuous')]
    except Exception:
        return []
class FKPublisher(Node):
    def __init__(self,topic,joint_names,thetas_deg,seconds,rate=25.0):
        super().__init__('fk_publisher')
        self.pub=self.create_publisher(JointState,topic,10)
        self.joint_names=joint_names
        self.positions=[deg2rad(t) for t in thetas_deg]
        self.t_end=time.time()+float(seconds)
        self.timer=self.create_timer(1.0/float(rate),self.tick)
        self.get_logger().info(f"Publishing FK on {topic} for {seconds:.1f}s with joints {joint_names}")
    def tick(self):
        if time.time()>=self.t_end:
            self.get_logger().info("Finished publishing FK pose")
            self.timer.cancel(); return
        msg=JointState()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.joint_names
        msg.position=self.positions
        self.pub.publish(msg)
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--topic",default="/joint_states")
    ap.add_argument("--seconds",type=float,default=10.0)
    ap.add_argument("--thetas",nargs='+',type=float,required=True)
    ap.add_argument("--joints",nargs='+',type=str,default=None)
    ap.add_argument("--urdf",type=str,default=None)
    a=ap.parse_args()
    if a.joints: names=a.joints
    elif a.urdf and os.path.isfile(a.urdf): names=movable_joints_from_urdf(a.urdf)
    else: raise SystemExit("Provide --urdf or --joints so joint order matches URDF")
    if len(names)!=len(a.thetas):
        raise SystemExit(f"len(joints)={len(names)} must equal len(thetas)={len(a.thetas)}")
    rclpy.init()
    node=FKPublisher(a.topic,names,a.thetas,a.seconds)
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=="__main__": main()
