#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_footprint', 'camera_link', rclpy.time.Time())
            self.get_logger().info(f'Transform from base_footprint to camera_link:')
            self.get_logger().info(f'  Translation: x={transform.transform.translation.x:.3f}, y={transform.transform.translation.y:.3f}, z={transform.transform.translation.z:.3f}')
            self.get_logger().info(f'  Rotation: x={transform.transform.rotation.x:.3f}, y={transform.transform.rotation.y:.3f}, z={transform.transform.rotation.z:.3f}, w={transform.transform.rotation.w:.3f}')
            return transform
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return None

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin_once(node, timeout_sec=1.0)
    transform = node.get_transform()
    node.destroy_node()
    rclpy.shutdown()
    return transform

if __name__ == '__main__':
    main()