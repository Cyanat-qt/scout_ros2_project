#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        odom_topic = self.get_parameter('odom_topic').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.tf_broadcaster = TransformBroadcaster(self)

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_callback, odom_qos)

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = self.child_frame_id or msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
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
