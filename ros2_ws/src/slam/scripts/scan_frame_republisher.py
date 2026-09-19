#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ScanFrameRepublisher(Node):
    def __init__(self):
        super().__init__('scan_frame_republisher')
        self.declare_parameter('input_scan_topic', '/scan_raw')
        self.declare_parameter('output_scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('stamp_offset_sec', 0.2)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('startup_delay_sec', 1.0)
        input_topic = self.get_parameter('input_scan_topic').value
        output_topic = self.get_parameter('output_scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.stamp_offset = max(0.0, float(self.get_parameter('stamp_offset_sec').value))
        publish_rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.publish_period_ns = int(1_000_000_000 / publish_rate)
        self.startup_delay_ns = int(max(0.0, float(self.get_parameter('startup_delay_sec').value)) * 1_000_000_000)
        self.start_time_ns = None
        self.last_publish_ns = 0
        self.latest_odom_stamp = None

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(LaserScan, output_topic, scan_qos)
        self.create_subscription(LaserScan, input_topic, self.scan_callback, scan_qos)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, scan_qos)

    def odom_callback(self, msg):
        self.latest_odom_stamp = Time.from_msg(msg.header.stamp)

    def scan_callback(self, msg):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if now_ns == 0:
            return

        if self.start_time_ns is None:
            self.start_time_ns = now_ns
        if now_ns - self.start_time_ns < self.startup_delay_ns:
            return
        if now_ns - self.last_publish_ns < self.publish_period_ns:
            return

        msg.header.frame_id = self.frame_id
        if self.latest_odom_stamp is not None:
            msg.header.stamp = self.latest_odom_stamp.to_msg()
        else:
            msg.header.stamp = (now + Duration(seconds=self.stamp_offset)).to_msg()
        msg.scan_time = max(msg.scan_time, self.publish_period_ns / 1_000_000_000.0)
        self.last_publish_ns = now_ns
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanFrameRepublisher()
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
