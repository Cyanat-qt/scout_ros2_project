#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock


class ClockRepublisher(Node):
    def __init__(self):
        super().__init__('clock_republisher')
        self.declare_parameter('input_topic', '/clock_raw')
        self.declare_parameter('output_topic', '/clock')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.last_stamp_ns = 0
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.publisher = self.create_publisher(Clock, output_topic, qos)
        self.subscription = self.create_subscription(
            Clock,
            input_topic,
            self.republish_clock,
            qos,
        )
        self.get_logger().info(f'Republishing monotonic clock {input_topic} to {output_topic}')

    def republish_clock(self, msg):
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + 1
        self.last_stamp_ns = stamp_ns

        out = Clock()
        out.clock = Time(nanoseconds=stamp_ns).to_msg()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = ClockRepublisher()
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
