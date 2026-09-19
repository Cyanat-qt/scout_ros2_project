#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState


class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')
        self.declare_parameter('input_topic', '/joint_states_raw')
        self.declare_parameter('output_topic', '/joint_states')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.last_stamp_ns = 0
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self.republish_joint_state,
            10,
        )
        self.get_logger().info(f'Republishing {input_topic} to {output_topic}')

    def republish_joint_state(self, msg):
        stamp_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + 1_000_000
        self.last_stamp_ns = stamp_ns

        out = JointState()
        out.header = msg.header
        out.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        out.name = msg.name
        out.position = msg.position
        out.velocity = msg.velocity
        out.effort = msg.effort
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = JointStateRepublisher()
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
