#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ScanConeVisualizer(Node):
    def __init__(self):
        super().__init__('scan_cone_visualizer')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/scan_cone')
        self.declare_parameter('max_visual_range', 6.0)
        self.declare_parameter('arc_segments', 48)

        scan_topic = self.get_parameter('scan_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.max_visual_range = float(self.get_parameter('max_visual_range').value)
        self.arc_segments = max(8, int(self.get_parameter('arc_segments').value))

        self.publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

    def scan_callback(self, msg):
        visual_range = msg.range_max
        if not math.isfinite(visual_range) or visual_range <= 0.0:
            visual_range = self.max_visual_range
        visual_range = min(visual_range, self.max_visual_range)

        angles = [
            msg.angle_min + (msg.angle_max - msg.angle_min) * i / self.arc_segments
            for i in range(self.arc_segments + 1)
        ]
        arc_points = [
            Point(x=visual_range * math.cos(angle), y=visual_range * math.sin(angle), z=0.01)
            for angle in angles
        ]
        origin = Point(x=0.0, y=0.0, z=0.01)

        fill = Marker()
        fill.header = msg.header
        fill.ns = 'scan_cone'
        fill.id = 0
        fill.type = Marker.TRIANGLE_LIST
        fill.action = Marker.ADD
        fill.scale.x = 1.0
        fill.scale.y = 1.0
        fill.scale.z = 1.0
        fill.color.r = 1.0
        fill.color.g = 0.82
        fill.color.b = 0.08
        fill.color.a = 0.18
        fill.lifetime = Duration(seconds=0.4).to_msg()
        fill.frame_locked = True

        for left, right in zip(arc_points[:-1], arc_points[1:]):
            fill.points.extend([origin, left, right])

        outline = Marker()
        outline.header = msg.header
        outline.ns = 'scan_cone'
        outline.id = 1
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.scale.x = 0.025
        outline.color.r = 1.0
        outline.color.g = 0.55
        outline.color.b = 0.0
        outline.color.a = 0.95
        outline.points = [origin, *arc_points, origin]
        outline.lifetime = Duration(seconds=0.4).to_msg()
        outline.frame_locked = True

        self.publisher.publish(MarkerArray(markers=[fill, outline]))


def main():
    rclpy.init()
    node = ScanConeVisualizer()
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
