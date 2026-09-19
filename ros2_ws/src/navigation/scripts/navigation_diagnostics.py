#!/usr/bin/env python3

import math
import re

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavigationDiagnostics(Node):
    def __init__(self):
        super().__init__('navigation_diagnostics')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('cmd_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('cmd_safe_topic', '/cmd_vel_safe')
        self.declare_parameter('route_status_topic', '/recognition_route_status')
        self.declare_parameter('marker_topic', '/navigation_diagnostics/markers')
        self.declare_parameter('log_period_sec', 0.5)
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_sector_start_deg', 55.0)
        self.declare_parameter('side_sector_end_deg', 125.0)
        self.declare_parameter('configured_robot_length', 0.17535744)
        self.declare_parameter('configured_robot_width', 0.11237406)
        self.declare_parameter('actual_robot_length', 0.17535744)
        self.declare_parameter('actual_robot_width', 0.11237406)
        self.declare_parameter('safety_stop_distance', 0.095)
        self.declare_parameter('safety_release_distance', 0.16)
        self.declare_parameter('side_stop_distance', 0.10)
        self.declare_parameter('side_release_distance', 0.15)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_nav_topic = str(self.get_parameter('cmd_nav_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.cmd_safe_topic = str(self.get_parameter('cmd_safe_topic').value)
        self.route_status_topic = str(self.get_parameter('route_status_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.log_period_sec = max(0.1, float(self.get_parameter('log_period_sec').value))
        self.map_to_odom_x = float(self.get_parameter('map_to_odom_x').value)
        self.map_to_odom_y = float(self.get_parameter('map_to_odom_y').value)
        self.map_to_odom_yaw = float(self.get_parameter('map_to_odom_yaw').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_sector_start = math.radians(
            float(self.get_parameter('side_sector_start_deg').value)
        )
        self.side_sector_end = math.radians(
            float(self.get_parameter('side_sector_end_deg').value)
        )
        self.configured_robot_length = float(
            self.get_parameter('configured_robot_length').value
        )
        self.configured_robot_width = float(
            self.get_parameter('configured_robot_width').value
        )
        self.actual_robot_length = float(self.get_parameter('actual_robot_length').value)
        self.actual_robot_width = float(self.get_parameter('actual_robot_width').value)
        self.safety_stop_distance = float(self.get_parameter('safety_stop_distance').value)
        self.safety_release_distance = float(self.get_parameter('safety_release_distance').value)
        self.side_stop_distance = float(self.get_parameter('side_stop_distance').value)
        self.side_release_distance = float(self.get_parameter('side_release_distance').value)

        self.current_pose_map = None
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_status = 'no status yet'
        self.last_status_route = None
        self.cmd_nav = Twist()
        self.cmd_smooth = Twist()
        self.cmd_safe = Twist()

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_subscription(Twist, self.cmd_nav_topic, self.cmd_nav_callback, 20)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 20)
        self.create_subscription(Twist, self.cmd_safe_topic, self.cmd_safe_callback, 20)
        self.create_subscription(String, self.route_status_topic, self.status_callback, 20)
        self.create_timer(self.log_period_sec, self.log_snapshot)
        self.create_timer(0.2, self.publish_markers)

        self.get_logger().info(
            'Navigation diagnostics active: '
            f'odom={self.odom_topic}, scan={self.scan_topic}, '
            f'cmd_nav={self.cmd_nav_topic}, cmd={self.cmd_topic}, '
            f'cmd_safe={self.cmd_safe_topic}, status={self.route_status_topic}'
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        odom_x = pose.position.x
        odom_y = pose.position.y
        odom_yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(self.map_to_odom_yaw)
        sin_yaw = math.sin(self.map_to_odom_yaw)
        map_x = self.map_to_odom_x + cos_yaw * odom_x - sin_yaw * odom_y
        map_y = self.map_to_odom_y + sin_yaw * odom_x + cos_yaw * odom_y
        map_yaw = normalize_angle(self.map_to_odom_yaw + odom_yaw)
        self.current_pose_map = (map_x, map_y, map_yaw)

    def scan_callback(self, msg):
        front_values = []
        left_values = []
        right_values = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                abs_angle = abs(angle)
                if abs_angle <= self.front_angle:
                    front_values.append(distance)
                elif self.side_sector_start <= angle <= self.side_sector_end:
                    left_values.append(distance)
                elif -self.side_sector_end <= angle <= -self.side_sector_start:
                    right_values.append(distance)
            angle += msg.angle_increment
        self.front_min = min(front_values) if front_values else math.inf
        self.left_min = min(left_values) if left_values else math.inf
        self.right_min = min(right_values) if right_values else math.inf

    def cmd_nav_callback(self, msg):
        self.cmd_nav = msg

    def cmd_callback(self, msg):
        self.cmd_smooth = msg

    def cmd_safe_callback(self, msg):
        self.cmd_safe = msg

    def status_callback(self, msg):
        self.last_status = msg.data
        match = re.search(r'id=(\d+)', msg.data)
        self.last_status_route = match.group(1) if match else None

    def safety_mode(self):
        safe = self.cmd_safe
        smooth = self.cmd_smooth
        center_error = (
            self.left_min - self.right_min
            if math.isfinite(self.left_min) and math.isfinite(self.right_min)
            else 0.0
        )

        if safe.linear.x < -0.01:
            return 'BACKUP'
        if (
            min(self.left_min, self.right_min) < self.side_release_distance
            and abs(center_error) > 0.03
            and abs(safe.angular.z - smooth.angular.z) > 0.03
        ):
            return 'CENTERING'
        if (
            self.front_min < self.safety_release_distance
            and smooth.linear.x > 0.01
            and safe.linear.x + 0.02 < smooth.linear.x
        ):
            return 'CREEP'
        if (
            min(self.left_min, self.right_min) < self.side_release_distance
            and abs(safe.angular.z - smooth.angular.z) > 0.05
        ):
            return 'SIDE_AVOID'
        if abs(safe.angular.z - smooth.angular.z) > 0.08 or abs(safe.linear.x - smooth.linear.x) > 0.03:
            return 'MODIFIED'
        if self.front_min < self.safety_stop_distance:
            return 'STOP_ZONE'
        if min(self.left_min, self.right_min) < self.side_stop_distance:
            return 'SIDE_STOP_ZONE'
        return 'PASS'

    def log_snapshot(self):
        if self.current_pose_map is None:
            return
        x, y, yaw = self.current_pose_map
        center_error = (
            self.left_min - self.right_min
            if math.isfinite(self.left_min) and math.isfinite(self.right_min)
            else math.nan
        )
        self.get_logger().info(
            'diag '
            f'pose_map=({x:.3f},{y:.3f},{yaw:.2f}) '
            f'front_min={self.front_min:.3f} '
            f'left_min={self.left_min:.3f} '
            f'right_min={self.right_min:.3f} '
            f'center_error={center_error:.3f} '
            f'cmd_nav=({self.cmd_nav.linear.x:.2f},{self.cmd_nav.angular.z:.2f}) '
            f'cmd=({self.cmd_smooth.linear.x:.2f},{self.cmd_smooth.angular.z:.2f}) '
            f'cmd_safe=({self.cmd_safe.linear.x:.2f},{self.cmd_safe.angular.z:.2f}) '
            f'safety={self.safety_mode()} '
            f'status=\"{self.last_status}\"'
        )

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        actual = Marker()
        actual.header.frame_id = 'base_footprint'
        actual.header.stamp = stamp
        actual.ns = 'nav_diag'
        actual.id = 1
        actual.type = Marker.LINE_STRIP
        actual.action = Marker.ADD
        actual.scale.x = 0.01
        actual.color.r = 0.0
        actual.color.g = 1.0
        actual.color.b = 0.0
        actual.color.a = 1.0
        half_l = self.actual_robot_length * 0.5
        half_w = self.actual_robot_width * 0.5
        actual.points = [
            Point(x=half_l, y=half_w, z=0.02),
            Point(x=half_l, y=-half_w, z=0.02),
            Point(x=-half_l, y=-half_w, z=0.02),
            Point(x=-half_l, y=half_w, z=0.02),
            Point(x=half_l, y=half_w, z=0.02),
        ]
        markers.markers.append(actual)

        configured = Marker()
        configured.header.frame_id = 'base_footprint'
        configured.header.stamp = stamp
        configured.ns = 'nav_diag'
        configured.id = 2
        configured.type = Marker.LINE_STRIP
        configured.action = Marker.ADD
        configured.scale.x = 0.008
        configured.color.r = 1.0
        configured.color.g = 0.65
        configured.color.b = 0.0
        configured.color.a = 1.0
        half_l = self.configured_robot_length * 0.5
        half_w = self.configured_robot_width * 0.5
        configured.points = [
            Point(x=half_l, y=half_w, z=0.03),
            Point(x=half_l, y=-half_w, z=0.03),
            Point(x=-half_l, y=-half_w, z=0.03),
            Point(x=-half_l, y=half_w, z=0.03),
            Point(x=half_l, y=half_w, z=0.03),
        ]
        markers.markers.append(configured)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = NavigationDiagnostics()
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
