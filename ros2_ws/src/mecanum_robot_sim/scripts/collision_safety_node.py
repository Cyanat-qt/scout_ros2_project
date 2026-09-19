#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class CollisionSafetyNode(Node):
    def __init__(self):
        super().__init__('collision_safety_node')
        self.declare_parameter('cmd_vel_in', '/cmd_vel')
        self.declare_parameter('cmd_vel_out', '/cmd_vel_safe')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_sector_start_deg', 55.0)
        self.declare_parameter('side_sector_end_deg', 125.0)
        self.declare_parameter('stop_distance', 0.09)
        self.declare_parameter('release_distance', 0.16)
        self.declare_parameter('side_stop_distance', 0.10)
        self.declare_parameter('side_release_distance', 0.15)
        self.declare_parameter('side_emergency_distance', 0.05)
        self.declare_parameter('side_emergency_release_distance', 0.08)
        self.declare_parameter('corridor_front_clear_distance', 0.22)
        self.declare_parameter('corridor_balance_distance', 0.45)
        self.declare_parameter('corridor_balance_deadband', 0.02)
        self.declare_parameter('corridor_centering_gain', 1.2)
        self.declare_parameter('max_centering_angular_bias', 0.10)
        self.declare_parameter('creep_speed', 0.12)
        self.declare_parameter('side_creep_speed', 0.10)
        self.declare_parameter('turning_drive_speed', 0.06)
        self.declare_parameter('holonomic_drive', True)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('backup_speed', -0.10)
        self.declare_parameter('backup_duration', 0.35)
        self.declare_parameter('unstick_turn_speed', 0.45)
        self.declare_parameter('wall_escape_turn_speed', 0.35)
        self.declare_parameter('unstick_toggle_period', 1.2)
        self.declare_parameter('cmd_timeout', 0.4)
        self.declare_parameter('scan_timeout', 0.8)
        self.declare_parameter('publish_rate', 20.0)

        self.cmd_vel_in = self.get_parameter('cmd_vel_in').value
        self.cmd_vel_out = self.get_parameter('cmd_vel_out').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_sector_start = math.radians(
            float(self.get_parameter('side_sector_start_deg').value)
        )
        self.side_sector_end = math.radians(
            float(self.get_parameter('side_sector_end_deg').value)
        )
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.release_distance = float(self.get_parameter('release_distance').value)
        self.side_stop_distance = float(self.get_parameter('side_stop_distance').value)
        self.side_release_distance = float(self.get_parameter('side_release_distance').value)
        self.side_emergency_distance = float(
            self.get_parameter('side_emergency_distance').value
        )
        self.side_emergency_release_distance = float(
            self.get_parameter('side_emergency_release_distance').value
        )
        self.corridor_front_clear_distance = float(
            self.get_parameter('corridor_front_clear_distance').value
        )
        self.corridor_balance_distance = float(
            self.get_parameter('corridor_balance_distance').value
        )
        self.corridor_balance_deadband = float(
            self.get_parameter('corridor_balance_deadband').value
        )
        self.corridor_centering_gain = float(
            self.get_parameter('corridor_centering_gain').value
        )
        self.max_centering_angular_bias = abs(
            float(self.get_parameter('max_centering_angular_bias').value)
        )
        self.creep_speed = abs(float(self.get_parameter('creep_speed').value))
        self.side_creep_speed = abs(float(self.get_parameter('side_creep_speed').value))
        self.turning_drive_speed = abs(
            float(self.get_parameter('turning_drive_speed').value)
        )
        self.holonomic_drive = bool(self.get_parameter('holonomic_drive').value)
        self.max_angular_speed = abs(
            float(self.get_parameter('max_angular_speed').value)
        )
        self.backup_speed = -abs(float(self.get_parameter('backup_speed').value))
        self.backup_duration = float(self.get_parameter('backup_duration').value)
        self.unstick_turn_speed = abs(float(self.get_parameter('unstick_turn_speed').value))
        self.wall_escape_turn_speed = abs(
            float(self.get_parameter('wall_escape_turn_speed').value)
        )
        self.unstick_toggle_period = max(
            0.2,
            float(self.get_parameter('unstick_toggle_period').value),
        )
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_scan_time_ns = 0
        self.backup_until_ns = 0
        self.last_cmd = Twist()
        self.last_cmd_time_ns = 0
        self.was_backing = False

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(Twist, self.cmd_vel_out, 10)
        self.create_subscription(Twist, self.cmd_vel_in, self.cmd_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_timer(1.0 / publish_rate, self.publish_safe_cmd)

        self.get_logger().info(
            f'Collision safety active: {self.cmd_vel_in} + {self.scan_topic} -> {self.cmd_vel_out}'
        )

    def unstick_angular(self, now_ns):
        if abs(self.last_cmd.angular.z) > 0.05:
            return max(
                -self.unstick_turn_speed,
                min(self.unstick_turn_speed, self.last_cmd.angular.z),
            )

        period_ns = int(self.unstick_toggle_period * 1_000_000_000)
        direction = 1.0 if (now_ns // period_ns) % 2 == 0 else -1.0
        return direction * self.unstick_turn_speed

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds

    def side_escape_direction(self, left_min, right_min):
        left_ratio = 0.0
        right_ratio = 0.0

        if left_min < self.side_emergency_release_distance:
            span = max(
                0.01,
                self.side_emergency_release_distance - self.side_emergency_distance,
            )
            left_ratio = min(
                1.0,
                (self.side_emergency_release_distance - left_min) / span,
            )
        if right_min < self.side_emergency_release_distance:
            span = max(
                0.01,
                self.side_emergency_release_distance - self.side_emergency_distance,
            )
            right_ratio = min(
                1.0,
                (self.side_emergency_release_distance - right_min) / span,
            )

        if left_ratio <= 0.0 and right_ratio <= 0.0:
            return 0.0
        if left_ratio > right_ratio:
            return -left_ratio
        if right_ratio > left_ratio:
            return right_ratio
        return 0.0

    def centering_bias(self, left_min, right_min):
        if not (math.isfinite(left_min) and math.isfinite(right_min)):
            return 0.0
        if left_min > self.corridor_balance_distance or right_min > self.corridor_balance_distance:
            return 0.0

        balance_error = left_min - right_min
        if abs(balance_error) <= self.corridor_balance_deadband:
            return 0.0

        raw_bias = self.corridor_centering_gain * balance_error
        return max(
            -self.max_centering_angular_bias,
            min(self.max_centering_angular_bias, raw_bias),
        )

    def scaled_limit(self, distance, stop_distance, release_distance, max_speed, min_speed):
        if not math.isfinite(distance):
            return max_speed
        if distance <= stop_distance:
            return min_speed
        if distance >= release_distance:
            return max_speed

        span = max(0.01, release_distance - stop_distance)
        ratio = (distance - stop_distance) / span
        return min_speed + ratio * max(0.0, max_speed - min_speed)

    def clamp_angular(self, angular):
        return max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular),
        )

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
        self.last_scan_time_ns = self.get_clock().now().nanoseconds

    def publish_safe_cmd(self):
        now_ns = self.get_clock().now().nanoseconds
        cmd_age = (now_ns - self.last_cmd_time_ns) / 1_000_000_000.0
        if self.last_cmd_time_ns == 0 or cmd_age > self.cmd_timeout:
            self.publisher.publish(Twist())
            self.was_backing = False
            return

        scan_age = (now_ns - self.last_scan_time_ns) / 1_000_000_000.0
        front_min = self.front_min
        left_min = self.left_min
        right_min = self.right_min
        if self.last_scan_time_ns == 0 or scan_age > self.scan_timeout:
            front_min = math.inf
            left_min = math.inf
            right_min = math.inf

        moving_forward = self.last_cmd.linear.x > 0.01
        turning_without_drive = (
            abs(self.last_cmd.linear.x) <= 0.01
            and abs(self.last_cmd.linear.y) <= 0.01
            and abs(self.last_cmd.angular.z) > 0.05
        )
        side_escape_direction = self.side_escape_direction(left_min, right_min)
        centering_bias = self.centering_bias(left_min, right_min)
        min_side = min(left_min, right_min)
        max_side = max(left_min, right_min)
        side_stop = min_side < self.side_stop_distance
        side_risk = min_side < self.side_release_distance
        side_emergency = min_side < self.side_emergency_distance
        corridor_mode = (
            moving_forward
            and math.isfinite(left_min)
            and math.isfinite(right_min)
            and front_min > self.corridor_front_clear_distance
            and max_side < self.corridor_balance_distance
        )
        turning_toward_wall = (
            left_min < right_min and self.last_cmd.angular.z > 0.05
        ) or (
            right_min < left_min and self.last_cmd.angular.z < -0.05
        )

        if moving_forward and front_min < self.stop_distance:
            self.backup_until_ns = max(
                self.backup_until_ns,
                now_ns + int(self.backup_duration * 1_000_000_000),
            )

        if now_ns < self.backup_until_ns:
            twist = Twist()
            twist.linear.x = self.backup_speed
            twist.angular.z = self.unstick_angular(now_ns)
            self.publisher.publish(twist)
            if not self.was_backing:
                self.get_logger().warn(
                    f'Obstacle {front_min:.2f} m ahead, backing up with steering.'
                )
                self.was_backing = True
            return

        if moving_forward and side_emergency:
            twist = Twist()
            twist.linear.x = min(
                self.last_cmd.linear.x,
                self.scaled_limit(
                    min_side,
                    self.side_emergency_distance,
                    self.side_emergency_release_distance,
                    self.side_creep_speed,
                    0.0,
                ),
            )
            twist.linear.y = self.last_cmd.linear.y
            twist.angular.z = self.clamp_angular(
                self.last_cmd.angular.z
                + (side_escape_direction * self.wall_escape_turn_speed)
                + centering_bias
            )
            self.publisher.publish(twist)
            self.was_backing = False
            return

        if moving_forward and front_min < self.release_distance:
            twist = Twist()
            twist.linear.x = min(
                self.last_cmd.linear.x,
                self.scaled_limit(
                    front_min,
                    self.stop_distance,
                    self.release_distance,
                    self.creep_speed,
                    self.turning_drive_speed,
                ),
            )
            twist.linear.y = self.last_cmd.linear.y
            twist.angular.z = (
                self.last_cmd.angular.z
                if abs(self.last_cmd.angular.z) > 0.05
                else self.unstick_angular(now_ns)
            )
            twist.angular.z += centering_bias
            self.publisher.publish(twist)
            self.was_backing = False
            return

        if moving_forward and corridor_mode:
            twist = Twist()
            twist.linear.x = self.last_cmd.linear.x
            twist.linear.y = self.last_cmd.linear.y
            twist.angular.z = self.clamp_angular(
                self.last_cmd.angular.z + centering_bias
            )
            self.publisher.publish(twist)
            self.was_backing = False
            return

        if moving_forward and side_risk and turning_toward_wall:
            twist = Twist()
            twist.linear.x = min(
                self.last_cmd.linear.x,
                self.scaled_limit(
                    min_side,
                    self.side_stop_distance,
                    self.side_release_distance,
                    self.side_creep_speed,
                    self.turning_drive_speed,
                ),
            )
            twist.linear.y = self.last_cmd.linear.y
            twist.angular.z = self.clamp_angular(
                self.last_cmd.angular.z + centering_bias
            )
            self.publisher.publish(twist)
            self.was_backing = False
            return

        if moving_forward and abs(centering_bias) > 0.01:
            twist = Twist()
            twist.linear.x = self.last_cmd.linear.x
            twist.linear.y = self.last_cmd.linear.y
            twist.angular.z = self.clamp_angular(
                self.last_cmd.angular.z + centering_bias
            )
            self.publisher.publish(twist)
            self.was_backing = False
            return

        if turning_without_drive:
            twist = Twist()
            if self.holonomic_drive:
                if side_emergency and turning_toward_wall and side_escape_direction != 0.0:
                    twist.angular.z = side_escape_direction * self.wall_escape_turn_speed
                else:
                    twist.angular.z = self.clamp_angular(self.last_cmd.angular.z)
            elif front_min < self.stop_distance:
                twist.linear.x = self.backup_speed
                twist.angular.z = self.unstick_angular(now_ns)
            elif side_emergency and turning_toward_wall and side_escape_direction != 0.0:
                twist.linear.x = 0.0
                twist.angular.z = side_escape_direction * self.wall_escape_turn_speed
            else:
                # Ackermann steering needs a little forward motion to build heading change.
                if front_min < self.release_distance or side_emergency:
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = self.turning_drive_speed
                twist.angular.z = self.clamp_angular(
                    self.unstick_angular(now_ns) + centering_bias
                )
            self.publisher.publish(twist)
            self.was_backing = False
            return

        self.was_backing = False
        self.publisher.publish(self.last_cmd)


def main():
    rclpy.init()
    node = CollisionSafetyNode()
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
