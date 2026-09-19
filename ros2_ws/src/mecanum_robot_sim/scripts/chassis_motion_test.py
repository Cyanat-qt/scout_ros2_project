#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(start, end):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ChassisMotionTest(Node):
    def __init__(self):
        super().__init__('chassis_motion_test')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', math.pi / 2.0)
        self.declare_parameter('duration_sec', 2.0)
        self.declare_parameter('settle_sec', 0.3)
        self.declare_parameter('odom_timeout_sec', 8.0)

        self.latest_pose = None
        self.sim_time = None
        self.publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            10,
        )
        self.create_subscription(Clock, '/clock', self.clock_callback, clock_qos)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = (position.x, position.y, yaw)

    def clock_callback(self, msg):
        self.sim_time = stamp_to_sec(msg.clock)

    def motion_time(self, use_sim_time):
        if use_sim_time and self.sim_time is not None:
            return self.sim_time
        return time.monotonic()

    def wait_for_odom(self):
        timeout = float(self.get_parameter('odom_timeout_sec').value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.latest_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_pose is None:
            raise RuntimeError('No odometry received on /odom')

    def stop(self, duration=None):
        settle_sec = float(self.get_parameter('settle_sec').value) if duration is None else duration
        use_sim_time = self.sim_time is not None
        deadline = self.motion_time(use_sim_time) + settle_sec
        wall_deadline = time.monotonic() + max(3.0, settle_sec * 5.0)
        stop = Twist()
        while (
            rclpy.ok()
            and self.motion_time(use_sim_time) < deadline
            and time.monotonic() < wall_deadline
        ):
            self.publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def hold_command(self, label, vx=0.0, vy=0.0, wz=0.0):
        duration = float(self.get_parameter('duration_sec').value)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        use_sim_time = self.sim_time is not None
        deadline = self.motion_time(use_sim_time) + duration
        wall_deadline = time.monotonic() + max(10.0, duration * 8.0)
        while (
            rclpy.ok()
            and self.motion_time(use_sim_time) < deadline
            and time.monotonic() < wall_deadline
        ):
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.stop()
        self.print_pose(label)

    def rotate_clockwise_180(self, label, angular_speed):
        start_yaw = self.latest_pose[2]
        target_yaw = normalize_angle(start_yaw - math.pi)
        tolerance = math.radians(3.0)
        max_duration = 8.0
        max_speed = abs(angular_speed)
        min_speed = 0.16
        heading_gain = 1.35
        twist = Twist()

        use_sim_time = self.sim_time is not None
        deadline = self.motion_time(use_sim_time) + max_duration
        wall_deadline = time.monotonic() + max(20.0, max_duration * 8.0)
        previous_yaw = self.latest_pose[2]
        cumulative_delta = 0.0
        stable_samples = 0

        while (
            rclpy.ok()
            and self.motion_time(use_sim_time) < deadline
            and time.monotonic() < wall_deadline
        ):
            current_yaw = self.latest_pose[2]
            heading_error = shortest_angle_delta(current_yaw, target_yaw)
            if abs(heading_error) <= tolerance:
                stable_samples += 1
                if stable_samples >= 6:
                    break
            else:
                stable_samples = 0

            command = max(-max_speed, min(max_speed, heading_gain * heading_error))
            if abs(command) < min_speed and abs(heading_error) > tolerance:
                command = math.copysign(min_speed, command)
            twist.angular.z = command
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            current_yaw = self.latest_pose[2]
            cumulative_delta += shortest_angle_delta(previous_yaw, current_yaw)
            previous_yaw = current_yaw
            time.sleep(0.03)

        self.stop()
        self.print_pose(label)
        return math.degrees(cumulative_delta)

    def print_pose(self, label):
        x, y, yaw = self.latest_pose
        print(f'{label}: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} deg', flush=True)

    def run(self):
        self.wait_for_odom()
        self.stop(0.6)
        self.print_pose('start')

        linear_speed = float(self.get_parameter('linear_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)
        start_x, start_y, start_yaw = self.latest_pose

        self.hold_command('forward_2s', vx=linear_speed)
        forward_x, forward_y, forward_yaw = self.latest_pose

        self.hold_command('left_strafe_2s', vy=linear_speed)
        strafe_x, strafe_y, strafe_yaw = self.latest_pose

        yaw_delta = self.rotate_clockwise_180('cw_180', angular_speed)
        rotate_x, rotate_y, rotate_yaw = self.latest_pose

        forward_distance = forward_x - start_x
        strafe_distance = strafe_y - forward_y

        failures = []
        if forward_distance < 0.25:
            failures.append(f'forward distance too small: {forward_distance:.3f} m')
        if abs(forward_y - start_y) > 0.10:
            failures.append(f'forward drift too large: {forward_y - start_y:.3f} m')
        if strafe_distance < 0.25:
            failures.append(f'left strafe distance too small: {strafe_distance:.3f} m')
        if abs(strafe_x - forward_x) > 0.10:
            failures.append(f'left strafe x drift too large: {strafe_x - forward_x:.3f} m')
        final_yaw_delta = math.degrees(shortest_angle_delta(strafe_yaw, rotate_yaw))
        if abs(abs(final_yaw_delta) - 180.0) > 12.0 or yaw_delta > 20.0:
            failures.append(
                'clockwise rotation not near 180 deg: '
                f'final_delta={final_yaw_delta:.1f} deg cumulative={yaw_delta:.1f} deg'
            )
        if math.hypot(rotate_x - strafe_x, rotate_y - strafe_y) > 0.12:
            failures.append('rotation translated too much')

        if failures:
            print('RESULT: FAIL', flush=True)
            for failure in failures:
                print(f'- {failure}', flush=True)
            return 1

        print('RESULT: PASS', flush=True)
        return 0


def main():
    rclpy.init()
    node = ChassisMotionTest()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        node.stop(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
