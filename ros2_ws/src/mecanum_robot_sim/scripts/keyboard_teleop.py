#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


HELP_TEXT = """
Keyboard teleop for mecanum_robot_sim

Moving:
       w
   a   k   d
       s

w/s     : forward/backward
a/d     : steer left/right
k/space : stop
q/z     : increase/decrease max linear speed
e/c     : increase/decrease max angular speed
release : auto stop shortly after key repeat stops
CTRL-C to quit
"""

PUBLISH_PERIOD = 0.05
COMMAND_TIMEOUT = 0.30

SPEED_BINDINGS = {
    'q': (1.10, 1.00),
    'z': (0.90, 1.00),
    'e': (1.00, 1.10),
    'c': (1.00, 0.90),
}


def open_keyboard_stream():
    try:
        stream = open('/dev/tty', 'r')
        if stream.isatty():
            return stream
        stream.close()
    except OSError:
        pass

    if sys.stdin.isatty():
        return sys.stdin

    return None


def read_key(stream, timeout):
    ready, _, _ = select.select([stream], [], [], timeout)
    if ready:
        return stream.read(1)
    return ''


def make_twist(x_direction, y_direction, yaw_direction, linear_speed, angular_speed):
    twist = Twist()
    twist.linear.x = x_direction * linear_speed
    twist.linear.y = y_direction * linear_speed
    twist.angular.z = yaw_direction * angular_speed
    return twist


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    stream = open_keyboard_stream()
    if stream is None:
        node.get_logger().error('No keyboard TTY available. Run this node in a terminal.')
        rclpy.shutdown()
        return

    settings = termios.tcgetattr(stream)
    linear_speed = 0.35
    angular_speed = 2.20
    x_direction = 0.0
    y_direction = 0.0
    yaw_direction = 0.0
    last_motion_time = 0.0
    last_publish_time = 0.0

    print(HELP_TEXT)
    print(f'linear={linear_speed:.2f} m/s, angular={angular_speed:.2f} rad/s')

    try:
        tty.setraw(stream.fileno())
        while rclpy.ok():
            now = time.monotonic()
            key = read_key(stream, 0.01)
            key = key.lower()

            if key == 'w':
                x_direction = 1.0
                y_direction = 0.0
                yaw_direction = 0.0
                last_motion_time = now
            elif key == 's':
                x_direction = -1.0
                y_direction = 0.0
                yaw_direction = 0.0
                last_motion_time = now
            elif key == 'a':
                if x_direction == 0.0:
                    x_direction = 1.0
                y_direction = 0.0
                yaw_direction = 1.0
                last_motion_time = now
            elif key == 'd':
                if x_direction == 0.0:
                    x_direction = 1.0
                y_direction = 0.0
                yaw_direction = -1.0
                last_motion_time = now
            elif key in ('k', ' '):
                x_direction = 0.0
                y_direction = 0.0
                yaw_direction = 0.0
                last_motion_time = 0.0
            elif key in SPEED_BINDINGS:
                linear_scale, angular_scale = SPEED_BINDINGS[key]
                linear_speed = max(0.05, min(1.50, linear_speed * linear_scale))
                angular_speed = max(0.10, min(5.00, angular_speed * angular_scale))
                print(f'\rlinear={linear_speed:.2f} m/s, angular={angular_speed:.2f} rad/s   ')
            elif key == '\x03':
                break

            if last_motion_time and now - last_motion_time > COMMAND_TIMEOUT:
                x_direction = 0.0
                y_direction = 0.0
                yaw_direction = 0.0
                last_motion_time = 0.0

            if now - last_publish_time >= PUBLISH_PERIOD:
                publisher.publish(
                    make_twist(
                        x_direction,
                        y_direction,
                        yaw_direction,
                        linear_speed,
                        angular_speed,
                    )
                )
                last_publish_time = now
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        publisher.publish(Twist())
        termios.tcsetattr(stream, termios.TCSADRAIN, settings)
        if stream is not sys.stdin:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
