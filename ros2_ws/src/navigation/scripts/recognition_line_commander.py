#!/usr/bin/env python3

import heapq
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Trigger


STATUS_SUCCEEDED = 'SUCCEEDED'
STATUS_ABORTED = 'ABORTED'
STATUS_CANCELED = 'CANCELED'


def clamp(value, low, high):
    return max(low, min(high, value))


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


class RecognitionLineCommander(Node):
    def __init__(self, **kwargs):
        super().__init__('recognition_line_commander', **kwargs)

        self.declare_parameter('routes_file', '')
        self.declare_parameter('route_id_topic', '/recognition_route_id')
        self.declare_parameter('route_status_topic', '/recognition_route_status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('path_topic', '/recognition_line_path')
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 2.10)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        self.declare_parameter('service_prefix', '/go_recognition_')
        self.declare_parameter('cancel_service', '/cancel_recognition_route')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('max_linear_speed', 0.55)
        self.declare_parameter('min_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.95)
        self.declare_parameter('linear_gain', 0.9)
        self.declare_parameter('heading_gain', 2.3)
        self.declare_parameter('cross_track_gain', 1.1)
        self.declare_parameter('side_wall_gain', 0.7)
        self.declare_parameter('max_linear_accel', 1.8)
        self.declare_parameter('max_linear_decel', 2.4)
        self.declare_parameter('max_angular_accel', 4.0)
        self.declare_parameter('slowdown_distance', 0.42)
        self.declare_parameter('corner_slowdown_speed', 0.34)
        self.declare_parameter('waypoint_tolerance', 0.08)
        self.declare_parameter('final_tolerance', 0.10)
        self.declare_parameter('yaw_tolerance', 0.22)
        self.declare_parameter('pre_rotate_angle', 0.45)
        self.declare_parameter('obstacle_stop_distance', 0.18)
        self.declare_parameter('obstacle_slow_distance', 0.32)
        self.declare_parameter('side_keep_distance', 0.18)
        self.declare_parameter('blocked_timeout_sec', 2.0)
        self.declare_parameter('scan_timeout_sec', 0.8)
        self.declare_parameter('use_graph_planner', True)
        self.declare_parameter('graph_margin', 0.12)

        routes_file_text = str(self.get_parameter('routes_file').value)
        self.routes_file = Path(routes_file_text).expanduser() if routes_file_text else None
        self.route_id_topic = str(self.get_parameter('route_id_topic').value)
        self.route_status_topic = str(self.get_parameter('route_status_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.map_to_odom_x = float(self.get_parameter('map_to_odom_x').value)
        self.map_to_odom_y = float(self.get_parameter('map_to_odom_y').value)
        self.map_to_odom_yaw = float(self.get_parameter('map_to_odom_yaw').value)
        self.service_prefix = str(self.get_parameter('service_prefix').value)
        self.cancel_service = str(self.get_parameter('cancel_service').value)
        control_rate = max(5.0, float(self.get_parameter('control_rate').value))

        self.max_linear_speed = abs(float(self.get_parameter('max_linear_speed').value))
        self.min_linear_speed = abs(float(self.get_parameter('min_linear_speed').value))
        self.max_angular_speed = abs(float(self.get_parameter('max_angular_speed').value))
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.cross_track_gain = float(self.get_parameter('cross_track_gain').value)
        self.side_wall_gain = float(self.get_parameter('side_wall_gain').value)
        self.max_linear_accel = abs(float(self.get_parameter('max_linear_accel').value))
        self.max_linear_decel = abs(float(self.get_parameter('max_linear_decel').value))
        self.max_angular_accel = abs(float(self.get_parameter('max_angular_accel').value))
        self.slowdown_distance = abs(float(self.get_parameter('slowdown_distance').value))
        self.corner_slowdown_speed = abs(float(self.get_parameter('corner_slowdown_speed').value))
        self.waypoint_tolerance = abs(float(self.get_parameter('waypoint_tolerance').value))
        self.final_tolerance = abs(float(self.get_parameter('final_tolerance').value))
        self.yaw_tolerance = abs(float(self.get_parameter('yaw_tolerance').value))
        self.pre_rotate_angle = abs(float(self.get_parameter('pre_rotate_angle').value))
        self.obstacle_stop_distance = abs(float(self.get_parameter('obstacle_stop_distance').value))
        self.obstacle_slow_distance = abs(float(self.get_parameter('obstacle_slow_distance').value))
        self.side_keep_distance = abs(float(self.get_parameter('side_keep_distance').value))
        self.blocked_timeout_sec = max(0.2, float(self.get_parameter('blocked_timeout_sec').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.use_graph_planner = bool(self.get_parameter('use_graph_planner').value)
        self.graph_margin = max(0.0, float(self.get_parameter('graph_margin').value))

        self.frame_id = 'map'
        self.routes = {}
        self.active_route_id = None
        self.active_route_name = ''
        self.active_points = []
        self.active_index = 0
        self.segment_start = None
        self.current_pose = None
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_scan_time_ns = 0
        self.last_scan_warn_ns = 0
        self.blocked_since_ns = None
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.control_dt = 1.0 / control_rate

        self.load_routes()

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.route_status_topic, 10)
        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 1)
        self.create_subscription(Int32, self.route_id_topic, self.route_id_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_service(Trigger, self.cancel_service, self.cancel_callback)

        for route_id in sorted(self.routes.keys(), key=self.route_sort_key):
            service_name = f'{self.service_prefix}{route_id}'
            self.create_service(
                Trigger,
                service_name,
                lambda request, response, rid=route_id: self.route_service_callback(
                    request,
                    response,
                    rid,
                ),
            )

        self.create_timer(1.0 / control_rate, self.control_tick)

        route_ids = ', '.join(sorted(self.routes.keys(), key=self.route_sort_key))
        self.publish_status(f'line commander ready routes=[{route_ids}]')
        self.get_logger().info(
            f'Recognition line commander ready. {self.route_id_topic} -> {self.cmd_vel_topic}, '
            f'path={self.path_topic}'
        )

    def route_sort_key(self, route_id):
        try:
            return int(route_id)
        except ValueError:
            return route_id

    def load_routes(self):
        if self.routes_file is None:
            raise RuntimeError('routes_file parameter is empty')
        if not self.routes_file.exists():
            raise RuntimeError(f'routes_file does not exist: {self.routes_file}')

        with self.routes_file.open('r', encoding='utf-8') as infp:
            config = yaml.safe_load(infp) or {}

        self.frame_id = str(config.get('frame_id', 'map'))
        raw_routes = config.get('routes', {})
        if not raw_routes:
            raise RuntimeError(f'No routes configured in {self.routes_file}')

        for route_id, route in raw_routes.items():
            route_id = str(route_id)
            waypoints = route.get('waypoints', [])
            if not waypoints:
                raise RuntimeError(f'Route {route_id} has no waypoints')
            self.routes[route_id] = {
                'name': str(route.get('name', route_id)),
                'description': str(route.get('description', '')),
                'waypoints': [self.normalize_waypoint(item) for item in waypoints],
            }

    def normalize_waypoint(self, item):
        return {
            'x': float(item['x']),
            'y': float(item['y']),
            'yaw': float(item.get('yaw', 0.0)),
        }

    def route_id_callback(self, msg):
        self.start_route(str(msg.data), source='topic')

    def route_service_callback(self, _request, response, route_id):
        accepted, message = self.start_route(route_id, source='service')
        response.success = accepted
        response.message = message
        return response

    def cancel_callback(self, _request, response):
        if self.active_route_id is None:
            response.success = False
            response.message = 'No active recognition line route to cancel.'
            return response
        route_id = self.active_route_id
        self.stop_robot()
        self.finish_route(route_id, STATUS_CANCELED, 'cancel requested')
        response.success = True
        response.message = f'Cancel requested for recognition route {route_id}.'
        return response

    def start_route(self, route_id, source):
        if route_id not in self.routes:
            message = f'Unknown recognition route id={route_id}'
            self.get_logger().error(message)
            self.publish_status(message)
            return False, message

        if self.current_pose is None:
            message = f'id={route_id} rejected: waiting for {self.odom_topic}'
            self.get_logger().warn(message)
            self.publish_status(message)
            return False, message

        route = self.routes[route_id]
        route_points = list(route['waypoints'])
        planned_points = self.make_plan(route_points)
        if not planned_points:
            message = f'id={route_id} stopped: {STATUS_ABORTED} no safe line plan'
            self.get_logger().error(message)
            self.publish_status(message)
            return False, message

        self.active_route_id = route_id
        self.active_route_name = route['name']
        self.active_points = planned_points
        self.active_index = 0
        self.segment_start = (self.current_pose[0], self.current_pose[1])
        self.blocked_since_ns = None

        message = (
            f'Start recognition line route id={route_id} name={route["name"]} '
            f'from={source} waypoints={len(planned_points)}'
        )
        self.get_logger().info(message)
        self.publish_status(message)
        self.publish_path()
        return True, message

    def make_plan(self, route_points):
        if not self.use_graph_planner or len(route_points) > 1:
            return route_points

        start = (self.current_pose[0], self.current_pose[1])
        goal = (route_points[-1]['x'], route_points[-1]['y'])
        graph_points = self.graph_nodes()
        path = self.shortest_visible_path(start, goal, graph_points)
        if not path:
            return []

        planned = []
        final_yaw = float(route_points[-1].get('yaw', 0.0))
        for index, (x, y) in enumerate(path[1:], start=1):
            yaw = final_yaw if index == len(path) - 1 else 0.0
            planned.append({'x': x, 'y': y, 'yaw': yaw})
        return self.remove_redundant_points(planned)

    def graph_nodes(self):
        return [
            (-1.35, 0.35),
            (0.00, 0.35),
            (1.30, 0.35),
            (1.30, 1.20),
            (1.30, 2.25),
            (1.30, 2.85),
            (1.30, 3.75),
            (0.65, 3.75),
            (-0.25, 3.75),
            (-1.35, 3.75),
            (-1.35, 2.85),
            (-1.35, 1.80),
            (-1.35, 0.85),
        ]

    def inflated_obstacles(self):
        margin = self.graph_margin
        rectangles = [
            (-0.50, 1.00, 0.65, 2.15),
            (-0.50, 0.50, 2.15, 2.65),
            (-0.90, -0.50, 2.20, 2.60),
            (0.00, 0.50, 2.65, 3.45),
        ]
        return [
            (xmin - margin, xmax + margin, ymin - margin, ymax + margin)
            for xmin, xmax, ymin, ymax in rectangles
        ]

    def segment_is_clear(self, a, b):
        ax, ay = a
        bx, by = b
        length = math.hypot(bx - ax, by - ay)
        steps = max(2, int(length / 0.03))
        obstacles = self.inflated_obstacles()
        for index in range(1, steps):
            t = index / steps
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            for xmin, xmax, ymin, ymax in obstacles:
                if xmin <= x <= xmax and ymin <= y <= ymax:
                    return False
        return True

    def shortest_visible_path(self, start, goal, graph_points):
        nodes = [start, goal] + graph_points
        node_count = len(nodes)
        edges = [[] for _ in range(node_count)]
        for i in range(node_count):
            for j in range(i + 1, node_count):
                if self.segment_is_clear(nodes[i], nodes[j]):
                    distance = self.distance(nodes[i], nodes[j])
                    edges[i].append((j, distance))
                    edges[j].append((i, distance))

        distances = [math.inf] * node_count
        previous = [-1] * node_count
        distances[0] = 0.0
        queue = [(0.0, 0)]
        while queue:
            current_distance, node = heapq.heappop(queue)
            if node == 1:
                break
            if current_distance > distances[node]:
                continue
            for neighbor, weight in edges[node]:
                new_distance = current_distance + weight
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = node
                    heapq.heappush(queue, (new_distance, neighbor))

        if previous[1] < 0:
            return []

        path = []
        node = 1
        while node >= 0:
            path.append(nodes[node])
            node = previous[node]
        path.reverse()
        return path

    def remove_redundant_points(self, points):
        if len(points) <= 2:
            return points
        cleaned = []
        for point in points:
            if cleaned:
                prev = cleaned[-1]
                if math.hypot(point['x'] - prev['x'], point['y'] - prev['y']) < 0.05:
                    cleaned[-1] = point
                    continue
            cleaned.append(point)
        return cleaned

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
        self.current_pose = (map_x, map_y, map_yaw)

    def scan_callback(self, msg):
        front = []
        left = []
        right = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                if abs(angle) <= math.radians(28.0):
                    front.append(distance)
                elif math.radians(55.0) <= angle <= math.radians(115.0):
                    left.append(distance)
                elif math.radians(-115.0) <= angle <= math.radians(-55.0):
                    right.append(distance)
            angle += msg.angle_increment
        self.front_min = min(front) if front else math.inf
        self.left_min = min(left) if left else math.inf
        self.right_min = min(right) if right else math.inf
        self.last_scan_time_ns = self.get_clock().now().nanoseconds

    def control_tick(self):
        if self.active_route_id is None:
            return
        if self.current_pose is None:
            self.stop_robot()
            return

        now_ns = self.get_clock().now().nanoseconds
        route_id = self.active_route_id
        scan_stale = (
            self.last_scan_time_ns == 0
            or (now_ns - self.last_scan_time_ns) / 1_000_000_000.0 > self.scan_timeout_sec
        )
        if scan_stale:
            self.stop_robot()
            if now_ns - self.last_scan_warn_ns > 1_000_000_000:
                self.last_scan_warn_ns = now_ns
                self.publish_status(f'id={route_id} waiting for fresh scan: {self.scan_topic}')
            return

        target = self.active_points[self.active_index]
        x, y, yaw = self.current_pose
        dx = target['x'] - x
        dy = target['y'] - y
        distance_to_target = math.hypot(dx, dy)
        final_waypoint = self.active_index == len(self.active_points) - 1
        tolerance = self.final_tolerance if final_waypoint else self.waypoint_tolerance

        if distance_to_target <= tolerance:
            if final_waypoint:
                self.stop_robot()
                self.finish_route(route_id, STATUS_SUCCEEDED)
                return

            self.active_index += 1
            self.segment_start = (x, y)
            self.blocked_since_ns = None
            self.publish_status(
                f'id={route_id} waypoint={self.active_index + 1}/{len(self.active_points)} '
                f'x={self.active_points[self.active_index]["x"]:.2f} '
                f'y={self.active_points[self.active_index]["y"]:.2f}'
            )
            return

        if self.front_min < self.obstacle_stop_distance:
            self.stop_robot()
            if self.blocked_since_ns is None:
                self.blocked_since_ns = now_ns
                self.get_logger().warn(
                    f'id={route_id} front obstacle {self.front_min:.2f} m, stop and wait.'
                )
            blocked_sec = (now_ns - self.blocked_since_ns) / 1_000_000_000.0
            if blocked_sec >= self.blocked_timeout_sec:
                self.finish_route(
                    route_id,
                    STATUS_ABORTED,
                    f'front obstacle {self.front_min:.2f} m',
                )
            return
        self.blocked_since_ns = None

        linear, angular = self.compute_control(target, distance_to_target, tolerance, final_waypoint)
        if self.front_min < self.obstacle_slow_distance:
            scale = clamp(
                (self.front_min - self.obstacle_stop_distance)
                / max(0.01, self.obstacle_slow_distance - self.obstacle_stop_distance),
                0.2,
                1.0,
            )
            linear *= scale
        angular += self.side_wall_correction()
        angular = clamp(angular, -self.max_angular_speed, self.max_angular_speed)
        self.publish_twist(linear, angular)

    def compute_control(self, target, distance_to_target, tolerance, final_waypoint):
        x, y, yaw = self.current_pose
        sx, sy = self.segment_start if self.segment_start is not None else (x, y)
        vx = target['x'] - sx
        vy = target['y'] - sy
        segment_length = math.hypot(vx, vy)
        if segment_length < 0.05:
            line_heading = math.atan2(target['y'] - y, target['x'] - x)
            cross_track = 0.0
        else:
            ux = vx / segment_length
            uy = vy / segment_length
            wx = x - sx
            wy = y - sy
            cross_track = ux * wy - uy * wx
            line_heading = math.atan2(uy, ux)

        target_heading = math.atan2(target['y'] - y, target['x'] - x)
        line_error = normalize_angle(line_heading - yaw)
        target_error = normalize_angle(target_heading - yaw)
        heading_error = target_error if distance_to_target < 0.22 else line_error

        angular = self.heading_gain * heading_error - self.cross_track_gain * cross_track
        angular = clamp(angular, -self.max_angular_speed, self.max_angular_speed)

        if abs(heading_error) > self.pre_rotate_angle and distance_to_target > 0.20:
            return min(self.min_linear_speed, self.corner_slowdown_speed), angular

        linear = clamp(
            self.linear_gain * distance_to_target,
            min(0.06, self.min_linear_speed),
            self.max_linear_speed,
        )

        remaining = max(0.0, distance_to_target - tolerance)
        brake_speed = math.sqrt(max(0.0, 2.0 * self.max_linear_decel * remaining))
        if distance_to_target < self.slowdown_distance:
            distance_scale = clamp(distance_to_target / max(0.01, self.slowdown_distance), 0.28, 1.0)
            linear = min(linear, self.max_linear_speed * distance_scale)
        linear = min(linear, brake_speed)

        if not final_waypoint and self.is_sharp_corner_ahead():
            corner_scale = clamp(distance_to_target / max(0.01, self.slowdown_distance), 0.45, 1.0)
            linear = min(linear, self.corner_slowdown_speed * corner_scale)

        turn_scale = clamp(1.0 - abs(heading_error) / 1.0, 0.38, 1.0)
        linear *= turn_scale
        if remaining > 0.015:
            linear = max(0.04, linear)
        return linear, angular

    def is_sharp_corner_ahead(self):
        if self.active_index >= len(self.active_points) - 1:
            return False
        if self.segment_start is None:
            return False
        current = self.active_points[self.active_index]
        nxt = self.active_points[self.active_index + 1]
        ax = current['x'] - self.segment_start[0]
        ay = current['y'] - self.segment_start[1]
        bx = nxt['x'] - current['x']
        by = nxt['y'] - current['y']
        if math.hypot(ax, ay) < 0.05 or math.hypot(bx, by) < 0.05:
            return False
        angle_a = math.atan2(ay, ax)
        angle_b = math.atan2(by, bx)
        return abs(normalize_angle(angle_b - angle_a)) > 0.65

    def side_wall_correction(self):
        correction = 0.0
        if self.left_min < self.side_keep_distance:
            correction -= self.side_wall_gain * (self.side_keep_distance - self.left_min)
        if self.right_min < self.side_keep_distance:
            correction += self.side_wall_gain * (self.side_keep_distance - self.right_min)
        return correction

    def publish_twist(self, linear_x, angular_z):
        twist = Twist()
        linear_x = self.limit_rate(
            float(linear_x),
            self.prev_linear,
            self.max_linear_accel,
            self.max_linear_decel,
        )
        angular_z = self.limit_rate(
            float(angular_z),
            self.prev_angular,
            self.max_angular_accel,
            self.max_angular_accel,
        )
        self.prev_linear = linear_x
        self.prev_angular = angular_z
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.cmd_pub.publish(Twist())

    def limit_rate(self, target, current, accel_limit, decel_limit):
        limit = accel_limit if abs(target) > abs(current) else decel_limit
        max_delta = max(0.0, limit) * self.control_dt
        return current + clamp(target - current, -max_delta, max_delta)

    def finish_route(self, route_id, status, reason=''):
        message = f'id={route_id} finished: {status}' if status == STATUS_SUCCEEDED else f'id={route_id} stopped: {status}'
        if reason:
            message = f'{message} {reason}'
        if status == STATUS_SUCCEEDED:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        self.publish_status(message)
        if self.active_route_id == route_id:
            self.active_route_id = None
            self.active_route_name = ''
            self.active_points = []
            self.active_index = 0
            self.segment_start = None
            self.blocked_since_ns = None
        self.publish_path(clear=True)

    def publish_status(self, message):
        self.status_pub.publish(String(data=message))

    def publish_path(self, clear=False):
        msg = PathMsg()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        if not clear:
            if self.current_pose is not None:
                msg.poses.append(self.make_pose_stamped(self.current_pose[0], self.current_pose[1], self.current_pose[2]))
            for point in self.active_points:
                msg.poses.append(self.make_pose_stamped(point['x'], point['y'], point.get('yaw', 0.0)))
        self.path_pub.publish(msg)

    def make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        half = yaw * 0.5
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        return pose

    def distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    rclpy.init()
    node = RecognitionLineCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
