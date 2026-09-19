#!/usr/bin/env python3

import math
from pathlib import Path
from typing import List
from typing import Optional
from typing import Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Trigger


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


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


class RecognitionRouteCommander(Node):
    def __init__(self):
        super().__init__('recognition_route_commander')

        self.declare_parameter('routes_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('route_id_topic', '/recognition_route_id')
        self.declare_parameter('route_status_topic', '/recognition_route_status')
        self.declare_parameter('nav_action_name', '/navigate_through_poses')
        self.declare_parameter('pose_nav_action_name', '/navigate_to_pose')
        self.declare_parameter('strict_follow_waypoints', False)
        self.declare_parameter('service_prefix', '/go_recognition_')
        self.declare_parameter('cancel_service', '/cancel_recognition_route')
        self.declare_parameter('cancel_on_new_goal', True)
        self.declare_parameter('retry_on_abort', True)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('retry_delay_sec', 1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        self.declare_parameter('heading_align_yaw_tolerance', 0.12)
        self.declare_parameter('alignment_send_delay_sec', 0.15)
        self.declare_parameter('alignment_rate_hz', 20.0)
        self.declare_parameter('alignment_timeout_sec', 6.0)
        self.declare_parameter('alignment_angular_gain', 1.4)
        self.declare_parameter('alignment_min_angular_speed', 0.12)
        self.declare_parameter('alignment_max_angular_speed', 0.85)

        routes_file_text = str(self.get_parameter('routes_file').value)
        self.routes_file = Path(routes_file_text).expanduser() if routes_file_text else None
        map_yaml_text = str(self.get_parameter('map_yaml').value)
        self.map_yaml = Path(map_yaml_text).expanduser() if map_yaml_text else None
        self.route_id_topic = self.get_parameter('route_id_topic').value
        self.route_status_topic = self.get_parameter('route_status_topic').value
        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.pose_nav_action_name = self.get_parameter('pose_nav_action_name').value
        self.strict_follow_waypoints = bool(self.get_parameter('strict_follow_waypoints').value)
        self.service_prefix = self.get_parameter('service_prefix').value
        self.cancel_service = self.get_parameter('cancel_service').value
        self.cancel_on_new_goal = bool(self.get_parameter('cancel_on_new_goal').value)
        self.retry_on_abort = bool(self.get_parameter('retry_on_abort').value)
        self.max_retries = max(0, int(self.get_parameter('max_retries').value))
        self.retry_delay_sec = max(0.1, float(self.get_parameter('retry_delay_sec').value))
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_to_odom_x = float(self.get_parameter('map_to_odom_x').value)
        self.map_to_odom_y = float(self.get_parameter('map_to_odom_y').value)
        self.map_to_odom_yaw = float(self.get_parameter('map_to_odom_yaw').value)
        self.heading_align_yaw_tolerance = max(
            0.02, float(self.get_parameter('heading_align_yaw_tolerance').value)
        )
        self.alignment_send_delay_sec = max(
            0.05, float(self.get_parameter('alignment_send_delay_sec').value)
        )
        self.alignment_rate_hz = max(5.0, float(self.get_parameter('alignment_rate_hz').value))
        self.alignment_timeout_sec = max(
            0.5, float(self.get_parameter('alignment_timeout_sec').value)
        )
        self.alignment_angular_gain = max(
            0.1, float(self.get_parameter('alignment_angular_gain').value)
        )
        self.alignment_min_angular_speed = abs(
            float(self.get_parameter('alignment_min_angular_speed').value)
        )
        self.alignment_max_angular_speed = abs(
            float(self.get_parameter('alignment_max_angular_speed').value)
        )

        self.frame_id = 'map'
        self.routes = {}
        self.active_route_id = None
        self.active_goal_handle = None
        self.active_retry_count = 0
        self.active_poses = []
        self.active_behavior_tree = ''
        self.active_waypoint_index = 0
        self.retry_timer = None
        self.current_pose_map = None
        self.alignment_route_id = None
        self.alignment_announced = False
        self.pending_alignment_timer = None
        self.alignment_timer = None
        self.alignment_started_ns = 0
        self.active_use_pose_navigation = False
        self.map_origin = (0.0, 0.0)
        self.map_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_pixels: List[int] = []

        self.load_routes()
        self.load_occupancy_map()

        self.action_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.nav_action_name,
        )
        self.pose_action_client = ActionClient(
            self,
            NavigateToPose,
            self.pose_nav_action_name,
        )
        self.status_pub = self.create_publisher(String, self.route_status_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Int32, self.route_id_topic, self.route_id_callback, 10)
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

        route_ids = ', '.join(sorted(self.routes.keys(), key=self.route_sort_key))
        self.publish_status(f'ready routes=[{route_ids}]')
        self.get_logger().info(
            f'Recognition route commander ready. Topic: {self.route_id_topic}, '
            f'services: {self.service_prefix}<id>'
        )

    def route_sort_key(self, route_id):
        try:
            return int(route_id)
        except ValueError:
            return route_id

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
                'name': route.get('name', route_id),
                'description': route.get('description', ''),
                'frame_id': route.get('frame_id', self.frame_id),
                'behavior_tree': route.get('behavior_tree', ''),
                'align_heading_on_success': bool(route.get('align_heading_on_success', False)),
                'arrival_polygon': route.get('arrival_polygon', []),
                'arrival_announce_text': route.get('arrival_announce_text', ''),
                'waypoints': waypoints,
            }

    def load_occupancy_map(self):
        if self.map_yaml is None:
            self.get_logger().warn('map_yaml is empty; task zone raycast checks are disabled.')
            return
        if not self.map_yaml.exists():
            self.get_logger().warn(
                f'map_yaml does not exist; task zone raycast checks are disabled: {self.map_yaml}'
            )
            return

        with self.map_yaml.open('r', encoding='utf-8') as infp:
            config = yaml.safe_load(infp) or {}

        image_path = Path(str(config.get('image', '')))
        if not image_path.is_absolute():
            image_path = self.map_yaml.parent / image_path
        if not image_path.exists():
            self.get_logger().warn(
                f'map image does not exist; task zone raycast checks are disabled: {image_path}'
            )
            return

        self.map_resolution = float(config.get('resolution', 0.0))
        origin = config.get('origin', [0.0, 0.0, 0.0])
        self.map_origin = (float(origin[0]), float(origin[1]))
        self.map_width, self.map_height, self.map_pixels = self.read_pgm(image_path)
        self.get_logger().info(
            f'Loaded task occupancy map {image_path} '
            f'{self.map_width}x{self.map_height} res={self.map_resolution:.3f}'
        )

    def read_pgm(self, path):
        def next_token(stream):
            token = bytearray()
            while True:
                char = stream.read(1)
                if not char:
                    return None
                if char == b'#':
                    stream.readline()
                    continue
                if char.isspace():
                    if token:
                        return token.decode('ascii')
                    continue
                token.extend(char)

        with path.open('rb') as infp:
            magic = next_token(infp)
            if magic != 'P5':
                raise RuntimeError(f'Only binary PGM P5 is supported: {path}')
            width = int(next_token(infp))
            height = int(next_token(infp))
            max_value = int(next_token(infp))
            if max_value > 255:
                raise RuntimeError(f'Unsupported PGM max value {max_value}: {path}')
            pixels = list(infp.read(width * height))
            if len(pixels) != width * height:
                raise RuntimeError(f'PGM pixel count mismatch: {path}')
        return width, height, pixels

    def route_id_callback(self, msg):
        self.start_route(str(msg.data), source='topic')

    def route_service_callback(self, _request, response, route_id):
        accepted, message = self.start_route(route_id, source='service')
        response.success = accepted
        response.message = message
        return response

    def cancel_callback(self, _request, response):
        self.cancel_retry_timer()
        if self.active_route_id is None:
            response.success = False
            response.message = 'No active recognition route to cancel.'
            return response

        route_id = self.active_route_id
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.reset_active_route_state(route_id)
        response.success = True
        response.message = f'Cancel requested for recognition route {route_id}.'
        self.publish_status(f'cancel requested id={route_id}')
        return response

    def start_route(self, route_id, source, retry_count=0):
        if route_id not in self.routes:
            message = f'Unknown recognition route id={route_id}'
            self.get_logger().error(message)
            self.publish_status(message)
            return False, message

        if source != 'retry':
            self.cancel_retry_timer()

        if self.active_goal_handle is not None:
            if not self.cancel_on_new_goal:
                message = (
                    f'Route {self.active_route_id} is still active; '
                    f'ignore new route {route_id}.'
                )
                self.get_logger().warn(message)
                self.publish_status(message)
                return False, message
            self.active_goal_handle.cancel_goal_async()

        route = self.routes[route_id]
        poses = self.make_poses(route)
        use_pose_navigation = self.use_pose_navigation(route, poses)
        action_client = self.pose_action_client if use_pose_navigation else self.action_client
        action_name = self.pose_nav_action_name if use_pose_navigation else self.nav_action_name
        if not action_client.wait_for_server(timeout_sec=3.0):
            message = f'Nav2 action server is not available: {action_name}'
            self.get_logger().error(message)
            self.publish_status(message)
            return False, message

        self.active_route_id = route_id
        self.active_retry_count = retry_count
        self.active_poses = poses
        self.active_behavior_tree = route.get('behavior_tree', '')
        self.active_waypoint_index = 0
        self.active_use_pose_navigation = use_pose_navigation
        self.alignment_route_id = None
        self.alignment_announced = False
        self.cancel_pending_alignment_timer()
        route_name = route.get('name', route_id)
        message = (
            f'Start recognition route id={route_id} name={route_name} '
            f'from={source} waypoints={len(poses)} strict={self.strict_follow_waypoints} '
            f'pose_mode={use_pose_navigation}'
        )
        self.get_logger().info(message)
        self.publish_status(message)

        if use_pose_navigation:
            self.send_next_pose_goal()
            return True, message

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = self.active_behavior_tree
        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(
            lambda future, rid=route_id: self.goal_response_callback(future, rid)
        )
        return True, message

    def send_next_pose_goal(self):
        route_id = self.active_route_id
        index = self.active_waypoint_index
        if route_id is None or index >= len(self.active_poses):
            return

        pose = self.active_poses[index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = self.active_behavior_tree

        total = len(self.active_poses)
        self.publish_status(
            f'id={route_id} waypoint={index + 1}/{total} '
            f'x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}'
        )
        send_future = self.pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.pose_feedback_callback,
        )
        send_future.add_done_callback(
            lambda future, rid=route_id, waypoint_index=index: self.pose_goal_response_callback(
                future,
                rid,
                waypoint_index,
            )
        )

    def start_alignment_phase(self, route_id, route):
        if self.current_pose_map is None or not self.active_poses:
            return

        self.alignment_route_id = route_id
        self.alignment_announced = False
        self.publish_status(f'id={route_id} navigation succeeded, aligning heading')
        self.get_logger().info(
            f'Route {route_id} navigation succeeded; stop navigation and rotate in place.'
        )

        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        self.cancel_pending_alignment_timer()
        self.pending_alignment_timer = self.create_timer(
            self.alignment_send_delay_sec,
            lambda rid=route_id: self.start_direct_alignment(rid),
        )

    def start_direct_alignment(self, route_id):
        self.cancel_pending_alignment_timer()
        if self.active_route_id != route_id or self.current_pose_map is None or not self.active_poses:
            return

        target_yaw = self.pose_yaw(self.active_poses[-1])
        yaw_error = normalize_angle(target_yaw - self.current_pose_map[2])
        if abs(yaw_error) <= self.heading_align_yaw_tolerance:
            self.finish_arrival_success(route_id)
            return

        self.alignment_started_ns = self.get_clock().now().nanoseconds
        self.cancel_alignment_timer()
        self.alignment_timer = self.create_timer(
            1.0 / self.alignment_rate_hz,
            lambda rid=route_id: self.direct_alignment_tick(rid),
        )
        self.publish_status(f'id={route_id} direct visual heading alignment started')

    def direct_alignment_tick(self, route_id):
        if self.active_route_id != route_id or self.alignment_route_id != route_id:
            self.stop_robot()
            self.cancel_alignment_timer()
            return
        if self.current_pose_map is None or not self.active_poses:
            self.stop_robot()
            return

        elapsed = (self.get_clock().now().nanoseconds - self.alignment_started_ns) / 1e9
        if elapsed > self.alignment_timeout_sec:
            self.stop_robot()
            self.get_logger().warn(
                f'Route {route_id} visual heading alignment timed out; finishing route.'
            )
            self.finish_arrival_success(route_id)
            return

        target_yaw = self.pose_yaw(self.active_poses[-1])
        yaw_error = normalize_angle(target_yaw - self.current_pose_map[2])
        if abs(yaw_error) <= self.heading_align_yaw_tolerance:
            self.stop_robot()
            self.finish_arrival_success(route_id)
            return

        angular = self.alignment_angular_gain * yaw_error
        angular = max(-self.alignment_max_angular_speed, min(self.alignment_max_angular_speed, angular))
        if abs(angular) < self.alignment_min_angular_speed:
            angular = math.copysign(self.alignment_min_angular_speed, angular)
        twist = Twist()
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def finish_arrival_success(self, route_id):
        route = self.routes.get(route_id, {})
        announce_text = str(route.get('arrival_announce_text', '')).strip()
        if announce_text and not self.alignment_announced:
            self.get_logger().info(announce_text)
            self.alignment_announced = True

        self.cancel_retry_timer()
        self.cancel_pending_alignment_timer()
        self.cancel_alignment_timer()
        self.stop_robot()
        self.publish_status(f'Recognition route id={route_id} finished: SUCCEEDED')
        self.reset_active_route_state(route_id)

    def cancel_pending_alignment_timer(self):
        if self.pending_alignment_timer is not None:
            self.pending_alignment_timer.cancel()
            self.destroy_timer(self.pending_alignment_timer)
            self.pending_alignment_timer = None

    def cancel_alignment_timer(self):
        if self.alignment_timer is not None:
            self.alignment_timer.cancel()
            self.destroy_timer(self.alignment_timer)
            self.alignment_timer = None

    def pose_goal_response_callback(self, future, route_id, waypoint_index):
        if self.active_route_id != route_id or self.active_waypoint_index != waypoint_index:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = f'Recognition route id={route_id} waypoint={waypoint_index + 1} rejected by Nav2.'
            self.get_logger().error(message)
            self.publish_status(message)
            self.finish_strict_route(route_id, GoalStatus.STATUS_ABORTED)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future_result, rid=route_id, waypoint_index=waypoint_index: self.pose_result_callback(
                future_result,
                rid,
                waypoint_index,
            )
        )

    def pose_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        total = len(self.active_poses)
        self.publish_status(
            f'id={self.active_route_id} waypoint={self.active_waypoint_index + 1}/{total} '
            f'distance={feedback.distance_remaining:.2f}'
        )

    def pose_result_callback(self, future, route_id, waypoint_index):
        if self.active_route_id != route_id or self.active_waypoint_index != waypoint_index:
            return

        result = future.result()
        if self.alignment_route_id == route_id and result.status == GoalStatus.STATUS_CANCELED:
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if waypoint_index + 1 < len(self.active_poses):
                self.active_waypoint_index += 1
                self.active_goal_handle = None
                self.send_next_pose_goal()
                return
            self.handle_navigation_success(route_id)
            return

        self.finish_strict_route(route_id, result.status)

    def handle_navigation_success(self, route_id):
        self.active_goal_handle = None
        route = self.routes.get(route_id, {})
        task_state = self.should_run_arrival_task(route_id, route)
        if task_state == 'skip':
            self.finish_strict_route(route_id, GoalStatus.STATUS_SUCCEEDED)
            return
        if task_state == 'fail':
            return
        self.start_alignment_phase(route_id, route)

    def should_run_arrival_task(self, route_id, route):
        if not route.get('align_heading_on_success', False):
            return 'skip'
        if self.current_pose_map is None or not self.active_poses:
            self.publish_status(f'Recognition route id={route_id} stopped: ABORTED no pose for task check')
            self.reset_active_route_state(route_id)
            return 'fail'

        if not self.current_pose_in_arrival_polygon(route):
            self.publish_status(
                f'Recognition route id={route_id} stopped: ABORTED outside arrival polygon'
            )
            self.reset_active_route_state(route_id)
            return 'fail'

        if not self.arrival_target_visible():
            self.publish_status(
                f'Recognition route id={route_id} stopped: ABORTED arrival target occluded by map'
            )
            self.reset_active_route_state(route_id)
            return 'fail'

        return 'run'

    def current_pose_in_arrival_polygon(self, route):
        polygon = route.get('arrival_polygon') or []
        if not polygon:
            return True
        point = (self.current_pose_map[0], self.current_pose_map[1])
        return self.point_in_polygon(point, [(float(item['x']), float(item['y'])) for item in polygon])

    def point_in_polygon(self, point, polygon):
        x, y = point
        inside = False
        count = len(polygon)
        for index in range(count):
            x1, y1 = polygon[index]
            x2, y2 = polygon[(index + 1) % count]
            if (y1 > y) == (y2 > y):
                continue
            denom = y2 - y1
            if abs(denom) < 1e-12:
                continue
            intersect_x = x1 + (y - y1) * (x2 - x1) / denom
            if intersect_x > x:
                inside = not inside
        return inside

    def arrival_target_visible(self):
        if self.map_resolution <= 0.0 or not self.map_pixels:
            return True
        start = (self.current_pose_map[0], self.current_pose_map[1])
        target = self.active_poses[-1].pose.position
        end = (target.x, target.y)
        return self.raycast_free(start, end)

    def world_to_map_cell(self, x, y) -> Optional[Tuple[int, int]]:
        origin_x, origin_y = self.map_origin
        col = int(math.floor((x - origin_x) / self.map_resolution))
        row_from_bottom = int(math.floor((y - origin_y) / self.map_resolution))
        if col < 0 or col >= self.map_width or row_from_bottom < 0 or row_from_bottom >= self.map_height:
            return None
        image_row = self.map_height - 1 - row_from_bottom
        return col, image_row

    def map_cell_occupied(self, col, image_row):
        value = self.map_pixels[image_row * self.map_width + col]
        return value < 128

    def raycast_free(self, start, end):
        sx, sy = start
        ex, ey = end
        distance = math.hypot(ex - sx, ey - sy)
        if distance < 1e-6:
            return True
        step = max(self.map_resolution * 0.5, 0.01)
        steps = max(2, int(math.ceil(distance / step)))
        for index in range(1, steps):
            ratio = index / steps
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            cell = self.world_to_map_cell(x, y)
            if cell is None:
                return False
            if self.map_cell_occupied(*cell):
                return False
        return True

    def finish_strict_route(self, route_id, status):
        status_name = STATUS_NAMES.get(status, str(status))
        if status == GoalStatus.STATUS_SUCCEEDED:
            message = f'Recognition route id={route_id} finished: {status_name}'
            self.get_logger().info(message)
            self.cancel_retry_timer()
        else:
            message = f'Recognition route id={route_id} stopped: {status_name}'
            self.get_logger().warn(message)
        self.publish_status(message)

        should_retry = (
            status == GoalStatus.STATUS_ABORTED
            and self.retry_on_abort
            and self.active_route_id == route_id
            and self.active_retry_count < self.max_retries
        )
        if should_retry:
            retry_count = self.active_retry_count + 1
            self.active_goal_handle = None
            self.active_poses = []
            self.schedule_retry(route_id, retry_count)
            return

        self.reset_active_route_state(route_id)

    def make_poses(self, route):
        frame_id = route.get('frame_id', self.frame_id)
        waypoints = route['waypoints']
        poses = []
        for index, waypoint in enumerate(waypoints):
            x = float(waypoint['x'])
            y = float(waypoint['y'])
            yaw = waypoint.get('yaw')
            if yaw is None:
                yaw = self.estimate_yaw(waypoints, index)
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = float(waypoint.get('z', 0.0))
            pose.pose.orientation = self.yaw_to_quaternion(float(yaw))
            poses.append(pose)
        return poses

    def pose_yaw(self, pose_stamped):
        return yaw_from_quaternion(pose_stamped.pose.orientation)

    def use_pose_navigation(self, route, poses):
        route_mode = str(route.get('nav_mode', '')).strip().lower()
        if route_mode == 'pose':
            return True
        if route_mode == 'through':
            return False
        return self.strict_follow_waypoints or len(poses) == 1

    def estimate_yaw(self, waypoints, index):
        current = waypoints[index]
        if index + 1 < len(waypoints):
            target = waypoints[index + 1]
        elif index > 0:
            target = current
            current = waypoints[index - 1]
        else:
            return 0.0
        return math.atan2(
            float(target['y']) - float(current['y']),
            float(target['x']) - float(current['x']),
        )

    def goal_response_callback(self, future, route_id):
        if self.active_route_id != route_id:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = f'Recognition route id={route_id} rejected by Nav2.'
            self.get_logger().error(message)
            self.publish_status(message)
            if self.active_route_id == route_id:
                self.active_route_id = None
                self.active_goal_handle = None
            return

        self.active_goal_handle = goal_handle
        message = f'Recognition route id={route_id} accepted by Nav2.'
        self.get_logger().info(message)
        self.publish_status(message)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future_result, rid=route_id: self.result_callback(future_result, rid)
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.publish_status(
            f'id={self.active_route_id} remaining_poses={feedback.number_of_poses_remaining} '
            f'distance={feedback.distance_remaining:.2f}'
        )

    def result_callback(self, future, route_id):
        if self.active_route_id != route_id and self.alignment_route_id != route_id:
            return
        result = future.result()
        if self.alignment_route_id == route_id and result.status == GoalStatus.STATUS_CANCELED:
            return

        status_name = STATUS_NAMES.get(result.status, str(result.status))
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.handle_navigation_success(route_id)
            return
        else:
            message = f'Recognition route id={route_id} stopped: {status_name}'
            self.get_logger().warn(message)
        self.publish_status(message)

        should_retry = (
            result.status == GoalStatus.STATUS_ABORTED
            and self.retry_on_abort
            and self.active_route_id == route_id
            and self.active_retry_count < self.max_retries
        )
        if should_retry:
            retry_count = self.active_retry_count + 1
            self.active_goal_handle = None
            self.schedule_retry(route_id, retry_count)
            return

        self.reset_active_route_state(route_id)

    def schedule_retry(self, route_id, retry_count):
        self.cancel_retry_timer()
        message = (
            f'Retry recognition route id={route_id} '
            f'attempt={retry_count}/{self.max_retries}'
        )
        self.get_logger().warn(message)
        self.publish_status(message)

        def retry_once():
            self.cancel_retry_timer()
            self.start_route(route_id, source='retry', retry_count=retry_count)

        self.retry_timer = self.create_timer(self.retry_delay_sec, retry_once)

    def cancel_retry_timer(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion

        quat = Quaternion()
        quat.z = math.sin(yaw * 0.5)
        quat.w = math.cos(yaw * 0.5)
        return quat

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def reset_active_route_state(self, route_id):
        if self.active_route_id != route_id:
            return
        self.cancel_pending_alignment_timer()
        self.cancel_alignment_timer()
        self.stop_robot()
        self.active_route_id = None
        self.active_goal_handle = None
        self.active_retry_count = 0
        self.active_poses = []
        self.active_behavior_tree = ''
        self.active_waypoint_index = 0
        self.active_use_pose_navigation = False
        self.alignment_route_id = None
        self.alignment_announced = False


def main():
    rclpy.init()
    node = RecognitionRouteCommander()
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
