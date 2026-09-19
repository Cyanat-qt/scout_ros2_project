#!/usr/bin/env python3

import math
import os
import json

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(first, second):
    return math.hypot(second[0] - first[0], second[1] - first[1])


def smooth_entry_speed(cruise_speed, approach_speed, remaining, slowdown, buffer):
    approach_speed = min(float(cruise_speed), float(approach_speed))
    if remaining <= slowdown or buffer <= slowdown:
        return approach_speed
    if remaining >= buffer:
        return float(cruise_speed)
    progress = clamp((buffer - remaining) / (buffer - slowdown), 0.0, 1.0)
    smooth = progress * progress * (3.0 - 2.0 * progress)
    return approach_speed + (cruise_speed - approach_speed) * (1.0 - smooth)


def stopping_distance(from_speed, to_speed, deceleration):
    from_speed = max(0.0, float(from_speed))
    to_speed = max(0.0, min(float(to_speed), from_speed))
    deceleration = max(float(deceleration), 1e-6)
    return max(0.0, (from_speed * from_speed - to_speed * to_speed) / (2.0 * deceleration))


def peak_speed_for_distance(start_speed, end_speed, distance_available, acceleration, deceleration):
    acceleration = max(float(acceleration), 1e-6)
    deceleration = max(float(deceleration), 1e-6)
    distance_available = max(0.0, float(distance_available))
    start_speed = max(0.0, float(start_speed))
    end_speed = max(0.0, float(end_speed))
    numerator = (
        2.0 * acceleration * deceleration * distance_available
        + deceleration * start_speed * start_speed
        + acceleration * end_speed * end_speed
    )
    return math.sqrt(max(0.0, numerator / (acceleration + deceleration)))


def point_from_yaml(data):
    return float(data['x']), float(data['y'])


class Segment:
    def __init__(self, data, defaults):
        self.id = str(data.get('id', 'segment'))
        self.type = str(data.get('type', 'track'))
        self.start = point_from_yaml(data['start'])
        self.end = point_from_yaml(data['end'])
        self.length = max(distance(self.start, self.end), 1e-6)
        self.unit = (
            (self.end[0] - self.start[0]) / self.length,
            (self.end[1] - self.start[1]) / self.length,
        )
        self.normal = (-self.unit[1], self.unit[0])
        self.speed = float(data.get('speed', defaults['default_speed']))
        self.approach_speed = float(
            data.get('approach_speed', defaults['transition_speed']))
        self.hold_yaw = float(data.get('hold_yaw', defaults['hold_yaw']))
        self.recognition_yaw = float(data.get(
            'recognition_yaw',
            math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0]),
        ))
        self.switch_tolerance = float(
            data.get('switch_tolerance', defaults['switch_tolerance']))
        self.recognition_stop = bool(data.get('recognition_stop', False))


class OrthogonalCenterlineController(Node):
    def __init__(self):
        super().__init__('orthogonal_centerline_controller')
        share = get_package_share_directory('navigation')
        default_track = os.path.join(share, 'config', 'centerline_track.yaml')

        self.declare_parameter('track_file', default_track)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('yolo_result_topic', '/yolo/detections')
        self.declare_parameter('yolo_active_topic', '/yolo/active')
        self.declare_parameter('recognition_wait_sec', 1.2)
        self.declare_parameter('recognition_timeout_sec', 6.0)
        self.declare_parameter('recognition_trigger_distance', 0.22)
        self.declare_parameter('recognition_yaw_tolerance', 0.08)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.track_file = str(self.get_parameter('track_file').value)

        self.settings, self.segments = self._load_track(self.track_file)
        self.control_rate = float(self.settings['control_rate'])
        self.pose_timeout = Duration(seconds=float(self.settings['pose_timeout']))
        self.commanded_speed = 0.0
        self.segment_index = 0
        self.running = bool(self.get_parameter('auto_start').value)
        self.completed = False
        self.last_status = None
        self.last_yolo_result = None
        self.last_yolo_wall_ns = 0
        self.recognition_active = False
        self.recognition_aligned = False
        self.recognition_started_ns = 0
        self.recognition_segment_index = None
        self.recognized_segments = set()

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.path_pub = self.create_publisher(
            Path, '/centerline_track/path', path_qos)
        self.status_pub = self.create_publisher(
            String, '/centerline_track/status', path_qos)
        self.speed_pub = self.create_publisher(
            Float32, '/centerline_track/target_speed', 10)
        self.segment_pub = self.create_publisher(
            String, '/centerline_track/active_segment', 10)
        yolo_active_qos = QoSProfile(depth=1)
        yolo_active_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        yolo_active_qos.reliability = ReliabilityPolicy.RELIABLE
        self.yolo_active_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('yolo_active_topic').value),
            yolo_active_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('yolo_result_topic').value),
            self._yolo_result_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, '/centerline_track/start', self._start)
        self.create_service(Trigger, '/centerline_track/stop', self._stop)
        self.path_pub.publish(self._make_path())
        self._set_yolo_active(False)
        self._set_status('running' if self.running else 'idle')
        self.create_timer(1.0 / self.control_rate, self._control_loop)

    def _set_yolo_active(self, active):
        self.yolo_active_pub.publish(Bool(data=bool(active)))

    def _yolo_result_callback(self, msg):
        try:
            self.last_yolo_result = json.loads(msg.data)
            self.last_yolo_wall_ns = self.get_clock().now().nanoseconds
        except (TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Invalid YOLO result message: {exc}')

    def _load_track(self, path):
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        self.frame_id = str(data.get('frame_id', self.frame_id))
        raw_settings = data.get('settings', {}) or {}
        settings = {
            'control_rate': 50.0,
            'default_speed': 3.0,
            'transition_speed': 1.2,
            'final_approach_speed': 0.12,
            'acceleration_limit': 2.4,
            'deceleration_limit': 3.0,
            'entry_buffer_distance': 0.55,
            'entry_slowdown_distance': 0.16,
            'stop_buffer_margin': 0.20,
            'cross_track_gain': 1.6,
            'yaw_gain': 2.0,
            'max_lateral_correction': 0.22,
            'max_angular_speed': 1.8,
            'recovery_speed': 0.25,
            'recovery_heading': 0.45,
            'recovery_cte': 0.16,
            'switch_tolerance': 0.08,
            'finish_tolerance': 0.10,
            'hold_yaw': 0.0,
            'pose_timeout': 0.30,
        }
        settings.update(raw_settings)
        segments = [Segment(item, settings) for item in data.get('segments', [])]
        if not segments:
            points = [point_from_yaml(item) for item in data.get('control_points', [])]
            for index, (start, end) in enumerate(zip(points, points[1:]), start=1):
                segments.append(Segment({
                    'id': f's{index}',
                    'type': 'track',
                    'start': {'x': start[0], 'y': start[1]},
                    'end': {'x': end[0], 'y': end[1]},
                }, settings))
        if not segments:
            raise ValueError('centerline track has no segments')
        return settings, segments

    def _make_path(self):
        message = Path()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        points = [self.segments[0].start] + [segment.end for segment in self.segments]
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            if index < len(self.segments):
                yaw = math.atan2(self.segments[index].unit[1], self.segments[index].unit[0])
            else:
                yaw = math.atan2(self.segments[-1].unit[1], self.segments[-1].unit[0])
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            message.poses.append(pose)
        return message

    def _lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(transform.header.stamp)
        now = self.get_clock().now()
        if stamp.nanoseconds > 0 and now - stamp > self.pose_timeout:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return t.x, t.y, yaw

    def _ramp_speed(self, target):
        dt = 1.0 / self.control_rate
        if target >= self.commanded_speed:
            self.commanded_speed = min(
                target,
                self.commanded_speed + float(self.settings['acceleration_limit']) * dt,
            )
        else:
            self.commanded_speed = max(
                target,
                self.commanded_speed - float(self.settings['deceleration_limit']) * dt,
            )
        self.speed_pub.publish(Float32(data=float(self.commanded_speed)))
        return self.commanded_speed

    def _segment_error(self, pose, segment):
        px, py, _yaw = pose
        dx = px - segment.start[0]
        dy = py - segment.start[1]
        progress = dx * segment.unit[0] + dy * segment.unit[1]
        cte = dx * segment.normal[0] + dy * segment.normal[1]
        remaining = segment.length - progress
        return progress, cte, remaining

    def _target_speed(self, segment, remaining, yaw_error, cte, final_segment):
        approach = segment.approach_speed
        if final_segment:
            approach = min(approach, float(self.settings['final_approach_speed']))
        acceleration = float(self.settings['acceleration_limit'])
        deceleration = float(self.settings['deceleration_limit'])
        margin = float(self.settings.get('stop_buffer_margin', 0.20))
        peak_speed = peak_speed_for_distance(
            self.commanded_speed,
            approach,
            max(0.0, remaining - margin),
            acceleration,
            deceleration,
        )
        cruise_speed = min(segment.speed, peak_speed)
        dynamic_buffer = max(
            float(self.settings['entry_buffer_distance']),
            stopping_distance(cruise_speed, approach, deceleration) + margin,
        )
        target = smooth_entry_speed(
            cruise_speed,
            approach,
            remaining,
            float(self.settings['entry_slowdown_distance']),
            dynamic_buffer,
        )
        if (
            abs(yaw_error) >= float(self.settings['recovery_heading'])
            or abs(cte) >= float(self.settings['recovery_cte'])
        ):
            target = min(target, float(self.settings['recovery_speed']))
        return max(0.0, target)

    def _publish_command(self, vx_world, vy_world, yaw_rate, yaw):
        command = Twist()
        command.linear.x = math.cos(yaw) * vx_world + math.sin(yaw) * vy_world
        command.linear.y = -math.sin(yaw) * vx_world + math.cos(yaw) * vy_world
        command.angular.z = yaw_rate
        self.cmd_pub.publish(command)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _set_status(self, status):
        if status == self.last_status:
            return
        self.last_status = status
        self.status_pub.publish(String(data=status))
        self.get_logger().info(status)

    def _segment_needs_recognition(self, segment):
        return segment.recognition_stop and self.segment_index not in self.recognized_segments

    def _start_recognition_stop(self, segment):
        self.recognition_active = True
        self.recognition_aligned = False
        self.recognition_started_ns = self.get_clock().now().nanoseconds
        self.recognition_segment_index = self.segment_index
        self.last_yolo_result = None
        self.last_yolo_wall_ns = 0
        self.commanded_speed = 0.0
        self._publish_stop()
        self._set_yolo_active(False)
        self._set_status(f'aligning_recognition_segment={segment.id}')

    def _complete_recognition_stop(self, segment, timed_out=False):
        counts = {}
        if self.last_yolo_result:
            counts = self.last_yolo_result.get('counts', {}) or {}
        enemy = int(counts.get('enemy', 0))
        ally = int(counts.get('ally', 0))
        hostage = int(counts.get('hostage', 0))
        prefix = 'YOLO timeout, using latest result' if timed_out else 'YOLO result'
        self.get_logger().info(
            f'{prefix} at {segment.id}: 战区敌军数量={enemy} 战区友军数量={ally} 战区人质数量={hostage}'
        )
        print(f'战区敌军数量：{enemy}', flush=True)
        print(f'战区友军数量：{ally}', flush=True)
        print(f'战区人质数量：{hostage}', flush=True)
        self.recognized_segments.add(self.segment_index)
        self._set_yolo_active(False)
        self.recognition_active = False
        self.recognition_aligned = False
        self.recognition_started_ns = 0
        self.recognition_segment_index = None
        self._set_status(f'recognized_segment={segment.id}')

    def _handle_recognition_stop(self, pose, segment):
        self.commanded_speed = 0.0
        now_ns = self.get_clock().now().nanoseconds

        if not self.recognition_aligned:
            yaw_error = normalize_angle(segment.recognition_yaw - pose[2])
            if abs(yaw_error) > float(self.get_parameter('recognition_yaw_tolerance').value):
                yaw_rate = clamp(
                    float(self.settings['yaw_gain']) * yaw_error,
                    -float(self.settings['max_angular_speed']),
                    float(self.settings['max_angular_speed']),
                )
                self._publish_command(0.0, 0.0, yaw_rate, pose[2])
                self._set_status(
                    f'aligning_recognition_segment={segment.id} yaw_error={yaw_error:.2f}')
                return False

            self.recognition_aligned = True
            self.recognition_started_ns = now_ns
            self.last_yolo_result = None
            self.last_yolo_wall_ns = 0
            self._publish_stop()
            self._set_yolo_active(True)
            self._set_status(f'recognizing_segment={segment.id}')
            return False

        self._publish_stop()
        elapsed = (now_ns - self.recognition_started_ns) / 1_000_000_000.0

        fresh_yolo = self.last_yolo_wall_ns >= self.recognition_started_ns
        if elapsed >= float(self.get_parameter('recognition_wait_sec').value) and fresh_yolo:
            self._complete_recognition_stop(segment)
            return True
        if elapsed >= float(self.get_parameter('recognition_timeout_sec').value):
            self._complete_recognition_stop(segment, timed_out=True)
            return True
        return False

    def _advance_if_needed(self, pose, segment, remaining):
        endpoint_distance = distance(pose[:2], segment.end)
        if remaining > segment.switch_tolerance and endpoint_distance > segment.switch_tolerance:
            return False
        if self.segment_index + 1 >= len(self.segments):
            if endpoint_distance <= float(self.settings['finish_tolerance']):
                self.running = False
                self.completed = True
                self.commanded_speed = 0.0
                self._publish_stop()
                self._set_status('completed_one_lap')
                return True
            return False
        self.segment_index += 1
        self.segment_pub.publish(String(data=self.segments[self.segment_index].id))
        self._set_status(f'segment={self.segments[self.segment_index].id}')
        return True

    def _control_loop(self):
        self.path_pub.publish(self._make_path())
        if not self.running:
            self.commanded_speed = 0.0
            self._publish_stop()
            return
        pose = self._lookup_pose()
        if pose is None:
            self.commanded_speed = 0.0
            self._publish_stop()
            self._set_status('waiting_for_pose')
            return

        segment = self.segments[self.segment_index]
        progress, cte, remaining = self._segment_error(pose, segment)
        if self.recognition_active:
            self._handle_recognition_stop(pose, segment)
            return

        recognition_distance = max(
            segment.switch_tolerance,
            float(self.get_parameter('recognition_trigger_distance').value),
        )
        if remaining <= recognition_distance and self._segment_needs_recognition(segment):
            self._start_recognition_stop(segment)
            return

        if self._advance_if_needed(pose, segment, remaining):
            return

        yaw_error = normalize_angle(segment.hold_yaw - pose[2])
        final_segment = self.segment_index + 1 >= len(self.segments)
        speed = self._ramp_speed(
            self._target_speed(segment, remaining, yaw_error, cte, final_segment))

        along_x = speed * segment.unit[0]
        along_y = speed * segment.unit[1]
        correction = clamp(
            -float(self.settings['cross_track_gain']) * cte,
            -float(self.settings['max_lateral_correction']),
            float(self.settings['max_lateral_correction']),
        )
        vx_world = along_x + correction * segment.normal[0]
        vy_world = along_y + correction * segment.normal[1]
        yaw_rate = clamp(
            float(self.settings['yaw_gain']) * yaw_error,
            -float(self.settings['max_angular_speed']),
            float(self.settings['max_angular_speed']),
        )
        self._publish_command(vx_world, vy_world, yaw_rate, pose[2])
        self.segment_pub.publish(String(data=segment.id))
        self._set_status(
            f'segment={segment.id} remaining={max(0.0, remaining):.2f} cte={cte:.3f}')

    def _start(self, _request, response):
        self.segment_index = 0
        self.commanded_speed = 0.0
        self.completed = False
        self.recognition_active = False
        self.recognition_aligned = False
        self.recognition_started_ns = 0
        self.recognition_segment_index = None
        self.recognized_segments.clear()
        self.last_yolo_result = None
        self.last_yolo_wall_ns = 0
        self.running = True
        self._set_yolo_active(False)
        self._set_status('running')
        response.success = True
        response.message = 'Centerline lap started'
        return response

    def _stop(self, _request, response):
        self.running = False
        self.completed = False
        self.commanded_speed = 0.0
        self._publish_stop()
        self._set_yolo_active(False)
        self._set_status('stopped')
        response.success = True
        response.message = 'Centerline controller stopped'
        return response

    def destroy_node(self):
        if rclpy.ok(context=self.context):
            self._publish_stop()
            self._set_yolo_active(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrthogonalCenterlineController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
