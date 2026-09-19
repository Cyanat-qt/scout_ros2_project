#!/usr/bin/env python3
"""
YOLO 相机检测节点 (HTTP 桥接版)
通过宿主机 yolo_server.py 的 HTTP API 做推理，不在容器内加载模型。
"""

import io
import json
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class YoloCameraDetector(Node):
    def __init__(self):
        super().__init__('yolo_camera_detector')

        # ── 可配置参数 ──
        self.declare_parameter('yolo_url', 'http://localhost:8765/detect')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('active_topic', '/yolo/active')
        self.declare_parameter('active_on_start', False)
        self.declare_parameter('conf_threshold', 0.15)
        self.declare_parameter('frame_stride', 3)
        self.declare_parameter('print_period_sec', 1.0)
        self.declare_parameter('timeout_sec', 2.0)
        self.declare_parameter('debug_image_width', 480)

        self.yolo_url = str(self.get_parameter('yolo_url').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.active_topic = str(self.get_parameter('active_topic').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.frame_stride = max(1, int(self.get_parameter('frame_stride').value))
        self.print_period_sec = max(0.2, float(self.get_parameter('print_period_sec').value))
        self.timeout_sec = max(0.5, float(self.get_parameter('timeout_sec').value))
        self.debug_image_width = max(160, int(self.get_parameter('debug_image_width').value))

        self.bridge = CvBridge()
        self.frame_index = 0
        self.last_print_ns = 0
        self.last_summary = None
        self.active = False
        self.image_subscription = None
        self.result_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.debug_image_pub = self.create_publisher(Image, '/yolo/debug_image', 1)

        self.image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        active_qos = QoSProfile(depth=1)
        active_qos.reliability = ReliabilityPolicy.RELIABLE
        active_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(Bool, self.active_topic, self.active_callback, active_qos)
        self._publish_idle_debug_image()
        if bool(self.get_parameter('active_on_start').value):
            self._set_active(True)

        self.get_logger().info(
            f'YOLO HTTP detector ready | topic={self.image_topic} | active_topic={self.active_topic} | url={self.yolo_url}'
        )

    def _publish_idle_debug_image(self):
        width = min(self.debug_image_width, 480)
        height = max(120, int(width * 0.56))
        debug = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(
            debug,
            'YOLO idle',
            (18, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def _set_active(self, active):
        active = bool(active)
        if active == self.active and (
            (active and self.image_subscription is not None)
            or (not active and self.image_subscription is None)
        ):
            return

        self.active = active
        self.frame_index = 0
        self.last_print_ns = 0
        self.last_summary = None

        if active:
            if self.image_subscription is None:
                self.image_subscription = self.create_subscription(
                    Image,
                    self.image_topic,
                    self.image_callback,
                    self.image_qos,
                )
            self.get_logger().info('YOLO activated for recognition task')
        else:
            if self.image_subscription is not None:
                self.destroy_subscription(self.image_subscription)
                self.image_subscription = None
            self._publish_idle_debug_image()
            self.get_logger().info('YOLO deactivated; camera subscription closed')

    def active_callback(self, msg):
        self._set_active(msg.data)

    def _publish_debug_image(self, frame, detections):
        debug = frame.copy()
        for detection in detections:
            bbox = detection.get('bbox') or []
            if len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [int(round(value)) for value in bbox]
            class_name = str(detection.get('class', 'object'))
            confidence = float(detection.get('confidence', 0.0))
            color = {
                'enemy': (0, 0, 255),
                'ally': (0, 255, 0),
                'hostage': (255, 180, 0),
            }.get(class_name, (255, 255, 0))
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                debug,
                f'{class_name} {confidence:.2f}',
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        height, width = debug.shape[:2]
        if width > self.debug_image_width:
            scale = self.debug_image_width / float(width)
            debug = cv2.resize(
                debug,
                (self.debug_image_width, max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def _infer(self, frame_bgr):
        """将 BGR 图像编码为 JPEG，POST 到 YOLO HTTP 服务，返回解析结果"""
        ok, jpg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return None

        boundary = b'----HermesYoloBoundary'
        body = (b'--' + boundary + b'\r\n'
                b'Content-Disposition: form-data; name="image"; filename="frame.jpg"\r\n'
                b'Content-Type: image/jpeg\r\n\r\n'
                + jpg.tobytes() +
                b'\r\n--' + boundary + b'--\r\n')

        req = urllib.request.Request(
            self.yolo_url + f'?conf={self.conf_threshold}',
            data=body,
            headers={'Content-Type': f'multipart/form-data; boundary={boundary.decode()}'},
            method='POST',
        )

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                return json.loads(resp.read().decode())
        except Exception as exc:
            self.get_logger().warn(f'YOLO HTTP call failed: {exc}', throttle_duration_sec=5)
            return None

    def image_callback(self, msg):
        if not self.active:
            return

        self.frame_index += 1
        if self.frame_index % self.frame_stride != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Frame conversion failed: {exc}')
            return

        if frame is None or frame.size == 0:
            return

        result = self._infer(frame)
        if result is None or not self.active:
            return

        counts = result.get('counts', {})
        detections = result.get('detections', [])
        normalized_counts = {
            'enemy': int(counts.get('enemy', 0)),
            'hostage': int(counts.get('hostage', 0)),
            'ally': int(counts.get('ally', 0)),
        }

        self.result_pub.publish(String(data=json.dumps({
            'stamp_sec': self.get_clock().now().nanoseconds / 1_000_000_000.0,
            'counts': normalized_counts,
            'detections': detections,
        }, ensure_ascii=False)))
        self._publish_debug_image(frame, detections)

        # 确保三类都有值
        summary_parts = [
            f"enemy={normalized_counts['enemy']}",
            f"hostage={normalized_counts['hostage']}",
            f"ally={normalized_counts['ally']}",
        ]
        summary = ' '.join(summary_parts)

        if detections:
            det_str = ', '.join(
                f"{d['class']}:{d['confidence']:.2f}" for d in detections[:8]
            )
            summary = f'{summary} | {det_str}'
        else:
            summary = f'{summary} | no detections'

        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_summary != summary
            or self.last_print_ns == 0
            or (now_ns - self.last_print_ns) >= int(self.print_period_sec * 1_000_000_000)
        ):
            self.get_logger().info(f'YOLO: {summary}')
            self.last_summary = summary
            self.last_print_ns = now_ns


def main():
    rclpy.init()
    node = YoloCameraDetector()
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
