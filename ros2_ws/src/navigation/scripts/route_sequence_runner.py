#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RouteSequenceRunner(Node):
    def __init__(self):
        super().__init__('route_sequence_runner')

        self.declare_parameter('route_ids', '101,102,103')
        self.declare_parameter('service_prefix', '/go_recognition_')
        self.declare_parameter('route_status_topic', '/recognition_route_status')
        self.declare_parameter('service_wait_timeout_sec', 1.0)
        self.declare_parameter('start_delay_sec', 1.0)

        route_ids_text = str(self.get_parameter('route_ids').value)
        self.route_ids = [item.strip() for item in route_ids_text.split(',') if item.strip()]
        if not self.route_ids:
            raise RuntimeError('route_ids is empty')

        self.service_prefix = str(self.get_parameter('service_prefix').value)
        self.route_status_topic = str(self.get_parameter('route_status_topic').value)
        self.service_wait_timeout_sec = max(
            0.2, float(self.get_parameter('service_wait_timeout_sec').value)
        )
        self.start_delay_sec = max(0.0, float(self.get_parameter('start_delay_sec').value))

        self.current_index = -1
        self.current_route_id = None
        self.active_client = None
        self.final_exit_code = 0
        self.finished = False

        self.create_subscription(String, self.route_status_topic, self.status_callback, 10)
        self.start_timer = self.create_timer(self.start_delay_sec, self.start_sequence)

        route_text = ' -> '.join(self.route_ids)
        self.get_logger().info(f'Route sequence runner ready: {route_text}')

    def start_sequence(self):
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None
        self.start_next_route()

    def start_next_route(self):
        if self.finished:
            return

        self.current_index += 1
        if self.current_index >= len(self.route_ids):
            self.get_logger().info('All mission routes finished successfully.')
            self.finish(0)
            return

        self.current_route_id = self.route_ids[self.current_index]
        service_name = f'{self.service_prefix}{self.current_route_id}'
        client = self.create_client(Trigger, service_name)
        self.active_client = client

        self.get_logger().info(f'Waiting for service {service_name}')
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            self.get_logger().error(f'Service not available: {service_name}')
            self.finish(1)
            return

        self.get_logger().info(
            f'Starting route {self.current_route_id} '
            f'({self.current_index + 1}/{len(self.route_ids)})'
        )
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self.start_route_done)

    def start_route_done(self, future):
        if self.finished:
            return
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - transport/runtime failure
            self.get_logger().error(f'Failed to start route {self.current_route_id}: {exc}')
            self.finish(1)
            return

        if not response.success:
            self.get_logger().error(
                f'Route {self.current_route_id} rejected: {response.message}'
            )
            self.finish(1)
            return

        self.get_logger().info(f'Route {self.current_route_id} accepted: {response.message}')

    def status_callback(self, msg):
        if self.finished or self.current_route_id is None:
            return

        text = msg.data
        success_token = f'id={self.current_route_id} finished: SUCCEEDED'
        stopped_token = f'id={self.current_route_id} stopped:'

        if success_token in text:
            self.get_logger().info(f'Route {self.current_route_id} completed.')
            self.start_next_route()
            return

        if stopped_token in text:
            self.get_logger().error(f'Route {self.current_route_id} failed: {text}')
            self.finish(1)

    def finish(self, exit_code):
        if self.finished:
            return
        self.finished = True
        self.final_exit_code = exit_code
        self.get_logger().info(f'Route sequence runner exiting with code {exit_code}')
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RouteSequenceRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Route sequence runner interrupted.')
            node.destroy_node()
            rclpy.shutdown()
        return 130
    return node.final_exit_code


if __name__ == '__main__':
    sys.exit(main())
