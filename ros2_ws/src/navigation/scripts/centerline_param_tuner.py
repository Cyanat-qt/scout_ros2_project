#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


PARAM_SPECS = {
    'default_speed': (0.60, 3.00, 0.20),
    'transition_speed': (0.30, 1.80, 0.15),
    'task_speed': (0.10, 0.60, 0.04),
    'acceleration_limit': (0.80, 4.00, 0.30),
    'deceleration_limit': (1.00, 5.00, 0.40),
    'entry_buffer_distance': (0.30, 3.50, 0.30),
    'entry_slowdown_distance': (0.05, 0.60, 0.05),
    'stop_buffer_margin': (0.05, 0.80, 0.05),
}

TUNED_KEYS = tuple(PARAM_SPECS.keys())


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def finite_ranges(msg, min_margin=0.0):
    lower = max(float(msg.range_min) + float(min_margin), 0.0)
    upper = float(msg.range_max)
    return [item for item in msg.ranges if math.isfinite(item) and lower <= item <= upper]


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def write_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def extract_params(track):
    settings = track.get('settings') or {}
    params = {}
    for key, (lower, upper, _step) in PARAM_SPECS.items():
        value = float(settings.get(key, lower))
        params[key] = round(clamp(value, lower, upper), 4)
    return params


def apply_params(track, params):
    tuned = copy.deepcopy(track)
    settings = tuned.setdefault('settings', {})
    for key in TUNED_KEYS:
        settings[key] = round(float(params[key]), 4)

    for segment in tuned.get('segments', []):
        segment_type = str(segment.get('type', 'track'))
        if segment_type == 'task_branch':
            speed = float(params['task_speed'])
            approach = speed
        else:
            speed = float(params['default_speed'])
            approach = min(float(params['transition_speed']), speed)
        segment['speed'] = round(speed, 4)
        segment['approach_speed'] = round(approach, 4)
    return tuned


def param_key(params):
    return tuple(round(float(params[key]), 4) for key in TUNED_KEYS)


@dataclass
class TrialResult:
    index: int
    params: dict
    outcome: str
    score: float
    elapsed_wall: float
    elapsed_sim: float
    min_scan: float
    last_status: str

    def as_dict(self):
        return {
            'index': self.index,
            'params': self.params,
            'outcome': self.outcome,
            'score': round(self.score, 4),
            'elapsed_wall': round(self.elapsed_wall, 3),
            'elapsed_sim': round(self.elapsed_sim, 3),
            'min_scan': None if math.isinf(self.min_scan) else round(self.min_scan, 4),
            'last_status': self.last_status,
        }


class TrialMonitor(Node):
    def __init__(self, args):
        super().__init__('centerline_tuning_monitor')
        self.args = args
        self.completed = False
        self.crashed = False
        self.stuck = False
        self.last_status = ''
        self.min_scan = math.inf
        self.close_scan_hits = 0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.odom_linear = 0.0
        self.odom_angular = 0.0
        self.start_wall = time.monotonic()
        self.last_motion_wall = self.start_wall
        self.sim_time = 0.0
        self.start_sim = None

        self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.cmd_callback, 10)
        self.create_subscription(String, '/centerline_track/status', self.status_callback, 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

    def elapsed_wall(self):
        return time.monotonic() - self.start_wall

    def elapsed_sim(self):
        if self.start_sim is None:
            return 0.0
        return max(0.0, self.sim_time - self.start_sim)

    def scan_callback(self, msg):
        values = finite_ranges(msg, self.args.scan_min_margin)
        if not values:
            return
        current_min = min(values)
        self.min_scan = min(self.min_scan, current_min)
        if self.elapsed_wall() < self.args.startup_grace:
            return
        if current_min <= self.args.crash_distance:
            self.close_scan_hits += 1
        else:
            self.close_scan_hits = max(0, self.close_scan_hits - 1)
        if self.close_scan_hits >= self.args.crash_hits:
            self.crashed = True

    def odom_callback(self, msg):
        twist = msg.twist.twist
        self.odom_linear = math.hypot(twist.linear.x, twist.linear.y)
        self.odom_angular = abs(twist.angular.z)
        if self.odom_linear >= self.args.motion_speed or self.odom_angular >= self.args.motion_angular:
            self.last_motion_wall = time.monotonic()

    def cmd_callback(self, msg):
        self.cmd_linear = math.hypot(msg.linear.x, msg.linear.y)
        self.cmd_angular = abs(msg.angular.z)

    def status_callback(self, msg):
        self.last_status = msg.data
        if 'completed_one_lap' in msg.data:
            self.completed = True

    def clock_callback(self, msg):
        self.sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self.start_sim is None and self.sim_time > 0.0:
            self.start_sim = self.sim_time

    def update_stuck(self):
        if self.elapsed_wall() < self.args.startup_grace:
            return
        command_active = (
            self.cmd_linear >= self.args.command_speed
            or self.cmd_angular >= self.args.command_angular
        )
        robot_static = (
            self.odom_linear < self.args.motion_speed
            and self.odom_angular < self.args.motion_angular
        )
        if command_active and robot_static:
            if time.monotonic() - self.last_motion_wall >= self.args.stuck_timeout:
                self.stuck = True
        else:
            self.last_motion_wall = time.monotonic()


def launch_trial(track_file):
    cmd = [
        'ros2', 'launch', 'navigation', 'centerline_navigation.launch.py',
        'gui:=false',
        'rviz:=false',
        'teleop:=false',
        f'track_file:={track_file}',
        'auto_start:=true',
    ]
    return subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop_process_tree(proc):
    if proc.poll() is not None:
        return
    for sig, delay in ((signal.SIGINT, 3.0), (signal.SIGTERM, 1.5), (signal.SIGKILL, 0.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        if delay <= 0:
            return
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.1)


def cleanup_leftovers():
    patterns = [
        '[r]os2 launch navigation centerline_navigation.launch.py',
        '[i]gn gazebo',
        '[g]z sim',
        '[p]arameter_bridge',
        '[o]rthogonal_centerline_controller',
        '[o]dom_tf_broadcaster',
        '[s]can_frame_republisher',
        '[c]lock_republisher',
        '[j]oint_state_republisher',
        '[c]ollision_safety_node',
        '[r]obot_state_publisher',
    ]
    for signal_name in ('-INT', '-TERM', '-KILL'):
        for pattern in patterns:
            subprocess.run(['pkill', signal_name, '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)


def score_result(outcome, elapsed, min_scan, args):
    if outcome == 'completed':
        clearance_penalty = 0.0
        if not math.isinf(min_scan) and min_scan < args.warning_distance:
            clearance_penalty = (args.warning_distance - min_scan) * args.clearance_weight
        return elapsed + clearance_penalty
    base = {
        'crashed': 10000.0,
        'stuck': 12000.0,
        'launch_failed': 15000.0,
        'timeout': 20000.0,
    }.get(outcome, 30000.0)
    progress_credit = min(elapsed, args.timeout) * 0.1
    return base - progress_credit


def run_trial(index, params, base_track, args, temp_dir):
    track = apply_params(base_track, params)
    track_file = Path(temp_dir) / f'centerline_trial_{index:03d}.yaml'
    write_yaml(track_file, track)

    cleanup_leftovers()
    proc = launch_trial(str(track_file))
    rclpy.init(args=None)
    monitor = TrialMonitor(args)
    outcome = 'timeout'
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            monitor.update_stuck()
            if proc.poll() is not None and monitor.elapsed_wall() > args.launch_grace:
                outcome = 'launch_failed'
                break
            if monitor.completed:
                outcome = 'completed'
                break
            if monitor.crashed:
                outcome = 'crashed'
                break
            if monitor.stuck:
                outcome = 'stuck'
                break
    finally:
        elapsed_wall = monitor.elapsed_wall()
        elapsed_sim = monitor.elapsed_sim()
        min_scan = monitor.min_scan
        last_status = monitor.last_status
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process_tree(proc)
        cleanup_leftovers()

    elapsed_for_score = elapsed_sim if elapsed_sim > 0.0 else elapsed_wall
    score = score_result(outcome, elapsed_for_score, min_scan, args)
    return TrialResult(
        index=index,
        params=copy.deepcopy(params),
        outcome=outcome,
        score=score,
        elapsed_wall=elapsed_wall,
        elapsed_sim=elapsed_sim,
        min_scan=min_scan,
        last_status=last_status,
    )


def candidate_variants(center, step_scale, seen):
    for key in TUNED_KEYS:
        lower, upper, step = PARAM_SPECS[key]
        for sign in (1.0, -1.0):
            candidate = copy.deepcopy(center)
            candidate[key] = round(clamp(candidate[key] + sign * step * step_scale, lower, upper), 4)
            if param_key(candidate) not in seen:
                yield candidate


def tune(base_track, args):
    base_params = extract_params(base_track)
    seen = set()
    results = []
    best = None
    best_params = copy.deepcopy(base_params)
    step_scale = 1.0

    with tempfile.TemporaryDirectory(prefix='centerline_tune_') as temp_dir:
        queue = [base_params]
        trial_index = 0
        while trial_index < args.trials and queue:
            params = queue.pop(0)
            key = param_key(params)
            if key in seen:
                continue
            seen.add(key)

            if args.dry_run:
                result = TrialResult(
                    index=trial_index + 1,
                    params=params,
                    outcome='dry_run',
                    score=0.0,
                    elapsed_wall=0.0,
                    elapsed_sim=0.0,
                    min_scan=math.inf,
                    last_status='',
                )
            else:
                result = run_trial(trial_index + 1, params, base_track, args, temp_dir)
            results.append(result)
            print(json.dumps(result.as_dict(), ensure_ascii=False), flush=True)

            if best is None or result.score < best.score:
                best = result
                best_params = copy.deepcopy(params)

            trial_index += 1
            if trial_index >= args.trials:
                break

            queue = list(candidate_variants(best_params, step_scale, seen))
            if not queue:
                step_scale *= 0.5
                if step_scale < 0.25:
                    break
                queue = list(candidate_variants(best_params, step_scale, seen))

    return best, results


def write_results(path, best, results):
    payload = {
        'best': None if best is None else best.as_dict(),
        'results': [item.as_dict() for item in results],
    }
    write_yaml(path, payload)


def parse_args():
    parser = argparse.ArgumentParser(description='Tune centerline speed and smooth-stop parameters headlessly.')
    parser.add_argument('--base-track', default='/ws/src/navigation/config/centerline_track.yaml')
    parser.add_argument('--output-track', default='/ws/src/navigation/config/centerline_track_tuned.yaml')
    parser.add_argument('--results-file', default='/ws/src/navigation/config/centerline_tuning_results.yaml')
    parser.add_argument('--trials', type=int, default=12)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--startup-grace', type=float, default=16.0)
    parser.add_argument('--launch-grace', type=float, default=12.0)
    parser.add_argument('--crash-distance', type=float, default=0.082)
    parser.add_argument('--scan-min-margin', type=float, default=0.015)
    parser.add_argument('--crash-hits', type=int, default=4)
    parser.add_argument('--warning-distance', type=float, default=0.13)
    parser.add_argument('--clearance-weight', type=float, default=50.0)
    parser.add_argument('--stuck-timeout', type=float, default=7.0)
    parser.add_argument('--command-speed', type=float, default=0.12)
    parser.add_argument('--command-angular', type=float, default=0.25)
    parser.add_argument('--motion-speed', type=float, default=0.025)
    parser.add_argument('--motion-angular', type=float, default=0.05)
    parser.add_argument('--apply-best', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    base_track = load_yaml(args.base_track)
    if not base_track.get('segments'):
        raise SystemExit(f'No segments in base track: {args.base_track}')

    print('tuning keys: ' + ', '.join(TUNED_KEYS), flush=True)
    print(f'base track: {args.base_track}', flush=True)
    best, results = tune(base_track, args)
    if best is None:
        raise SystemExit('No tuning trial was executed')

    tuned = apply_params(base_track, best.params)
    write_yaml(args.output_track, tuned)
    write_results(args.results_file, best, results)
    print(f'best: {json.dumps(best.as_dict(), ensure_ascii=False)}', flush=True)
    print(f'wrote tuned track: {args.output_track}', flush=True)
    print(f'wrote results: {args.results_file}', flush=True)

    if args.apply_best and best.outcome == 'completed':
        backup = str(args.base_track) + '.bak'
        write_yaml(backup, base_track)
        write_yaml(args.base_track, tuned)
        print(f'applied best track to {args.base_track}; backup={backup}', flush=True)
    elif args.apply_best:
        print('best trial did not complete; not applying to base track', flush=True)


if __name__ == '__main__':
    main()
