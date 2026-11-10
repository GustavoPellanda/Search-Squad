#!/usr/bin/env python3
"""Check public goal navigation, cancellation and reactive fallback in the project world."""
import argparse
from functools import partial
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from squad_interfaces.msg import ControlLease
from squad_interfaces.srv import SetControl
import yaml

from scenario_evaluation import distance_to_box, path_has_detour


class NavigationObserver(Node):
    """Observe actual action results, authorized sources and generated paths."""

    def __init__(self) -> None:
        super().__init__('navigation_verifier')
        self.map_ready = False
        self.source = ''
        self.speed = 1.0
        self.paths = []
        self.trajectory = []
        self.commands = {}
        self.motion_trace = []
        self.last_trace_time = 0.0
        self.action = ActionClient(self, NavigateToPose, '/explorer_1/navigate_to_pose')
        self.control = self.create_client(SetControl, '/explorer_1/set_control')
        self._subscriptions_owned = [
            self.create_subscription(OccupancyGrid, '/map', self.map_callback,
                                     QoSProfile(depth=1,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(ControlLease, '/explorer_1/control_lease',
                                     self.lease_callback, 10),
            self.create_subscription(Odometry, '/explorer_1/odom', self.odom_callback, 10),
            self.create_subscription(NavPath, '/explorer_1/nav2/plan', self.path_callback, 10),
            self.create_subscription(ModelStates, '/simulation/model_states',
                                     self.states_callback, 10),
        ]
        for topic in ('cmd_vel_nav', 'cmd_vel_guard_input', 'cmd_vel'):
            self._subscriptions_owned.append(self.create_subscription(
                Twist, '/explorer_1/' + topic, partial(self.command_callback, topic), 10))

    def command_callback(self, topic: str, message: Twist) -> None:
        """Distinguish controller output, gate forwarding and final protected commands."""
        self.commands[topic] = [message.linear.x, message.angular.z]

    def map_callback(self, message: OccupancyGrid) -> None:
        """Wait for observed geometry rather than an initially empty publication."""
        self.map_ready = sum(value >= 0 for value in message.data) > 400

    def lease_callback(self, message: ControlLease) -> None:
        """Record the supervisor's current exclusive source."""
        self.source = message.source

    def odom_callback(self, message: Odometry) -> None:
        """Observe physical-wheel stopping after an explicit HOLD request."""
        self.speed = abs(message.twist.twist.linear.x) + abs(message.twist.twist.angular.z)

    def path_callback(self, message: NavPath) -> None:
        """Record planned paths for detour inspection."""
        self.paths.append([(pose.pose.position.x, pose.pose.position.y)
                           for pose in message.poses])

    def states_callback(self, message: ModelStates) -> None:
        """Record actual pose separately from the action server's success status."""
        if 'explorer_1' in message.name:
            pose = message.pose[message.name.index('explorer_1')]
            self.trajectory.append((pose.position.x, pose.position.y))
            if time.monotonic() - self.last_trace_time > 1.0:
                self.last_trace_time = time.monotonic()
                self.motion_trace.append({'position': self.trajectory[-1],
                                          'yaw': 2 * math.atan2(pose.orientation.z,
                                                                pose.orientation.w),
                                          'source': self.source, 'speed': self.speed,
                                          'commands': self.commands.copy()})

    def wait(self, predicate, timeout: float) -> None:
        """Keep collecting feedback until a condition or its wall deadline is reached."""
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > end:
                raise TimeoutError('Navigation verification condition timed out')
            rclpy.spin_once(self, timeout_sec=0.05)

    def future(self, future, timeout: float):
        """Bound an asynchronous test request without blocking ROS collection."""
        self.wait(future.done, timeout)
        return future.result()

    def send(self, x: float, y: float):
        """Submit a normalized map-frame goal to the public supervisor endpoint."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        handle = self.future(self.action.send_goal_async(goal), 10)
        assert handle.accepted, 'Public navigation goal was rejected'
        return handle


def main() -> None:
    """Reach two goals, cancel another, verify fallback, then latch a stationary hold."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('/tmp/search_squad_navigation.json'))
    parser.add_argument('--test-map', default='')
    parser.add_argument('--room-passages', action='store_true',
                        help='Also traverse the narrow passages behind room furniture')
    parser.add_argument('--reference', type=Path, default=Path('artifacts/scenario/reference.yaml'))
    args = parser.parse_args()
    if args.room_passages and not args.test_map:
        parser.error('--room-passages requires the independently generated --test-map')
    for name, value in {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'ROS_DOMAIN_ID': '71',
                        'GAZEBO_MASTER_URI': 'http://127.0.0.1:11471',
                        'ROS_LOG_DIR': '/tmp/search_squad_navigation_ros_logs',
                        'GAZEBO_LOG_PATH': '/tmp/search_squad_navigation_gazebo_logs'}.items():
        os.environ.setdefault(name, value)
    rclpy.init()
    observer = NavigationObserver()
    report = {'result': 'failed', 'goals': [], 'test_map': args.test_map}
    launch = None
    log_path = args.output.with_suffix('.launch.log')
    try:
        with log_path.open('w') as stream:
            command = ['ros2', 'launch', 'mission', 'fleet.launch.py', 'mode:=navigation',
                       'gui:=false', 'rviz:=false', 'log_terminal:=false']
            if args.test_map:
                command.append('test_map:=' + str(Path(args.test_map).resolve()))
            launch = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            observer.wait(lambda: observer.map_ready and observer.source == 'reactive', 90)
            odom_subscribers = observer.get_subscriptions_info_by_topic('/explorer_1/odom')
            assert any(endpoint.node_name == 'controller_server'
                       for endpoint in odom_subscribers), 'Nav2 controller lacks real odometry'
            goals = [(-3.7, -0.45), (-2.8, 0.0)]
            if args.test_map:
                # This route's direct chord crosses the north-west room's screen:
                goals.extend([(-3.2, 3.1), (-5.15, 3.1)])
                if args.room_passages:
                    goals.extend([(-3.95, 4.5), (-2.85, 4.55), (-0.55, 4.55), (-0.55, 3.3)])
            for x, y in goals:
                handle = observer.send(x, y)
                result = observer.future(handle.get_result_async(), 150)
                report['goals'].append({'x': x, 'y': y, 'status': result.status})
                assert result.status == 4, report['goals']
                assert observer.trajectory, 'Missing simulator trajectory'
                assert math.dist(observer.trajectory[-1], (x, y)) < 0.22
                observer.wait(lambda: observer.source == 'reactive', 5)
            handle = observer.send(-1.0, 0.3)
            observer.wait(lambda: observer.source == 'nav', 5)
            start = time.monotonic()
            observer.wait(lambda: time.monotonic() - start > 1.0, 3)
            cancellation = observer.future(handle.cancel_goal_async(), 5)
            assert cancellation.goals_canceling, 'Cancellation was not accepted'
            result = observer.future(handle.get_result_async(), 10)
            assert result.status == 5, 'Public goal did not finish CANCELED'
            observer.wait(lambda: observer.source == 'reactive', 5)
            report['cancel_and_reactive_fallback'] = 'passed'
            response = observer.future(
                observer.control.call_async(SetControl.Request(mode='stop')), 5
            )
            assert response.accepted
            observer.wait(lambda: observer.source == 'hold' and observer.speed < 0.02, 5)
            report['explicit_stop'] = 'passed'
            report['plans'] = observer.paths
            report['trajectory'] = observer.trajectory
            if args.test_map:
                reference = yaml.safe_load(args.reference.read_text())
                report['static_obstacle_detour'] = any(
                    path_has_detour(path, reference['boxes']) for path in observer.paths
                )
                assert report['static_obstacle_detour'], 'No clear Nav2 detour was recorded'
                report['minimum_center_clearance'] = min(
                    distance_to_box(x, y, box)
                    for x, y in observer.trajectory for box in reference['boxes']
                )
                assert report['minimum_center_clearance'] > 0.14
            report['result'] = 'passed'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        report['plans'] = observer.paths
        report['trajectory'] = observer.trajectory
        report['motion_trace'] = observer.motion_trace
        if launch is not None and launch.poll() is None:
            os.kill(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=8)
        if log_path.exists():
            report['shutdown_errors'] = [line for line in log_path.read_text().splitlines()
                                         if 'failed to terminate' in line or
                                         'process has died' in line]
            if report['shutdown_errors']:
                report['result'] = 'failed'
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: value for key, value in report.items()
                          if key not in ('plans', 'trajectory', 'motion_trace')}, indent=2))
        observer.destroy_node()
        rclpy.shutdown()
    if report['result'] != 'passed':
        raise AssertionError('Navigation check failed')


if __name__ == '__main__':
    main()
