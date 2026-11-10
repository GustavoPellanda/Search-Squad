#!/usr/bin/env python3
"""Exercise real safety gate, Humble Collision Monitor and published inspection states."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from squad_interfaces.msg import SafetyState
from tf2_msgs.msg import TFMessage

from verify_faults import Fixture, make_scan, make_transform, record_check, stop_nodes, zero


class SafetyFixture(Fixture):
    """Supply independent requests, obstacles and momentum to the real protection launch."""

    def __init__(self) -> None:
        super().__init__()
        self.request = (0.12, 0.0)
        self.measured = (0.0, 0.0)
        self.points = []
        self.send_scan = True
        self.send_odom = True
        self.bad_scan = ''
        self.status = None
        self.final_samples = []
        self._status_subscription = self.create_subscription(
            SafetyState, '/probe/safety_state', self.status_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._final_subscription = self.create_subscription(
            Twist, '/probe/cmd_vel', self.final_callback, 20)

    def status_callback(self, message: SafetyState) -> None:
        """Record the public decision and its measurement-based explanation."""
        self.status = message

    def final_callback(self, message: Twist) -> None:
        """Observe the existing final publisher independently of the gate output."""
        self.final_samples.append((message.linear.x, message.angular.z))

    def _publish_inputs(self, kind: str, *, commands: bool, scans: bool, transforms: bool) -> None:
        """Publish synthetic time and TF, then independently fault the requested stream."""
        # Build the inputs directly so an absent odometry stream is not refreshed by the parent:
        seconds = 40.0 + time.monotonic() - self._begin_wall_time
        stamp = int(seconds * 10**9)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(stamp, 10**9)
        self._clock_publisher.publish(clock)
        if transforms:
            self._tf_publisher.publish(TFMessage(transforms=[
                make_transform('probe/odom', 'probe/base_footprint', stamp)]))
        if commands:
            command = Twist()
            command.linear.x, command.angular.z = self.request
            self._command_publisher.publish(command)
        if self.send_odom:
            odom = Odometry()
            odom.header.stamp = clock.clock
            odom.header.frame_id = 'probe/odom'
            odom.child_frame_id = 'probe/base_footprint'
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.measured
            self._odom_publisher.publish(odom)
        if scans and self.send_scan:
            scan = make_scan(self.bad_scan or kind, stamp)
            for x, y in self.points:
                ray = round((math.atan2(y, x) % (2 * math.pi)) / scan.angle_increment) % 360
                scan.ranges[ray] = math.hypot(x, y)
            self._scan_publisher.publish(scan)

    def check(self, checks: list, name: str, state: str, reason: str = '') -> None:
        """Require actual gate decisions and outputs; observe the final monitor when moving."""
        gate, _ = self.phase(seconds=1.0)
        assert self.status is not None, 'Missing public safety_state'
        record_check(checks, name, self.status.state == state and bool(gate))
        if reason:
            assert self.status.reason == reason, (name, self.status.reason)
        if state == 'STOP':
            assert zero(gate), name
        else:
            scale = 0.4 if state == 'SLOW' else 1.0
            assert any(math.isclose(x, self.request[0] * scale, abs_tol=1e-5) and
                       math.isclose(w, self.request[1] * scale, abs_tol=1e-5)
                       for x, w in gate), (name, gate)
            assert self.final_samples and any(
                abs(x) + abs(w) > 0.01 for x, w in self.final_samples[-4:]), name


def main() -> None:
    """Verify directional risk, dynamics, stream faults and public status in one fixture."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path,
                        default=Path('/tmp/search_squad_lidar_safety.json'))
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '79')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    rclpy.init()
    fixture = SafetyFixture()
    processes = []
    report = {'result': 'failed', 'checks': []}
    try:
        with args.output.with_suffix('.launch.log').open('w') as stream:
            processes.append(subprocess.Popen(
                ['ros2', 'launch', 'navigation', 'protection.launch.py',
                 'robot:=probe', 'source:=test'], stdout=stream, stderr=subprocess.STDOUT,
                start_new_session=True))
        fixture.phase(seconds=4)
        checks = report['checks']
        fixture.check(checks, 'clear +Inf scan preserves motion', 'CLEAR')
        fixture.points = [(0.3, 0.0)]
        fixture.check(checks, 'front warning reduces speed', 'SLOW')
        fixture.points = [(0.17, 0.0)]
        fixture.check(checks, 'front risk stops forward motion', 'STOP', 'BRAKING_ENVELOPE')
        fixture.request = (-0.15, 0.0)
        fixture.check(checks, 'front obstacle permits reverse escape', 'CLEAR')
        fixture.points = [(-0.18, 0.0)]
        fixture.check(checks, 'rear risk stops reverse motion', 'STOP', 'BRAKING_ENVELOPE')
        fixture.request = (0.15, 0.0)
        fixture.check(checks, 'rear obstacle permits forward escape', 'CLEAR')
        fixture.points = [(0.17, 0.0)]
        fixture.request = (-0.15, 0.0)
        fixture.measured = (0.15, 0.0)
        fixture.check(checks, 'momentum remains protected after command reversal', 'STOP')
        fixture.measured = (0.0, 0.0)
        fixture.points = [(0.17, 0.06)]
        fixture.request = (0.0, -0.8)
        fixture.check(checks, 'swept corner blocks clockwise rotation', 'STOP')
        fixture.request = (0.0, 0.8)
        fixture.check(checks, 'opposite rotation stays available', 'CLEAR')
        fixture.points = []
        fixture.request = (0.12, 0.0)
        fixture.bad_scan = 'nan'
        fixture.check(checks, 'invalid reading publishes a stop reason', 'STOP', 'SCAN_INVALID')
        fixture.bad_scan = ''
        fixture.phase()
        fixture.send_scan = False
        fixture.check(checks, 'scan timeout stops despite continuing commands',
                      'STOP', 'SCAN_STALE')
        fixture.send_scan = True
        fixture.check(checks, 'fresh sensors recover protection', 'CLEAR')
        fixture.send_odom = False
        fixture.check(checks, 'lost measured velocity fails closed',
                      'STOP', 'ODOM_INVALID_OR_STALE')
        report['result'] = 'passed'
    finally:
        stop_nodes(processes)
        fixture.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
