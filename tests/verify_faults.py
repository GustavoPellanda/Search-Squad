#!/usr/bin/env python3
"""Inject sensor, TF and command faults into isolated C++ nodes, without Gazebo."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

NANOSECONDS_PER_SECOND = 10**9
SCAN_RAYS = 360
FRONT_HALF_ANGLE_RAD = 0.45
SIDE_OUTER_ANGLE_RAD = 1.75
TAIL_SAMPLE_COUNT = 4
ZERO_SPEED_TOLERANCE = 0.001


def make_transform(parent: str, child: str, nanoseconds: int) -> TransformStamped:
    """Build an identity TF edge at a synthetic ROS timestamp."""
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(
        nanoseconds, NANOSECONDS_PER_SECOND
    )
    transform.transform.rotation.w = 1.0
    return transform


def obstacle_ranges(scan: LaserScan, kind: str) -> list[float]:
    """Place a front obstacle and optionally block the left, right or both laser sectors."""
    ranges = []
    for beam_index in range(SCAN_RAYS):
        bearing_rad = math.remainder(beam_index * scan.angle_increment, 2 * math.pi)
        distance_m = 0.35 if abs(bearing_rad) <= FRONT_HALF_ANGLE_RAD else 3.0
        in_left_sector = FRONT_HALF_ANGLE_RAD < bearing_rad < SIDE_OUTER_ANGLE_RAD
        in_right_sector = -SIDE_OUTER_ANGLE_RAD < bearing_rad < -FRONT_HALF_ANGLE_RAD
        if in_left_sector and kind in ('left_blocked', 'blocked'):
            distance_m = 0.2
        if in_right_sector and kind in ('right_blocked', 'blocked'):
            distance_m = 0.2
        ranges.append(distance_m)
    return ranges


def make_scan(kind: str, nanoseconds: int) -> LaserScan:
    """Build a full-circle clear scan, then inject exactly the requested scan fault."""
    scan = LaserScan()
    scan.header.frame_id = 'probe/base_scan'
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(
        nanoseconds, NANOSECONDS_PER_SECOND
    )
    scan.angle_min = 0.0
    scan.angle_increment = 2 * math.pi / SCAN_RAYS
    scan.angle_max = (SCAN_RAYS - 1) * scan.angle_increment
    scan.range_min = 0.12
    scan.range_max = 3.5
    scan.ranges = [math.inf] * SCAN_RAYS

    invalid_ranges = {'nan': math.nan, 'negative_inf': -math.inf, 'too_close': 0.01}
    if kind in invalid_ranges:
        scan.ranges[0] = invalid_ranges[kind]
    elif kind == 'empty':
        scan.ranges = []
    elif kind == 'wrong_frame':
        scan.header.frame_id = 'other/base_scan'
    elif kind == 'stale':
        scan.header.stamp.sec -= 5
    elif kind == 'future':
        scan.header.stamp.sec += 5
    elif kind == 'negative_timestamp':
        scan.header.stamp.sec = -1
    elif kind == 'invalid_nanoseconds':
        # This normalizes to the current time but violates the ROS Time field contract:
        scan.header.stamp.sec -= 1
        scan.header.stamp.nanosec += NANOSECONDS_PER_SECOND
    elif kind == 'invalid_metadata':
        scan.angle_increment = math.nan
    elif kind in ('left_blocked', 'right_blocked', 'blocked'):
        scan.ranges = obstacle_ranges(scan, kind)
    return scan


class Fixture(Node):
    """Own synthetic input publishers and collect gate/controller velocity observations."""

    def __init__(self) -> None:
        super().__init__('search_squad_fault_fixture')
        self._gate_samples = []
        self._reactive_samples = []
        self._initialize_communication()
        self._begin_wall_time = time.monotonic()

        sensor_mount = make_transform('probe/base_footprint', 'probe/base_scan', 0)
        self._static_publisher.publish(TFMessage(transforms=[sensor_mount]))

    def _initialize_communication(self) -> None:
        """Connect the same synthetic topics and QoS used by the original fault fixture."""
        self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self._scan_publisher = self.create_publisher(
            LaserScan, '/probe/scan', qos_profile_sensor_data
        )
        self._command_publisher = self.create_publisher(Twist, '/probe/cmd_vel_test', 1)
        self._tf_publisher = self.create_publisher(TFMessage, '/tf', 10)
        self._static_publisher = self.create_publisher(
            TFMessage,
            '/tf_static',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._gate_subscription = self.create_subscription(
            Twist,
            '/probe/cmd_vel_guard_input',
            self._gate_callback,
            10,
        )
        self._reactive_subscription = self.create_subscription(
            Twist,
            '/probe/cmd_vel_reactive',
            self._reactive_callback,
            10,
        )

    def _gate_callback(self, message: Twist) -> None:
        """Record the gate output as planar linear/angular velocities."""
        self._gate_samples.append((message.linear.x, message.angular.z))

    def _reactive_callback(self, message: Twist) -> None:
        """Record the reactive output independently of gate selection."""
        self._reactive_samples.append((message.linear.x, message.angular.z))

    def _publish_inputs(
        self,
        kind: str,
        *,
        commands: bool,
        scans: bool,
        transforms: bool,
    ) -> None:
        """Advance ROS time and publish whichever inputs remain available in this phase."""
        ros_seconds = 40.0 + time.monotonic() - self._begin_wall_time
        nanoseconds = int(ros_seconds * NANOSECONDS_PER_SECOND)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(nanoseconds, NANOSECONDS_PER_SECOND)
        self._clock_publisher.publish(clock)
        if transforms:
            odom_to_base = make_transform('probe/odom', 'probe/base_footprint', nanoseconds)
            self._tf_publisher.publish(TFMessage(transforms=[odom_to_base]))
        if commands:
            command = Twist()
            command.linear.x = math.nan if kind == 'nan_command' else 0.12
            self._command_publisher.publish(command)
        if scans:
            self._scan_publisher.publish(make_scan(kind, nanoseconds))

    def phase(
        self,
        kind: str = 'clear',
        seconds: float = 0.85,
        *,
        commands: bool = True,
        scans: bool = True,
        transforms: bool = True,
    ) -> tuple[list, list]:
        """Drive one phase for wall seconds and return its last four outputs per node."""
        first_gate = len(self._gate_samples)
        first_reactive = len(self._reactive_samples)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._publish_inputs(kind, commands=commands, scans=scans, transforms=transforms)
            rclpy.spin_once(self, timeout_sec=0.025)

        # Ignore the phase's transient prefix, but never reuse an earlier phase's samples:
        gate_tail = max(first_gate, len(self._gate_samples) - TAIL_SAMPLE_COUNT)
        reactive_tail = max(first_reactive, len(self._reactive_samples) - TAIL_SAMPLE_COUNT)
        return self._gate_samples[gate_tail:], self._reactive_samples[reactive_tail:]


def zero(velocities: list[tuple[float, float]]) -> bool:
    """Require observations to exist and both planar speeds to be near zero."""
    return bool(velocities) and all(
        abs(linear) < ZERO_SPEED_TOLERANCE and abs(angular) < ZERO_SPEED_TOLERANCE
        for linear, angular in velocities
    )


def record_check(checks: list[str], name: str, passed: bool) -> None:
    """Fail at the first violated expectation and retain the successful check names."""
    assert passed, name
    checks.append(name)


def start_nodes(processes: list[subprocess.Popen], log_path: Path) -> None:
    """Start only this fixture's namespaced nodes, retaining handles even on partial failure."""
    configurations = (
        ('navigation', 'command_gate', {
            'source': 'test',
            'scan_frame': 'probe/base_scan',
            'base_frame': 'probe/base_footprint',
            'odom_frame': 'probe/odom',
        }),
        ('explorer', 'reactive_demo', {'duration': '120.0'}),
    )
    for package, executable, parameters in configurations:
        executable_path = Path(get_package_prefix(package)) / 'lib' / package / executable
        command = [
            str(executable_path), '--ros-args', '-r', '__ns:=/probe', '-p', 'use_sim_time:=true'
        ]
        for name, value in parameters.items():
            command.extend(['-p', name + ':=' + value])
        with log_path.open('a') as log:
            process = subprocess.Popen(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(process)


def verify_fault_reactions(fixture: Fixture, checks: list[str]) -> None:
    """Exercise sensor, timestamp, command and TF faults with recovery after each scan fault."""
    gate, reactive = fixture.phase(seconds=3)
    record_check(
        checks,
        'clear +Inf rays permit forward commands',
        any(linear > 0.1 for linear, _ in gate) and any(linear > 0.1 for linear, _ in reactive),
    )
    scan_faults = (
        'nan', 'negative_inf', 'too_close', 'empty', 'wrong_frame', 'stale', 'future',
        'invalid_metadata',
        'negative_timestamp', 'invalid_nanoseconds',
    )
    for kind in scan_faults:
        gate, reactive = fixture.phase(kind)
        record_check(
            checks, kind + ' scan stops gate and reactive controller', zero(gate) and zero(reactive)
        )
        gate, reactive = fixture.phase()
        record_check(
            checks,
            kind + ' recovery with valid scan',
            any(linear > 0.1 for linear, _ in gate) and any(linear > 0.1 for linear, _ in reactive),
        )

    gate, _ = fixture.phase('nan_command')
    record_check(checks, 'NaN command stops gate', zero(gate))
    fixture.phase()
    gate, _ = fixture.phase(seconds=1.2, commands=False)
    record_check(checks, 'lost command stops gate', zero(gate))
    fixture.phase()
    gate, reactive = fixture.phase(seconds=1.3, scans=False)
    record_check(
        checks, 'lost LiDAR stops gate and reactive controller', zero(gate) and zero(reactive)
    )
    fixture.phase()
    gate, _ = fixture.phase(seconds=1.0, transforms=False)
    record_check(checks, 'missing current TF stops gate', zero(gate))

    fixture.phase()
    _, reactive = fixture.phase('left_blocked')
    record_check(
        checks,
        'obstacle with blocked left sector turns right',
        any(angular < -0.1 and linear == 0 for linear, angular in reactive),
    )
    fixture.phase()
    _, reactive = fixture.phase('right_blocked')
    record_check(
        checks,
        'obstacle with blocked right sector turns left',
        any(angular > 0.1 and linear == 0 for linear, angular in reactive),
    )
    _, reactive = fixture.phase('blocked')
    record_check(checks, 'no free side stops reactive controller', zero(reactive))


def stop_nodes(processes: list[subprocess.Popen]) -> None:
    """Interrupt fixture-owned process groups, using the original five-second fallback."""
    for process in processes:
        if process.poll() is not None:
            continue
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main() -> None:
    """Parse the report destination, run the fixture and always release owned resources."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/search_squad_faults.json')
    args = parser.parse_args()
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID', '62')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/search_squad_fault_logs')
    report = {'checks': []}
    processes = []
    log_path = Path(args.output).with_suffix('.nodes.log')
    rclpy.init()
    fixture = Fixture()
    try:
        start_nodes(processes, log_path)
        verify_fault_reactions(fixture, report['checks'])
        report['result'] = 'passed'
    except (AssertionError, OSError, RuntimeError, ValueError) as error:
        # Report the failure at the tool boundary, then propagate it without recovery:
        report['result'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        stop_nodes(processes)
        fixture.destroy_node()
        rclpy.shutdown()
        report['node_log'] = str(log_path)
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
