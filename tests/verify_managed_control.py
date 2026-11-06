#!/usr/bin/env python3
"""Inject ownership, stale-command and supervisor-loss faults into managed control."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from squad_interfaces.msg import ControlLease
from verify_faults import Fixture, record_check, stop_nodes, zero


class ManagedFixture(Fixture):
    """Reuse the sensor/TF fixture while independently controlling both command sources."""

    def __init__(self) -> None:
        super().__init__()
        self.source = 'hold'
        self.generation = 1
        self.send_lease = True
        self.send_nav = True
        self.stale_lease = False
        self.nav_speed = 0.12
        self.reactive_mode = 'roam'
        self._lease_publisher = self.create_publisher(ControlLease, '/probe/control_lease', 1)
        self._nav_publisher = self.create_publisher(Twist, '/probe/cmd_vel_nav', 1)

    def _publish_inputs(self, kind: str, *, commands: bool, scans: bool, transforms: bool) -> None:
        """Publish current sensors while allowing supervision or navigation input to disappear."""
        super()._publish_inputs(kind, commands=commands, scans=scans, transforms=transforms)
        if self.send_lease:
            lease = ControlLease()
            seconds = 40.0 + time.monotonic() - self._begin_wall_time
            if self.stale_lease:
                seconds -= 5.0
            nanoseconds = int(seconds * 10**9)
            lease.header.stamp.sec, lease.header.stamp.nanosec = divmod(nanoseconds, 10**9)
            lease.source = self.source
            lease.reactive_mode = self.reactive_mode if self.source == 'reactive' else 'hold'
            lease.generation = self.generation
            self._lease_publisher.publish(lease)
        if self.send_nav:
            command = Twist()
            command.linear.x = self.nav_speed
            self._nav_publisher.publish(command)


def start_managed_nodes(processes: list, log_path: Path) -> None:
    """Start only fixture-owned gate and reactive processes in managed mode."""
    configurations = [
        ('navigation', 'command_gate', {'source': 'managed', 'scan_frame': 'probe/base_scan',
                                      'base_frame': 'probe/base_footprint',
                                      'odom_frame': 'probe/odom'}),
        ('explorer', 'reactive_controller', {'managed': 'true'}),
    ]
    for package, executable, parameters in configurations:
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        command = [str(path), '--ros-args', '-r', '__ns:=/probe', '-p', 'use_sim_time:=true']
        for key, value in parameters.items():
            command.extend(['-p', key + ':=' + value])
        with log_path.open('a') as stream:
            processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                              start_new_session=True))


def main() -> None:
    """Verify exclusive control and fail-closed handover without starting Gazebo."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('/tmp/search_squad_managed.json'))
    args = parser.parse_args()
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID', '69')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/search_squad_managed_ros_logs')
    rclpy.init()
    fixture = ManagedFixture()
    processes = []
    checks = []
    report = {'checks': checks, 'result': 'failed'}
    try:
        start_managed_nodes(processes, args.output.with_suffix('.nodes.log'))
        gate, reactive = fixture.phase(seconds=3)
        record_check(checks, 'hold rejects both active inputs', zero(gate) and zero(reactive))
        fixture.source = 'nav'
        fixture.generation += 1
        gate, reactive = fixture.phase()
        record_check(checks, 'navigation owns motion while reactive holds',
                     any(x > 0.1 for x, _ in gate) and zero(reactive))
        fixture.source = 'reactive'
        fixture.nav_speed = 0.3
        fixture.generation += 1
        gate, reactive = fixture.phase()
        record_check(checks, 'reactive owns motion and competing nav is ignored',
                     all(0.1 < x < 0.14 for x, _ in gate) and any(x > 0.1 for x, _ in reactive))
        fixture.source = 'nav'
        fixture.send_nav = False
        fixture.generation += 1
        gate, reactive = fixture.phase()
        record_check(checks, 'handover does not reuse a cached navigation command', zero(gate))
        fixture.send_nav = True
        fixture.nav_speed = 0.12
        fixture.phase()
        fixture.send_lease = False
        gate, reactive = fixture.phase(seconds=1)
        record_check(checks, 'lost supervisor stops despite continuing nav commands',
                     zero(gate) and zero(reactive))
        fixture.send_lease = True
        fixture.source = 'reactive'
        fixture.stale_lease = True
        fixture.generation += 1
        gate, reactive = fixture.phase()
        record_check(checks, 'stale lease stops gate and reactive controller',
                     zero(gate) and zero(reactive))
        fixture.stale_lease = False
        gate, reactive = fixture.phase()
        record_check(checks, 'fresh lease and fresh commands recover',
                     any(x > 0.1 for x, _ in gate) and any(x > 0.1 for x, _ in reactive))
        fixture.reactive_mode = 'scan'
        fixture.generation += 1
        gate, reactive = fixture.phase()
        record_check(checks, 'managed observation rotates in a clear full circle',
                     any(abs(turn) > 0.4 for _, turn in gate) and
                     all(abs(speed) < 1e-6 for speed, _ in gate))
        gate, reactive = fixture.phase('rear_blocked')
        record_check(checks, 'observation stays still when rear rotation clearance is blocked',
                     zero(gate) and zero(reactive))
        gate, reactive = fixture.phase(scans=False, seconds=1.2)
        record_check(checks, 'sensor loss stops both managed layers', zero(gate) and zero(reactive))
        report['result'] = 'passed'
    finally:
        stop_nodes(processes)
        fixture.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
