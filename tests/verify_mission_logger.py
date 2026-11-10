#!/usr/bin/env python3
"""Check robot activity transitions and filtering in the central mission log."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from squad_interfaces.msg import RobotEvent


class LogFixture(Node):
    """Inject noisy low-level traffic and independently inspect the collector's file."""

    def __init__(self) -> None:
        super().__init__('mission_log_verifier')
        self.events = self.create_publisher(RobotEvent, '/fleet/events', 50)
        self.warnings = self.create_publisher(Log, '/rosout', 50)

    def wait(self, seconds: float) -> None:
        """Keep ROS discovery and publications alive during a bounded wall interval."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def event(self, phase: str, task: str = '', mapping_status: str = '') -> None:
        """Publish one task or safety transition with an unambiguous robot identity."""
        message = RobotEvent(robot='explorer_1', task_id=task, phase=phase,
                             mapping_status=mapping_status)
        message.header.stamp = self.get_clock().now().to_msg()
        if phase == 'SAFETY_STOP':
            message.detail = 'BRAKING_ENVELOPE'
        self.events.publish(message)


def main() -> None:
    """Verify noisy streams, persistent stops, recovery, warnings and immediate task outcomes."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('/tmp/search_squad_log_filter.json'))
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '80')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    log_path = args.output.with_suffix('.events.log')
    rclpy.init()
    fixture = LogFixture()
    process = None
    report = {'result': 'failed'}
    try:
        executable = Path(get_package_prefix('mission')) / 'lib/mission/mission_logger'
        with args.output.with_suffix('.node.log').open('w') as stream:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '-p', 'log_file:=' + str(log_path),
                 '-p', 'progress_interval:=2.0', '-p', 'warning_interval:=2.0',
                 '-p', 'safety_interval:=2.0', '-p', 'safety_settle_time:=0.15'],
                stdout=stream, stderr=subprocess.STDOUT,
            )
        deadline = time.monotonic() + 8
        while fixture.events.get_subscription_count() == 0:
            fixture.wait(0.05)
            assert time.monotonic() < deadline, 'Logger did not become ready'
        fixture.wait(0.2)
        fixture.event('ASSIGNED', 'room_1')
        for index in range(60):
            fixture.event(('OBSERVING', 'PLANNING_OBSERVATION', 'EXPLORING')[index % 3], 'room_1')
            fixture.event('SAFETY_CLEAR' if index % 2 else 'SAFETY_SLOW')
            fixture.event('NAVIGATING')
            fixture.event('GOAL_REACHED')
            fixture.warnings.publish(Log(
                name='probe', level=int(LoggingSeverity.WARN),
                msg='KeepoutFilter: New filter mask arrived from /probe/room_mask'))
            fixture.wait(0.005)
        fixture.wait(0.3)
        text = log_path.read_text()
        assert text.count('| EXPLORING') == 1, text
        assert '| NAVIGATING' not in text and '| GOAL_REACHED' not in text
        assert 'SAFETY_' not in text and 'KeepoutFilter' not in text

        for _ in range(20):
            fixture.warnings.publish(Log(
                name='probe', level=int(LoggingSeverity.WARN), msg='Repeated warning'))
            fixture.wait(0.005)
        fixture.warnings.publish(Log(
            name='probe', level=int(LoggingSeverity.ERROR), msg='Important failure'))
        fixture.event('SAFETY_STOP')
        fixture.wait(0.6)
        text = log_path.read_text()
        assert text.count('Repeated warning') == 1
        assert 'Important failure' in text and text.count('| SAFETY_STOP') == 1
        fixture.event('SAFETY_CLEAR')
        fixture.wait(0.6)
        assert log_path.read_text().count('SAFETY_RECOVERED') == 1

        fixture.event('SAFETY_STOP')
        fixture.wait(0.03)
        fixture.event('SAFETY_CLEAR')
        for phase in ('RETURNING', 'EXITING', 'COMPLETED', 'FAILED'):
            fixture.event(phase, 'room_1')
        fixture.wait(0.4)
        text = log_path.read_text()
        assert text.count('| SAFETY_STOP') == 1, 'Transient stop leaked into the central log'
        for phase in ('RETURNING', 'EXITING', 'COMPLETED', 'FAILED'):
            assert '| ' + phase in text, phase

        # Room heartbeats must not overwrite navigation or hide repeated goal cycles:
        fixture.event('IDLE', 'room_1', 'PARTIAL')
        fixture.event('REACTIVE_NAVIGATION')
        for _ in range(2):
            fixture.event('NAVIGATING')
            fixture.event('IDLE', mapping_status='NOT_STARTED')
            fixture.event('IDLE', 'room_1', 'PARTIAL')
            fixture.event('GOAL_REACHED')
            fixture.event('REACTIVE_NAVIGATION')
        fixture.event('GOAL_REJECTED')
        fixture.event('HOLD')
        fixture.event('WAITING_FOR_TASK')
        fixture.wait(0.3)
        text = log_path.read_text()
        assert text.splitlines()[0] == 'SEARCH SQUAD | Mission log', text
        assert '| IDLE' not in text and 'NOT_STARTED' not in text, text
        phases = [line.split(' | ')[1] for line in text.splitlines()[1:]
                  if 'explorer_1 | ' in line]
        expected = ['REACTIVE_NAVIGATION', 'NAVIGATING', 'GOAL_REACHED',
                    'REACTIVE_NAVIGATION', 'NAVIGATING', 'GOAL_REACHED',
                    'REACTIVE_NAVIGATION', 'GOAL_REJECTED', 'HOLD', 'WAITING_FOR_TASK']
        assert phases[-len(expected):] == expected, phases
        report.update(result='passed', lines=len(text.splitlines()), noisy_cycles=60,
                      navigation_cycles=2, room_idle_suppressed=True)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        fixture.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
