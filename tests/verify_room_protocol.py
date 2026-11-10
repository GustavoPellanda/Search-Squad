#!/usr/bin/env python3
"""Inject task-level faults and delayed release into the actual C++ room node."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from squad_interfaces.action import ExploreRoom
from squad_interfaces.msg import ControlLease, RobotEvent
from squad_interfaces.srv import SetControl
from tf2_msgs.msg import TFMessage

from verify_faults import make_scan, make_transform, record_check, stop_nodes


class RoomFixture(Node):
    """Replace external services and sensors, while leaving room decisions in C++."""

    def __init__(self, scenario: str, *, real_supervisor: bool = False) -> None:
        super().__init__('room_protocol_' + scenario)
        self.real_supervisor = real_supervisor
        self.robot = 'owner_probe' if real_supervisor else 'probe'
        prefix = '/' + self.robot
        self.send_heartbeat = True
        self.fault = ''
        self.phase = ''
        self.source = ''
        self.requests = []
        self.pending_navigation = []
        self.release_requested = False
        self.allow_release = False
        self.release_future = Future()
        self.group = ReentrantCallbackGroup()
        self._map_publisher = self.create_publisher(
            OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._scan_publisher = self.create_publisher(
            LaserScan, prefix + '/scan', qos_profile_sensor_data)
        self._tf_publisher = self.create_publisher(TFMessage, '/tf', 10)
        self._lease_publisher = self.create_publisher(ControlLease, prefix + '/control_lease', 1)
        self._odom_publisher = self.create_publisher(Odometry, prefix + '/odom', 1)
        self._heartbeat_publisher = self.create_publisher(Empty, prefix + '/task_heartbeat', 1)
        self._lease_subscription = self.create_subscription(
            ControlLease, prefix + '/control_lease', self.lease_callback, 1)
        self._event_subscription = self.create_subscription(
            RobotEvent, '/fleet/events', self.event_callback, 20)
        if not real_supervisor:
            self._control_service = self.create_service(
                SetControl, prefix + '/set_control',
                self.control_callback, callback_group=self.group)
        self.control = self.create_client(SetControl, prefix + '/set_control')
        self._nav_server = ActionServer(
            self, NavigateToPose,
            prefix + ('/nav2/navigate_to_pose' if real_supervisor else '/navigate_to_pose'),
            self.execute_navigation,
            cancel_callback=self.cancel_navigation, callback_group=self.group)
        self.action = ActionClient(self, ExploreRoom, prefix + '/explore_room')
        self.navigation = ActionClient(self, NavigateToPose, prefix + '/navigate_to_pose')
        self.timer = self.create_timer(0.05, self.publish_inputs)

    def event_callback(self, message: RobotEvent) -> None:
        """Observe the actual node's task transitions."""
        if message.robot == self.robot and (message.task_id or self.real_supervisor):
            self.phase = message.phase

    def lease_callback(self, message: ControlLease) -> None:
        """Observe real supervisor ownership during the task-owner-loss check."""
        self.source = message.source

    async def control_callback(self, request, response):
        """Withhold only release acknowledgement until the verifier permits it."""
        self.requests.append(request.mode)
        if request.mode == 'release':
            self.release_requested = True
            if not self.allow_release:
                await self.release_future
        response.accepted = True
        response.message = request.mode
        return response

    def cancel_navigation(self, _handle):
        """Accept the room node's controlled cancellation of its pending approach."""
        return CancelResponse.ACCEPT

    async def execute_navigation(self, handle):
        """Keep the approach active until canceled, without simulating physical motion."""
        completion = Future()
        self.pending_navigation.append((handle, completion))
        await completion
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return NavigateToPose.Result()

    def publish_inputs(self) -> None:
        """Publish current inputs except the one deliberately faulted in this scenario."""
        stamp = self.get_clock().now().nanoseconds
        if self.fault != 'lost_pose':
            pose_stamp = stamp + (5 * 10**9 if self.fault == 'future_pose' else 0)
            pose = make_transform('map', self.robot + '/base_footprint', pose_stamp)
            pose.transform.translation.y = -0.6
            self._tf_publisher.publish(TFMessage(transforms=[pose]))
        if self.fault != 'lost_scan':
            kind = {'invalid_scan': 'nan', 'future_scan': 'future'}.get(self.fault, 'clear')
            scan = make_scan(kind, stamp)
            scan.header.frame_id = self.robot + '/base_scan'
            self._scan_publisher.publish(scan)
        if self.fault != 'lost_map':
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            map_stamp = stamp + (5 * 10**9 if self.fault == 'future_map' else 0)
            grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(map_stamp, 10**9)
            grid.info.resolution = 0.05
            grid.info.width = 100
            grid.info.height = 100
            grid.info.origin.position.x = -2.5
            grid.info.origin.position.y = -2.5
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * 10000
            self._map_publisher.publish(grid)
        if not self.real_supervisor and self.fault != 'lost_supervisor':
            lease = ControlLease(source='hold', reactive_mode='hold', generation=1)
            lease_stamp = stamp + (5 * 10**9 if self.fault == 'future_lease' else 0)
            lease.header.stamp.sec, lease.header.stamp.nanosec = divmod(lease_stamp, 10**9)
            self._lease_publisher.publish(lease)
        if self.real_supervisor:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            self._odom_publisher.publish(odom)
            if self.send_heartbeat:
                self._heartbeat_publisher.publish(Empty())
        for handle, completion in self.pending_navigation:
            if handle.is_cancel_requested and not completion.done():
                completion.set_result(True)
            elif self.real_supervisor and not completion.done():
                feedback = NavigateToPose.Feedback()
                feedback.current_pose.header.frame_id = 'map'
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                feedback.current_pose.pose.position.y = -0.6
                feedback.current_pose.pose.orientation.w = 1.0
                handle.publish_feedback(feedback)

    def wait(self, predicate, timeout: float) -> None:
        """Keep all fixture callbacks live while waiting for a bounded observation."""
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Room protocol condition timed out: ' + self.phase)
            rclpy.spin_once(self, timeout_sec=0.02)

    def future(self, future, timeout: float):
        """Read a completed asynchronous operation without blocking the publisher timer."""
        self.wait(future.done, timeout)
        return future.result()

    def send_task(self):
        """Supply only the explicit doorway contract, without reference room geometry."""
        goal = ExploreRoom.Goal(task_id='protocol_test', doorway_width=1.2)
        for pose in (goal.approach, goal.threshold, goal.entry):
            pose.header.frame_id = 'map'
            pose.pose.orientation.z = 2**-0.5
            pose.pose.orientation.w = 2**-0.5
        goal.approach.pose.position.y = -0.6
        goal.entry.pose.position.y = 0.65
        return self.future(self.action.send_goal_async(goal), 5)


def run_case(kind: str, log_path: Path) -> dict:
    """Give every fatal scenario a fresh real room process and always release it."""
    fixture = RoomFixture(kind)
    processes = []
    try:
        executable = Path(get_package_prefix('explorer')) / 'lib/explorer/room_explorer'
        with log_path.open('a') as stream:
            processes.append(subprocess.Popen(
                [str(executable), '--ros-args', '-r', '__ns:=/' + fixture.robot],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        fixture.wait(fixture.action.server_is_ready, 10)
        handle = fixture.send_task()
        assert handle.accepted
        fixture.wait(lambda: fixture.phase == 'GOING_TO_ROOM', 8)
        result_future = handle.get_result_async()
        if kind in ('release_ack', 'release_timeout'):
            assert fixture.future(handle.cancel_goal_async(), 5).goals_canceling
            fixture.wait(lambda: fixture.release_requested, 5)
            deadline = time.monotonic() + 0.6
            fixture.wait(lambda: time.monotonic() > deadline, 2)
            assert not result_future.done(), 'Room action completed before release was acknowledged'
            assert fixture.phase == 'INCOMPLETE'
            if kind == 'release_ack':
                fixture.allow_release = True
                fixture.release_future.set_result(True)
        else:
            fixture.fault = kind
        wrapped = fixture.future(result_future, 6)
        outcome = wrapped.result
        if kind == 'release_ack':
            assert wrapped.status == 5 and outcome.returned and outcome.passage_clear
            fixture.wait(lambda: fixture.phase == 'IDLE', 2)
            assert fixture.send_task().accepted, 'Second assignment rejected after release'
        else:
            expected_reason = 'DATA_INVALID'
            if kind in ('lost_supervisor', 'future_lease'):
                expected_reason = 'SUPERVISOR_LOST'
            elif kind == 'release_timeout':
                expected_reason = 'CONTROL_ACK_TIMEOUT'
            assert outcome.reason == expected_reason, outcome.reason
            assert outcome.returned == (kind == 'release_timeout')
            assert outcome.passage_clear == outcome.returned
            fixture.wait(lambda: 'stop' in fixture.requests, 2)
            fixture.wait(lambda: fixture.phase == 'FAILED', 2)
            assert not fixture.send_task().accepted, 'Faulted room silently became available'
        return {'scenario': kind, 'reason': outcome.reason,
                'returned': outcome.returned, 'action_status': wrapped.status}
    finally:
        # Let any deferred fake-service coroutine end before destroying the fixture:
        if not fixture.release_future.done():
            fixture.release_future.set_result(True)
        stop_nodes(processes)
        for _handle, completion in fixture.pending_navigation:
            if not completion.done():
                completion.set_result(True)
        for _ in range(4):
            rclpy.spin_once(fixture, timeout_sec=0.02)
        fixture.action.destroy()
        fixture.navigation.destroy()
        fixture._nav_server.destroy()
        fixture.destroy_node()


def verify_owner_loss(log_path: Path) -> dict:
    """Expire a real supervisor's room-owner heartbeat while its backend goal is active."""
    fixture = RoomFixture('owner_loss', real_supervisor=True)
    processes = []
    try:
        executable = (Path(get_package_prefix('indoor_navigation')) /
                      'lib/indoor_navigation/navigation_supervisor')
        with log_path.open('a') as stream:
            processes.append(subprocess.Popen(
                [str(executable), '--ros-args', '-r', '__ns:=/' + fixture.robot],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        fixture.wait(fixture.control.service_is_ready, 10)
        fixture.wait(lambda: fixture.source == 'hold', 5)
        response = fixture.future(fixture.control.call_async(SetControl.Request(mode='task')), 5)
        assert response.accepted
        fixture.wait(fixture.navigation.server_is_ready, 5)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = -0.6
        goal.pose.pose.orientation.w = 1.0
        handle = fixture.future(fixture.navigation.send_goal_async(goal), 5)
        assert handle.accepted
        fixture.wait(lambda: fixture.source == 'nav', 5)
        fixture.send_heartbeat = False
        wrapped = fixture.future(handle.get_result_async(), 5)
        assert wrapped.status == 6
        fixture.wait(lambda: fixture.source == 'hold' and fixture.phase == 'FAULT', 2)
        response = fixture.future(
            fixture.control.call_async(SetControl.Request(mode='release')), 2)
        assert not response.accepted and response.message == 'FAULT'
        return {'scenario': 'owner_loss', 'source': fixture.source,
                'action_status': wrapped.status, 'release_rejected': True}
    finally:
        stop_nodes(processes)
        fixture.action.destroy()
        fixture.navigation.destroy()
        fixture._nav_server.destroy()
        fixture.destroy_node()


def main() -> None:
    """Separate protocol evidence from the world-based physical room benchmark."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path,
                        default=Path('/tmp/search_squad_room_protocol.json'))
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '77')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    rclpy.init()
    report = {'result': 'failed', 'checks': [], 'scenarios': []}
    try:
        for kind in ('lost_scan', 'invalid_scan', 'future_scan', 'lost_map', 'future_map',
                     'lost_pose', 'future_pose', 'lost_supervisor', 'future_lease',
                     'release_ack', 'release_timeout'):
            report['scenarios'].append(run_case(kind, args.output.with_suffix('.nodes.log')))
            record_check(report['checks'], kind, True)
        report['scenarios'].append(verify_owner_loss(args.output.with_suffix('.nodes.log')))
        record_check(report['checks'], 'owner_loss', True)
        report['result'] = 'passed'
    finally:
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        rclpy.shutdown()


if __name__ == '__main__':
    main()
