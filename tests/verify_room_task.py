#!/usr/bin/env python3
"""Run one project-world room task and evaluate it using separate, ignored reference geometry."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.set_message import set_message_fields
from squad_interfaces.action import ExploreRoom
from squad_interfaces.msg import RobotEvent
import yaml

from scenario_evaluation import distance_to_box, path_has_detour


class Observer(Node):
    """Collect live-map revisions and task events without exposing reference geometry to robots."""

    def __init__(self, robot: str) -> None:
        super().__init__('room_task_verifier')
        self.map = None
        self.events = []
        self.phase = ''
        self.robot = robot
        self.map_at_entry = 0
        self.trajectory = []
        self.paths = []
        self._map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._event_subscription = self.create_subscription(
            RobotEvent, '/fleet/events', self.event_callback, 50,
        )
        self._state_subscription = self.create_subscription(
            ModelStates, '/simulation/model_states', self.states_callback, 10,
        )
        self._path_subscription = self.create_subscription(
            NavPath, '/' + robot + '/nav2/plan', self.path_callback, 10,
        )
        self.action = ActionClient(self, ExploreRoom, '/' + robot + '/explore_room')
        self.states = self.create_client(GetEntityState, '/simulation/get_entity_state')

    def states_callback(self, message: ModelStates) -> None:
        """Record actual motion without feeding evaluator geometry into the runtime graph."""
        if self.robot in message.name:
            pose = message.pose[message.name.index(self.robot)]
            self.trajectory.append((pose.position.x, pose.position.y))

    def path_callback(self, message: NavPath) -> None:
        """Retain Nav2 routes so obstacle detours can be checked independently."""
        self.paths.append([(pose.pose.position.x, pose.pose.position.y)
                           for pose in message.poses])

    def map_callback(self, message: OccupancyGrid) -> None:
        """Retain only the latest sensor-built grid."""
        self.map = message

    def event_callback(self, message: RobotEvent) -> None:
        """Record task-owned transitions, not unrelated navigation events."""
        if message.robot != self.robot or not message.task_id:
            return
        if message.phase != self.phase:
            print(message.phase, message.mapping_status, message.frontier_count,
                  message.known_cells, flush=True)
            self.phase = message.phase
            if message.phase == 'ENTERING' and self.map is not None:
                self.map_at_entry = sum(value >= 0 for value in self.map.data)
        self.events.append({'phase': message.phase, 'mapping_status': message.mapping_status,
                            'frontiers': message.frontier_count, 'known': message.known_cells,
                            'detail': message.detail})

    def state(self, robot: str):
        """Query world state only in the external evaluator."""
        request = GetEntityState.Request(name=robot, reference_frame='world')
        future = self.states.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2)
        if not future.done() or not future.result().success:
            raise RuntimeError('Cannot query simulator entity ' + robot)
        return future.result().state


def wait_future(observer: Observer, future, timeout: float):
    """Bound asynchronous test operations while continuing to collect ROS messages."""
    rclpy.spin_until_future_complete(observer, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError('ROS operation timed out')
    return future.result()


def coverage(grid: OccupancyGrid, reference: dict) -> dict:
    """Measure observed reachable floor separately from the controller's stopping criterion."""
    resolution = grid.info.resolution
    x = grid.info.origin.position.x + (np.arange(grid.info.width) + 0.5) * resolution
    y = grid.info.origin.position.y + (np.arange(grid.info.height) + 0.5) * resolution
    xx, yy = np.meshgrid(x, y)
    eligible = ((xx > reference['x_min'] + 0.15) & (xx < reference['x_max'] - 0.15) &
                (yy > reference['y_min'] + 0.15) & (yy < reference['y_max'] - 0.15))
    for box in reference['boxes']:
        dx, dy = xx - box['x'], yy - box['y']
        local_x = dx * math.cos(box['yaw']) + dy * math.sin(box['yaw'])
        local_y = -dx * math.sin(box['yaw']) + dy * math.cos(box['yaw'])
        eligible &= ~((abs(local_x) <= box['sx'] / 2 + 0.12) &
                      (abs(local_y) <= box['sy'] / 2 + 0.12))
    values = np.array(grid.data).reshape(xx.shape)
    total = int(eligible.sum())
    known = int(((values >= 0) & eligible).sum())
    return {'eligible_cells': total, 'known_cells': known,
            'known_fraction': known / total if total else 0.0,
            'definition': 'known reachable-floor reference cells, 0.12 m obstacle clearance'}


def doorway_depth(point, assignment: dict) -> float:
    """Measure the actual robot center against the commanded doorway half-plane."""
    threshold = assignment['threshold']['pose']
    orientation = threshold['orientation']
    yaw = 2 * math.atan2(orientation['z'], orientation['w'])
    return ((point[0] - threshold['position']['x']) * math.cos(yaw) +
            (point[1] - threshold['position']['y']) * math.sin(yaw))


def run_assignment(observer: Observer, assignment: dict, timeout: float,
                   launch, cancel_at: str = '') -> dict:
    """Cancel at an observed stage and independently verify the eventual physical exit."""
    goal = ExploreRoom.Goal()
    set_message_fields(goal, assignment)
    handle = wait_future(observer, observer.action.send_goal_async(goal), 10)
    assert handle.accepted, 'Room task rejected, including after previous release'
    result_future = handle.get_result_async()
    deadline = time.monotonic() + timeout
    cancellation = None
    cancel_depth = None
    while not result_future.done():
        rclpy.spin_once(observer, timeout_sec=0.1)
        if launch.poll() is not None or time.monotonic() > deadline:
            raise TimeoutError('Room task did not finish; inspect launch log')
        phase_matches = observer.phase == cancel_at
        depth = doorway_depth(observer.trajectory[-1], assignment)
        if cancel_at == 'ENTERING':
            # Cancel with the center already inside, before the entry goal completes:
            phase_matches = phase_matches and depth > 0.05
        if cancel_at and phase_matches and cancellation is None:
            cancel_depth = depth
            cancellation = handle.cancel_goal_async()
    wrapped = result_future.result()
    outcome = wrapped.result
    result = {'mapping_status': outcome.mapping_status, 'reason': outcome.reason,
              'returned': outcome.returned, 'passage_clear': outcome.passage_clear,
              'action_status': wrapped.status, 'cancel_at': cancel_at,
              'cancel_depth': cancel_depth}
    if cancel_at:
        assert cancellation is not None, 'Requested cancellation stage was never observed'
        assert wait_future(observer, cancellation, 5).goals_canceling
        assert wrapped.status == 5, result
        expected = 'NOT_STARTED' if cancel_at in ('GOING_TO_ROOM', 'ENTERING') else 'PARTIAL'
        assert outcome.mapping_status == expected, result
    else:
        assert wrapped.status == 4, result
    assert outcome.returned and outcome.passage_clear, result
    state = observer.state(observer.robot)
    position = [state.pose.position.x, state.pose.position.y]
    result['exit_position'] = position
    result['exit_depth'] = doorway_depth(position, assignment)
    assert result['exit_depth'] < -0.40, 'Action finished while the doorway was occupied'
    return result


def main() -> None:
    """Start one isolated headless fleet, submit the room action and verify the outcome."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=Path, default=Path('artifacts/scenario/room_task.yaml'))
    parser.add_argument('--reference', type=Path, default=Path('artifacts/scenario/reference.yaml'))
    parser.add_argument('--robot', default='explorer_1')
    parser.add_argument('--output', type=Path, default=Path('/tmp/search_squad_room_task.json'))
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--exploration-timeout', type=float)
    parser.add_argument('--expected', choices=['MAPPED', 'PARTIAL'], default='MAPPED')
    parser.add_argument('--cancellations', action='store_true',
                        help='Cancel approach, mid-entry, exploration and return in one fleet')
    args = parser.parse_args()
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID', '67')
    os.environ.setdefault('GAZEBO_MASTER_URI', 'http://127.0.0.1:11467')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/search_squad_room_ros_logs')
    os.environ.setdefault('GAZEBO_LOG_PATH', '/tmp/search_squad_room_gazebo_logs')
    rclpy.init()
    observer = Observer(args.robot)
    log_path = args.output.with_suffix('.launch.log')
    report = {'result': 'failed', 'task': str(args.task), 'launch_log': str(log_path)}
    launch = None
    try:
        with log_path.open('w') as log:
            launch = subprocess.Popen(
                ['ros2', 'launch', 'mission', 'fleet.launch.py', 'mode:=room',
                 'robot:=' + args.robot, 'gui:=false', 'rviz:=false', 'log_terminal:=false',
                 'log_file:=' + str(args.output.with_suffix('.events.log'))],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            deadline = time.monotonic() + 90
            while (observer.map is None or sum(value >= 0 for value in observer.map.data) < 400
                   or not observer.action.server_is_ready()):
                rclpy.spin_once(observer, timeout_sec=0.1)
                if launch.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Fleet did not become ready; inspect launch log')
            initial = {robot: observer.state(robot) for robot in
                       ('scout', 'explorer_1', 'explorer_2')}
            odom_subscribers = observer.get_subscriptions_info_by_topic('/' + args.robot + '/odom')
            assert any(endpoint.node_name == 'controller_server'
                       for endpoint in odom_subscribers), 'Nav2 controller lacks real odometry'
            assignment = yaml.safe_load(args.task.read_text())
            if args.exploration_timeout is not None:
                assignment['exploration_timeout'] = args.exploration_timeout
            if args.cancellations:
                report['cancellations'] = []
                for phase in ('GOING_TO_ROOM', 'ENTERING', 'EXPLORING', 'RETURNING'):
                    assignment['task_id'] = 'cancel_' + phase.lower()
                    assignment['exploration_timeout'] = 12.0
                    report['cancellations'].append(run_assignment(
                        observer, assignment, args.timeout, launch, phase))
                report['outcome'] = report['cancellations'][-1]
            else:
                report['outcome'] = run_assignment(observer, assignment, args.timeout, launch)
            report['events'] = observer.events
            report['map_known_at_entry'] = observer.map_at_entry
            report['map_known_at_end'] = sum(value >= 0 for value in observer.map.data)
            reference = yaml.safe_load(args.reference.read_text())
            report['coverage'] = coverage(observer.map, reference)
            report['trajectory'] = observer.trajectory
            report['planned_detour_observed'] = any(
                path_has_detour(path, reference['boxes']) for path in observer.paths
            )
            report['minimum_center_clearance'] = min(
                distance_to_box(x, y, box)
                for x, y in observer.trajectory for box in reference['boxes']
            )
            final = {robot: observer.state(robot) for robot in initial}
            report['final_positions'] = {robot: [state.pose.position.x, state.pose.position.y]
                                         for robot, state in final.items()}
            for robot in initial:
                if robot == args.robot:
                    continue
                delta = math.hypot(final[robot].pose.position.x - initial[robot].pose.position.x,
                                   final[robot].pose.position.y - initial[robot].pose.position.y)
                assert delta < 0.01, robot + ' moved without selection'
            if not args.cancellations:
                assert report['outcome']['mapping_status'] == args.expected, report['outcome']
            assert report['minimum_center_clearance'] > 0.14, 'Robot contacted a static obstacle'
            if not args.cancellations and args.expected == 'MAPPED':
                assert report['coverage']['known_fraction'] >= 0.95, report['coverage']
                assert report['map_known_at_end'] > report['map_known_at_entry']
            report['result'] = 'passed'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        if launch is not None and launch.poll() is None:
            os.kill(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=8)
        if log_path.exists():
            text = log_path.read_text()
            report['shutdown_errors'] = [line for line in text.splitlines()
                                         if 'failed to terminate' in line or
                                         'process has died' in line]
            if report['shutdown_errors']:
                report['result'] = 'failed'
        if observer.map is not None:
            snapshot = args.output.with_suffix('.map.json')
            snapshot.write_text(json.dumps({
                'resolution': observer.map.info.resolution,
                'width': observer.map.info.width,
                'height': observer.map.info.height,
                'origin': [observer.map.info.origin.position.x,
                           observer.map.info.origin.position.y],
                'data': list(observer.map.data),
            }))
            report['map_snapshot'] = str(snapshot)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: value for key, value in report.items()
                          if key not in ('events', 'trajectory')}, indent=2))
        observer.destroy_node()
        rclpy.shutdown()
    if report['result'] != 'passed':
        raise AssertionError('Room task verification failed')


if __name__ == '__main__':
    main()
