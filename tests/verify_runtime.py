#!/usr/bin/env python3
"""Verify an owned fleet launch; never control a simulation started by another user.

Ground truth is queried only here for evaluation, never by robot controllers.
"""
import argparse
from functools import partial
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any, Callable, NamedTuple

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage

ROBOTS = ('scout', 'explorer_1', 'explorer_2')
SENSOR_TOPICS = ('scan', 'odom', 'joint_states', 'imu')
OBSERVED_TOPICS = (
    ('scan', LaserScan),
    ('odom', Odometry),
    ('joint_states', JointState),
    ('imu', Imu),
    ('cmd_vel', Twist),
    ('cmd_vel_guard_input', Twist),
    ('cmd_vel_reactive', Twist),
)
SIMULATION_TIME_NODES = (
    'command_gate', 'robot_state_publisher', 'collision_monitor', 'timed_diff_drive',
    'turtlebot3_laserscan', 'turtlebot3_imu', 'protection_manager',
)


class Sample(NamedTuple):
    """A received ROS message with distinct simulated and monotonic receipt times."""

    ros_seconds: float
    wall_seconds: float
    message: Any


class Observer(Node):
    """Collect fleet messages and TF, and spin while waiting for time or service responses."""

    def __init__(self) -> None:
        super().__init__(
            'search_squad_verifier',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self._samples = {}
        self._edges = set()
        self._observation_subscriptions = []
        self._initialize_subscriptions()

    def _initialize_subscriptions(self) -> None:
        """Use sensor QoS for observations and retained QoS for static transforms."""
        for robot in ROBOTS:
            self._samples[robot] = {}
            for topic, message_type in OBSERVED_TOPICS:
                self._samples[robot][topic] = []
                subscription = self.create_subscription(
                    message_type,
                    '/' + robot + '/' + topic,
                    partial(self._message_callback, robot, topic),
                    qos_profile_sensor_data,
                )
                self._observation_subscriptions.append(subscription)

        transform_topics = (
            ('/tf', qos_profile_sensor_data),
            ('/tf_static', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        )
        for topic, qos in transform_topics:
            subscription = self.create_subscription(
                TFMessage, topic, self._transforms_callback, qos
            )
            self._observation_subscriptions.append(subscription)

    def _message_callback(self, robot: str, topic: str, message: Any) -> None:
        """Record a message with receipt times without modifying its sensor timestamp."""
        sample = Sample(
            ros_seconds=self.get_clock().now().nanoseconds / 1e9,
            wall_seconds=time.monotonic(),
            message=message,
        )
        self._samples[robot][topic].append(sample)

    def _transforms_callback(self, message: TFMessage) -> None:
        """Record observed parent/child edges independently of transform authority."""
        for transform in message.transforms:
            self._edges.add((transform.header.frame_id, transform.child_frame_id))

    def history(self, robot: str, topic: str) -> tuple[Sample, ...]:
        """Return a snapshot so checks cannot change the observer's sample history."""
        return tuple(self._samples[robot][topic])

    def latest(self, robot: str, topic: str) -> Any:
        """Return the latest received message after the caller has established readiness."""
        return self._samples[robot][topic][-1].message

    def tf_edges(self) -> list[tuple[str, str]]:
        """Return the observed TF graph in stable report order."""
        return sorted(self._edges)

    def sensors_are_ready(self) -> bool:
        """Require all four sensor streams for every robot before integration checks."""
        for robot in ROBOTS:
            for topic in SENSOR_TOPICS:
                if not self._samples[robot][topic]:
                    return False
        return True

    def wait(
        self,
        seconds: float,
        action: Callable[[], None] | None = None,
        *,
        sim: bool = False,
    ) -> None:
        """Spin for wall or simulated seconds, bounding a stalled simulation by wall time."""
        begin = self.get_clock().now().nanoseconds / 1e9 if sim else time.monotonic()
        deadline = time.monotonic() + max(45.0, seconds * 8)
        while True:
            current = self.get_clock().now().nanoseconds / 1e9 if sim else time.monotonic()
            if current - begin >= seconds:
                break
            if time.monotonic() > deadline:
                raise AssertionError('Simulation clock did not make progress')
            if action is not None:
                action()
            rclpy.spin_once(self, timeout_sec=0.025)

    def result(self, future: Future, timeout: float = 8) -> Any:
        """Spin until a service response arrives or the wall-time deadline expires."""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        require(future.done(), 'Service response timed out')
        return future.result()


def require(condition: bool, message: str) -> None:
    """Stop the verification at a violated expectation with a diagnostic message."""
    if not condition:
        raise AssertionError(message)


def descendants(pid: int) -> list[int]:
    """Find only descendants of the verifier-owned launch, using Linux process metadata."""
    process_ids = []
    children_path = Path(f'/proc/{pid}/task/{pid}/children')
    if not children_path.exists():
        return process_ids
    for child in children_path.read_text().split():
        child_pid = int(child)
        process_ids.append(child_pid)
        process_ids.extend(descendants(child_pid))
    return process_ids


def parse_arguments() -> argparse.Namespace:
    """Choose the existing integration scenario, selected explorer and report destination."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('baseline', 'demo', 'protection'), default='baseline')
    parser.add_argument('--robot', choices=('explorer_1', 'explorer_2'), default='explorer_1')
    parser.add_argument('--output', default='/tmp/search_squad_runtime.json')
    parser.add_argument(
        '--gui',
        action='store_true',
        help='Also start Gazebo GUI and the selected local RViz view',
    )
    return parser.parse_args()


def configure_environment() -> None:
    """Set isolated defaults, respect caller choices and reject course overlays."""
    defaults = {
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'ROS_DOMAIN_ID': '61',
        'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
        'GAZEBO_MASTER_URI': 'http://127.0.0.1:11461',
        'ROS_LOG_DIR': '/tmp/search_squad_ros_logs',
        'GAZEBO_LOG_PATH': '/tmp/search_squad_gazebo_logs',
    }
    for name, value in defaults.items():
        os.environ.setdefault(name, value)
    require(
        'robotica-movel' not in os.environ.get('AMENT_PREFIX_PATH', ''),
        'Old overlay in environment',
    )


def launch_command(args: argparse.Namespace) -> list[str]:
    """Select the fleet composition without bypassing the protected input topics."""
    launch_file = 'reactive_demo.launch.py' if args.mode == 'demo' else 'fleet.launch.py'
    command = [
        'ros2', 'launch', 'mission', launch_file,
        'gui:=true' if args.gui else 'gui:=false',
        'rviz:=true' if args.gui else 'rviz:=false',
    ]
    if args.mode == 'demo':
        command.extend(['robot:=' + args.robot, 'duration:=30.0'])
    elif args.mode == 'protection':
        command.extend(['source:=test', 'robot:=' + args.robot, 'demo_pose:=true'])
    return command


def wait_for_fleet(observer: Observer, launch: subprocess.Popen, log_path: Path) -> None:
    """Bound sensor discovery and fail early if this test's launch exits."""
    deadline = time.monotonic() + 90
    while not observer.sensors_are_ready():
        rclpy.spin_once(observer, timeout_sec=0.1)
        require(launch.poll() is None, 'Launch exited; inspect ' + str(log_path))
        require(
            time.monotonic() < deadline,
            'Fleet sensors did not become ready; inspect ' + str(log_path),
        )
    observer.wait(2.0)


def verify_sensor_frames(observer: Observer, robot: str) -> None:
    """Check sensor headers and the local odom-to-laser TF chain for one robot."""
    scan = observer.latest(robot, 'scan')
    odom = observer.latest(robot, 'odom')
    joints = observer.latest(robot, 'joint_states')
    imu = observer.latest(robot, 'imu')
    require(
        scan.header.frame_id == robot + '/base_scan' and len(scan.ranges) == 360,
        robot + ' scan frame/rays',
    )
    require(
        odom.header.frame_id == robot + '/odom' and
        odom.child_frame_id == robot + '/base_footprint',
        robot + ' odom frames',
    )
    require(
        joints.header.frame_id == robot + '/base_footprint' and len(joints.name) == 2,
        robot + ' joints',
    )
    require(imu.header.frame_id == robot + '/imu_link', robot + ' imu frame')
    required_edges = (
        (robot + '/odom', robot + '/base_footprint'),
        (robot + '/base_footprint', robot + '/base_link'),
        (robot + '/base_link', robot + '/base_scan'),
    )
    for edge in required_edges:
        require(edge in observer.tf_edges(), 'Missing TF ' + str(edge))


def verify_command_graph(observer: Observer, robot: str) -> None:
    """Require a sole final publisher and no actuator subscription upstream of protection."""
    final_topic = '/' + robot + '/cmd_vel'
    publishers = observer.get_publishers_info_by_topic(final_topic)
    subscribers = observer.get_subscriptions_info_by_topic(final_topic)
    require(
        len(publishers) == 1 and publishers[0].node_name == 'collision_monitor',
        robot + ' final publisher',
    )
    require(
        any(subscription.node_name == 'timed_diff_drive' for subscription in subscribers),
        robot + ' actuator final subscription',
    )
    for topic in ('cmd_vel_test', 'cmd_vel_reactive', 'cmd_vel_nav', 'cmd_vel_guard_input'):
        upstream_subscribers = observer.get_subscriptions_info_by_topic('/' + robot + '/' + topic)
        actuator_bypasses_protection = any(
            subscription.node_name == 'timed_diff_drive' for subscription in upstream_subscribers
        )
        require(not actuator_bypasses_protection, 'Actuator bypass')


def verify_lifecycle_and_clock(observer: Observer, robot: str) -> None:
    """Wait for the monitor to be active, then query all seven nodes' use_sim_time settings."""
    lifecycle = observer.create_client(GetState, '/' + robot + '/collision_monitor/get_state')
    require(lifecycle.wait_for_service(timeout_sec=8), 'Collision Monitor lifecycle missing')
    activation_deadline = time.monotonic() + 20
    while observer.result(lifecycle.call_async(GetState.Request())).current_state.label != 'active':
        require(time.monotonic() < activation_deadline, robot + ' monitor not active')
        observer.wait(0.2)

    for node_name in SIMULATION_TIME_NODES:
        node_path = '/' + robot + '/' + node_name
        parameters = observer.create_client(GetParameters, node_path + '/get_parameters')
        require(parameters.wait_for_service(timeout_sec=5), node_path + ' parameter service')
        request = GetParameters.Request()
        request.names = ['use_sim_time']
        use_sim_time = observer.result(parameters.call_async(request)).values[0]
        require(use_sim_time.bool_value, node_path + ' not using simulation time')


def world_state(observer: Observer, state_client: Client, robot: str) -> EntityState:
    """Query simulator world pose strictly as external verification evidence."""
    request = GetEntityState.Request()
    request.name = robot
    request.reference_frame = 'world'
    response = observer.result(state_client.call_async(request))
    require(response.success, 'Missing Gazebo entity ' + robot)
    return response.state


def verify_demo(observer: Observer, robot: str, report: dict) -> None:
    """Observe advance, slowing, turning and the existing duration-based stop."""
    observer.wait(34, sim=True)
    commands = [sample.message for sample in observer.history(robot, 'cmd_vel_reactive')]
    require(any(command.linear.x > 0.02 for command in commands), 'Demo did not advance')
    require(
        any(abs(command.angular.z) > 0.1 for command in commands),
        'Demo did not react by turning',
    )
    require(any(0.001 < command.linear.x < 0.11 for command in commands), 'Demo did not slow')
    final_command_stopped = (
        bool(commands) and abs(commands[-1].linear.x) < 0.001 and
        abs(commands[-1].angular.z) < 0.001
    )
    require(final_command_stopped, 'Demo did not finish stopped')
    report['reactive_behavior'] = 'advance, slow, turn and duration stop observed'


def owned_monitor_pid(launch: subprocess.Popen, robot: str) -> int:
    """Locate the selected monitor only among descendants of this verifier's launch."""
    candidates = []
    for pid in descendants(launch.pid):
        try:
            command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        except FileNotFoundError:
            continue
        is_monitor = '/navigation/collision_monitor_runner ' in command
        is_selected_robot = '__ns:=/' + robot in command
        if is_monitor and is_selected_robot:
            candidates.append(pid)
    require(len(candidates) == 1, 'Could not identify owned collision monitor process')
    return candidates[0]


def verify_actuator_timeout(
    observer: Observer,
    launch: subprocess.Popen,
    robot: str,
    publish_command: Callable[[], None],
) -> None:
    """Freeze only the owned monitor and require the independent drive timeout to stop wheels."""
    pid = owned_monitor_pid(launch, robot)
    try:
        os.kill(pid, signal.SIGSTOP)
        observer.wait(1.1, publish_command)
        stopped = observer.latest(robot, 'odom').twist.twist
        require(
            abs(stopped.linear.x) < 0.01 and abs(stopped.angular.z) < 0.03,
            'Actuator timeout did not stop wheels',
        )
    finally:
        os.kill(pid, signal.SIGCONT)


def verify_protection(
    observer: Observer,
    launch: subprocess.Popen,
    robot: str,
    report: dict,
) -> None:
    """Exercise drive timeout, source watchdog and monitor slowdown/stop in sequence."""
    publisher = observer.create_publisher(Twist, '/' + robot + '/cmd_vel_test', 1)
    command = Twist()
    command.linear.x = 0.10
    publish_command = partial(publisher.publish, command)
    observer.wait(2, publish_command, sim=True)
    require(
        abs(observer.latest(robot, 'odom').twist.twist.linear.x) > 0.04,
        'Test input did not move',
    )
    verify_actuator_timeout(observer, launch, robot, publish_command)
    report['actuator_timeout'] = 'monitor frozen for 1.1 wall seconds; measured velocity <0.01 m/s'

    observer.wait(1.5, publish_command, sim=True)
    require(observer.latest(robot, 'cmd_vel').linear.x > 0.005, 'Monitor did not recover')
    observer.wait(1.0)
    gate_output = observer.latest(robot, 'cmd_vel_guard_input')
    require(
        gate_output.linear.x == 0 and gate_output.angular.z == 0,
        'Input watchdog did not command zero',
    )
    require(
        abs(observer.latest(robot, 'odom').twist.twist.linear.x) < 0.01,
        'Loss of commands did not stop motion',
    )
    report['command_timeout'] = (
        'source silent for 1.0 wall second; gate zero and odometry <0.01 m/s'
    )

    command.linear.x = 0.12
    first_output = len(observer.history(robot, 'cmd_vel'))
    observer.wait(28, publish_command, sim=True)
    outputs = observer.history(robot, 'cmd_vel')[first_output:]
    require(
        any(0.001 < sample.message.linear.x < 0.09 for sample in outputs),
        'Collision Monitor slowdown not observed',
    )
    require(outputs[-1].message.linear.x == 0, 'Collision Monitor stop not observed')
    require(
        observer.latest(robot, 'cmd_vel_guard_input').linear.x > 0.10,
        'Gate should still request forward motion for monitor stop proof',
    )
    speed = observer.latest(robot, 'odom').twist.twist.linear.x
    require(abs(speed) < 0.01, 'Robot did not physically stop at obstacle')
    report['collision_protection'] = (
        'raw/gate 0.12 m/s, final slowdown and zero, measured stop before obstacle'
    )
    command.linear.x = 0.0
    observer.wait(0.5, publish_command)


def verify_gui_processes(observer: Observer, launch: subprocess.Popen, report: dict) -> None:
    """Check GUI process presence; this alone does not establish rendering or clean shutdown."""
    live_commands = []
    for pid in descendants(launch.pid):
        command_path = Path(f'/proc/{pid}/cmdline')
        if command_path.exists():
            live_commands.append(command_path.read_bytes().replace(b'\0', b' ').decode())
    require(any('gzclient' in command for command in live_commands), 'Gazebo GUI not running')
    require(any('/rviz2 ' in command for command in live_commands), 'RViz not running')
    report['gui_processes'] = 'Gazebo client and RViz alive at end of baseline'
    observer.wait(8)


def verify_motion(
    observer: Observer,
    state_client: Client,
    initial_world: dict[str, EntityState],
    args: argparse.Namespace,
    report: dict,
) -> None:
    """Compare simulator displacement and encoder motion, including unselected-robot hold."""
    report['motion'] = {}
    report['world_motion'] = {}
    for robot in ROBOTS:
        world = world_state(observer, state_client, robot)
        first_world = initial_world[robot]
        delta_x_m = world.pose.position.x - first_world.pose.position.x
        delta_y_m = world.pose.position.y - first_world.pose.position.y
        displacement_m = math.hypot(delta_x_m, delta_y_m)
        orientation = world.pose.orientation
        pitch_sine = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
        pitch_rad = abs(math.asin(max(-1.0, min(1.0, pitch_sine))))
        body_speed_m_s = math.hypot(world.twist.linear.x, world.twist.linear.y)
        report['world_motion'][robot] = {
            'displacement_m': displacement_m,
            'final_position': [world.pose.position.x, world.pose.position.y, world.pose.position.z],
            'final_pitch_rad': pitch_rad,
            'final_speed_m_s': body_speed_m_s,
        }
        require(pitch_rad < 0.04, robot + ' body tilt is inconsistent with planar TF')
        should_hold = args.mode == 'baseline' or robot != args.robot
        if should_hold:
            require(displacement_m < 0.01, robot + ' moved in simulator without selection')
        else:
            require(
                displacement_m > 0.1,
                'Selected robot encoders moved without simulator translation',
            )
            require(body_speed_m_s < 0.01, 'Simulator body is not stopped')

        maximum_displacement_m = 0.0
        maximum_yaw_rad = 0.0
        for sample in observer.history(robot, 'odom'):
            pose = sample.message.pose.pose
            local_distance_m = math.hypot(pose.position.x, pose.position.y)
            local_yaw_rad = abs(2 * math.atan2(pose.orientation.z, pose.orientation.w))
            maximum_displacement_m = max(maximum_displacement_m, local_distance_m)
            maximum_yaw_rad = max(maximum_yaw_rad, local_yaw_rad)
        final_twist = observer.latest(robot, 'odom').twist.twist
        final_speed = abs(final_twist.linear.x)
        final_turn = abs(final_twist.angular.z)
        report['motion'][robot] = {
            'maximum_local_displacement_m': maximum_displacement_m,
            'maximum_abs_yaw_rad': maximum_yaw_rad,
            'final_linear_m_s': final_speed,
            'final_angular_rad_s': final_turn,
        }
        if should_hold:
            require(
                maximum_displacement_m < 0.005 and maximum_yaw_rad < 0.02,
                robot + ' moved without selection',
            )
        else:
            require(maximum_displacement_m > 0.10, 'Selected robot did not translate')
            require(final_speed < 0.01 and final_turn < 0.03, 'Selected robot not stopped at end')


def verify_scenario(
    observer: Observer,
    launch: subprocess.Popen,
    args: argparse.Namespace,
    report: dict,
) -> None:
    """Check fleet readiness and contracts, run the selected scenario, then measure motion."""
    wait_for_fleet(observer, launch, Path(report['launch_log']))
    report['scan_details'] = {}
    for robot in ROBOTS:
        scan = observer.latest(robot, 'scan')
        report['scan_details'][robot] = {'frame': scan.header.frame_id, 'rays': len(scan.ranges)}
        verify_sensor_frames(observer, robot)
        verify_command_graph(observer, robot)
        verify_lifecycle_and_clock(observer, robot)

    state_client = observer.create_client(GetEntityState, '/simulation/get_entity_state')
    require(state_client.wait_for_service(timeout_sec=8), 'Simulator evaluation service missing')
    initial_world = {}
    for robot in ROBOTS:
        initial_world[robot] = world_state(observer, state_client, robot)
    report['frames_and_command_graph'] = 'passed'
    report['sim_time_nodes'] = '7 nodes per robot verified'

    if args.mode == 'baseline':
        observer.wait(6, sim=True)
    elif args.mode == 'demo':
        verify_demo(observer, args.robot, report)
    else:
        verify_protection(observer, launch, args.robot, report)
    if args.gui:
        verify_gui_processes(observer, launch, report)
    verify_motion(observer, state_client, initial_world, args, report)
    report['tf_edges'] = observer.tf_edges()


def stop_launch(launch: subprocess.Popen | None) -> None:
    """Interrupt the owned launch, preserving the original 18/5/5-second escalation sequence."""
    if launch is None or launch.poll() is not None:
        return
    os.kill(launch.pid, signal.SIGINT)
    try:
        launch.wait(timeout=18)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid, signal.SIGTERM)
        try:
            launch.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait(timeout=5)


def report_shutdown(log_path: Path, report: dict) -> list[str]:
    """Record launch cleanup failures and prevent a scenario pass from hiding them."""
    shutdown_errors = []
    for line in log_path.read_text().splitlines():
        if 'failed to terminate' in line or 'process has died' in line:
            shutdown_errors.append(line)
    report['shutdown'] = 'passed' if not shutdown_errors else shutdown_errors
    if shutdown_errors and report.get('result') == 'passed':
        report['result'] = 'failed'
        report['error'] = 'Shutdown required escalation or a node crashed'
    return shutdown_errors


def main() -> None:
    """Configure, run and verify the owned launch, then clean up and report."""
    args = parse_arguments()
    configure_environment()
    command = launch_command(args)
    log_path = Path(args.output).with_suffix('.launch.log')
    environment_names = (
        'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'ROS_DOMAIN_ID', 'GAZEBO_MASTER_URI',
        'RMW_IMPLEMENTATION',
    )
    report = {
        'mode': args.mode,
        'robot': args.robot,
        'command': command,
        'environment': {name: os.environ.get(name, '') for name in environment_names},
        'launch_log': str(log_path),
    }
    rclpy.init()
    observer = Observer()
    launch = None
    try:
        with log_path.open('w') as log:
            launch = subprocess.Popen(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        verify_scenario(observer, launch, args, report)
        report['result'] = 'passed'
    except (AssertionError, OSError, RuntimeError, ValueError) as error:
        # Preserve the diagnostic at the tool boundary and propagate the original failure:
        report['result'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        stop_launch(launch)
        observer.destroy_node()
        rclpy.shutdown()
        shutdown_errors = report_shutdown(log_path, report) if launch is not None else []
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        if shutdown_errors:
            raise AssertionError('Shutdown errors; inspect ' + str(log_path))


if __name__ == '__main__':
    main()
