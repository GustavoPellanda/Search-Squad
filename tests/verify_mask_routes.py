#!/usr/bin/env python3
"""Check footprint clearance of real Nav2 plans around an observed-room mask."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import CostmapFilterInfo
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
import yaml

from scenario_evaluation import distance_to_box


class MaskRouteProbe(Node):
    """Supply a virtual room edge and observe the planner's resulting route."""

    def __init__(self) -> None:
        super().__init__('mask_route_verifier')
        self._costmap = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_publisher = self.create_publisher(OccupancyGrid, '/probe/map', qos)
        self._mask_publisher = self.create_publisher(OccupancyGrid, '/probe/mask', qos)
        self._info_publisher = self.create_publisher(CostmapFilterInfo, '/probe/filter_info', qos)
        self._scan_publisher = self.create_publisher(LaserScan, '/probe/scan', 10)
        self.obstacle_present = True
        self._scan_timer = self.create_timer(0.1, self._publish_scan)
        self._subscription = self.create_subscription(
            OccupancyGrid,
            '/probe/nav2/global_costmap/costmap',
            self._costmap_callback,
            qos,
        )
        self.lifecycle = self.create_client(ChangeState, '/probe/nav2/planner_server/change_state')
        self.planner = ActionClient(self, ComputePathToPose, '/probe/nav2/compute_path_to_pose')
        self._broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'probe/base_footprint'
        transform.transform.translation.x = 1.2
        transform.transform.translation.y = 3.0
        transform.transform.rotation.w = 1.0
        sensor = TransformStamped()
        sensor.header.frame_id = 'probe/base_footprint'
        sensor.child_frame_id = 'probe/base_scan'
        sensor.transform.translation.z = 0.182
        sensor.transform.rotation.w = 1.0
        self._broadcaster.sendTransform([transform, sensor])
        self._publish_fixture()

    def _publish_scan(self) -> None:
        """Place an elevated LiDAR hit in structural free space, outside the keepout box."""
        scan = LaserScan()
        scan.header.frame_id = 'probe/base_scan'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_increment = 2 * math.pi / 360
        scan.angle_max = 359 * scan.angle_increment
        scan.range_min = 0.12
        scan.range_max = 3.5
        scan.ranges = [math.inf] * 360
        if self.obstacle_present:
            for ray in range(88, 93):
                scan.ranges[ray] = 1.0
        self._scan_publisher.publish(scan)

    def _publish_fixture(self) -> None:
        """Keep structural unknown space distinct from a costmap-only forbidden rectangle."""
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = 0.05
        grid.info.width = 120
        grid.info.height = 120
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 14400
        mask = copy.deepcopy(grid)
        mask.data = [-1] * 14400
        for row in range(120):
            for column in range(120):
                cell = row * 120 + column
                if row in (0, 119) or column in (0, 119):
                    grid.data[cell] = 100
                if 48 <= column < 72 and 20 <= row < 92:
                    grid.data[cell] = -1
                    mask.data[cell] = 100
        info = CostmapFilterInfo()
        info.header = grid.header
        info.type = 0
        info.base = 0.0
        info.multiplier = 1.0
        info.filter_mask_topic = '/probe/mask'
        self._map_publisher.publish(grid)
        self._mask_publisher.publish(mask)
        self._info_publisher.publish(info)

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        """Wait until the keepout mask has actually reached the combined costmap."""
        self._costmap = message

    def wait(self, predicate, timeout: float) -> None:
        """Bound lifecycle and planning checks with a wall deadline."""
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Planner/mask did not become ready')
            rclpy.spin_once(self, timeout_sec=0.05)

    def future(self, future, timeout: float):
        """Collect asynchronous ROS results without blocking fixture callbacks."""
        self.wait(future.done, timeout)
        return future.result()

    def activate_planner(self) -> None:
        """Configure and activate only the real planner, without a moving robot."""
        self.wait(self.lifecycle.service_is_ready, 15)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            assert self.future(self.lifecycle.call_async(request), 15).success
        self.wait(lambda: self._costmap is not None and
                  self._costmap.data[60 * 120 + 60] == 100, 10)

    def check_route(self) -> dict:
        """Measure body clearance independently against the known test rectangle."""
        self.wait(self.planner.server_is_ready, 5)
        goal = ComputePathToPose.Goal()
        goal.start.header.frame_id = 'map'
        goal.start.pose.position.x = 1.2
        goal.start.pose.position.y = 3.0
        goal.start.pose.orientation.w = 1.0
        goal.goal = copy.deepcopy(goal.start)
        goal.goal.pose.position.x = 4.8
        goal.use_start = True
        goal.planner_id = 'GridBased'
        handle = self.future(self.planner.send_goal_async(goal), 5)
        assert handle.accepted, 'Planner rejected the route'
        result = self.future(handle.get_result_async(), 10)
        assert result.status == 4, 'Planner failed to find a route around the mask'
        path = [(pose.pose.position.x, pose.pose.position.y)
                for pose in result.result.path.poses]
        obstacle = {'x': 3.0, 'y': 2.8, 'sx': 1.2, 'sy': 3.6, 'yaw': 0.0}
        clearance = min(distance_to_box(x, y, obstacle) for x, y in path)
        return {'minimum_clearance': clearance, 'path': path,
                'result': 'passed' if clearance >= 0.32 else 'failed',
                'required_clearance': '0.35 m circle, allowing 0.03 m grid discretization'}

    def check_live_obstacle(self) -> dict:
        """Prove scan height filtering and +Inf clearing work in the actual obstacle layer."""
        # Scan beam 90 hits map (1.2, 4.0), independently of mask and static geometry:
        cells = [row * 120 + column for row in range(78, 82) for column in range(22, 26)]
        self.wait(lambda: self._costmap is not None and
                  any(self._costmap.data[cell] == 100 for cell in cells), 8)
        marked = max(self._costmap.data[cell] for cell in cells)
        self.obstacle_present = False
        self.wait(lambda: all(self._costmap.data[cell] == 0 for cell in cells), 8)
        return {'marked_cost': marked, 'cleared_cost': 0,
                'point_height': 0.182}


def main() -> None:
    """Use the project's actual layer ordering in an isolated planner fixture."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('/tmp/search_squad_mask_routes.json'))
    args = parser.parse_args()
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID', '76')
    root = Path(__file__).resolve().parents[1]
    parameters = yaml.safe_load((root / 'src/indoor_navigation/config/nav2.yaml').read_text())
    costmap = parameters['global_costmap']['global_costmap']['ros__parameters']
    costmap['use_sim_time'] = False
    costmap['robot_base_frame'] = 'probe/base_footprint'
    costmap['obstacle_layer']['scan']['topic'] = '/probe/scan'
    costmap['static_layer']['map_topic'] = '/probe/map'
    costmap['room_filter']['filter_info_topic'] = '/probe/filter_info'
    planner_parameters = parameters['planner_server']['ros__parameters']
    planner_parameters['use_sim_time'] = False
    planner_parameters['bond_heartbeat_period'] = 0.0
    resolved = {'probe': {'nav2': {
        'planner_server': {'ros__parameters': planner_parameters},
        'global_costmap': {'global_costmap': {'ros__parameters': costmap}},
    }}}
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(resolved, stream)
        parameter_path = Path(stream.name)
    rclpy.init()
    observer = MaskRouteProbe()
    process = None
    report = {'result': 'failed'}
    try:
        with args.output.with_suffix('.planner.log').open('w') as log:
            process = subprocess.Popen(
                [str(Path(get_package_prefix('nav2_planner')) / 'lib/nav2_planner/planner_server'),
                 '--ros-args',
                 '-r', '__ns:=/probe/nav2', '--params-file', str(parameter_path)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            observer.activate_planner()
            live_obstacle = observer.check_live_obstacle()
            report = observer.check_route()
            report['live_obstacle'] = live_obstacle
    except Exception as error:
        report['error'] = str(error)
    finally:
        if process is not None and process.poll() is None:
            os.kill(process.pid, signal.SIGINT)
            process.wait(timeout=15)
        parameter_path.unlink(missing_ok=True)
        observer.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: value for key, value in report.items() if key != 'path'}, indent=2))
    if report['result'] != 'passed':
        raise AssertionError('Nav2 route does not preserve footprint clearance around the mask')


if __name__ == '__main__':
    main()
