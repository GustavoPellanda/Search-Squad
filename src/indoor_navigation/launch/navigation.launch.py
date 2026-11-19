"""Start namespaced Nav2 using the live map and the protected command chain.

The fleet launch supplies simulation, localization and mapping. Nav2 runs in a
private <robot>/nav2 namespace; the supervisor exposes the public navigation
interface and controls handover to the reactive controller.
"""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare(context: LaunchContext) -> list:
    """Prepare robot-specific parameters, start the servers and register file cleanup."""
    robot = LaunchConfiguration('robot').perform(context)
    test_map = LaunchConfiguration('test_map').perform(context)
    roam_on_idle = LaunchConfiguration('roam_on_idle').perform(context) == 'true'

    parameters = load_navigation_parameters(robot, test_map)
    if LaunchConfiguration('use_protection_footprint').perform(context) == 'true':
        radius = planning_radius_from_protection()
        for costmap in ('local_costmap', 'global_costmap'):
            parameters[costmap][costmap]['ros__parameters']['robot_radius'] = radius
    # Door-approach tests do not start RoomExplorer or publish its room-mask input:
    if LaunchConfiguration('enable_room_filter').perform(context) == 'false':
        for costmap in ('local_costmap', 'global_costmap'):
            parameters[costmap][costmap]['ros__parameters']['room_filter']['enabled'] = False
    if LaunchConfiguration('clear_static_footprint').perform(context) == 'true':
        # SLAM raster artifacts under the robot must not trap its own footprint.
        # Clear only the robot's current footprint in the private static layers;
        # the structural /map and the downstream LiDAR protection are unchanged:
        for costmap in ('local_costmap', 'global_costmap'):
            parameters[costmap][costmap]['ros__parameters']['static_layer'][
                'footprint_clearing_enabled'] = True
    # Nav2 nodes share one namespaced YAML file, owned by this launch instance:
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump({robot: {'nav2': parameters}}, stream)
        parameters_path = stream.name

    servers = [
        ('nav2_planner', 'planner_server'),
        ('nav2_controller', 'controller_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    if test_map:
        servers.insert(0, ('nav2_map_server', 'map_server'))

    # Nav2 produces an intermediate input; only Collision Monitor publishes final cmd_vel:
    actions = []
    for package, executable in servers:
        server = Node(
            package=package,
            executable=executable,
            namespace=robot + '/nav2',
            name=executable,
            parameters=[parameters_path],
            remappings=[('cmd_vel', '/' + robot + '/cmd_vel_nav')],
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
        )
        actions.append(server)

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=robot + '/nav2',
        name='navigation_manager',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [executable for _, executable in servers],
            'bond_timeout': 4.0,
        }],
        output='screen',
    )
    supervisor = Node(
        package='indoor_navigation',
        executable='navigation_supervisor',
        namespace=robot,
        parameters=[{'use_sim_time': True, 'roam_on_idle': roam_on_idle}],
        output='screen',
    )
    parameter_cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=cleanup, kwargs={'path': parameters_path}),
    ]))
    return actions + [lifecycle_manager, supervisor, parameter_cleanup]


def planning_radius_from_protection() -> float:
    """Enclose the actual final StopZone at every yaw using its farthest corner."""
    with open(os.path.join(
        get_package_share_directory('lidar_safety'), 'config', 'safety.yaml'
    ), encoding='utf-8') as stream:
        safety = yaml.safe_load(stream)
    with open(os.path.join(
        get_package_share_directory('navigation'), 'config', 'protection.yaml'
    ), encoding='utf-8') as stream:
        protection = yaml.safe_load(stream)
    margin = protection['monitor_margin']
    longitudinal = max(safety['safety.footprint_front'], safety['safety.footprint_rear']) + margin
    lateral = safety['safety.footprint_half_width'] + margin
    # The circle contains all four guarded footprint corners: r = sqrt(x_max² + y_max²).
    # Nav2's existing footprint_padding supplies its additional planning allowance:
    radius = math.hypot(longitudinal, lateral)
    if not math.isfinite(radius) or longitudinal <= 0.0 or lateral <= 0.0:
        raise ValueError('Protection dimensions must define a finite positive planning radius')
    return radius


def load_navigation_parameters(robot: str, test_map: str) -> dict:
    """Resolve frames, topics and behavior trees without changing the installed YAML."""
    navigation_share = get_package_share_directory('indoor_navigation')
    config_path = os.path.join(navigation_share, 'config', 'nav2.yaml')
    with open(config_path, encoding='utf-8') as stream:
        parameters = resolve_parameters(yaml.safe_load(stream), robot)

    navigator_parameters = parameters['bt_navigator']['ros__parameters']
    navigator_parameters['default_nav_to_pose_bt_xml'] = os.path.join(
        navigation_share, 'behavior_trees', 'navigate_pose.xml'
    )

    # Both explorers consume the same absolute structural-map topic, regardless
    # of their private Nav2 namespaces. No per-robot map or costmap is a substitute.
    # The explicit saved-map diagnostic is the only exception; it cannot run in room mode:
    map_topic = '/navigation_test_map' if test_map else '/map'
    for costmap in ('global_costmap', 'local_costmap'):
        costmap_parameters = parameters[costmap][costmap]['ros__parameters']
        costmap_parameters['static_layer']['map_topic'] = map_topic

    if test_map:
        parameters['map_server'] = {'ros__parameters': {
            'use_sim_time': True,
            'yaml_filename': test_map,
            'topic_name': '/navigation_test_map',
            'frame_id': 'map',
        }}

    # Encoder odometry belongs to the robot namespace, outside the private Nav2 namespace:
    navigator_parameters['odom_topic'] = '/' + robot + '/odom'
    parameters['controller_server']['ros__parameters']['odom_topic'] = '/' + robot + '/odom'
    return parameters


def resolve_parameters(value, robot: str):
    """Recursively replace configured local frame IDs and topics with robot-specific names."""
    if isinstance(value, dict):
        return {key: resolve_parameters(item, robot) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve_parameters(item, robot) for item in value]
    if value in ('base_footprint', 'odom'):
        return robot + '/' + value
    if value in ('scan', 'room_filter_info'):
        return '/' + robot + '/' + value
    return value


def cleanup(_context: LaunchContext, path: str) -> list:
    """Remove only the parameter file owned by this launch instance."""
    if os.path.exists(path):
        os.unlink(path)
    return []


def generate_launch_description() -> LaunchDescription:
    """Expose one robot namespace; the fleet launch supplies simulation and sensors."""
    arguments = [
        DeclareLaunchArgument('robot', default_value='explorer_1'),
        DeclareLaunchArgument('roam_on_idle', default_value='false'),
        DeclareLaunchArgument('test_map', default_value=''),
        DeclareLaunchArgument('enable_room_filter', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('clear_static_footprint', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_protection_footprint', default_value='false',
                              choices=['true', 'false']),
    ]
    navigation_setup = OpaqueFunction(function=prepare)
    return LaunchDescription(arguments + [navigation_setup])
