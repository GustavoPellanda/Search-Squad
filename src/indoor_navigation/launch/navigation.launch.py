"""Start namespaced Nav2 using the live map and the existing protected command chain."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_parameters(value, robot: str):
    """Apply robot frame IDs and absolute topics to a nested Nav2 parameter structure."""
    if isinstance(value, dict):
        return {key: resolve_parameters(item, robot) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve_parameters(item, robot) for item in value]
    if value in ('base_footprint', 'odom'):
        return robot + '/' + value
    if value in ('scan', 'room_filter_info'):
        return '/' + robot + '/' + value
    return value


def prepare(context) -> list:
    """Resolve the robot frames and keep every Nav2 output upstream of protection."""
    robot = LaunchConfiguration('robot').perform(context)
    path = os.path.join(get_package_share_directory('indoor_navigation'), 'config', 'nav2.yaml')
    with open(path, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)

    parameters = resolve_parameters(parameters, robot)
    parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
        get_package_share_directory('indoor_navigation'), 'behavior_trees', 'navigate_pose.xml'
    )
    test_map = LaunchConfiguration('test_map').perform(context)
    if test_map:
        for costmap in ('global_costmap', 'local_costmap'):
            costmap_parameters = parameters[costmap][costmap]['ros__parameters']
            costmap_parameters['static_layer']['map_topic'] = '/navigation_test_map'
        parameters['map_server'] = {'ros__parameters': {
            'use_sim_time': True, 'yaml_filename': test_map,
            'topic_name': '/navigation_test_map', 'frame_id': 'map'}}
    parameters['bt_navigator']['ros__parameters']['odom_topic'] = '/' + robot + '/odom'
    parameters['controller_server']['ros__parameters']['odom_topic'] = '/' + robot + '/odom'
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump({robot: {'nav2': parameters}}, stream)
        params_path = stream.name
    actions = []
    servers = [('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
               ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')]
    if test_map:
        servers.insert(0, ('nav2_map_server', 'map_server'))
    for package, executable in servers:
        remappings = [('cmd_vel', '/' + robot + '/cmd_vel_nav')]
        actions.append(Node(
            package=package,
            executable=executable,
            namespace=robot + '/nav2',
            name=executable,
            parameters=[params_path],
            remappings=remappings,
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
        ))
    roam_on_idle = LaunchConfiguration('roam_on_idle').perform(context) == 'true'
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=robot + '/nav2',
        name='navigation_manager',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': [server for _, server in servers], 'bond_timeout': 4.0}],
        output='screen',
    ))
    actions.append(Node(
        package='indoor_navigation',
        executable='navigation_supervisor',
        namespace=robot,
        parameters=[{'use_sim_time': True, 'roam_on_idle': roam_on_idle}],
        output='screen',
    ))
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=cleanup, kwargs={'path': params_path}),
    ])))
    return actions


def cleanup(_context, path):
    """Remove only the parameter file owned by this launch instance."""
    if os.path.exists(path):
        os.unlink(path)
    return []


def generate_launch_description():
    """Expose a single robot namespace; the main fleet launch supplies simulation and sensors."""
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='explorer_1'),
        DeclareLaunchArgument('roam_on_idle', default_value='false'),
        DeclareLaunchArgument('test_map', default_value=''),
        OpaqueFunction(function=prepare),
    ])
