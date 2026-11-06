"""Compose the fleet, live mapping, supervised navigation and room exploration.

The stationary profile preserves the triangle. Navigation and room profiles add
one controlled explorer; Scout perception and dispatch remain future components.
"""
import json
import os
import shutil
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package: str, filename: str, **arguments) -> GroupAction:
    """Keep each robot's launch arguments in an isolated scope."""
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    child = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=arguments.items()
    )
    return GroupAction([child])


def status_terminal(event, _context, log_file: str):
    """Follow the collector file; tail exits when launch stops the collector PID."""
    return [ExecuteProcess(
        cmd=['gnome-terminal', '--wait', '--title=Search Squad Mission Log', '--',
             'tail', '--pid=' + str(event.pid), '-n', '+1', '-F', log_file],
        output='screen',
    )]


def compose(context: LaunchContext) -> list:
    """Resolve runtime arguments before selecting poses and assembling child launches."""
    robot = LaunchConfiguration('robot').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    source = LaunchConfiguration('source').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    terminal = LaunchConfiguration('log_terminal').perform(context) == 'true'
    test_pose = LaunchConfiguration('obstacle_test_pose').perform(context) == 'true'
    task_file = LaunchConfiguration('task_file').perform(context)
    test_map = LaunchConfiguration('test_map').perform(context)
    if test_map and mode != 'navigation':
        raise ValueError('test_map is restricted to mode:=navigation, never room exploration')
    mission_share = get_package_share_directory('mission')
    with open(os.path.join(mission_share, 'config', 'fleet.yaml'), encoding='utf-8') as stream:
        fleet = yaml.safe_load(stream)
    managed = mode in ('navigation', 'room')
    if mode == 'reactive':
        source = 'reactive'
    if managed:
        source = 'managed'
    if mode != 'hold' and robot == 'scout':
        raise ValueError('Select explorer_1 or explorer_2 for this navigation stage')

    # Include order schedules processes; sensor watchdogs and lifecycle readiness govern motion:
    actions = [include('simulation', 'world.launch.py', gui=gui)]
    for name, initial_pose in fleet['robots'].items():
        pose = fleet['obstacle_test_pose'] if test_pose and name == robot else initial_pose
        arguments = {coordinate: str(value) for coordinate, value in pose.items()}
        actions.append(include('simulation', 'robot.launch.py', robot=name, **arguments))
        actions.append(include('navigation', 'protection.launch.py', robot=name,
                               source=source if name == robot else 'hold'))

    if managed:
        # No saved map is loaded: the adapter supplies poses, while LiDAR supplies geometry:
        for executable in ('sim_pose_adapter', 'occupancy_mapper'):
            actions.append(Node(
                package='mapping', executable=executable,
                parameters=[{'use_sim_time': True}], output='screen',
            ))
        actions.append(include('indoor_navigation', 'navigation.launch.py', robot=robot,
                               roam_on_idle='true' if mode == 'navigation' else 'false',
                               test_map=test_map))
        actions.append(Node(
            package='explorer', executable='room_explorer', namespace=robot,
            parameters=[os.path.join(get_package_share_directory('explorer'),
                                    'config', 'room_exploration.yaml'),
                        {'use_sim_time': True, 'observation_behavior_tree': os.path.join(
                get_package_share_directory('indoor_navigation'),
                'behavior_trees', 'observe_position.xml',
            )}], output='screen',
        ))
    if managed or mode == 'reactive':
        actions.append(include('explorer', 'reactive.launch.py', robot=robot,
                               managed='true' if managed else 'false'))

    # The collector is a launch-owned node. Its terminal is only a viewer of its flushed file:
    log_file = LaunchConfiguration('log_file').perform(context)
    logger = Node(
        package='mission', executable='mission_logger',
        parameters=[{'use_sim_time': True, 'log_file': log_file}], output='screen',
    )
    if terminal and os.environ.get('DISPLAY') and shutil.which('gnome-terminal'):
        actions.append(RegisterEventHandler(OnProcessStart(
            target_action=logger,
            on_start=lambda event, ctx: status_terminal(event, ctx, log_file),
        )))
    actions.append(logger)

    # RViz's map view includes all localized robots, costmaps, frontiers and the Nav2 tools:
    if rviz:
        view = 'navigation.rviz' if managed else robot + '.rviz'
        remappings = [('goal_pose', '/' + robot + '/goal_pose')]
        for topic in ('nav2/global_costmap/costmap', 'nav2/plan', 'room_markers'):
            remappings.append(('/explorer_1/' + topic, '/' + robot + '/' + topic))
        actions.append(Node(
            package='rviz2', executable='rviz2', namespace=robot, name='fleet_rviz',
            arguments=['-d', os.path.join(mission_share, 'rviz', view)],
            parameters=[{'use_sim_time': True}], output='screen',
            remappings=remappings,
        ))
    if task_file:
        if mode != 'room':
            raise ValueError('task_file requires mode:=room')
        # A test entrance is explicit input, never an embedded world-coordinate lookup:
        with open(task_file, encoding='utf-8') as stream:
            task = yaml.safe_load(stream)
        actions.append(ExecuteProcess(
            cmd=[sys.executable, '/opt/ros/humble/bin/ros2', 'action', 'send_goal',
                 '/' + robot + '/explore_room', 'squad_interfaces/action/ExploreRoom',
                 json.dumps(task)], output='screen',
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    """Expose profiles while keeping fleet.launch.py as the single application entry point."""
    arguments = [
        DeclareLaunchArgument('mode', default_value='room',
                              choices=['hold', 'reactive', 'navigation', 'room']),
        DeclareLaunchArgument('robot', default_value='explorer_1',
                              choices=['scout', 'explorer_1', 'explorer_2']),
        DeclareLaunchArgument('source', default_value='hold',
                              choices=['hold', 'test', 'nav', 'reactive']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('log_terminal', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('log_file', default_value='/tmp/search_squad_mission/events.log'),
        DeclareLaunchArgument('obstacle_test_pose', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('test_map', default_value='',
                              description='Test-only map, restricted to navigation mode.'),
        DeclareLaunchArgument('task_file', default_value='',
                              description='Optional explicit room assignment YAML; never a map.'),
    ]
    middleware = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    return LaunchDescription(arguments + [middleware, OpaqueFunction(function=compose)])
