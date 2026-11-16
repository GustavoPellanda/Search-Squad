"""Assemble the simulation and the controllers for one selected fleet robot.

All modes spawn and protect the three robots. Hold keeps the selected command
source fixed; reactive starts standalone roaming. Navigation and room add live
mapping and a supervisor that owns command selection. Navigation roams when idle;
room waits for an assignment. The other robots keep their hold command source.
The default starts fuzzy Scout following, parallel individual SLAM and one shared
map. Explorers are localized but held until a separate test or a future task
controller supplies destinations. Other legacy profiles retain assisted mapping.
"""
import json
import os
import shutil
import sys
from functools import partial

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def compose(context: LaunchContext) -> list:
    """Resolve the profile, then assemble simulation, control and operator interfaces.

    OpaqueFunction supplies the runtime values needed for YAML pose selection and
    Python profile branches. Returning actions schedules startup; their order does
    not establish sensor readiness. Watchdogs and lifecycle checks govern motion.
    """
    robot = LaunchConfiguration('robot').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    if robot == 'auto':
        robot = 'scout' if mode == 'scout' else 'explorer_1'
    source = LaunchConfiguration('source').perform(context)
    mapping_backend = LaunchConfiguration('mapping_backend').perform(context)
    if mapping_backend == 'auto':
        mapping_backend = 'parallel_slam' if mode == 'scout' else 'assisted'
    gui = LaunchConfiguration('gui').perform(context)
    rviz_enabled = LaunchConfiguration('rviz').perform(context) == 'true'
    terminal_enabled = LaunchConfiguration('log_terminal').perform(context) == 'true'
    obstacle_test_pose = LaunchConfiguration('obstacle_test_pose').perform(context) == 'true'
    task_file = LaunchConfiguration('task_file').perform(context)
    test_map = LaunchConfiguration('test_map').perform(context)
    log_file = LaunchConfiguration('log_file').perform(context)

    # Each SLAM owns its map->odom edge; fusion alone owns the public /map.
    # Global explorer task/navigation profiles remain separate from this mapping increment:
    if mapping_backend in ('scout_slam', 'parallel_slam'):
        if robot != 'scout' or mode not in ('hold', 'scout'):
            raise ValueError('SLAM backends require robot:=scout and mode:=hold|scout')
        if mode == 'hold' and source not in ('hold', 'test'):
            raise ValueError('SLAM teleop requires source:=hold|test')
        if obstacle_test_pose or test_map or task_file:
            raise ValueError('scout_slam uses the normal spawn and accepts no test_map/task_file')

    # A saved map is test input for navigation, never room-exploration knowledge:
    if test_map and mode != 'navigation':
        raise ValueError('test_map is restricted to mode:=navigation, never room exploration')
    if mode not in ('hold', 'scout') and robot == 'scout':
        raise ValueError('Select explorer_1 or explorer_2 for this navigation stage')
    if mode == 'scout' and robot != 'scout':
        raise ValueError('mode:=scout requires robot:=scout')

    # Reactive overrides the fixed source; managed profiles delegate it to the supervisor:
    managed_navigation = mode in ('navigation', 'room')
    if mode == 'reactive':
        source = 'reactive'
    if managed_navigation:
        source = 'managed'
    elif mode == 'scout':
        source = 'corridor'

    mission_share = get_package_share_directory('mission')
    fleet_path = os.path.join(mission_share, 'config', 'fleet.yaml')
    with open(fleet_path, encoding='utf-8') as stream:
        fleet = yaml.safe_load(stream)

    # Every robot gets its own spawn and protection chain; only one source is selected:
    actions = [include('simulation', 'world.launch.py', gui=gui)]
    actions.extend(create_robot_actions(fleet, robot, source, obstacle_test_pose))

    # Mapping backends are exclusive; parallel SLAM also aligns the explorers' map frames:
    if mapping_backend == 'scout_slam':
        actions.append(include('shared_mapping', 'scout_slam.launch.py'))
    elif mapping_backend == 'parallel_slam':
        actions.append(include('shared_mapping', 'parallel_slam.launch.py'))

    # Mapping and supervised Nav2 are shared by navigation and room profiles:
    if managed_navigation:
        actions.extend(create_managed_navigation_actions(robot, mode, test_map))
    if mode == 'scout':
        # Start exactly one map authority. Local fuzzy geometry still uses encoder odom:
        if mapping_backend == 'assisted':
            actions.extend(create_mapping_actions())
        actions.append(include(
            'scout', 'scout.launch.py', robot=robot,
            autostart=LaunchConfiguration('scout_autostart').perform(context),
        ))
    if managed_navigation or mode == 'reactive':
        actions.append(include(
            'explorer',
            'reactive.launch.py',
            robot=robot,
            managed='true' if managed_navigation else 'false',
        ))

    actions.extend(create_logging_actions(log_file, terminal_enabled))
    if rviz_enabled:
        actions.append(create_rviz_node(robot, mission_share, mode, mapping_backend))
    if task_file:
        if mode != 'room':
            raise ValueError('task_file requires mode:=room')
        actions.append(create_room_assignment(task_file, robot))
    return actions


def include(package: str, filename: str, **arguments) -> GroupAction:
    """Isolate child arguments so one robot's settings cannot leak into another."""
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    child = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )
    return GroupAction([child])


def create_robot_actions(
    fleet: dict, selected_robot: str, selected_source: str, obstacle_test_pose: bool
) -> list:
    """Spawn each Burger at its configured pose and connect its protected command input."""
    actions = []
    for robot, initial_pose in fleet['robots'].items():
        # Test placement replaces only the selected robot's normal triangle pose:
        pose = initial_pose
        source = 'hold'
        if robot == selected_robot:
            source = selected_source
            if obstacle_test_pose:
                pose = fleet['obstacle_test_pose']
        pose_arguments = {coordinate: str(value) for coordinate, value in pose.items()}
        actions.append(include('simulation', 'robot.launch.py', robot=robot, **pose_arguments))

        # Selected input -> command gate -> Collision Monitor -> cmd_vel -> timed drive:
        actions.append(include(
            'navigation',
            'protection.launch.py',
            robot=robot,
            source=source,
        ))
    return actions


def create_mapping_actions() -> list:
    """Share measured-pose localization and live LiDAR mapping across behavior profiles."""
    # The adapter supplies map->odom from measured poses; the mapper uses LiDAR geometry.
    # A saved map reaches Nav2 only through the explicit navigation-test option:
    pose_adapter = Node(
        package='mapping',
        executable='sim_pose_adapter',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    occupancy_mapper = Node(
        package='mapping',
        executable='occupancy_mapper',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return [pose_adapter, occupancy_mapper]


def create_managed_navigation_actions(robot: str, mode: str, test_map: str) -> list:
    """Connect existing localization/mapping, supervised Nav2 and room tasks."""
    navigation = include(
        'indoor_navigation',
        'navigation.launch.py',
        robot=robot,
        roam_on_idle='true' if mode == 'navigation' else 'false',
        test_map=test_map,
    )

    # Room goals use the supervisor; observation positions use this dedicated behavior tree:
    room_parameters = os.path.join(
        get_package_share_directory('explorer'), 'config', 'room_exploration.yaml'
    )
    observation_tree = os.path.join(
        get_package_share_directory('indoor_navigation'),
        'behavior_trees',
        'observe_position.xml',
    )
    room_explorer = Node(
        package='explorer',
        executable='room_explorer',
        namespace=robot,
        parameters=[room_parameters, {
            'use_sim_time': True,
            'observation_behavior_tree': observation_tree,
        }],
        output='screen',
    )
    return create_mapping_actions() + [navigation, room_explorer]


def create_logging_actions(log_file: str, terminal_enabled: bool) -> list:
    """Always start the collector; optionally attach a viewer once its PID is known."""
    logger = Node(
        package='mission',
        executable='mission_logger',
        parameters=[{'use_sim_time': True, 'log_file': log_file}],
        output='screen',
    )
    actions = []
    if terminal_enabled and os.environ.get('DISPLAY') and shutil.which('gnome-terminal'):
        # Register before starting the logger so the viewer receives its process-start event:
        viewer_handler = RegisterEventHandler(OnProcessStart(
            target_action=logger,
            on_start=partial(status_terminal, log_file=log_file),
        ))
        actions.append(viewer_handler)
    actions.append(logger)
    return actions


def status_terminal(event, _context: LaunchContext, log_file: str) -> list:
    """Follow the flushed log file until the launch-owned collector PID exits."""
    viewer = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--wait', '--title=Search Squad Mission Log', '--',
            'tail', '--pid=' + str(event.pid), '-n', '+1', '-F', log_file,
        ],
        output='screen',
    )
    return [viewer]


def create_rviz_node(
    robot: str, mission_share: str, mode: str, mapping_backend: str = 'assisted'
) -> Node:
    """Select the map or odometry view and route goals and overlays to the selected robot."""
    view = robot + '.rviz'
    if mapping_backend == 'parallel_slam':
        view = 'fleet_mapping.rviz'
    elif mapping_backend == 'scout_slam' and mode == 'hold':
        view = 'scout_slam.rviz'
    elif mode in ('navigation', 'room'):
        view = 'navigation.rviz'
    elif mode == 'scout':
        view = 'corridor.rviz'
    remappings = [('goal_pose', '/' + robot + '/goal_pose')]
    # The shared navigation view is authored for explorer_1; redirect its selected overlays:
    if view == 'navigation.rviz':
        for topic in ('nav2/global_costmap/costmap', 'nav2/plan', 'room_markers'):
            remappings.append(('/explorer_1/' + topic, '/' + robot + '/' + topic))
    return Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot,
        name='fleet_rviz',
        arguments=['-d', os.path.join(mission_share, 'rviz', view)],
        parameters=[{'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )


def create_room_assignment(task_file: str, robot: str) -> ExecuteProcess:
    """Send the explicit entrance YAML as an ExploreRoom action goal through the ROS CLI."""
    with open(task_file, encoding='utf-8') as stream:
        task = yaml.safe_load(stream)
    return ExecuteProcess(
        cmd=[
            sys.executable, '/opt/ros/humble/bin/ros2', 'action', 'send_goal',
            '/' + robot + '/explore_room', 'squad_interfaces/action/ExploreRoom',
            json.dumps(task),
        ],
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    """Declare the application interface and defer profile assembly until values resolve."""
    # Profile and command ownership; source is used directly only in hold mode:
    profile_arguments = [
        DeclareLaunchArgument(
            'mode',
            default_value='scout',
            choices=['hold', 'reactive', 'navigation', 'room', 'scout'],
            description='Hold, reactive, navigation, room assignment or Scout corridor behavior.',
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='auto',
            choices=['auto', 'scout', 'explorer_1', 'explorer_2'],
            description='Auto selects Scout in scout mode and explorer_1 in other profiles.',
        ),
        DeclareLaunchArgument(
            'mapping_backend',
            default_value='auto',
            choices=['auto', 'assisted', 'scout_slam', 'parallel_slam'],
            description='Auto uses parallel SLAM and a shared map in scout mode; other profiles retain assisted mapping.',
        ),
        DeclareLaunchArgument(
            'source',
            default_value='hold',
            choices=['hold', 'test', 'nav', 'reactive'],
            description='Fixed command input in hold mode; other modes choose their own source.',
        ),
    ]
    viewer_arguments = [
        DeclareLaunchArgument(
            'scout_autostart', default_value='true', choices=['true', 'false'],
            description='Start Scout once its inputs and supervisor are ready.',
        ),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('log_terminal', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('log_file', default_value='/tmp/search_squad_mission/events.log'),
    ]
    test_arguments = [
        DeclareLaunchArgument(
            'obstacle_test_pose',
            default_value='false',
            choices=['true', 'false'],
            description='Place only the selected robot at the configured obstacle-test pose.',
        ),
        DeclareLaunchArgument(
            'test_map',
            default_value='',
            description='Test-only map, restricted to navigation mode.',
        ),
        DeclareLaunchArgument(
            'task_file',
            default_value='',
            description='Optional explicit room assignment YAML; never a map.',
        ),
    ]

    # Apply the validated middleware to child processes before scheduling their launches:
    middleware = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    fleet_composition = OpaqueFunction(function=compose)
    return LaunchDescription(
        profile_arguments + viewer_arguments + test_arguments + [middleware, fleet_composition]
    )
