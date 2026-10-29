"""Default to three stationary robots; motion requires an explicitly selected source."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package: str, filename: str, **arguments) -> GroupAction:
    """Include a package launch in its own scope so per-robot arguments do not leak."""
    launch_path = os.path.join(get_package_share_directory(package), 'launch', filename)
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=arguments.items(),
    )
    return GroupAction([included_launch])


def compose(context: LaunchContext) -> list:
    """Select spawn poses and wire protection for each independent robot namespace."""
    selected_robot = LaunchConfiguration('robot').perform(context)
    selected_source = LaunchConfiguration('source').perform(context)
    run_demo = LaunchConfiguration('demo').perform(context).lower() == 'true'
    duration = LaunchConfiguration('duration').perform(context)
    use_demo_pose = LaunchConfiguration('demo_pose').perform(context).lower() == 'true'
    if run_demo:
        selected_source = 'reactive'
        use_demo_pose = True
    if selected_source == 'reactive' and selected_robot == 'scout':
        raise ValueError('Reactive demonstration requires an explorer')

    mission_share = get_package_share_directory('mission')
    fleet_path = os.path.join(mission_share, 'config', 'fleet.yaml')
    with open(fleet_path, encoding='utf-8') as stream:
        fleet = yaml.safe_load(stream)

    world = include('simulation', 'world.launch.py', gui=LaunchConfiguration('gui'))
    actions = [world]
    for robot, waiting_pose in fleet['robots'].items():
        spawn_pose = waiting_pose
        if use_demo_pose and robot == selected_robot:
            spawn_pose = fleet['demo_pose']
        pose_arguments = {coordinate: str(value) for coordinate, value in spawn_pose.items()}
        robot_spawn = include('simulation', 'robot.launch.py', robot=robot, **pose_arguments)
        protection = include(
            'navigation',
            'protection.launch.py',
            robot=robot,
            source=selected_source if robot == selected_robot else 'hold',
        )
        actions.extend([robot_spawn, protection])

    # RViz uses one local odom tree; this does not establish a shared global frame:
    rviz_view = Node(
        package='rviz2',
        executable='rviz2',
        name='fleet_rviz',
        output='screen',
        arguments=['-d', os.path.join(mission_share, 'rviz', selected_robot + '.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    actions.append(rviz_view)
    if run_demo:
        reactive_controller = include(
            'explorer',
            'reactive.launch.py',
            robot=selected_robot,
            duration=duration,
        )
        actions.append(reactive_controller)
    return actions


def generate_launch_description() -> LaunchDescription:
    """Expose fleet selection and defer YAML/pose evaluation until arguments are available."""
    arguments = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'robot',
            default_value='explorer_1',
            choices=['scout', 'explorer_1', 'explorer_2'],
        ),
        DeclareLaunchArgument(
            'source',
            default_value='hold',
            choices=['hold', 'reactive', 'test', 'nav'],
        ),
        DeclareLaunchArgument(
            'demo',
            default_value='false',
            choices=['false', 'true'],
            description='Run the bounded reactive demonstration with the selected explorer.',
        ),
        DeclareLaunchArgument('duration', default_value='30.0'),
        DeclareLaunchArgument('demo_pose', default_value='false', choices=['false', 'true']),
    ]
    # Select the validated middleware before any robot or simulator process starts:
    middleware = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    return LaunchDescription(arguments + [middleware, OpaqueFunction(function=compose)])
