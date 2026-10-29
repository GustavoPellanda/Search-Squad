"""Connect one selected source, the freshness gate and the Humble Collision Monitor."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Preserve the initial discovery delay and lifecycle bond timeout in wall seconds.
DISCOVERY_DELAY_SECONDS = 1.0
BOND_TIMEOUT_SECONDS = 2.0


def configure(context: LaunchContext) -> list:
    """Resolve namespace-dependent frames and assemble the protected command chain."""
    robot = LaunchConfiguration('robot').perform(context)
    source = LaunchConfiguration('source').perform(context)
    config_path = os.path.join(
        get_package_share_directory('navigation'), 'config', 'protection.yaml'
    )
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    gate_parameters = dict(
        config['command_gate'],
        use_sim_time=True,
        source=source,
        scan_frame=robot + '/base_scan',
        base_frame=robot + '/base_footprint',
        odom_frame=robot + '/odom',
    )
    monitor_parameters = dict(
        config['collision_monitor'],
        base_frame_id=robot + '/base_footprint',
        odom_frame_id=robot + '/odom',
    )
    command_gate = Node(
        package='navigation',
        executable='command_gate',
        name='command_gate',
        namespace=robot,
        output='screen',
        parameters=[gate_parameters],
    )
    collision_monitor = Node(
        package='navigation',
        executable='collision_monitor_runner',
        name='collision_monitor',
        namespace=robot,
        output='screen',
        parameters=[monitor_parameters],
    )
    protection_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='protection_manager',
        namespace=robot,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['collision_monitor'],
            'bond_timeout': BOND_TIMEOUT_SECONDS,
        }],
    )
    # Let discovery settle before the first lifecycle service request:
    delayed_manager = TimerAction(
        period=DISCOVERY_DELAY_SECONDS,
        actions=[protection_manager],
    )
    return [command_gate, collision_monitor, delayed_manager]


def generate_launch_description() -> LaunchDescription:
    """Declare fixed source selection and evaluate robot-specific configuration at launch."""
    robot_argument = DeclareLaunchArgument('robot')
    source_argument = DeclareLaunchArgument(
        'source',
        default_value='hold',
        choices=['hold', 'reactive', 'test', 'nav'],
    )
    return LaunchDescription([
        robot_argument,
        source_argument,
        OpaqueFunction(function=configure),
    ])
