"""Independent SLAM per robot with measured initial alignment and one fused /map.

The parent supplies robots, raw sensors, encoder TF and command protection.
Only initialization queries Gazebo poses; no observations or graph constraints
are exchanged between the individual SLAM estimators.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Filter structural scans, estimate three trajectories and merge replaceable grids."""
    share = get_package_share_directory('shared_mapping')
    with open(os.path.join(share, 'config', 'scout_slam.yaml'), encoding='utf-8') as stream:
        base_parameters = yaml.safe_load(stream)['slam_toolbox']['ros__parameters']
    fleet_parameters = os.path.join(share, 'config', 'parallel_mapping.yaml')
    actions = [Node(
        package='shared_mapping',
        executable='slam_frontend',
        name='slam_frontend',
        parameters=[fleet_parameters],
        output='screen',
    )]
    for robot in ('scout', 'explorer_1', 'explorer_2'):
        parameters = dict(base_parameters)
        parameters.update({
            'map_frame': 'map' if robot == 'scout' else robot + '/slam_map',
            'odom_frame': robot + '/odom',
            'base_frame': robot + '/base_footprint',
            'scan_topic': '/' + robot + '/scan_mapping',
            'map_name': '/' + robot + '/local_map',
            'enable_interactive_mode': False,
            'use_map_saver': False,
            # Parallel walls weakly constrain longitudinal scan matching. Prefer
            # nearby encoder predictions while keeping laser matching/loop closure active.
            'distance_variance_penalty': 0.01,
            'angle_variance_penalty': 0.10,
        })
        actions.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot,
            parameters=[parameters],
            output='screen',
        ))
    actions.append(Node(
        package='shared_mapping',
        executable='map_fusion',
        name='map_fusion',
        parameters=[fleet_parameters],
        output='screen',
    ))
    return LaunchDescription(actions)
