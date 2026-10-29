"""Compatibility entry point for the reactive mode of the main fleet launch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Forward existing demonstration arguments to the canonical mission entry point."""
    mission_share = get_package_share_directory('mission')
    arguments = [
        DeclareLaunchArgument(
            'robot',
            default_value='explorer_1',
            choices=['explorer_1', 'explorer_2'],
        ),
        DeclareLaunchArgument('duration', default_value='30.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    fleet_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mission_share, 'launch', 'fleet.launch.py')),
        launch_arguments={
            'demo': 'true',
            'robot': LaunchConfiguration('robot'),
            'duration': LaunchConfiguration('duration'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    return LaunchDescription(arguments + [GroupAction([fleet_demo])])
