"""Compose the permanent Scout behavior and its local corridor perception."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """The main fleet launch owns simulation, mapping and protection."""
    robot = LaunchConfiguration('robot')
    perception_path = os.path.join(
        get_package_share_directory('perception'), 'launch', 'perception.launch.py'
    )
    config = os.path.join(
        get_package_share_directory('scout'), 'config', 'corridor_following.yaml'
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_path),
        launch_arguments={'robot': robot}.items(),
    )
    follower = Node(
        package='scout',
        executable='corridor_follower',
        namespace=robot,
        parameters=[config, {
            'use_sim_time': True,
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='scout'),
        DeclareLaunchArgument('autostart', default_value='false', choices=['true', 'false']),
        perception,
        follower,
    ])
