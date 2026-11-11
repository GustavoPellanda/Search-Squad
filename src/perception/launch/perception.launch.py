"""Start local corridor interpretation with the protection layer's physical footprint."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def compose(context: LaunchContext) -> list:
    """Resolve the robot and share footprint configuration without duplicating geometry."""
    robot = LaunchConfiguration('robot').perform(context)
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'corridor_perception.yaml'
    )
    safety_path = os.path.join(
        get_package_share_directory('lidar_safety'), 'config', 'safety.yaml'
    )
    with open(safety_path, encoding='utf-8') as stream:
        safety = yaml.safe_load(stream)
    perception = Node(
        package='perception',
        executable='corridor_perception',
        namespace=robot,
        parameters=[config, {
            'use_sim_time': True,
            'footprint_front': safety['safety.footprint_front'],
            'footprint_half_width': safety['safety.footprint_half_width'],
        }],
        output='screen',
    )
    return [perception]


def generate_launch_description() -> LaunchDescription:
    """The fleet launch supplies sensors and encoder odometry."""
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='scout'),
        OpaqueFunction(function=compose),
    ])
