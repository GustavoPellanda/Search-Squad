"""Start the permanent reactive controller upstream of the robot's protection."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Select continuous standalone roaming or supervisor-managed maneuvers."""
    robot = LaunchConfiguration('robot')
    managed = ParameterValue(LaunchConfiguration('managed'), value_type=bool)
    config_path = os.path.join(
        get_package_share_directory('explorer'), 'config', 'reactive.yaml'
    )
    arguments = [
        DeclareLaunchArgument('robot', default_value='explorer_1'),
        DeclareLaunchArgument('managed', default_value='false', choices=['true', 'false']),
    ]

    # In managed mode the controller waits for supervisor leases before issuing maneuvers:
    controller = Node(
        package='explorer',
        executable='reactive_controller',
        namespace=robot,
        name='reactive_controller',
        output='screen',
        parameters=[config_path, {'managed': managed}],
    )
    return LaunchDescription(arguments + [controller])
