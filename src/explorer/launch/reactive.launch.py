"""Start the bounded reactive controller upstream of the selected robot's protection."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Load the reactive thresholds and expose the existing duration override."""
    robot = LaunchConfiguration('robot')
    duration = LaunchConfiguration('duration')
    config_path = os.path.join(
        get_package_share_directory('explorer'), 'config', 'reactive.yaml'
    )
    robot_argument = DeclareLaunchArgument(
        'robot',
        default_value='explorer_1',
        choices=['explorer_1', 'explorer_2'],
    )
    duration_argument = DeclareLaunchArgument('duration', default_value='30.0')

    reactive_controller = Node(
        package='explorer',
        executable='reactive_demo',
        name='reactive_demo',
        namespace=robot,
        output='screen',
        parameters=[
            config_path,
            {'duration': ParameterValue(duration, value_type=float)},
        ],
    )
    return LaunchDescription([robot_argument, duration_argument, reactive_controller])
