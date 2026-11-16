"""Run Scout-only SLAM; the parent launch supplies simulation, odometry and protection.

SLAM Toolbox consumes /scout/scan and scan-time scout/odom TF, then owns /map
and map->scout/odom. Do not run the assisted mapper or pose adapter beside it.
Explorers are not localized by this node and must remain in hold.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Connect the official Humble asynchronous executable to explicit Scout frames."""
    config_path = os.path.join(
        get_package_share_directory('shared_mapping'), 'config', 'scout_slam.yaml'
    )
    # Keep this node at the root: /map and /tf are shared; frames are set in YAML.
    # The encoder drive remains the sole authority for scout/odom->scout/base_footprint:
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='/',
        parameters=[config_path],
        output='screen',
    )
    return LaunchDescription([slam])
