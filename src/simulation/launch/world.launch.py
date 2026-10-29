"""Start the original four-room world using resources from installed packages."""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SIMULATION_SEED = '7'


def generate_launch_description() -> LaunchDescription:
    """Configure model/plugin discovery and start the server with an optional GUI."""
    simulation_share = get_package_share_directory('simulation')
    gazebo_share = get_package_share_directory('gazebo_ros')
    plugin_directory = os.path.join(get_package_prefix('simulation'), 'lib')
    plugin_path = plugin_directory + os.pathsep + os.environ.get('GAZEBO_PLUGIN_PATH', '')
    gui_argument = DeclareLaunchArgument('gui', default_value='true')

    # Walls, floor and light are in the world; no model database is required:
    model_database = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    plugin_environment = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(simulation_share, 'worlds', 'four_rooms.world'),
            'server_required': 'true',
            'seed': SIMULATION_SEED,
            'params_file': os.path.join(simulation_share, 'config', 'gazebo.yaml'),
        }.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    return LaunchDescription([
        gui_argument,
        model_database,
        plugin_environment,
        gazebo_server,
        gazebo_client,
    ])
