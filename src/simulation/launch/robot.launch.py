"""Instantiate the pinned Burger template with explicit topic and frame prefixes."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Preserve the small spawn lift, service timeout and state-publisher frequency.
SPAWN_HEIGHT_M = '0.005'
SPAWN_TIMEOUT_SECONDS = '90.0'
STATE_PUBLISH_FREQUENCY_HZ = 30.0


def prepare_robot_model(robot: str, simulation_share: str) -> tuple[str, str]:
    """Render a namespaced SDF into a temporary file and load the unchanged joint URDF."""
    mesh_root = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_common'
    )
    template_path = os.path.join(simulation_share, 'models', 'burger', 'model.sdf.in')
    with open(template_path, encoding='utf-8') as stream:
        model = stream.read().replace('@ROBOT@', robot)
        model = model.replace('@MESH_ROOT@', 'file://' + mesh_root)
    with open(os.path.join(simulation_share, 'urdf', 'burger.urdf'), encoding='utf-8') as stream:
        description = stream.read()

    with tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.sdf',
        prefix='search_squad_',
        delete=False,
    ) as stream:
        stream.write(model)
        model_path = stream.name
    return model_path, description


def cleanup_model(_context: LaunchContext, model_path: str) -> list:
    """Remove this launch instance's temporary SDF after shutdown."""
    if os.path.exists(model_path):
        os.unlink(model_path)
    return []


def spawn_robot(context: LaunchContext) -> list:
    """Connect the state publisher and simulator spawn to one rendered robot model."""
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('scout', 'explorer_1', 'explorer_2'):
        raise ValueError('Unknown fleet robot')
    simulation_share = get_package_share_directory('simulation')
    model_path, description = prepare_robot_model(robot, simulation_share)

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_prefix': robot + '/',
            'robot_description': description,
            'publish_frequency': STATE_PUBLISH_FREQUENCY_HZ,
        }],
    )
    entity_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_' + robot,
        parameters=[{'use_sim_time': True}],
        output='screen',
        arguments=[
            '-entity', robot,
            '-file', model_path,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', SPAWN_HEIGHT_M,
            '-Y', LaunchConfiguration('yaw'),
            '-timeout', SPAWN_TIMEOUT_SECONDS,
        ],
    )
    model_cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=cleanup_model, kwargs={'model_path': model_path}),
    ]))
    return [state_publisher, entity_spawn, model_cleanup]


def generate_launch_description() -> LaunchDescription:
    """Declare simulator pose arguments before rendering the robot-specific resources."""
    arguments = [
        DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=spawn_robot)])
