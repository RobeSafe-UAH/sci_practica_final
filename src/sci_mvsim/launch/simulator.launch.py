"""Launch the independent SCI world and its ROS 2 simulation bridge.

Per-vehicle remapping follows rbsf-racing's rbsf_mvsim launch adapter pattern.
"""

from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sci_mvsim.world import build_world, load_config


def simulation_remappings(robot):
    topics = ('cmd_vel', 'odom', 'base_pose_ground_truth', 'scan', 'tf', 'tf_static',
              'collision', *(f'sonar{index}_scan' for index in range(8)))
    return [(f'/{robot}/{topic}', f'/_sim/{robot}/{topic}') for topic in topics]


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    config = load_config(config_file)
    robot = config['robot']['name']
    headless = LaunchConfiguration('headless').perform(context).lower()
    if headless not in ('true', 'false'):
        raise ValueError('headless must be true or false')
    world_file = build_world(config, Path(tempfile.mkdtemp(prefix='sci_mvsim_')))
    # MVSim already prefixes every topic with its vehicle name. A node namespace
    # here would duplicate that prefix; the bridge itself runs under /<robot>.
    simulator = Node(
        package='mvsim', executable='mvsim_node', name='sci_simulator',
        parameters=[{
            'world_file': str(world_file),
            'headless': headless == 'true',
            'force_publish_vehicle_namespace': True,
            'do_fake_localization': False,
            'publish_tf_odom2baselink': False,
            'use_sim_time': False,
        }],
        remappings=simulation_remappings(robot), output='screen',
        on_exit=[Shutdown(reason='MVSim stopped')],
    )
    bridge = Node(
        package='sci_mvsim', executable='simulation_bridge', namespace=robot,
        parameters=[{'config_file': str(config_file), 'use_sim_time': True}],
        output='screen', on_exit=[Shutdown(reason='The simulation bridge stopped')],
    )
    return [LogInfo(msg=f'SCI world: {world_file}'), simulator, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='/opt/sci/config/scenario.yaml'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
