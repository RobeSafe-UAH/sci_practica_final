"""Launch the independent SCI world and its ROS 2 simulation bridge.

Per-vehicle remapping follows rbsf-racing's rbsf_mvsim launch adapter pattern.
"""

import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sci_mvsim.world import build_world, load_config, prepare_world


def simulation_remappings(robot):
    topics = ('cmd_vel', 'odom', 'base_pose_ground_truth', 'scan', 'tf', 'tf_static', 'collision')
    return [(f'/{robot}/{topic}', f'/_sim/{robot}/{topic}') for topic in topics]


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    config = load_config(config_file)
    robot = config['robot']['name']
    rate_hz = float(config['simulation']['rate_hz'])
    headless = LaunchConfiguration('headless').perform(context).lower()
    if headless not in ('true', 'false'):
        raise ValueError('headless must be true or false')
    run_directory = Path(tempfile.mkdtemp(prefix='sci_mvsim_'))
    cache_directory = os.environ.get('SCI_WORLD_CACHE')
    if cache_directory:
        cached_world = prepare_world(config, cache_directory)
        world = ET.parse(cached_world)
        signal = run_directory / 'scenario.start'
        signal.write_text('0\n', encoding='ascii')
        world.find("./element[@class='sci_start_gate']/start_file").text = str(signal)
        world_file = run_directory / 'world.xml'
        world.write(world_file, encoding='utf-8', xml_declaration=True)
    else:
        world_file = build_world(config, run_directory)
    start_gate = Path(get_package_prefix('sci_start_gate')) / 'lib' / 'libsci_start_gate.so'
    if not start_gate.is_file():
        raise RuntimeError(f'The native start gate is not installed: {start_gate}')
    # MVSim already prefixes every topic with its vehicle name. A node namespace
    # here would duplicate that prefix; the bridge itself runs under /<robot>.
    simulator = Node(
        package='mvsim', executable='mvsim_node', name='sci_simulator',
        parameters=[{
            'world_file': str(world_file),
            # MVSim skips ticks shorter than the physics step. Polling more
            # often avoids losing alternate steps to timer jitter.
            'simul_rate': 10.0 * rate_hz,
            # Publish state only when physics advances.
            'period_ms_publish_tf': 0.0,
            'gui_refresh_period': 1000.0 / rate_hz,
            'headless': headless == 'true',
            'force_publish_vehicle_namespace': True,
            'do_fake_localization': False,
            'publish_tf_odom2baselink': False,
            'use_sim_time': False,
        }],
        remappings=simulation_remappings(robot), output='screen',
        additional_env={'LD_PRELOAD': os.pathsep.join(filter(None, (
            str(start_gate), os.environ.get('LD_PRELOAD'))))},
        on_exit=[Shutdown(reason='MVSim stopped')],
    )
    bridge = Node(
        package='sci_mvsim', executable='simulation_bridge', namespace=robot,
        parameters=[{'config_file': str(config_file), 'use_sim_time': True,
                     'start_signal_file': str(world_file.parent / 'scenario.start')}],
        output='screen', on_exit=[Shutdown(reason='The simulation bridge stopped')],
    )
    return [LogInfo(msg=f'SCI world: {world_file}'), simulator, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='/opt/sci/config/scenario.yaml'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
