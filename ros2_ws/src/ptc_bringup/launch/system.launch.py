"""Four application owners; simulation/controllers remain deployment resources."""
import json
import os
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def _nodes(context):
    path = str(Path(LaunchConfiguration('settings_file').perform(context)).resolve())
    binary = Path(get_package_prefix('ptc_app')) / 'lib' / 'ptc_app' / ('settings_check.exe' if os.name == 'nt' else 'settings_check')
    settings = json.loads(subprocess.check_output([str(binary), path, '--export-json'], text=True))
    parameters = {'settings_file': path, 'use_sim_time': settings['use_sim_time']}
    continuous = LaunchConfiguration('continuous_operation').perform(context).lower()
    if continuous not in ('true', 'false'):
        raise ValueError('continuous_operation must be true or false')
    if LaunchConfiguration('task_workflow').perform(context) != 'pick-only':
        raise ValueError('task_workflow must be pick-only')
    task_parameters = {**parameters, 'continuous_operation': continuous == 'true', 'task_workflow': LaunchConfiguration('task_workflow').perform(context)}
    namespace = settings['namespace']
    vision_remaps = []
    observer_namespace = LaunchConfiguration('observer_namespace').perform(context).strip('/')
    if observer_namespace:
        for key in ('observer_rgb', 'observer_depth', 'observer_rgb_intrinsics', 'observer_depth_intrinsics'):
            endpoint = settings['endpoints'][key]
            vision_remaps.append((endpoint, '/' + observer_namespace + '/' + endpoint))
    shared_tracks_topic = LaunchConfiguration('shared_tracks_topic').perform(context)
    shared_remaps = [(settings['endpoints']['shared_tracks'], shared_tracks_topic)] if shared_tracks_topic else []
    vision_remaps.extend(shared_remaps)
    nodes = [Node(package='ptc_vision', executable='vision', namespace=namespace, parameters=[parameters], remappings=vision_remaps, output='screen'),
             Node(package='ptc_task', executable='task_coordinator', namespace=namespace, parameters=[task_parameters], remappings=shared_remaps, output='screen')]
    if not shared_tracks_topic:
        nodes.append(Node(package='ptc_inference', executable='inference', namespace=namespace,
                          parameters=[parameters], output='screen'))
    if settings['motion_backend'] == 'mock':
        nodes.append(Node(package='ptc_mock', executable='arm_gateway_mock', namespace=namespace, parameters=[parameters], output='screen'))
    elif os.name == 'nt':
        raise RuntimeError('MoveIt backend is supported by the Linux/WSL build, not Windows Pixi.')
    else:
        launch = Path(get_package_share_directory('ptc_moveit_config')) / 'launch' / 'moveit.launch.py'
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments={'settings_file': path, 'external_motion_stack': LaunchConfiguration('external_motion_stack').perform(context)}.items()))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('settings_file', default_value='config/settings.yaml'), DeclareLaunchArgument('external_motion_stack', default_value='false'), DeclareLaunchArgument('observer_namespace', default_value=''), DeclareLaunchArgument('shared_tracks_topic', default_value=''), DeclareLaunchArgument('continuous_operation', default_value='false'), DeclareLaunchArgument('task_workflow', default_value='pick-only'), OpaqueFunction(function=_nodes)])
