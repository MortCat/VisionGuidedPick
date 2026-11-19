import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_normalized_urdf(urdf_path: Path) -> str:
    urdf_path = urdf_path.resolve()
    urdf_xml = urdf_path.read_text(encoding='utf-8')
    identity_path = Path(f'{urdf_path}.identity.yaml')
    identity = yaml.safe_load(identity_path.read_text(encoding='utf-8')) or {}
    dependencies = {
        (urdf_path.parent / item['path']).resolve(): item['sha256']
        for item in (identity.get('dependencies') or [])
    }
    root = ET.fromstring(urdf_xml)
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if not filename:
            continue
        if '://' in filename:
            raise RuntimeError('resolve mesh URI to local asset before deployment')
        resolved = (urdf_path.parent / filename).resolve()
        expected_sha256 = dependencies.get(resolved)
        if not expected_sha256 or hashlib.sha256(resolved.read_bytes()).hexdigest() != expected_sha256:
            raise RuntimeError('URDF mesh missing from verified dependency manifest')
        uri = f'file://{resolved.as_posix()}'
        for quote in ('"', "'"):
            urdf_xml = urdf_xml.replace(f'filename={quote}{filename}{quote}', f'filename={quote}{uri}{quote}')
    return urdf_xml


def _nodes(context):
    path = str(Path(LaunchConfiguration('settings_file').perform(context)).resolve())
    checker = Path(get_package_prefix('ptc_app')) / 'lib' / 'ptc_app' / 'settings_check'
    settings = json.loads(subprocess.check_output([str(checker), path, '--export-json'], text=True))
    if settings['motion_backend'] != 'moveit':
        raise RuntimeError('This launcher requires runtime.motion_backend=moveit.')
    asset = yaml.safe_load(Path(settings['moveit_config_path']).read_text(encoding='utf-8'))
    if not isinstance(asset, dict) or set(asset) != {'move_group', 'servo', 'gateway'}:
        raise RuntimeError('MoveIt parameter asset must contain move_group, servo and gateway maps.')
    if any(not isinstance(value, dict) for value in asset.values()):
        raise RuntimeError('MoveIt parameter sections must be maps.')
    robot = {'robot_description': _load_normalized_urdf(Path(settings['robot_description_path'])),
             'robot_description_semantic': Path(settings['srdf_path']).read_text(encoding='utf-8'), 'use_sim_time': settings['use_sim_time']}
    namespace = settings['namespace']
    if LaunchConfiguration("external_motion_stack").perform(context).lower() == "true":
        return [Node(package="ptc_motion_moveit", executable="arm_gateway_moveit", namespace=namespace,
                     parameters=[asset["gateway"], robot, {"settings_file": path}], output="screen")]
    return [Node(package='moveit_ros_move_group', executable='move_group', namespace=namespace, parameters=[asset['move_group'], robot], output='screen'),
            Node(package='moveit_servo', executable='servo_node', namespace=namespace, parameters=[asset['servo'], robot],
                 remappings=[('~/pause_servo', settings['endpoints']['servo_pause']), ('~/switch_command_type', settings['endpoints']['servo_command_type'])], output='screen'),
            Node(package='ptc_motion_moveit', executable='arm_gateway_moveit', namespace=namespace,
                 parameters=[asset['gateway'], robot, {'settings_file': path}], output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('settings_file'), DeclareLaunchArgument('external_motion_stack', default_value='false'), OpaqueFunction(function=_nodes)])
