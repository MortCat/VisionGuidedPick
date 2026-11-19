"""Use the same validated robot assets for MoveIt, Servo and the gateway."""
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'runtime'))
from model_variant import settings_file


def build_config(name, base, directory):
    directory = Path(directory)
    selected_settings = settings_file(directory.parents[2], name)
    checker = Path(get_package_prefix('ptc_app')) / 'lib/ptc_app/settings_check'
    settings = json.loads(subprocess.check_output(
        [str(checker), str(selected_settings), '--export-json'], text=True))
    source = Path(settings['robot_description_path'])
    urdf = ET.parse(source).getroot()
    # MoveIt and Servo retrieve URDF meshes by URI, including local files.
    for mesh in urdf.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename and '://' not in filename:
            mesh.set('filename', (source.parent / filename).resolve().as_uri())
    cfg = MoveItConfigsBuilder('panda', package_name='moveit_resources_panda_moveit_config').to_moveit_configs()
    cfg.robot_description['robot_description'] = ET.tostring(urdf, encoding='unicode')
    cfg.robot_description_semantic['robot_description_semantic'] = Path(settings['srdf_path']).read_text()
    (directory / (name + '_description.urdf')).write_text(cfg.robot_description['robot_description'])
    return cfg
