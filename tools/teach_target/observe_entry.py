"""Prepare only the configured ROS environment; never start motion components."""
import os
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'deployment/runtime'))
    import runtime_profile
    selection = os.environ.get('PTC_RUNTIME_PROFILE') or ('windows_wsl' if runtime_profile.is_wsl() else 'linux_local')
    profile, profile_path, xml_path, revision = runtime_profile.load_profile(selection)
    locations, _ = runtime_profile.load_locations(os.environ.get('PTC_RUNTIME_LOCATIONS'), profile, 'application')
    if 'application_profile_path' in profile['dds']:
        xml_path = runtime_profile.native_path(profile['dds']['application_profile_path'], profile_path.parent, 'dds.application_profile_path')
    setup = Path(locations['ros_prefix']) / 'setup.bash'
    if not setup.is_file(): raise RuntimeError('ROS setup is missing: ' + str(setup))
    overlay = root / 'native_install/local_setup.bash'
    raw = subprocess.check_output([
        'bash', '--noprofile', '--norc', '-c',
        'source "$1" >&2 || exit; if [[ -f "$2" ]]; then source "$2" >&2 || exit; fi; env -0',
        'observe', str(setup), str(overlay),
    ])
    environment = dict(item.split('=', 1) for item in raw.decode().split('\0') if '=' in item)
    environment.update(ROS_DOMAIN_ID=str(profile['ros']['domain_id']),
                       RMW_IMPLEMENTATION=profile['ros']['rmw_implementation'],
                       FASTDDS_DEFAULT_PROFILES_FILE=str(xml_path), FASTRTPS_DEFAULT_PROFILES_FILE=str(xml_path),
                       ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET')
    environment.pop('ROS_LOCALHOST_ONLY', None)
    os.execve(sys.executable, [sys.executable, '-u', str(root / 'tools/teach_target/observe_worker.py')], environment)


if __name__ == '__main__': main()
