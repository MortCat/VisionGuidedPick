"""Deployment-only profile validation and process startup; no application state."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET
from model_variant import model_selection, VARIANTS

DIRECTORY = Path(__file__).resolve().parent
CANONICAL_ROOT = DIRECTORY.parents[1]
COMPONENTS = ('bridge', 'devices', 'motion', 'arm_01', 'arm_02')
ROS_KEYS = ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_BUILTIN_TRANSPORTS', 'ROS_LOCALHOST_ONLY',
            'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS',
            'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
            'RMW_FASTRTPS_PUBLICATION_MODE', 'ROS_SECURITY_ENABLE',
            'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH')


class RuntimeProfileError(ValueError):
    pass


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise RuntimeProfileError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'), object_pairs_hook=unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          RuntimeProfileError('non-finite JSON value: ' + value)))


def keys(value, required, optional=(), field='profile'):
    if not isinstance(value, dict):
        raise RuntimeProfileError(field + ': expected object')
    missing = set(required) - value.keys()
    unknown = value.keys() - set(required) - set(optional)
    if missing or unknown:
        raise RuntimeProfileError(f'{field}: missing={sorted(missing)}, unknown={sorted(unknown)}')


def exact(value, expected, field):
    if type(value) is not type(expected) or value != expected:
        raise RuntimeProfileError(f'{field}: expected {expected!r}')


def native_path(value, base, field):
    if not isinstance(value, str) or not value.strip():
        raise RuntimeProfileError(field + ': expected nonempty path')
    return (base / value).resolve()


def validate_udp_profile(path):
    ns = {'d': 'http://www.eprosima.com'}
    tree = ET.parse(path)
    participant = tree.find('.//d:participant', ns)
    descriptor = tree.find('.//d:transport_descriptor', ns)
    if (participant is None or participant.get('is_default_profile') != 'true'
            or participant.findtext('d:rtps/d:useBuiltinTransports', namespaces=ns) != 'false'
            or descriptor is None or descriptor.findtext('d:type', namespaces=ns) != 'UDPv4'
            or participant.findtext('d:rtps/d:userTransports/d:transport_id', namespaces=ns)
            != descriptor.findtext('d:transport_id', namespaces=ns)):
        raise RuntimeProfileError(str(path) + ': default UDPv4 participant required')


def load_profile(selection):
    path = DIRECTORY / 'profiles' / (selection + '.json') if selection in (
        'linux_local', 'windows_wsl') else Path(selection).resolve()
    value = read_json(path)
    keys(value, ('schema_version', 'profile_id', 'ros', 'dds', 'simulator', 'application', 'network'))
    exact(value['schema_version'], 1, 'schema_version')
    identity = value['profile_id']
    if identity not in ('linux_local', 'windows_wsl'):
        raise RuntimeProfileError('profile_id: unsupported profile')
    for field, expected in {
        'ros': {'distro': 'jazzy', 'rmw_implementation': 'rmw_fastrtps_cpp', 'domain_id': 0},
        'application': {'host_environment': 'wsl' if identity == 'windows_wsl' else 'linux'},
        'simulator': {'host_os': 'windows' if identity == 'windows_wsl' else 'linux',
                      'ros_library_source': 'bundled_jazzy' if identity == 'windows_wsl' else 'native_jazzy',
                      'bridge_extension': 'isaacsim.ros2.bridge'},
        'network': {'wsl_networking_modes': ['nat', 'mirrored']} if identity == 'windows_wsl' else {},
    }.items():
        keys(value[field], expected.keys(), field=field)
        for key, expected_value in expected.items():
            exact(value[field][key], expected_value, field + '.' + key)
    keys(value['dds'], ('transport', 'profile_path'), ('application_profile_path',), field='dds')
    exact(value['dds']['transport'], 'udp_v4', 'dds.transport')
    xml_path = native_path(value['dds']['profile_path'], path.parent, 'dds.profile_path')
    digest = hashlib.sha256()
    sources = [path, xml_path]
    if 'application_profile_path' in value['dds']:
        sources.append(native_path(value['dds']['application_profile_path'], path.parent,
                                   'dds.application_profile_path'))
    for source in sources[1:]:
        validate_udp_profile(source)
    for source in sources:
        raw = source.read_bytes()
        if raw.startswith(b'\xef\xbb\xbf') or b'\r' in raw:
            raise RuntimeProfileError(str(source) + ': UTF-8/LF without BOM required')
        raw.decode('utf-8')
        digest.update(len(raw).to_bytes(8, 'big'))
        digest.update(raw)
    return value, path.resolve(), xml_path, 'sha256:' + digest.hexdigest()


def is_wsl():
    return 'microsoft' in platform.release().lower()


def load_locations(path, profile, purpose):
    explicit = bool(path)
    path = Path(path).resolve() if path else DIRECTORY / 'local/locations.json'
    if path.is_file():
        value = read_json(path)
    elif purpose == 'application' and not explicit:
        value = {'schema_version': 1, 'project_root': str(CANONICAL_ROOT),
                 'ros_prefix': '/opt/ros/jazzy',
                 'host_role': 'wsl_application' if is_wsl() else 'linux_local'}
    else:
        raise RuntimeProfileError(f'locations missing: {path}; copy the matching example and edit paths')
    keys(value, ('schema_version', 'host_role', 'project_root'),
         ('ros_prefix', 'isaac_root', 'scene_path', 'wsl_distribution', 'mcp'), 'locations')
    exact(value['schema_version'], 1, 'locations.schema_version')
    role = value['host_role']
    allowed = ('windows_simulator', 'wsl_application') if profile['profile_id'] == 'windows_wsl' else ('linux_local',)
    if role not in allowed:
        raise RuntimeProfileError('locations.host_role: incompatible with profile')
    if purpose == 'application' and role == 'windows_simulator':
        raise RuntimeProfileError('locations.host_role: application must run in Linux/WSL')
    if purpose == 'isaac' and role == 'wsl_application':
        raise RuntimeProfileError('locations.host_role: this profile runs Isaac on Windows')
    if role != 'windows_simulator' and 'ros_prefix' not in value:
        raise RuntimeProfileError('locations.ros_prefix: required for Linux/WSL')
    result = dict(value)
    for key in ('project_root', 'ros_prefix', 'isaac_root', 'scene_path'):
        if key in value:
            result[key] = str(native_path(value[key], path.parent, 'locations.' + key))
    if 'mcp' in value:
        keys(value['mcp'], ('extension_root', 'host', 'port'), field='locations.mcp')
        exact(value['mcp']['host'], 'localhost', 'locations.mcp.host')
        port = value['mcp']['port']
        if type(port) is not int or not 1 <= port <= 65535:
            raise RuntimeProfileError('locations.mcp.port: integer 1..65535 required')
        result['mcp'] = dict(value['mcp'], extension_root=str(native_path(
            value['mcp']['extension_root'], path.parent, 'locations.mcp.extension_root')))
    if role == 'windows_simulator':
        distro = value.get('wsl_distribution')
        if not isinstance(distro, str) or not distro.strip() or distro.startswith('-'):
            raise RuntimeProfileError('locations.wsl_distribution: expected distro name')
    return result, path


def network_check(profile, locations, errors, warnings):
    if profile['profile_id'] == 'windows_wsl':
        command = ['wslinfo', '--networking-mode'] if os.name != 'nt' else [
            'wsl.exe', '-d', locations['wsl_distribution'], '--', 'wslinfo', '--networking-mode']
        try:
            mode = subprocess.run(command, capture_output=True, encoding='utf-8', errors='replace',
                                  timeout=15, check=True).stdout.strip().lower()
        except (OSError, subprocess.SubprocessError) as error:
            errors.append('network_mode_unavailable: ' + str(error))
            mode = 'unknown'
        if mode not in profile['network']['wsl_networking_modes']:
            errors.append(f'network_profile_mismatch: actual={mode}, supported=nat/mirrored; see deployment/runtime/README.md')
        elif mode == 'nat':
            warnings.append('NAT detected: verify actual DDS discovery and image payload; preflight does not prove transport readiness')
    else:
        mode = 'local'
    if os.name != 'nt':
        for name in ('rmem_max', 'wmem_max'):
            path = Path('/proc/sys/net/core') / name
            if path.exists() and int(path.read_text()) < 4194304:
                warnings.append(f'buffer_capacity: {name}={path.read_text().strip()}, requested=4194304')
    return mode


def application_settings(root, ros_prefix, variant=None):
    """Validate through the existing typed loader, never duplicate the YAML schema."""
    command = ['bash', '--noprofile', '--norc', '-c',
               'set -e; source "$1/setup.bash"; source "$2/native_install/local_setup.bash"; exec "$2/native_install/ptc_app/lib/ptc_app/settings_check" "$3" --export-json',
               'settings-preflight', str(ros_prefix), str(root)]
    environment = {'HOME': os.environ.get('HOME', str(Path.home())), 'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8'}
    from gripper_contact_model import resolve_contact_settings
    resolve_contact_settings(model_selection(root, variant)[1])
    from tempfile import NamedTemporaryFile
    from semantic_settings import resolve
    product = os.environ.get('PTC_SEMANTIC_TRIAL') == 'true'
    result = {}
    for name in ('arm_01', 'arm_02'):
        path = model_selection(root, variant)[1] / (name + '.yaml')
        if product:
            # Keep the temporary file beside its base so relative assets resolve identically.
            with NamedTemporaryFile(suffix='.yaml', dir=path.parent, delete=False) as temporary:
                resolved = Path(temporary.name)
                try:
                    resolve(path, resolved)
                    child = subprocess.run(command + [str(resolved)], env=environment,
                                           capture_output=True, encoding='utf-8', timeout=15)
                finally:
                    resolved.unlink(missing_ok=True)
        else:
            child = subprocess.run(command + [str(path)], env=environment, capture_output=True,
                                   encoding='utf-8', timeout=15)
        if child.returncode:
            raise RuntimeProfileError('application_settings_invalid: ' + name + ': ' + child.stderr.strip())
        settings = json.loads(child.stdout)
        if settings.get('use_sim_time') is not True or settings.get('namespace') != name:
            raise RuntimeProfileError('application_settings_mismatch: simulator requires use_sim_time=true and matching arm namespace')
        result[name] = settings
    return result



def local_control_arguments(locations):
    """Existing App-close lifecycle needs a loopback-only simulator channel."""
    if 'mcp' in locations:
        return []  # Preserve the explicitly configured extension / port owner.
    return ['--enable', 'isaacsim.code_editor.python_server',
            '--/exts/isaacsim.code_editor.python_server/host=127.0.0.1',
            '--/exts/isaacsim.code_editor.python_server/port=8226']


def plan(selection, locations_path=None, purpose='application', variant=None):
    if not selection:
        selection = 'windows_wsl' if is_wsl() else 'linux_local'
    profile, profile_path, xml_path, revision = load_profile(selection)
    if purpose == 'application' and 'application_profile_path' in profile['dds']:
        xml_path = native_path(profile['dds']['application_profile_path'], profile_path.parent,
                               'dds.application_profile_path')
    locations, path = load_locations(locations_path, profile, purpose)
    errors, warnings = [], []
    root = Path(locations['project_root'])
    selected_variant, settings_dir, selected_scene = model_selection(root, variant)
    if purpose == 'isaac' and 'scene_path' in locations:
        configured_scene = Path(locations['scene_path'])
        if configured_scene == selected_scene.with_name('FactoryRabbitDemo.usda'):
            locations['scene_path'] = str(selected_scene)
    environment = {'ROS_DISTRO': profile['ros']['distro'],
                   'RMW_IMPLEMENTATION': profile['ros']['rmw_implementation'], 'ROS_DOMAIN_ID': '0',
                   'FASTDDS_DEFAULT_PROFILES_FILE': str(xml_path), 'FASTRTPS_DEFAULT_PROFILES_FILE': str(xml_path),
                   'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'}
    if profile['profile_id'] == 'windows_wsl':
        # Applies to rmw_fastrtps writers; SDK direct DDS writers may differ.
        environment['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'
    environment.update(PTC_RUNTIME_PROFILE=str(profile_path), PTC_RUNTIME_PROFILE_REVISION=revision,
                       PTC_RUNTIME_HOST_ROLE=locations['host_role'])
    if purpose == 'application':
        if os.name == 'nt' or (profile['profile_id'] == 'windows_wsl') != is_wsl():
            errors.append('host_role_mismatch: profile requires its declared Linux/WSL application host')
        if root != CANONICAL_ROOT:
            errors.append('runtime_root_mismatch: expected ' + str(CANONICAL_ROOT))
        ros_prefix = locations.get('ros_prefix', '')
        for item in (Path(ros_prefix) / 'setup.bash', root / 'native_install/local_setup.bash',
                     root / 'native_install/.native_build_complete', root / 'native_build/sim_servo/agent_sim_servo'):
            if not item.is_file():
                errors.append('missing_runtime_resource: ' + str(item))
        if not errors:
            try:
                application_settings(root, ros_prefix, selected_variant)
            except (RuntimeProfileError, OSError, ValueError, subprocess.SubprocessError) as error:
                errors.append(str(error))
        environment.update(PTC_RUNTIME_PROFILE=str(profile_path), PTC_RUNTIME_LOCATIONS=str(path),
                           PTC_RUNTIME_PROFILE_REVISION=revision, PTC_RUNTIME_HOST_ROLE=locations['host_role'],
                           PTC_RUNTIME_ROS_PREFIX=ros_prefix)
        argv = []
    else:
        for key in ('isaac_root', 'scene_path'):
            if key not in locations:
                raise RuntimeProfileError('locations.' + key + ': required for Isaac launcher')
        sdk = Path(locations['isaac_root'])
        windows = locations['host_role'] == 'windows_simulator'
        if windows != (os.name == 'nt') or is_wsl():
            errors.append('host_role_mismatch: Isaac must run on the declared native host')
        launcher = sdk / ('isaac-sim.bat' if windows else 'isaac-sim.sh')
        required = [launcher, Path(locations['scene_path'])]
        if windows:
            prefix = sdk / 'exts/isaacsim.ros2.core/jazzy'
            required += [prefix / 'lib/rmw_fastrtps_cpp.dll', sdk / 'kit/python/python.exe']
            environment.update(AMENT_PREFIX_PATH=str(prefix), PTC_RUNTIME_ROS_LIB=str(prefix / 'lib'))
        else:
            required += [Path(locations.get('ros_prefix', '')) / 'setup.bash']
        if windows:
            executable = sdk / 'kit/kit.exe'
            app = sdk / 'apps/isaacsim.exp.full.kit'
            required += [executable, app]
            argv = [str(executable), str(app)]
        else:
            argv = [str(launcher)]
        argv += ['--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge']
        argv += local_control_arguments(locations)
        if 'mcp' in locations:
            mcp = locations['mcp']
            required += [Path(mcp['extension_root'])]
            argv += ['--ext-folder', mcp['extension_root'], '--enable', 'isaac.sim.mcp_extension',
                     '--/exts/isaac.sim.mcp/server.host=' + mcp['host'],
                     '--/exts/isaac.sim.mcp/server.port=' + str(mcp['port'])]
        # The Kit exec file CLI opens a stage without pressing Play.
        environment['PTC_RUNTIME_SCENE'] = locations['scene_path']
        argv += ['--exec', str(DIRECTORY / 'open_stage.py')]
        for item in required:
            if not item.exists():
                errors.append('missing_runtime_resource: ' + str(item))
    environment.update(PTC_RUNTIME_MODEL_VARIANT=selected_variant, PTC_RUNTIME_SETTINGS_DIR=str(settings_dir))
    if purpose != 'application' and selected_variant != 'original':
        environment['PTC_RUNTIME_SCENE'] = str(selected_scene)
    mode = network_check(profile, locations, errors, warnings)
    return {'profile_id': profile['profile_id'], 'runtime_profile_revision': revision,
            'profile_path': str(profile_path), 'locations_path': str(path), 'locations': locations,
            'network_mode': mode, 'environment': environment, 'argv': argv,
            'errors': errors, 'warnings': warnings,
            'status': 'preflight_failed' if errors else 'launch_validated'}


def require_valid(value):
    if value['errors']:
        raise RuntimeProfileError('\n'.join(value['errors']))


def process_start(pid):
    return (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()[19]


def entrypoint_revision(root):
    digest = hashlib.sha256()
    for name in ('deployment/linux/run_factory_ros.sh', 'deployment/runtime/runtime_profile.py',
                 'deployment/runtime/gripper_contact_model.py', 'deployment/runtime/semantic_settings.py',
                 'deployment/linux/rotate_component_log.py',
                 'deployment/linux/runtime/inspection_bridge.py', 'deployment/linux/runtime/joint_trajectory_bridge.py',
                 'deployment/linux/runtime/inspection_launch.py', 'deployment/linux/runtime/device_interfaces.launch.py'):
        digest.update((root / name).read_bytes())
    return digest.hexdigest()


def check_running(value, components, require_running=False):
    import fcntl
    root = Path(value['locations']['project_root'])
    for component in components:
        lock = root / 'logs/factory_ros/locks' / (component + '.lock')
        locked = False
        if lock.exists():
            with lock.open('r') as stream:
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    locked = True
        if not locked:
            if require_running:
                raise RuntimeProfileError('component_not_running: ' + component)
            continue
        try:
            record = read_json(lock.with_suffix('.runtime.json'))
            match = (record['runtime_profile_revision'] == value['runtime_profile_revision']
                     and record['entrypoint_revision'] == entrypoint_revision(root)
                     and record.get('model_variant', VARIANTS[0]) == value['environment'].get('PTC_RUNTIME_MODEL_VARIANT', VARIANTS[0])
                     and record['start_time'] == process_start(record['pid'])
                     and (Path('/proc') / str(record['pid']) / 'fd/9').stat().st_ino == lock.stat().st_ino)
        except (OSError, ValueError, KeyError, IndexError):
            match = False
        if match and component in ('arm_01', 'arm_02') and (record.get('continuous_operation') != (os.environ.get('PTC_CONTINUOUS_OPERATION', 'true') == 'true') or record.get('task_workflow', 'inspect-pick') != os.environ.get('PTC_TASK_WORKFLOW', 'pick-only') or record.get('semantic_product', False) != (os.environ.get('PTC_SEMANTIC_TRIAL') == 'true')):
            match = False
        if not match:
            raise RuntimeProfileError('component_restart_required: ' + component + '; Stop Isaac, stop.sh, then restart')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('app-plan', 'isaac-plan', 'run-component', 'check-running', 'record-component'))
    parser.add_argument('components', nargs='*')
    parser.add_argument('--runtime-profile')
    parser.add_argument('--locations')
    parser.add_argument('--model-variant', choices=VARIANTS)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--require-running', action='store_true')
    args = parser.parse_args()
    try:
        if any(component not in COMPONENTS for component in args.components):
            raise RuntimeProfileError('Unknown runtime component')
        if args.command == 'record-component':
            if len(args.components) != 1 or args.dry_run:
                raise RuntimeProfileError('record-component: one component required; dry-run unsupported')
            root = CANONICAL_ROOT
            pid = os.getppid()
            record = {'runtime_profile_revision': os.environ['PTC_RUNTIME_PROFILE_REVISION'],
                      'profile_path': os.environ['PTC_RUNTIME_PROFILE'], 'host_role': os.environ['PTC_RUNTIME_HOST_ROLE'],
                      'entrypoint_revision': entrypoint_revision(root), 'pid': pid, 'start_time': process_start(pid),
                      'continuous_operation': os.environ.get('PTC_CONTINUOUS_OPERATION', 'true') == 'true',
                      'task_workflow': os.environ.get('PTC_TASK_WORKFLOW', 'pick-only'),
                      'semantic_product': os.environ.get('PTC_SEMANTIC_TRIAL') == 'true',
                      'model_variant': os.environ.get('PTC_RUNTIME_MODEL_VARIANT', VARIANTS[0])}
            target = root / 'logs/factory_ros/locks' / (args.components[0] + '.runtime.json')
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(record, indent=2) + '\n')
            temporary.replace(target)
            return 0
        value = plan(args.runtime_profile, args.locations, 'isaac' if args.command == 'isaac-plan' else 'application', variant=args.model_variant)
        if args.command in ('app-plan', 'isaac-plan') or args.dry_run:
            print(json.dumps(value, indent=2))
            return 2 if value['errors'] else 0
        require_valid(value)
        if args.command == 'check-running':
            check_running(value, args.components or COMPONENTS, args.require_running)
            return 0
        if len(args.components) != 1:
            raise RuntimeProfileError('run-component: exactly one component required')
        environment = {'HOME': os.environ.get('HOME', str(Path.home())),
                       'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin', 'LANG': 'C.UTF-8'}
        environment.update(value['environment'])
        continuous = os.environ.get('PTC_CONTINUOUS_OPERATION', 'true')
        if continuous not in ('true', 'false'):
            raise RuntimeProfileError('PTC_CONTINUOUS_OPERATION must be true or false')
        workflow = os.environ.get('PTC_TASK_WORKFLOW', 'pick-only')
        if workflow != 'pick-only':
            raise RuntimeProfileError('PTC_TASK_WORKFLOW must be pick-only')
        environment['PTC_SEMANTIC_TRIAL'] = os.environ.get('PTC_SEMANTIC_TRIAL', 'false')
        environment['PTC_TASK_WORKFLOW'] = workflow
        environment['PTC_CONTINUOUS_OPERATION'] = continuous
        os.execvpe('bash', ['bash', '--noprofile', '--norc', str(CANONICAL_ROOT / 'deployment/linux/run_factory_ros.sh'),
                           '--clean', args.components[0]], environment)
    except (RuntimeProfileError, OSError, json.JSONDecodeError, ET.ParseError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
