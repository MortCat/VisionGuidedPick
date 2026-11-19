"""Provision the application explicitly; daily startup never installs packages."""
import argparse
from contextlib import contextmanager
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
LOCAL = Path('deployment/runtime/local')
MINIMUM_PYTHON = (3, 12)


def require_python():
    if sys.version_info[:2] < MINIMUM_PYTHON:
        raise ValueError('Python 3.12 or newer is required; no setup changes were made')


def start_entry(root):
    spec = importlib.util.spec_from_file_location('project_start_check', root / 'deployment/runtime/application.py')
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    return entry


def run(argv, root=ROOT, **kwargs):
    print('+ ' + subprocess.list2cmdline([str(x) for x in argv]), flush=True)
    subprocess.run([str(x) for x in argv], cwd=root, check=True, **kwargs)


def write_new_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        saved = json.loads(path.read_text(encoding='utf-8-sig'))
        if not isinstance(saved, dict):
            raise ValueError(f'{path}: expected a JSON object')
        return saved
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
    return value


def require_separate_wsl_root(path):
    value = PurePosixPath(path)
    if not value.is_absolute() or '..' in value.parts or any(c in path for c in '\r\n\x00'):
        raise ValueError('WSL project root must be an absolute Linux path')
    if value.is_relative_to(PurePosixPath('/mnt')):
        raise ValueError('Use a separate WSL Linux checkout, not a /mnt Windows checkout')
    return str(value)


def linux_locations(root, isaac_root, ros_prefix, wsl):
    path = root / LOCAL / 'locations.json'
    role = 'wsl_application' if wsl else 'linux_local'
    value = {'schema_version': 1, 'host_role': role, 'project_root': str(root),
             'ros_prefix': str(ros_prefix)}
    if not wsl:
        value.update(isaac_root=str(isaac_root), scene_path=str(root /
            'deployment/factory_rabbit_20261005/scene/FactoryRabbitDemo.camera-bracket-v2.usda'))
    saved = write_new_json(path, value)
    selected = Path(saved.get('project_root', str(root))).expanduser()
    selected = selected if selected.is_absolute() else path.parent / selected
    if selected.resolve() != root.resolve():
        raise ValueError(f'{path}: existing project_root selects a different source directory; preserved it')
    if saved.get('host_role') != role:
        raise ValueError(f'{path}: existing host_role differs; preserve and review it')
    return saved


def native_setup(args, root=ROOT):
    override = os.environ.get('PTC_TEACH_CONFIG')
    if override and Path(override).expanduser().absolute() != (root / LOCAL / 'teaching.yaml').absolute():
        raise ValueError('Setup uses local/teaching.yaml; unset PTC_TEACH_CONFIG or reconcile that override explicitly')
    wsl = bool(os.environ.get('WSL_DISTRO_NAME'))
    location = root / LOCAL / 'locations.json'
    old = read_existing_object(location)
    ros = Path(old.get('ros_prefix', args.ros_prefix))
    sdk = Path(old.get('isaac_root', args.isaac_root or '/opt/isaacsim'))
    missing = [name for name in ('bash', 'git', 'cmake', 'g++', 'colcon') if not shutil.which(name)]
    if not (ros / 'setup.bash').is_file():
        missing.append(f'ROS Jazzy at {ros}; install ROS / MoveIt and development dependencies from docs/getting-started.md')
    if not wsl and not (sdk / 'isaac-sim.sh').is_file():
        missing.append(f'Isaac Sim at {sdk}; supply --isaac-root or review local/locations.json')
    if sys.version_info < (3, 12):
        missing.append('Python 3.12 or newer')
    if missing:
        raise ValueError('Missing prerequisites:\n- ' + '\n- '.join(missing))
    demo_bundle = getattr(args, 'demo_model_bundle', None)
    demo_acquire = ['python3', 'deployment/runtime/default_model.py', '--acquire']
    if demo_bundle:
        demo_acquire += ['--bundle', str(demo_bundle)]
    commands = [demo_acquire,
        ['python3', 'prepare_scene_assets.py', str(args.asset_root)] if args.asset_root else
            ['python3', 'prepare_scene_assets.py'],
        ['python3', '-m', 'venv', '.teach_api_env'],
        ['.teach_api_env/bin/python', '-m', 'pip', 'install', '-r', 'tools/teach_target/requirements.txt',
            '-c', 'tools/teach_target/constraints-api-linux-py312.txt'],
    ]
    if not args.ml_python:
        commands.extend([
            ['python3', '-m', 'venv', '.teach_ml_env'],
            ['.teach_ml_env/bin/python', '-m', 'pip', 'install', '--no-cache-dir', 'torch==2.11.0', 'torchvision==0.26.0',
                '--index-url', 'https://download.pytorch.org/whl/cu128'],
            ['.teach_ml_env/bin/python', '-m', 'pip', 'install', '--no-cache-dir', '-r', 'tools/teach_target/requirements-ml.txt',
                '-c', 'tools/teach_target/constraints-ml-linux-cu128.txt'],
        ])
    if args.dry_run:
        print(json.dumps({'root': str(root), 'host_role': 'wsl_application' if wsl else 'linux_local',
                          'commands': commands, 'remaining': ['default-model registration', 'pip check', 'CUDA/import check', 'SAM setup', 'native build']}, indent=2))
        return 0
    if args.check:
        check_installation(root, args.ml_python)
        return 0
    saved = linux_locations(root, sdk, ros, wsl)
    existing_api = root / '.teach_api_env/bin/python'
    if existing_api.is_file() and subprocess.run(
            [str(existing_api), str(root / 'deployment/runtime/default_model.py'), '--check'],
            cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        commands.remove(demo_acquire)
    # Validate the selected ROS dependencies before downloading ML wheels.
    run(['bash', '--noprofile', '--norc', '-c',
         'set -e; source "$1/setup.bash"; for p in moveit_ros_move_group moveit_servo moveit_configs_utils moveit_resources_panda_moveit_config control_msgs realtime_tools; do ros2 pkg prefix "$p" >/dev/null; done',
         'setup-prerequisites', str(ros)], root)
    for command in commands:
        # Reuse local venvs without clearing or replacing their files.
        if command[:3] == ['python3', '-m', 'venv']:
            python = root / command[3] / 'bin/python'
            if python.is_file() and subprocess.run([str(python), '-m', 'pip', '--version'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                continue
        run(command, root)
    api = root / '.teach_api_env/bin/python'
    teaching = root / LOCAL / 'teaching.yaml'
    default_ml = str(Path(args.ml_python).expanduser().absolute()) if args.ml_python else str(root / '.teach_ml_env/bin/python')
    config_code = '''import sys,yaml
from pathlib import Path
root=Path(sys.argv[1]); target=root/'deployment/runtime/local/teaching.yaml'
if not target.exists():
 cfg=yaml.safe_load((root/'config/teaching.yaml').read_text())
 cfg['training']['python']=sys.argv[2];cfg['annotation']['python']=sys.argv[2]
 with target.open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
'''
    run([api, '-c', config_code, str(root), default_ml], root)
    ml = configured_ml(root, api)
    if args.ml_python and Path(ml).expanduser().absolute() != Path(args.ml_python).expanduser().absolute():
        raise ValueError('Existing teaching.yaml selects a different ML interpreter; preserved it. Review before retrying.')
    demo_install = [api, 'deployment/runtime/default_model.py', '--install']
    if demo_bundle:
        demo_install += ['--bundle', str(demo_bundle)]
    run(demo_install, root)
    run([ml, '-m', 'pip', 'check'], root)
    run([ml, '-c', 'import torch,rfdetr,pytorch_lightning,pycocotools,cv2; assert torch.cuda.is_available(), "CUDA unavailable"; print(torch.__version__, "CUDA ready")'], root)
    run([api, 'tools/teach_target/setup_sam.py', '--config', str(teaching)], root)
    environment = dict(os.environ, PTC_BUILD_ROS_PREFIX=str(ros))
    run(['bash', 'deployment/linux/build_native.sh'], root, env=environment)
    check_installation(root, ml)
    print('Setup complete. Open the simulator separately, then run bash start.sh.')
    return 0


def configured_ml(root, api):
    code = "from teaching_config import load_config; from pathlib import Path; import sys; print(load_config(Path(sys.argv[1]))['training']['python'])"
    result = subprocess.run([str(api), '-c', code, str(root)], cwd=root / 'tools/teach_target',
                            text=True, capture_output=True, check=True)
    return result.stdout.strip()


def check_installation(root, ml_override=None):
    api = root / '.teach_api_env/bin/python'
    if not api.is_file():
        raise ValueError('API venv missing; run Setup without --check first')
    ml = configured_ml(root, api)
    if ml_override and Path(ml).expanduser().absolute() != Path(ml_override).expanduser().absolute():
        raise ValueError('Configured ML interpreter differs from --ml-python')
    run([api, '-m', 'pip', 'check'], root)
    run([ml, '-m', 'pip', 'check'], root)
    run([ml, '-c', 'import torch,rfdetr,pytorch_lightning,pycocotools,cv2; assert torch.cuda.is_available(), "CUDA unavailable"; print(torch.__version__, "CUDA ready")'], root)
    annotation_check = '''import sys,hashlib,subprocess
from pathlib import Path
root=Path(sys.argv[1]);sys.path.insert(0,str(root/'tools/teach_target'))
from teaching_config import load_config
from setup_sam import CHECKSUM,REVISION
cfg=load_config(root)['annotation'];checkpoint=root/cfg['checkpoint'];source=root/cfg['source'];deps=root/cfg['dependencies']
assert checkpoint.is_file() and hashlib.sha256(checkpoint.read_bytes()).hexdigest()==CHECKSUM, 'SAM checkpoint missing or changed; rerun Setup'
assert deps.is_dir(), 'SAM annotation dependencies missing'
assert subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()==REVISION, 'SAM source revision differs'
print('SAM source and checkpoint verified')
'''
    run([api, '-c', annotation_check, str(root)], root)
    run([api, 'deployment/runtime/default_model.py', '--check'], root)
    run(['python3', 'prepare_scene_assets.py', '--check'], root)
    if not (root / 'native_install/.native_build_complete').is_file():
        raise ValueError('Native build missing; run Setup without --check first')
    run(['python3', 'deployment/runtime/release_source_manifest.py', '--check', 'native_install/.source_manifest.json'], root)
    print('Setup verification passed. No services or robot tasks were started.')


def windows_setup(args, root=ROOT):
    require_python()
    if not (root / '.git').exists():
        raise ValueError('Windows Setup requires a Git clone; clone the repository instead of extracting a ZIP')
    if not all(shutil.which(name) for name in ('wsl.exe', 'pwsh.exe', 'git')):
        raise ValueError('Install Git, WSL Ubuntu 24.04 and PowerShell 7 first; see docs/getting-started.md')
    path = root / LOCAL / 'locations.json'
    old = read_existing_object(path)
    distro = old.get('wsl_distribution', args.wsl_distribution)
    sdk = Path(old.get('isaac_root', args.isaac_root or 'C:/isaacsim'))
    if not (sdk / 'isaac-sim.bat').is_file():
        raise ValueError(f'Isaac Sim missing at {sdk}; supply --isaac-root')
    def wsl(*argv, **kwargs):
        return subprocess.run(['wsl.exe', '-d', distro, '--exec', *argv], check=True, **kwargs)
    launcher = root / LOCAL / 'launcher.json'
    prior = read_existing_object(launcher)
    target = args.wsl_project_root or prior.get('wsl_project_root')
    if not target:
        home = wsl('printenv', 'HOME', capture_output=True, text=True).stdout.strip()
        target = str(PurePosixPath(home) / 'workspace/VisionGuidedPick')
    target = require_separate_wsl_root(target)
    if old and (old.get('schema_version') != 1 or old.get('host_role') != 'windows_simulator'):
        raise ValueError('Existing Windows locations.json has an incompatible schema or host role; preserved it')
    if prior and prior.get('wsl_project_root') != target:
        raise ValueError('Existing launcher.json selects a different checkout; preserved it')
    start = start_entry(root)
    if args.dry_run:
        changed, _, _ = start.source_drift(root, distro, target)
        if changed:
            raise ValueError(start.describe_drift(target, changed))
        print(json.dumps({'host': 'windows', 'wsl_distribution': distro, 'wsl_project_root': target,
                          'actions': ['preserve host settings', 'prepare Windows scene assets',
                                      'stage source only if WSL destination is absent', 'run WSL Setup']}, indent=2))
        return 0
    if args.check:
        start.windows_application(root)
        start.require_matching_wsl_source(root, distro, target)
        run([sys.executable, 'prepare_scene_assets.py', '--check'], root)
        wsl('python3', str(PurePosixPath(target) / 'deployment/runtime/setup.py'), '--check')
        return 0
    # Bootstrap a separate source snapshot, never overwrite an existing WSL tree.
    probe = wsl('python3', '-c', 'from pathlib import Path;import sys; print(int(Path(sys.argv[1]).exists()))', target,
                text=True, capture_output=True).stdout.strip()
    if probe == '0':
        names = subprocess.check_output(['git', 'ls-files', '--cached', '--others', '--exclude-standard'], cwd=root, text=True).splitlines()
        with tempfile.TemporaryFile() as archive:
            with tarfile.open(fileobj=archive, mode='w') as bundle:
                for name in sorted(set(names)):
                    p = root / name
                    if p.is_file() and not p.is_symlink():
                        bundle.add(p, arcname=name, recursive=False)
            archive.seek(0)
            extract = "import sys,tarfile,tempfile;from pathlib import Path;p=Path(sys.argv[1]);p.parent.mkdir(parents=True,exist_ok=True);tmp=tempfile.TemporaryDirectory(prefix=p.name+'.partial-',dir=p.parent);stage=Path(tmp.name)/'source';stage.mkdir();tarfile.open(fileobj=sys.stdin.buffer,mode='r|').extractall(stage,filter='data');stage.rename(p);tmp.cleanup()"
            wsl('python3', '-c', extract, target, stdin=archive)
    start.require_matching_wsl_source(root, distro, target)
    demo_bundle = getattr(args, 'demo_model_bundle', None)
    demo_command = [sys.executable, 'deployment/runtime/default_model.py', '--acquire']
    if demo_bundle:
        demo_command += ['--bundle', str(demo_bundle)]
    run(demo_command, root)
    demo_source = Path(demo_bundle).resolve() if demo_bundle else (root / 'artifacts/default-model-cache' /
        json.loads((root / 'config/default_model.json').read_text())['archive_name'])
    mapped_demo = wsl('wslpath', '-a', '-u', str(demo_source), text=True, capture_output=True).stdout.strip()
    asset_command = [sys.executable, 'prepare_scene_assets.py']
    if args.asset_root:
        asset_command.append(str(args.asset_root))
    run(asset_command, root)
    mapped_assets = wsl('wslpath', '-a', '-u', str(root / 'deployment/factory_rabbit_20261005/assets/official/isaac_6_1'),
                        text=True, capture_output=True).stdout.strip()
    command = ['python3', str(PurePosixPath(target) / 'deployment/runtime/setup.py'),
               '--ros-prefix', args.ros_prefix, '--asset-root', mapped_assets, '--demo-model-bundle', mapped_demo]
    if args.ml_python:
        command += ['--ml-python', args.ml_python]
    wsl(*command)
    saved = write_new_json(path, {'schema_version': 1, 'host_role': 'windows_simulator',
        'project_root': str(root), 'isaac_root': str(sdk), 'scene_path': str(root /
        'deployment/factory_rabbit_20261005/scene/FactoryRabbitDemo.camera-bracket-v2.usda'),
        'wsl_distribution': distro})
    if saved.get('host_role') != 'windows_simulator':
        raise ValueError('Existing Windows locations.json has a different host_role; preserved it')
    saved_launcher = write_new_json(launcher, {'schema_version': 1, 'wsl_project_root': target})
    if saved_launcher.get('wsl_project_root') != target:
        raise ValueError('Existing launcher.json selects a different checkout; preserved it')
    print('Setup complete. Run ./start.ps1 on Windows.')
    return 0


def read_existing_object(path):
    if not path.exists():
        return {}
    value = json.loads(path.read_text(encoding='utf-8-sig'))
    if not isinstance(value, dict):
        raise ValueError(f'{path}: expected a JSON object')
    return value


@contextmanager
def provisioning_lock(root):
    import fcntl
    path = root / 'logs/factory_ros/project-start.lock'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError('Startup or shutdown is in progress; stop the project before Setup') from error
        for component in ('bridge', 'devices', 'motion', 'arm_01', 'arm_02', 'monitor'):
            component_lock = path.parent / 'locks' / (component + '.lock')
            if component_lock.exists():
                with component_lock.open('r') as stream:
                    try:
                        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError as error:
                        raise ValueError('Backend is running; stop this project before Setup') from error
        record = root / 'logs/teach_target/process.json'
        if record.is_file():
            row = read_existing_object(record)
            try:
                stat = Path('/proc', str(int(row['pid'])), 'stat').read_text().rsplit(')', 1)[1].split()
                if stat[0] != 'Z' and str(row['start']) == stat[19]:
                    raise ValueError('Web service is running; stop this project before Setup')
            except (FileNotFoundError, KeyError, IndexError):
                pass
        yield


def main(argv=None):
    try:
        require_python()
    except ValueError as error:
        print(f'Setup failed: {error}', file=sys.stderr)
        return 2
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-root', help='Installed simulator path; used only when creating local settings')
    parser.add_argument('--ros-prefix', default='/opt/ros/jazzy')
    parser.add_argument('--wsl-distribution', default='Ubuntu-24.04')
    parser.add_argument('--wsl-project-root', help='Separate absolute Linux source directory; Windows only')
    parser.add_argument('--demo-model-bundle', type=Path, help='Verified demonstration ZIP for offline/private model setup')
    parser.add_argument('--asset-root', type=Path, help='Authorized local Isaac asset source for offline Linux setup')
    parser.add_argument('--ml-python', help='Reuse and verify an existing Linux ML interpreter instead of installing one')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='Verify existing setup without installing or building')
    mode.add_argument('--dry-run', action='store_true', help='Show setup actions without writing or installing')
    args = parser.parse_args(argv)
    try:
        if sys.platform == 'win32':
            return windows_setup(args)
        if sys.platform == 'linux':
            if args.check or args.dry_run:
                return native_setup(args)
            with provisioning_lock(ROOT):
                return native_setup(args)
        raise ValueError('Supported hosts: native Linux or Windows with WSL Ubuntu')
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'Setup failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
