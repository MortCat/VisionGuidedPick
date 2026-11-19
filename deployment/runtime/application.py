"""Daily application entry: backend first, then the optional desktop App.

Isaac Sim is managed separately. This entry never starts simulation or arm tasks.
"""
import argparse
import importlib.util
import json
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
MINIMUM_PYTHON = (3, 12)
# Runs inside WSL with this checkout's rules, so old rules cannot hide drift.
SOURCE_PROBE = r'''
import hashlib, json, sys, types
from pathlib import Path
root = Path(sys.argv[1])
state = {'exists': root.is_dir()}
if state['exists']:
    root = root.resolve()
    rules = types.ModuleType('release_source_manifest')
    exec(compile(MODULE, 'release_source_manifest.py', 'exec'), rules.__dict__)
    state['files'] = {}
    for path in rules.source_files(root):
        data = path.read_bytes()
        name = path.relative_to(root).as_posix()
        state['files'][name] = hashlib.sha256(data).hexdigest()
print(json.dumps(state))
'''


def read_object(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError) as error:
        raise ValueError(f'{path}: {error}. Run Setup first (setup.sh on Linux; setup.ps1 on Windows).') from error
    if not isinstance(value, dict):
        raise ValueError(f'{path}: expected a JSON object. Run Setup first (setup.sh on Linux; setup.ps1 on Windows).')
    return value


def windows_application(root):
    """Read host-owned settings; a Linux checkout must own its own local files."""
    directory = root / 'deployment/runtime/local'
    locations = read_object(directory / 'locations.json')
    if locations.get('schema_version') != 1 or locations.get('host_role') != 'windows_simulator':
        raise ValueError('Windows requires its own windows_simulator locations.json; run Setup.')
    distribution = locations.get('wsl_distribution')
    if (not isinstance(distribution, str) or not distribution.strip()
            or distribution.startswith('-') or any(ord(c) < 32 for c in distribution)):
        raise ValueError('Invalid wsl_distribution in locations.json; run Setup.')
    launcher = read_object(directory / 'launcher.json')
    if launcher.get('schema_version') != 1:
        raise ValueError('Unsupported launcher.json schema; run Setup.')
    application = launcher.get('wsl_project_root')
    if (not isinstance(application, str) or not application.strip()
            or any(ord(c) < 32 for c in application)
            or not PurePosixPath(application).is_absolute()
            or '..' in PurePosixPath(application).parts):
        raise ValueError('wsl_project_root must be an absolute Linux checkout path; run Setup.')
    if re.match(r'^/mnt/[a-z](?:/|$)', application, flags=re.IGNORECASE):
        raise ValueError('Use a separate checkout in the WSL Linux filesystem, not a Windows drive; run Setup.')
    return distribution, application


def local_source_files(root):
    """Deployable source identity of this checkout, by the shared manifest rules."""
    spec = importlib.util.spec_from_file_location(
        'project_source_identity', root / 'deployment/runtime/release_source_manifest.py')
    rules = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rules)
    return rules.collect(root)['files']


def wsl_source_state(root, distribution, application):
    rules = (root / 'deployment/runtime/release_source_manifest.py').read_text(encoding='utf-8')
    program = f'MODULE = {rules!r}\n' + SOURCE_PROBE
    # Bytes keep LF line endings; WSL python3 reads the program from stdin.
    result = subprocess.run(['wsl.exe', '-d', distribution, '--exec', 'python3', '-', application],
                            input=program.encode('utf-8'), capture_output=True)
    if result.returncode:
        detail = result.stderr.decode('utf-8', 'replace').strip()
        raise ValueError(f'Could not read WSL source {application}: {detail}')
    return json.loads(result.stdout.decode('utf-8'))


def source_drift(root, distribution, application):
    """Return (changed names or None when absent, WSL state, local files)."""
    expected = local_source_files(root)
    state = wsl_source_state(root, distribution, application)
    if not state['exists']:
        return None, state, expected
    actual = state['files']
    changed = sorted(name for name in expected.keys() | actual.keys() if expected.get(name) != actual.get(name))
    return changed, state, expected


def describe_drift(application, changed):
    shown = ', '.join(changed[:8]) + (f' and {len(changed) - 8} more' if len(changed) > 8 else '')
    return (f'WSL backend source {application} differs from this Windows checkout '
            f'({len(changed)} files: {shown}). It was preserved. See '
            'docs/getting-started.md#updating-windows-and-wsl for the safe parallel-checkout procedure.')


def require_matching_wsl_source(root, distribution, application):
    changed, _, _ = source_drift(root, distribution, application)
    if changed is None:
        raise ValueError(f'WSL source {application} is missing; run Setup (setup.sh on Linux; setup.ps1 on Windows).')
    if changed:
        raise ValueError(describe_drift(application, changed) + ' Nothing was started.')


def command(args, root=ROOT, system=None):
    system = system or platform.system()
    stopping = args.stop or args.status
    confirmed_stopped = getattr(args, 'simulation_stopped', False)
    if confirmed_stopped and not args.stop:
        raise ValueError('--simulation-stopped is only valid with Stop')
    if system == 'Linux':
        result = ['bash', str(root / ('stop.sh' if stopping else 'start.sh'))]
        if args.status:
            result.append('--status')
        if args.no_app and not stopping:
            result.append('--no-app')
        if confirmed_stopped:
            result.append('--simulation-stopped')
        if args.dry_run:
            result.append('--dry-run')
        return result
    if system != 'Windows':
        raise ValueError('Supported application hosts are native Linux and Windows + WSL.')
    distribution, application = windows_application(root)
    if stopping:
        result = ['wsl.exe', '-d', distribution, '--cd', application, '--exec',
                  'bash', './stop.sh']
        if args.status:
            result.append('--status')
        if confirmed_stopped:
            result.append('--simulation-stopped')
        if args.dry_run:
            result.append('--dry-run')
        return result
    pwsh = shutil.which('pwsh') or shutil.which('pwsh.exe')
    if not pwsh:
        raise ValueError('PowerShell 7.2 or newer is required on Windows; run Setup checks.')
    # Stop/status stay usable across drift; every start, including dry-run, compares first.
    require_matching_wsl_source(root, distribution, application)
    result = [pwsh, '-NoProfile', '-File', str(root / 'deployment/windows/start_app.ps1'),
              '-SkipIsaac', '-WslProjectRoot', application]
    if args.no_app:
        result.append('-NoApp')
    if args.dry_run:
        result.append('-DryRun')
    return result


def app_close_command(args, root=ROOT, system=None):
    """After a Windows Stop, close the Windows App window; native stop.sh closes its own."""
    system = system or platform.system()
    pwsh = shutil.which('pwsh') or shutil.which('pwsh.exe')
    if system != 'Windows' or not args.stop or args.dry_run or not pwsh:
        return None
    return [pwsh, '-NoProfile', '-File', str(root / 'deployment/windows/start_app.ps1'), '-CloseApp']


def main(argv=None):
    if sys.version_info[:2] < MINIMUM_PYTHON:
        print(f'Startup failed: Python 3.12 or newer is required; found {platform.python_version()}.',
              file=sys.stderr)
        return 2
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--no-app', action='store_true', help='Keep backend/Web running without an App window')
    mode.add_argument('--stop', action='store_true', help='Stop this checkout through the existing lifecycle')
    mode.add_argument('--status', action='store_true', help='Show this checkout service status')
    parser.add_argument('--simulation-stopped', action='store_true', help='Operator confirms Isaac is already stopped; Stop only')
    parser.add_argument('--dry-run', action='store_true', help='Validate without starting services')
    args = parser.parse_args(argv)
    try:
        result = subprocess.run(command(args), cwd=ROOT, check=False).returncode
        close_app = app_close_command(args) if result == 0 else None
        if close_app:
            result = subprocess.run(close_app, cwd=ROOT, check=False).returncode
        return result
    except (OSError, ValueError) as error:
        print(f'Startup failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
