"""Fingerprint deployable sources without host paths or diagnostic payloads."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path, PurePosixPath

DIRECTORIES = (
    'ros2_ws/src', 'config', 'deployment/runtime', 'deployment/linux', 'deployment/windows',
    'tools/teach_target', 'deployment/factory_rabbit_20261005/scene/runtime',
)
ENTRY_FILES = (
    'deployment/runtime/application.py', 'deployment/runtime/setup.py',
    'setup.sh', 'setup.ps1', 'start.sh', 'start.ps1', 'stop.sh', 'stop.ps1',
    'deployment/linux/web_service.sh', 'prepare_scene_assets.py',
    'deployment/windows/start_app.ps1', 'pixi.toml', 'pixi.lock',
    'deployment/factory_rabbit_20261005/production_manifest.json',
    'deployment/factory_rabbit_20261005/scene/FactoryRabbitDemo.camera-bracket-v2.usda',
    'deployment/factory_rabbit_20261005/scene/factory_base.usda',
)
SOURCE_SUFFIXES = {'.cpp', '.hpp', '.h', '.py', '.sh', '.ps1', '.xml', '.txt',
                   '.json', '.yaml', '.yml', '.toml', '.js', '.css', '.html', '.cjs'}
EXCLUDED_PARTS = {'__pycache__', '.pytest_cache', 'local', 'diagnostics',
                  'artifacts', 'logs', 'reports', 'recordings', 'node_modules', 'build', 'install'}
GENERATED_NAMES = {'inspection_bridge_report.json', 'servo_rejection.json',
                   'feeder_report.json', 'monitor_report.json'}


def source_files(root):
    paths = {root / name for name in ENTRY_FILES if (root / name).is_file()}
    package = root / 'deployment/factory_rabbit_20261005'
    manifest = package / 'production_manifest.json'
    if manifest.is_file():
        scene = json.loads(manifest.read_text(encoding='utf-8'))
        for name in scene['usd_dependencies'] + scene['runtime_sidecars']:
            relative = PurePosixPath(name)
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError(f'Invalid scene dependency path: {name}')
            if relative.is_relative_to(PurePosixPath('assets/official')):
                continue  # Acquired payloads have their own checksum verifier.
            path = package / relative
            if path.is_symlink() or not path.resolve().is_relative_to(package.resolve()):
                raise ValueError(f'Scene dependency escapes the package: {name}')
            if not path.is_file():
                raise ValueError(f'Missing authored scene dependency: {name}')
            paths.add(path)
    for directory in DIRECTORIES:
        base = root / directory
        if not base.is_dir():
            continue
        for path in base.rglob('*'):
            relative = path.relative_to(root)
            if any(part in EXCLUDED_PARTS or part.startswith(('native_', '.teach_'))
                   for part in relative.parts):
                continue
            if path.name.endswith('.semantic.yaml'):
                continue  # Derived from profile + product overlay on startup.
            if path.is_file() and path.suffix in SOURCE_SUFFIXES and path.name not in GENERATED_NAMES:
                if path.is_symlink() or not path.resolve().is_relative_to(root):
                    raise ValueError(f'Source link escapes the manifest root: {relative.as_posix()}')
                paths.add(path)
    return sorted(paths, key=lambda path: path.relative_to(root).as_posix())


def collect(root):
    root = root.resolve()
    files = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
             for path in source_files(root)}
    try:
        revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                                           stderr=subprocess.DEVNULL, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    encoded = json.dumps(files, sort_keys=True, separators=(',', ':')).encode()
    return {'schema_version': 1, 'git_revision': revision, 'files': files,
            'source_sha256': hashlib.sha256(encoded).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path, help='Explicit output file; default prints JSON')
    parser.add_argument('--check', type=Path, help='Compare sources with a previous manifest')
    args = parser.parse_args()
    actual = collect(args.root)
    if args.check:
        expected = json.loads(args.check.read_text(encoding='utf-8'))
        if expected.get('schema_version') != 1 or not isinstance(expected.get('files'), dict):
            parser.error('Unsupported source manifest')
        before, after = expected['files'], actual['files']
        differences = [name for name in sorted(before.keys() | after.keys())
                       if before.get(name) != after.get(name)]
        print(json.dumps({'matches': not differences, 'changed_files': differences}, indent=2))
        return 1 if differences else 0
    text = json.dumps(actual, indent=2, sort_keys=True) + '\n'
    if args.output:
        args.output.write_text(text, encoding='utf-8')
    else:
        print(text, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
