"""Acquire the pinned scene assets from NVIDIA or an authorized local asset tree."""
import argparse
import hashlib
import json
import posixpath
import re
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import urllib.error
import urllib.parse
import urllib.request

PACKAGE = Path(__file__).resolve().parents[1] / 'factory_rabbit_20261005'
PREFIX = PurePosixPath('assets/official/isaac_6_1')
CHECKSUMS = Path(__file__).with_name('isaac_assets.sha256.json')
OFFICIAL_ROOT = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/'


def dependencies():
    manifest = json.loads((PACKAGE / 'production_manifest.json').read_text(encoding='utf-8'))
    paths = []
    for item in manifest['usd_dependencies']:
        path = PurePosixPath(item)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError(f'Invalid manifest asset path: {item}')
        if path.is_relative_to(PREFIX):
            paths.append(path)
    return sorted(set(paths))



def validate_material_dependencies(paths, locate):
    """Check literal texture resources in the pinned MDL subset, not SDK modules."""
    known = set(paths)
    errors = []
    for path in paths:
        if path.suffix.lower() != '.mdl':
            continue
        text = locate(path).read_text(encoding='utf-8')
        for resource in re.findall(r'\btexture_(?:2d|3d|cube|ptex)\s*\(\s*"([^"\n]+)"', text):
            relative = PurePosixPath(resource)
            if relative.is_absolute() or '\\' in resource or ':' in resource:
                errors.append(f'Unsupported material resource path: {path}: {resource}')
                continue
            dependency = PurePosixPath(posixpath.normpath(str(path.parent / relative)))
            if not dependency.is_relative_to(PREFIX) or dependency not in known:
                errors.append(f'Untracked material dependency: {path} -> {dependency}; update the production and checksum manifests')
            elif not locate(dependency).is_file():
                errors.append(f'Missing material dependency: {path} -> {dependency}')
    if errors:
        raise ValueError('\n'.join(errors))


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def pinned_assets():
    paths = dependencies()
    recorded = json.loads(CHECKSUMS.read_text(encoding='utf-8'))
    if recorded.get('schema_version') != 1:
        raise ValueError('Unsupported Isaac checksum manifest')
    expected = recorded['sha256']
    if set(expected) != {path.as_posix() for path in paths}:
        raise ValueError('Isaac dependency and checksum manifests differ')
    legacy = recorded.get('legacy_sha256', {})
    if not set(legacy).issubset(expected):
        raise ValueError('Legacy checksum contains an unknown asset')
    return paths, expected, legacy


def accepted_hashes(path, expected, legacy):
    values = {expected[path.as_posix()]}
    if path.as_posix() in legacy:
        values.add(legacy[path.as_posix()])
    return values


def stage_assets(asset_root, *, check=False):
    paths, expected, legacy = pinned_assets()
    asset_root = asset_root.resolve()
    errors, planned = [], []
    for path in paths:
        source = asset_root / path.relative_to(PREFIX)
        destination = PACKAGE / path
        digest = expected[path.as_posix()]
        if not source.is_file():
            errors.append(f'Missing source: {source}')
        else:
            digest = sha256(source)
            if digest not in accepted_hashes(path, expected, legacy):
                errors.append(f'Source checksum mismatch: {path.relative_to(PREFIX)}')
        if not destination.resolve().is_relative_to(PACKAGE.resolve()):
            errors.append(f'Destination escapes package: {path}')
        elif destination.exists():
            if not destination.is_file() or sha256(destination) not in accepted_hashes(path, expected, legacy):
                errors.append(f'Existing destination checksum mismatch: {path}')
        else:
            planned.append((source, destination, digest))
    if errors:
        raise ValueError('\n'.join(errors))
    validate_material_dependencies(paths, lambda path: asset_root / path.relative_to(PREFIX))
    if not check:
        for source, destination, digest in planned:
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
                    temporary = Path(stream.name)
                shutil.copyfile(source, temporary)
                if sha256(temporary) != digest:
                    raise ValueError(f'Asset changed during copy: {source}')
                temporary.replace(destination)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
    return len(paths), 0 if check else len(planned)


def pending_assets():
    paths, expected, legacy = pinned_assets()
    pending, errors = [], []
    for path in paths:
        destination = PACKAGE / path
        if not destination.resolve().is_relative_to(PACKAGE.resolve()):
            errors.append(f'Destination escapes package: {path}')
        elif destination.exists():
            if not destination.is_file() or sha256(destination) not in accepted_hashes(path, expected, legacy):
                errors.append(f'Existing destination checksum mismatch: {path}')
        else:
            pending.append(path)
    if errors:
        raise ValueError('\n'.join(errors))
    return paths, expected, pending


def verify_assets():
    paths, _, missing = pending_assets()
    if missing:
        raise ValueError('Missing scene assets; run with --download or provide a local asset root:\n' +
                         '\n'.join(path.as_posix() for path in missing))
    validate_material_dependencies(paths, lambda path: PACKAGE / path)
    return len(paths), 0


def download_assets():
    paths, expected, pending = pending_assets()
    for index, path in enumerate(pending, 1):
        relative = path.relative_to(PREFIX).as_posix()
        url = OFFICIAL_ROOT + urllib.parse.quote(relative, safe='/')
        destination = PACKAGE / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            print(f'[{index}/{len(pending)}] Downloading {relative}', flush=True)
            with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
                temporary = Path(stream.name)
                with urllib.request.urlopen(url, timeout=30) as response:
                    shutil.copyfileobj(response, stream, length=1024 * 1024)
            if sha256(temporary) != expected[path.as_posix()]:
                raise ValueError(f'Official asset checksum mismatch: {relative}; existing files were preserved')
            temporary.replace(destination)
        except (OSError, urllib.error.URLError) as error:
            raise ValueError(f'Could not download {relative}: {error}. '
                             'Rerun to continue with already verified files, or use an authorized local asset root.') from error
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    validate_material_dependencies(paths, lambda path: PACKAGE / path)
    return len(paths), len(pending)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_root', nargs='?', type=Path, help='Authorized local directory containing Isaac/')
    parser.add_argument('--download', action='store_true', help='Download the pinned files directly from NVIDIA; retain their terms')
    parser.add_argument('--check', action='store_true', help='Verify installed files, or the supplied source tree, without writing')
    args = parser.parse_args()
    if args.download and (args.asset_root or args.check):
        parser.error('--download cannot be combined with asset_root or --check')
    if not args.download and not args.asset_root and not args.check:
        parser.error('Use --download, --check, or an authorized local asset_root')
    try:
        if args.download:
            count, copied = download_assets()
        elif args.asset_root:
            count, copied = stage_assets(args.asset_root, check=args.check)
        else:
            count, copied = verify_assets()
    except (OSError, ValueError, KeyError) as error:
        parser.exit(2, str(error) + '\n')
    print(f'{count} Isaac asset files verified; {copied} acquired.')


if __name__ == '__main__':
    main()
