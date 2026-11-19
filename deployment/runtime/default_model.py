"""Acquire and register the pinned demonstration model during explicit Setup."""
import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile

PROJECT = Path(__file__).resolve().parents[2]


def descriptor(project=PROJECT):
    spec = json.loads((project / 'config/default_model.json').read_text(encoding='utf-8'))
    if spec.get('schema_version') != 1:
        raise ValueError('Unsupported default model descriptor schema')
    for key in ('model_version', 'target_id'):
        if not isinstance(spec.get(key), str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', spec[key]):
            raise ValueError('Invalid default model identity')
    for key in ('archive_sha256', 'checkpoint_sha256', 'contract_sha256'):
        if not isinstance(spec.get(key), str) or not re.fullmatch(r'[0-9a-f]{64}', spec[key]):
            raise ValueError('Invalid default model checksum')
    for key in ('archive_bytes', 'checkpoint_bytes'):
        if type(spec.get(key)) is not int or spec[key] <= 0:
            raise ValueError('Invalid default model size')
    return spec


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def cache_path(project, spec):
    name = spec['archive_name']
    if Path(name).name != name or '/' in name or '\\' in name:
        raise ValueError('Invalid default model archive name')
    return project / 'artifacts/default-model-cache' / name


def token(project):
    value = os.environ.get('GH_TOKEN') or os.environ.get('GITHUB_TOKEN')
    if value:
        return value
    if shutil.which('gh'):
        result = subprocess.run(['gh', 'auth', 'token'], capture_output=True, text=True, timeout=20)
        if result.returncode == 0:
            return result.stdout.strip()
    if shutil.which('git'):
        try:
            result = subprocess.run(['git', '-c', 'credential.interactive=never', 'credential', 'fill'],
                input='protocol=https\nhost=github.com\npath=MortCat/VisionGuidedPick.git\n\n',
                capture_output=True, text=True, timeout=20, cwd=project,
                env=dict(os.environ, GIT_TERMINAL_PROMPT='0'))
            if result.returncode == 0:
                fields = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
                return fields.get('password')
        except subprocess.TimeoutExpired:
            pass
    return None


class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, url):
        parsed = urllib.parse.urlparse(url)
        allowed = {'api.github.com', 'github.com', 'objects.githubusercontent.com',
                   'release-assets.githubusercontent.com'}
        if parsed.scheme != 'https' or parsed.hostname not in allowed:
            raise ValueError('Unexpected model download redirect')
        redirected = super().redirect_request(request, fp, code, message, headers, url)
        if parsed.hostname != urllib.parse.urlparse(request.full_url).hostname:
            redirected.remove_header('Authorization')
        return redirected


def acquire(project, spec, bundle=None):
    if bundle:
        path = Path(bundle).expanduser().resolve()
        if not path.is_file() or digest(path) != spec['archive_sha256']:
            raise ValueError('Supplied demonstration bundle is missing or has a different SHA-256')
        return path
    path = cache_path(project, spec)
    if path.exists():
        if digest(path) != spec['archive_sha256']:
            raise ValueError(f'Demonstration cache checksum mismatch; remove {path} and retry Setup')
        return path
    url = spec['asset_api_url']
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme != 'https' or parsed.hostname != 'api.github.com':
        raise ValueError('Expected the pinned GitHub asset API URL')
    headers = {'Accept': 'application/octet-stream', 'User-Agent': 'VisionGuidedPick-Setup',
               'X-GitHub-Api-Version': '2022-11-28', 'Cache-Control': 'no-cache'}
    opener = urllib.request.build_opener(SafeRedirect())
    def open_range(start):
        end = min(start + 4 * 1024 * 1024, spec['archive_bytes']) - 1
        request_headers = headers | {'Range': f'bytes={start}-{end}'}
        try:
            return opener.open(urllib.request.Request(url, headers=request_headers), timeout=30), end
        except urllib.error.HTTPError as error:
            if error.code not in (401, 403, 404) or 'Authorization' in headers:
                raise
            credential = token(project)
            if not credential:
                raise ValueError('The demo asset is not accessible. Check the repository visibility/release, sign in with gh/Git Credential Manager, '
                                 'set GH_TOKEN, or supply --demo-model-bundle /path/to/the/downloaded.zip') from None
            credential = credential.strip()
            if not credential.isascii() or any(c.isspace() or ord(c) < 33 for c in credential):
                raise ValueError('GitHub credential has an invalid format')
            headers['Authorization'] = 'Bearer ' + credential
            return open_range(start)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, suffix='.partial', delete=False) as stream:
            temporary = Path(stream.name)
            total = 0
            deadline = time.monotonic() + 600
            while total < spec['archive_bytes']:
                start = total
                response, end = open_range(start)
                with response:
                    status = getattr(response, 'status', 200)
                    if status == 206:
                        if response.headers.get('Content-Range') != f'bytes {start}-{end}/{spec["archive_bytes"]}':
                            raise ValueError('Unexpected demonstration download byte range')
                        expected = end - start + 1
                    elif status == 200 and start == 0:
                        expected = spec['archive_bytes']
                    else:
                        raise ValueError('Server did not honor the demonstration download range')
                    while chunk := response.read1(1024 * 1024):
                        if time.monotonic() > deadline:
                            raise ValueError('Demonstration download timed out; retry Setup or use an offline ZIP')
                        total += len(chunk)
                        if total - start > expected:
                            raise ValueError('Demonstration download exceeds its pinned size')
                        stream.write(chunk)
                if total - start != expected:
                    raise ValueError('Incomplete demonstration download range')
                print(f'Demonstration download: {total}/{spec["archive_bytes"]} bytes', flush=True)
        if total != spec['archive_bytes'] or digest(temporary) != spec['archive_sha256']:
            raise ValueError('Demonstration download size or SHA-256 differs from the pinned release')
        os.replace(temporary, path)
        print(f'Demonstration bundle acquired and verified: {total} bytes')
        return path
    finally:
        if temporary:
            temporary.unlink(missing_ok=True)


def bundle_module(project):
    spec = importlib.util.spec_from_file_location('default_model_bundle', project / 'tools/teach_target/model_bundle.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_model(project, storage, spec):
    directory = storage / 'models' / spec['model_version']
    row = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    if (row.get('model_version') != spec['model_version'] or row.get('target_id') != spec['target_id']
            or row.get('checkpoint_sha256') != spec['checkpoint_sha256']
            or row.get('contract_sha256') != spec['contract_sha256'] or row.get('state') != 'published'):
        raise ValueError('Existing demonstration model differs; it was preserved')
    bundle_module(project).verify(directory, row)
    return row


def check_installed(project, storage, spec):
    check_model(project, storage, spec)
    profile = json.loads((storage / 'targets' / spec['target_id'] / 'profile.json').read_text(encoding='utf-8'))
    if profile.get('target_id') != spec['target_id']:
        raise ValueError('Demonstration target profile differs')


@contextmanager
def storage_owner(storage):
    storage.mkdir(parents=True, exist_ok=True)
    with (storage / '.owner.lock').open('a+b') as lock:
        try:
            if os.name == 'nt':
                import msvcrt
                if lock.tell() == 0:
                    lock.write(b'0');lock.flush()
                lock.seek(0);msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            raise ValueError('Web service owns this model storage; stop the project before Setup') from None
        yield


def install(project, storage, spec, bundle=None):
    with storage_owner(storage):
        destination = storage / 'models' / spec['model_version']
        profile = storage / 'targets' / spec['target_id'] / 'profile.json'
        if profile.exists() and json.loads(profile.read_text())['target_id'] != spec['target_id']:
            raise ValueError('Existing target profile differs; it was preserved')
        if destination.exists():
            check_model(project, storage, spec)
        else:
            archive = acquire(project, spec, bundle)
            prefix = f'models/{spec["model_version"]}/'
            allowed = {prefix + 'checkpoint.pth', prefix + 'manifest.json',
                       f'targets/{spec["target_id"]}/profile.json', 'MODEL_CARD.md', 'SHA256SUMS'}
            with zipfile.ZipFile(archive) as zipped:
                if set(zipped.namelist()) != allowed or len(zipped.namelist()) != len(allowed):
                    raise ValueError('Unexpected paths in demonstration bundle')
                if sum(info.file_size for info in zipped.infolist()) > spec['checkpoint_bytes'] + 2 * 1024 * 1024:
                    raise ValueError('Unexpected expanded model size')
                destination.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.TemporaryDirectory(dir=destination.parent, prefix='.demo-install-') as temporary:
                    stage = Path(temporary) / spec['model_version'];stage.mkdir()
                    for name in ('checkpoint.pth', 'manifest.json'):
                        with zipped.open(prefix + name) as source, (stage / name).open('wb') as output:
                            shutil.copyfileobj(source, output)
                    with zipped.open('MODEL_CARD.md') as source, (stage / 'MODEL_CARD.md').open('wb') as output:
                        shutil.copyfileobj(source, output)
                    row = json.loads((stage / 'manifest.json').read_text())
                    if (row.get('checkpoint_sha256') != spec['checkpoint_sha256']
                            or row.get('contract_sha256') != spec['contract_sha256']
                            or row.get('model_version') != spec['model_version']
                            or row.get('target_id') != spec['target_id'] or row.get('state') != 'published'):
                        raise ValueError('Demonstration manifest does not match the pinned contract')
                    bundle_module(project).verify(stage, row)
                    stage.rename(destination)
        if not profile.exists():
            profile.parent.mkdir(parents=True, exist_ok=True)
            row = dict(target_id=spec['target_id'], name=spec['name'], draft_revision=1,
                       images=[], latest_job_id=None)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=profile.parent,
                                             delete=False, suffix='.tmp') as stream:
                json.dump(row, stream);temporary = Path(stream.name)
            os.replace(temporary, profile)
        check_installed(project, storage, spec)
        print('Demonstration model ready; existing target data preserved. Observe and arm tasks remain stopped.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--acquire', action='store_true')
    mode.add_argument('--install', action='store_true')
    mode.add_argument('--check', action='store_true')
    parser.add_argument('--bundle', type=Path)
    args = parser.parse_args(argv)
    try:
        spec = descriptor()
        if args.acquire:
            print(acquire(PROJECT, spec, args.bundle))
        else:
            sys.path.insert(0, str(PROJECT / 'tools/teach_target'))
            from teaching_config import load_config
            storage = (PROJECT / load_config(PROJECT)['storage_root']).resolve()
            if args.check:
                check_installed(PROJECT, storage, spec)
                print('Demonstration model and target verified')
            else:
                install(PROJECT, storage, spec, args.bundle)
        return 0
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as error:
        print(f'Demonstration model setup failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
