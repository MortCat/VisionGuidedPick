"""Provision only annotation assets/dependencies; never upgrade Torch/ROS or train."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import urllib.request
from teaching_config import load_config

ROOT = Path(__file__).resolve().parents[2]
REVISION = '2b90b9f5ceec907a1c18123530e92e794ad901a4'
CHECKSUM = '6d1aa6f30de5c92224f8172114de081d104bbd23dd9dc5c58996f0cad5dc4d38'
URL = 'https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    args = parser.parse_args()
    cfg = load_config(ROOT, args.config)['annotation']
    repo = ROOT / cfg['source']
    repo.parent.mkdir(parents=True, exist_ok=True)
    if not repo.exists():
        subprocess.run(['git', 'clone', '--no-checkout', 'https://github.com/facebookresearch/sam2.git', str(repo)], check=True)
        subprocess.run(['git', 'checkout', '--detach', REVISION], cwd=repo, check=True)
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    if revision != REVISION:
        raise RuntimeError('Annotation source revision differs; preserve and inspect the local checkout')
    subprocess.run([cfg['python'], '-m', 'pip', 'install', '--target', str(ROOT / cfg['dependencies']),
                    'hydra-core==1.3.2', 'iopath==0.1.10'], check=True)
    checkpoint = ROOT / cfg['checkpoint']
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    if not checkpoint.exists():
        temporary = checkpoint.with_suffix('.download')
        urllib.request.urlretrieve(URL, temporary)
        if hashlib.sha256(temporary.read_bytes()).hexdigest() != CHECKSUM:
            raise RuntimeError('Downloaded annotation checkpoint checksum mismatch')
        temporary.replace(checkpoint)
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != CHECKSUM:
        raise RuntimeError('Annotation checkpoint checksum mismatch')
    (repo.parent / 'installation.json').write_text(json.dumps({
        'source': 'https://github.com/facebookresearch/sam2', 'revision': revision,
        'checkpoint_sha256': CHECKSUM, 'checkpoint_bytes': checkpoint.stat().st_size}, indent=2))
    print('SAM annotation runtime ready. No training or robot action performed.')


if __name__ == '__main__':
    main()
