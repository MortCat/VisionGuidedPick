"""Self-contained Teach Target model bundle: build, verify and explicit legacy migration.

A published model directory holds checkpoint.pth and manifest.json. Activation reads
only that directory; training jobs and images remain traceability records.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import uuid

BUNDLE_FORMAT = 1
CONTRACT_FIELDS = ('model_version', 'checkpoint_sha256', 'class_names', 'class_mapping',
                   'preprocessing', 'evaluation')
EVALUATION_FIELDS = ('metrics_source', 'selected_epoch', 'evaluation_threshold', 'train_count',
                     'augmented_train_count', 'validation_count', 'metrics', 'deployment_validated')


class BundleError(ValueError):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code = status, code


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
    os.replace(tmp, path)


def file_sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def contract_sha256(row):
    contract = {key: row[key] for key in CONTRACT_FIELDS}
    return hashlib.sha256(json.dumps(contract, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def class_mapping(categories):
    # RF-DETR Roboflow loader remaps sorted COCO category IDs to contiguous output labels.
    return {str(i): c['name'] for i, c in enumerate(sorted(categories, key=lambda c: c['id']))}


def bundle_fields(job_folder, resolved_config, class_names):
    """Collect everything activation needs from a finished job, at bundle time only."""
    job_folder = Path(job_folder)
    mapping = class_mapping(read_json(job_folder / 'dataset/train/_annotations.coco.json')['categories'])
    if sorted(mapping.values()) != sorted(class_names):
        raise BundleError(409, 'CLASS_MAPPING_MISMATCH', 'Dataset categories differ from model class names')
    result = read_json(job_folder / 'result.json')
    training = resolved_config['training']
    return dict(
        bundle_format=BUNDLE_FORMAT,
        class_mapping=mapping,
        preprocessing=dict(architecture='RFDETRSegSmall', resolution=training['resolution'],
                           color_order='RGB', input='PIL RGB at original size',
                           normalization='RFDETR.predict library resize/normalization',
                           mask_space='original_rgb_pixels'),
        evaluation={key: result.get(key) for key in EVALUATION_FIELDS},
        source=dict(training=training, library_versions=result.get('library_versions')),
    )


def seal(row):
    return dict(row, contract_sha256=contract_sha256(row))


def missing_fields(row):
    required = CONTRACT_FIELDS + ('bundle_format', 'contract_sha256', 'source')
    return [key for key in required if key not in row]


def verify(directory, row):
    """Check a manifest and checkpoint without reading any job or image."""
    if row.get('bundle_format') != BUNDLE_FORMAT or missing_fields(row):
        raise BundleError(409, 'MODEL_BUNDLE_LEGACY',
                          'Model predates self-contained bundles; migrate it with '
                          f'tools/teach_target/model_bundle.py migrate {row.get("model_version")}')
    if contract_sha256(row) != row['contract_sha256']:
        raise BundleError(503, 'MODEL_BUNDLE_MISMATCH', 'Model manifest contract checksum mismatch')
    checkpoint = Path(directory) / 'checkpoint.pth'
    if not checkpoint.is_file():
        raise BundleError(503, 'CHECKPOINT_MISSING', 'Published checkpoint is missing')
    if file_sha256(checkpoint) != row['checkpoint_sha256']:
        raise BundleError(503, 'CHECKPOINT_HASH_MISMATCH', 'Published checkpoint checksum mismatch')
    return checkpoint


def migrate(root, version):
    """Explicit operator migration: rebuild bundle fields from the retained job record.

    Keeps the original manifest beside the new one and never changes state.
    """
    directory = Path(root) / 'models' / version
    manifest = directory / 'manifest.json'
    row = read_json(manifest)
    if row.get('bundle_format') == BUNDLE_FORMAT:
        verify(directory, row)
        return row, False
    job_folder = Path(root) / 'jobs' / row['job_id']
    required = [job_folder / name for name in
                ('dataset/train/_annotations.coco.json', 'result.json', 'resolved_config.yaml')]
    absent = [str(path) for path in required if not path.is_file()]
    if absent:
        raise BundleError(409, 'MODEL_BUNDLE_SOURCE_MISSING',
                          'Cannot rebuild bundle; job records are missing: ' + ', '.join(absent))
    checkpoint = directory / 'checkpoint.pth'
    if not checkpoint.is_file() or file_sha256(checkpoint) != row['checkpoint_sha256']:
        raise BundleError(503, 'CHECKPOINT_HASH_MISMATCH', 'Checkpoint missing or changed; not migrated')
    import yaml
    resolved = yaml.safe_load((job_folder / 'resolved_config.yaml').read_text(encoding='utf-8'))
    updated = seal(dict(row, **bundle_fields(job_folder, resolved, row['class_names']),
                        migrated_from_legacy=True))
    backup = directory / 'manifest.legacy.json'
    if not backup.exists():
        atomic_json(backup, row)
    atomic_json(manifest, updated)
    return updated, True


def main(argv=None):
    import argparse
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from teaching_config import config_path, load_config
    project = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    for name in ('check', 'migrate'):
        command = sub.add_parser(name)
        command.add_argument('model_version', nargs='*', help='Default: every model in storage')
        command.add_argument('--storage-root', help='Default: storage_root from the Teach Target config')
        command.add_argument('--config')
    args = parser.parse_args(argv)
    root = Path(args.storage_root) if args.storage_root else (
        project / load_config(project, args.config or config_path(project))['storage_root'])
    versions = args.model_version or sorted(p.parent.name for p in root.glob('models/*/manifest.json'))
    failed = False
    for version in versions:
        try:
            if args.action == 'migrate':
                _, changed = migrate(root, version)
                print(f'{version}: {"migrated" if changed else "already self-contained"}')
            else:
                directory = root / 'models' / version
                verify(directory, read_json(directory / 'manifest.json'))
                print(f'{version}: self-contained')
        except (BundleError, OSError, KeyError, ValueError) as error:
            failed = True
            print(f'{version}: {getattr(error, "code", type(error).__name__)}: {error}', file=sys.stderr)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
