"""Self-contained model bundle and live-outcome checks; no FastAPI, GPU, ROS or motion."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil
import tempfile

import yaml

HERE = Path(__file__).resolve().parent


def load(name):
    spec = importlib.util.spec_from_file_location('test_' + name, HERE / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bundle = load('model_bundle')
runtime = load('observe_runtime')


def expect(code, fn, *args):
    try:
        fn(*args)
    except (bundle.BundleError, runtime.ObserveFailure) as error:
        assert error.code == code, (error.code, code)
        return error
    raise AssertionError('expected ' + code)


def write_job(folder, name='Rabbit', resolution=432):
    (folder / 'dataset/train').mkdir(parents=True)
    (folder / 'dataset/train/_annotations.coco.json').write_text(
        json.dumps({'categories': [{'id': 1, 'name': name}]}))
    (folder / 'result.json').write_text(json.dumps({
        'metrics_source': 'held_out', 'selected_epoch': 3, 'evaluation_threshold': .5,
        'metrics': {'mean_union_iou': .8}, 'deployment_validated': False,
        'library_versions': {'rfdetr': 'recorded-by-worker'}, 'validation_images': [{'image_id': 'x'}]}))
    (folder / 'resolved_config.yaml').write_text(yaml.safe_dump({'training': {'resolution': resolution}}))


with tempfile.TemporaryDirectory() as directory:
    # A custom storage root outside any checkout.
    root = Path(directory) / 'custom-store'
    job = root / 'jobs/job1'
    write_job(job)
    model = root / 'models/v1'
    model.mkdir(parents=True)
    (model / 'checkpoint.pth').write_bytes(b'weights')
    fields = bundle.bundle_fields(job, yaml.safe_load((job / 'resolved_config.yaml').read_text()), ['Rabbit'])
    assert fields['class_mapping'] == {'0': 'Rabbit'}
    assert fields['preprocessing']['resolution'] == 432
    assert fields['evaluation']['metrics'] == {'mean_union_iou': .8}
    assert 'validation_images' not in fields['evaluation']
    assert fields['source']['library_versions'] == {'rfdetr': 'recorded-by-worker'}
    manifest = bundle.seal(dict(model_version='v1', target_id='t', job_id='job1', state='published',
                                checkpoint_sha256=bundle.file_sha256(model / 'checkpoint.pth'),
                                class_names=['Rabbit'], **fields))
    (model / 'manifest.json').write_text(json.dumps(manifest))
    expect('CLASS_MAPPING_MISMATCH', bundle.bundle_fields, job, {'training': {'resolution': 1}}, ['Other'])

    # Move the store and remove jobs/images: activation still resolves.
    moved = Path(directory) / 'moved-store'
    shutil.move(str(root), moved)
    shutil.rmtree(moved / 'jobs')
    resolved = runtime.resolve_published_model(moved, 'v1')
    assert resolved['checkpoint_path'] == str(moved / 'models/v1/checkpoint.pth')
    assert resolved['class_mapping'] == {'0': 'Rabbit'} and resolved['preprocessing']['resolution'] == 432

    # Manifest edits that change the activation contract are detected.
    tampered = dict(manifest, class_mapping={'0': 'Other'})
    (moved / 'models/v1/manifest.json').write_text(json.dumps(tampered))
    expect('MODEL_BUNDLE_MISMATCH', runtime.resolve_published_model, moved, 'v1')
    (moved / 'models/v1/manifest.json').write_text(json.dumps(dict(manifest, state='candidate')))
    expect('MODEL_NOT_PUBLISHED', runtime.resolve_published_model, moved, 'v1')
    (moved / 'models/v1/manifest.json').write_text(json.dumps(manifest))
    (moved / 'models/v1/checkpoint.pth').write_bytes(b'changed')
    expect('CHECKPOINT_HASH_MISMATCH', runtime.resolve_published_model, moved, 'v1')

    # Legacy manifests are refused, not silently rewritten.
    legacy_root = Path(directory) / 'legacy-store'
    legacy_dir = legacy_root / 'models/old'
    legacy_dir.mkdir(parents=True)
    (legacy_dir / 'checkpoint.pth').write_bytes(b'old-weights')
    legacy = dict(model_version='old', target_id='t', job_id='job2', state='published',
                  checkpoint_sha256=hashlib.sha256(b'old-weights').hexdigest(), class_names=['Rabbit'])
    (legacy_dir / 'manifest.json').write_text(json.dumps(legacy))
    expect('MODEL_BUNDLE_LEGACY', runtime.resolve_published_model, legacy_root, 'old')
    assert json.loads((legacy_dir / 'manifest.json').read_text()) == legacy
    # Migration without the retained job reports exactly what is missing.
    error = expect('MODEL_BUNDLE_SOURCE_MISSING', bundle.migrate, legacy_root, 'old')
    assert 'result.json' in str(error)
    assert json.loads((legacy_dir / 'manifest.json').read_text()) == legacy
    # Explicit migration keeps the original manifest and the published state.
    write_job(legacy_root / 'jobs/job2', resolution=512)
    migrated, changed = bundle.migrate(legacy_root, 'old')
    assert changed and migrated['state'] == 'published' and migrated['migrated_from_legacy']
    assert json.loads((legacy_dir / 'manifest.legacy.json').read_text()) == legacy
    assert bundle.migrate(legacy_root, 'old') == (migrated, False)
    shutil.rmtree(legacy_root / 'jobs')
    assert runtime.resolve_published_model(legacy_root, 'old')['preprocessing']['resolution'] == 512
    assert bundle.main(['check', '--storage-root', str(legacy_root)]) == 0
print('Model bundle: custom/moved storage, no job records, tamper/hash checks and explicit legacy migration passed')


# Auto Pick outcome comes from live TaskStatus.last_result only.
observer = runtime.ObserveRuntime(Path('/nonexistent-project'), Path('/nonexistent-store'),
                                  {'stale_after_sec': 1}, 'python')
identity = dict(epoch='2', cycle_id='7', track_id='42')
assert observer._cycle_outcome(dict(identity, phase=0)) is None
assert observer._cycle_outcome(dict(identity, phase=0, last_result=dict(identity, outcome=5))) == 5
assert observer._cycle_outcome(dict(identity, phase=0, last_result=dict(identity, cycle_id='6', outcome=1))) is None
observer.object_history['42'] = dict(track_id='42', display_id='1', state='placing',
                                     arm='arm_01', epoch='2', cycle_id='7')
observer._update_object_outcome('arm_01', dict(identity, phase=0), None)
assert observer.object_history['42']['state'] == 'placing'  # No evidence yet.
observer._update_object_outcome('arm_01', dict(identity, phase=0, last_result=dict(identity, outcome=1)), None)
assert observer.object_history['42']['state'] == 'unverified'  # Release report is not bin evidence.
observer._accept_bin_placement('arm_01', dict(identity, state='placed'))
assert observer.object_history['42']['state'] == 'placed'
print('Auto Pick outcome: live last_result without diagnostic manifests; release report stays unverified passed')
