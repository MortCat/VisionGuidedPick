"""Embedded in the USD Script Node; resolves sidecar code from the root file."""
import hashlib
import importlib.util
import pathlib
import sys

import omni.usd
from pxr import Sdf


def setup(db):
    stage = omni.usd.get_context().get_stage()
    factory = stage.GetPrimAtPath('/World/FactoryDemo')
    value = factory.GetAttribute('factory:runtimeDirectory').Get()
    directory = pathlib.Path(Sdf.ComputeAssetPathRelativeToLayer(stage.GetRootLayer(), value.path))
    path = directory / 'factory_runtime.py'
    name = 'factory_autoplay_' + hashlib.sha256(str(path).encode()).hexdigest()[:12]
    source = path.read_bytes()
    source_digest = hashlib.sha256(source).hexdigest()
    module = sys.modules.get(name)
    if module is None or getattr(module, '_source_digest', None) != source_digest:
        previous_manager = getattr(module, '_manager', None)
        if previous_manager is not None:
            previous_manager.shutdown()
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        try:
            exec(compile(source, str(path), 'exec'), module.__dict__)
            module._source_digest = source_digest
        except Exception:
            sys.modules.pop(name, None)
            raise
    db.per_instance_state.manager = module.get_manager(stage, directory)


def compute(db):
    manager = db.per_instance_state.manager
    if manager.error:
        db.log_error(manager.error)
        return False
    return True


def cleanup(db):
    manager = getattr(db.per_instance_state, 'manager', None)
    if manager is not None:
        manager.shutdown()
