"""Resolve the product configuration and its project-relative interpreters."""
import os
from pathlib import Path

import yaml


def config_path(root):
    override = os.environ.get('PTC_TEACH_CONFIG')
    local = root / 'deployment/runtime/local/teaching.yaml'
    if override:
        return Path(override).expanduser().resolve()
    return local if local.is_file() else root / 'config/teaching.yaml'


def load_config(root, path=None):
    config = yaml.safe_load(Path(path or config_path(root)).read_text(encoding='utf-8'))
    for section in ('training', 'annotation'):
        executable = Path(config[section]['python']).expanduser()
        # Keep the venv executable path, including its symlink.
        config[section]['python'] = str(root / executable)
    return config
