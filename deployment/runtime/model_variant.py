"""One deployment selection for the simulator and every application model consumer."""
import os
from pathlib import Path

VARIANTS = ('camera_bracket_v2',)
FACTORY_SCENE = 'FactoryRabbitDemo.camera-bracket-v2.usda'


def model_selection(root, variant=None):
    root = Path(root).resolve()
    variant = variant or os.environ.get('PTC_RUNTIME_MODEL_VARIANT', VARIANTS[0])
    if variant not in VARIANTS:
        raise ValueError('Unknown model variant: ' + variant)
    settings = root / 'config/factory_demo/variants/camera_bracket_v2'
    scene = root / 'deployment/factory_rabbit_20261005/scene' / FACTORY_SCENE
    return variant, settings, scene


def settings_file(root, arm):
    if arm not in ('arm_01', 'arm_02'):
        raise ValueError('Unknown arm: ' + arm)
    return model_selection(root)[1] / (arm + '.yaml')
