"""Apply the explicit product settings overlay to an arm profile."""
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile

import yaml


def merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value


def resolve(source, destination):
    overlay = source.with_suffix(".product.yaml")
    settings = yaml.safe_load(source.read_text(encoding="utf-8"))
    merge(settings, yaml.safe_load(overlay.read_text(encoding="utf-8")))
    with NamedTemporaryFile(mode="w", encoding="utf-8", dir=destination.parent,
                            delete=False) as output:
        staged = Path(output.name)
        try:
            yaml.safe_dump(settings, output)
            output.close()
            staged.replace(destination)
        finally:
            staged.unlink(missing_ok=True)


if __name__ == "__main__":
    resolve(*(Path(arg) for arg in sys.argv[1:]))
