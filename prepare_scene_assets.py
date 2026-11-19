"""Prepare the factory assets from NVIDIA; optional arguments use the shared CLI."""
from pathlib import Path
import runpy
import sys


def main():
    if len(sys.argv) == 1:
        sys.argv.append('--download')
    source = Path(__file__).resolve().parent / 'deployment/runtime/stage_isaac_assets.py'
    runpy.run_path(str(source), run_name='__main__')


if __name__ == '__main__':
    main()
