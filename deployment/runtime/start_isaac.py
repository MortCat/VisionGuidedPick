"""Native Linux Isaac launcher using the common deployment profile."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from runtime_profile import ROS_KEYS, RuntimeProfileError, plan, require_valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-profile', default='linux_local')
    parser.add_argument('--locations')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    try:
        value = plan(args.runtime_profile, args.locations, 'isaac')
        print(json.dumps(value, indent=2), flush=True)
        if args.dry_run:
            return 2 if value['errors'] else 0
        require_valid(value)
        sdk = Path(value['locations']['isaac_root'])
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                if (process / 'exe').resolve() == sdk / 'kit/kit':
                    raise RuntimeProfileError('Isaac SDK already running; close normally before changing ROS context')
            except (OSError, RuntimeError):
                continue
        environment = {key: val for key, val in os.environ.items()
                       if key not in ROS_KEYS and not key.startswith('ZENOH_')}
        environment['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
        environment.update(value['environment'])
        # Source only the declared native Jazzy prefix; no ambient overlay or eval.
        command = ['bash', '--noprofile', '--norc', '-c',
                   'set -e; source "$1/setup.bash"; shift; exec "$@"', 'isaac-runtime',
                   value['locations']['ros_prefix']] + value['argv']
        return subprocess.call(command, env=environment, cwd=sdk)
    except (RuntimeProfileError, OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
