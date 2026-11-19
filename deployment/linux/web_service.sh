#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
api_python="${PTC_TEACH_PYTHON:-$root/.teach_api_env/bin/python}"
[[ -x "$api_python" ]] || { echo "Create $root/.teach_api_env and install tools/teach_target/requirements.txt first." >&2; exit 1; }
unset PYTHONPATH PYTHONHOME
exec "$api_python" -I "$root/tools/teach_target/lifecycle.py" "$@"
