#!/usr/bin/env bash
set -eo pipefail
entry_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
case "${1:-}" in
  --pick-runtime) shift ;;
  --status) exec bash "$entry_root/stop.sh" --status ;;
  --stop) shift; exec bash "$entry_root/stop.sh" "$@" ;;
  --help)
    echo 'Usage: bash start.sh [--no-app|--dry-run]'
    echo 'Default: Web UI and V2 semantic dual-arm backend, idle until an Auto Pick command.'
    echo 'Train new models from Teach Target. Development runtime: --pick-runtime --help.'
    exit 0
    ;;
  *) set -- --semantic-dual --single-cycle --model-variant camera_bracket_v2 "$@" ;;
esac
root="$entry_root"
actual="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
dry=false; open_browser=true; continuous=true; profile_args=(); semantic_trial=false; semantic_dual=false
while (($#)); do
  case "$1" in
    --runtime-profile|--locations|--model-variant) [[ $# -ge 2 ]] || { echo "Missing value: $1" >&2; exit 2; }; profile_args+=("$1" "$2"); shift 2 ;;
    --dry-run) dry=true; shift ;;
    --workflow) [[ $# -ge 2 ]] || { echo "Missing workflow" >&2; exit 2; }; [[ "$2" == pick-only ]] || { echo "Unknown workflow: $2" >&2; exit 2; }; shift 2 ;;
    --semantic-trial) semantic_trial=true; shift ;;
    --semantic-dual) semantic_trial=true; semantic_dual=true; shift ;;
    --single-cycle) continuous=false; shift ;;
    --no-app|--no-browser) open_browser=false; shift ;;
    --help) echo 'Usage: bash start.sh [--runtime-profile linux_local|windows_wsl] [--locations JSON] [--model-variant camera_bracket_v2] [--workflow pick-only] [--no-browser] [--single-cycle] [--semantic-trial|--semantic-dual] [--dry-run] (default: both arms, continuous pick-only)'; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if $semantic_trial; then
  [[ "$continuous" == false ]] || {
    echo 'Semantic runtime requires --single-cycle and pick-only' >&2; exit 2;
  }
fi
export PTC_SEMANTIC_TRIAL="$semantic_trial"
export PTC_TASK_WORKFLOW=pick-only
export PTC_CONTINUOUS_OPERATION="$continuous"
resolver="$actual/deployment/runtime/runtime_profile.py"
runtime_plan="$(python3 "$resolver" app-plan "${profile_args[@]}")" || { printf '%s\n' "$runtime_plan"; exit 2; }
printf '%s\n' "$runtime_plan"
runtime_locations="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["locations_path"])' <<<"$runtime_plan")"
if [[ -f "$runtime_locations" ]]; then export PTC_RUNTIME_LOCATIONS="$runtime_locations"; else unset PTC_RUNTIME_LOCATIONS; fi
export PTC_RUNTIME_PROFILE="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["profile_path"])' <<<"$runtime_plan")"
if $dry; then
  echo "Only runtime root: $root; install: $root/native_install"
  echo 'Native backend: bridge devices motion arm_01 arm_02 (product UI separate)'
  echo "Product UI + workflow=pick-only app_window=$open_browser; continuous operation=$continuous; dry-run starts no services or motion."
  exit 0
fi
[[ -f "$root/native_install/.native_build_complete" && -f "$root/native_install/local_setup.bash" && -x "$root/native_build/sim_servo/agent_sim_servo" ]] || {
  echo 'Run bash deployment/linux/build_native.sh first.' >&2; exit 1;
}
mkdir -p "$root/logs/factory_ros"
exec 8>"$root/logs/factory_ros/project-start.lock"
flock -n 8 || { echo 'Another project startup is in progress.' >&2; exit 3; }
web_start="$(bash "$root/deployment/linux/web_service.sh" --no-browser)"
printf '%s\n' "$web_start"
url="$(sed -n 's/^Teach Target: //p' <<<"$web_start")"
components=(bridge devices motion arm_01)
if ! $semantic_trial || $semantic_dual; then components+=(arm_02); fi
python3 "$resolver" check-running "${components[@]}" "${profile_args[@]}"
for component in "${components[@]}"; do
  lock="$root/logs/factory_ros/locks/$component.lock"
  if [[ -f "$lock" ]] && ! flock -n "$lock" true; then
    echo "Reusing running component: $component"
    continue
  fi
  nohup bash "$root/deployment/linux/run_factory_ros.sh" "$component" "${profile_args[@]}" \
    > "$root/logs/factory_ros/$component.startup.log" 2>&1 8>&- &
done
ready=false
for ((attempt=0; attempt<30; attempt++)); do
  if python3 "$resolver" check-running "${components[@]}" "${profile_args[@]}" --require-running 2>/dev/null; then ready=true; break; fi
  sleep 0.5
done
$ready || { echo "Backend processes unavailable; check $root/logs/factory_ros." >&2; exit 1; }
echo "Backend started; workflow=pick-only; product UI: $url; awaiting simulation data. Logs: $root/logs/factory_ros"
if $open_browser; then
  bash "$root/deployment/linux/web_service.sh" --open-app 8>&- ||
    echo "App window unavailable. Open $url in your browser."
fi
echo 'Status: bash stop.sh --status | Stop ROS and Web: bash stop.sh'
if $semantic_trial; then echo 'After Isaac Play and Start Observe, click Start in Auto Pick to run continuously.'
elif $continuous; then echo 'Continuous operation enabled: after Isaac Play and readiness, process eligible rabbits and skip expired targets.'
else echo 'Single-cycle debug mode: Isaac Play and explicit Task START are required.'; fi
