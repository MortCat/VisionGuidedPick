#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
if [[ "${1:-}" != --clean ]]; then
  actual="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
  exec python3 "$actual/deployment/runtime/runtime_profile.py" run-component "$@"
fi
shift
component="${1:?Usage: run_factory_ros.sh bridge|devices|motion|arm_01|arm_02}"
case "$component" in bridge|devices|motion|arm_01|arm_02) ;; *) exit 2 ;; esac
mkdir -p "$root/logs/factory_ros/locks"
exec 9>"$root/logs/factory_ros/locks/$component.lock"
flock -n 9 || { echo "Component already running: $component" >&2; exit 3; }
exec > >(python3 "$root/deployment/linux/rotate_component_log.py" "$root/logs/factory_ros/$component.log" 9>&-) 2>&1
: "${PTC_RUNTIME_PROFILE_REVISION:?Use runtime profile entry point}"
runtime_rmw="$RMW_IMPLEMENTATION"; runtime_dds="$FASTDDS_DEFAULT_PROFILES_FILE"; runtime_domain="$ROS_DOMAIN_ID"
source "$PTC_RUNTIME_ROS_PREFIX/setup.bash"
source "$root/native_install/local_setup.bash"
export RMW_IMPLEMENTATION="$runtime_rmw" ROS_DOMAIN_ID="$runtime_domain"
export FASTDDS_DEFAULT_PROFILES_FILE="$runtime_dds" FASTRTPS_DEFAULT_PROFILES_FILE="$runtime_dds"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS ROS_DISCOVERY_SERVER ROS_STATIC_PEERS
python3 "$root/deployment/runtime/runtime_profile.py" record-component "$component"
runtime="$root/deployment/linux/runtime"
cd "$root"
case "$component" in
  bridge) exec python3 "$runtime/inspection_bridge.py" --ros-args -p use_sim_time:=true ;;
  devices) exec ros2 launch "$runtime/device_interfaces.launch.py" ;;
  motion) exec ros2 launch "$runtime/inspection_launch.py" ;;
  arm_01|arm_02)
    if [[ "${PTC_SEMANTIC_TRIAL:-false}" == true ]]; then
      settings="$PTC_RUNTIME_SETTINGS_DIR/$component.semantic.yaml"
      python3 "$root/deployment/runtime/semantic_settings.py" \
        "$PTC_RUNTIME_SETTINGS_DIR/$component.yaml" "$settings"
      shared_tracks=/factory/observe/pick_preview_tracks
      if [[ "$component" == arm_02 ]]; then shared_tracks+=_arm_02; fi
      exec ros2 launch ptc_bringup system.launch.py settings_file:="$settings" external_motion_stack:=true shared_tracks_topic:="$shared_tracks" continuous_operation:=false task_workflow:=pick-only
    fi
    exec ros2 launch ptc_bringup system.launch.py settings_file:="$PTC_RUNTIME_SETTINGS_DIR/$component.yaml" external_motion_stack:=true observer_namespace:=factory continuous_operation:="${PTC_CONTINUOUS_OPERATION:-true}" task_workflow:="${PTC_TASK_WORKFLOW:-pick-only}" ;;
esac
