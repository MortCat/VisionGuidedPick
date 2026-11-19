#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
if [[ "${1:-}" != --clean ]]; then
  exec env -i HOME="${HOME:?HOME is required}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 PTC_BUILD_ROS_PREFIX="${PTC_BUILD_ROS_PREFIX:-/opt/ros/jazzy}" bash --noprofile --norc "$root/deployment/linux/build_native.sh" --clean
fi
source "${PTC_BUILD_ROS_PREFIX:-/opt/ros/jazzy}/setup.bash"
cd "$root"
rm -f native_install/.native_build_complete
mkdir -p native_build
python3 deployment/runtime/release_source_manifest.py --output native_build/source-manifest.json
for package in moveit_ros_move_group moveit_servo moveit_configs_utils moveit_resources_panda_moveit_config control_msgs realtime_tools; do
  ros2 pkg prefix "$package" >/dev/null || { echo "Missing native ROS package: $package" >&2; exit 1; }
done
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base native_log build --base-paths ros2_ws/src deployment/linux/runtime/gripper_adapter \
  --build-base native_build --install-base native_install --executor sequential \
  --packages-skip ptc_description_tools \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
source native_install/local_setup.bash
cmake -S deployment/linux/runtime/sim_servo -B native_build/sim_servo -DCMAKE_BUILD_TYPE=Release
cmake --build native_build/sim_servo -j2
python3 deployment/runtime/release_source_manifest.py --check native_build/source-manifest.json
cp native_build/source-manifest.json native_install/.source_manifest.json
touch native_install/.native_build_complete
echo 'Native build complete; run bash start.sh. No services or motion were started.'
