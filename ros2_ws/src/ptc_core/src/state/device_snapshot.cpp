#include "ptc_core/state/device_snapshot.hpp"

#include <cmath>
#include <algorithm>
#include <set>

namespace ptc_core
{
double vector_norm(const Vector3d & v) {return std::hypot(v.x_m, v.y_m, v.z_m);}
bool inside_workspace(const Pose3d & p, const MotionConfig & c)
{
  const auto & v = p.position;
  return is_valid_pose(p) && v.x_m >= c.bounds_min_m.x_m && v.x_m <= c.bounds_max_m.x_m &&
    v.y_m >= c.bounds_min_m.y_m && v.y_m <= c.bounds_max_m.y_m &&
    v.z_m >= c.bounds_min_m.z_m && v.z_m <= c.bounds_max_m.z_m;
}
bool arm_snapshot_ready(const ArmSnapshot & a, const SettingsSnapshot & settings, const TimeContext & time)
{
  const auto & s = settings.get();
  if (a.epoch != time.epoch || !a.tf_valid || !a.controller_ready || !a.state.status.ready ||
    a.state.status.data_stale || a.state.status.health_state != DeviceHealthState::kHealthy ||
    a.state.status.connection_state != DeviceConnectionState::kConnected ||
    a.state.base_frame != s.frames.robot_base || a.state.tcp_frame != s.frames.tcp ||
    !is_valid_pose(a.state.tcp_pose) || a.state.timestamp != a.tcp_timestamp || a.state.joints.empty() ||
    !source_is_fresh(a.joint_timestamp, time, s.motion.joint_state_max_age_sec, s.time.sensor_future_tolerance_sec) ||
    !source_is_fresh(a.tcp_timestamp, time, s.motion.tcp_state_max_age_sec, s.time.sensor_future_tolerance_sec)) {return false;}
  std::set<std::string> names;
  for (const auto & joint : a.state.joints) {
    if (joint.name.empty() || !names.insert(joint.name).second || !std::isfinite(joint.position) ||
      (joint.velocity && !std::isfinite(*joint.velocity)) || (joint.effort && !std::isfinite(*joint.effort))) {return false;}
  }
  return true;
}
bool gripper_snapshot_ready(const GripperState & g, const SettingsSnapshot & settings, const TimeContext & t)
{
  return g.epoch == t.epoch && g.status.ready && !g.status.data_stale &&
    g.status.connection_state == DeviceConnectionState::kConnected && g.status.health_state == DeviceHealthState::kHealthy &&
    std::isfinite(g.width_m) && g.width_m >= 0 &&
    source_is_fresh(g.timestamp, t, settings.get().motion.joint_state_max_age_sec, settings.get().time.sensor_future_tolerance_sec);
}
bool gripper_measured_closed_empty(const GripperState & g, const SystemSettings & s)
{
  return g.stopped && !g.contact_confirmed && std::isfinite(g.width_m) && g.width_m >= 0 &&
    g.width_m <= s.grasp.close_width_m + gripper_closed_empty_tolerance_m;
}
bool gripper_measured_open_empty(const GripperState & g, const SystemSettings & s)
{
  return g.stopped && g.empty_confirmed && !g.contact_confirmed && std::isfinite(g.width_m) &&
    g.width_m >= s.grasp.open_width_m - s.motion.position_tolerance_m;
}
bool rgbd_frame_valid(const RgbdFrame & f, const SettingsSnapshot & settings, const TimeContext & t)
{
  const auto & s = settings.get();
  if (f.color_timestamp.count() < 0 || f.depth_timestamp.count() < 0) {return false;}
  const auto gap = f.color_timestamp > f.depth_timestamp ? f.color_timestamp - f.depth_timestamp : f.depth_timestamp - f.color_timestamp;
  return f.frame_id != 0 && f.epoch == t.epoch && f.color_frame == s.frames.color_optical &&
    (s.schema_version==2 ? f.depth_frame==s.frames.color_optical && f.depth_source_frame==s.frames.depth_optical :
    f.depth_frame==s.frames.depth_optical) && f.calibration_id == s.calibration.expected_calibration_id &&
    f.depth_registered && f.intrinsics_valid && f.transform_valid && is_valid_pose(f.camera_in_world) &&
    source_is_fresh(f.color_timestamp, t, s.vision.frame_max_age_sec, s.time.sensor_future_tolerance_sec) &&
    source_is_fresh(f.depth_timestamp, t, s.vision.frame_max_age_sec, s.time.sensor_future_tolerance_sec) &&
    gap <= *seconds_to_duration(s.vision.rgb_depth_sync_tolerance_sec);
}
DecisionStatus FrameBuffer::push(const RgbdFrame & f, const SettingsSnapshot & settings, const TimeContext & t)
{
  if (!rgbd_frame_valid(f,settings,t)) {return {Disposition::kRejected,"invalid RGB-D frame"};}
  if (!frames.empty() && frames.front().epoch != t.epoch) {frames.clear();}
  if (std::any_of(frames.begin(),frames.end(),[&](const auto & old) {return old.frame_id == f.frame_id;})) {return {Disposition::kRejected,"duplicate buffered frame"};}
  if (frames.size() >= settings.get().vision.max_buffered_frames) {return {Disposition::kRejected,"frame buffer capacity reached"};}
  frames.push_back(f); return {Disposition::kApplied,"frame buffered"};
}
std::optional<RgbdFrame> FrameBuffer::take(std::uint64_t id)
{
  const auto found = std::find_if(frames.begin(),frames.end(),[&](const auto & f) {return f.frame_id == id;});
  if (found == frames.end()) {return std::nullopt;} const auto frame = *found; frames.erase(found); return frame;
}
}  // namespace ptc_core
