#pragma once

#include "ptc_core/arm_state.hpp"
#include "ptc_core/config/system_settings.hpp"
#include "ptc_core/state/event_context.hpp"

namespace ptc_core
{
struct GripperState
{
  Nanoseconds timestamp{0}; std::uint64_t epoch{0}, evidence_sequence{0};
  DeviceStatus status; double width_m{kUnset};
  bool stopped{false}, empty_confirmed{false}, contact_confirmed{false}, visual_follow_confirmed{false};
};
struct ArmSnapshot
{
  ArmState state; Nanoseconds joint_timestamp{0}, tcp_timestamp{0}; std::uint64_t epoch{0};
  bool tf_valid{false}, controller_ready{false}, stopped{false};
};
struct RgbdFrame
{
  std::uint64_t frame_id{0}, epoch{0}; Nanoseconds color_timestamp{0}, depth_timestamp{0};
  std::string color_frame, depth_frame, calibration_id;
  bool depth_registered{false}, intrinsics_valid{false}, transform_valid{false};
  Pose3d camera_in_world;
  std::string depth_source_frame; // schema v2 registered depth is expressed in RGB optical frame
  Pose3d tcp_in_world;
};
bool arm_snapshot_ready(const ArmSnapshot &, const SettingsSnapshot &, const TimeContext &);
bool gripper_snapshot_ready(const GripperState &, const SettingsSnapshot &, const TimeContext &);
// Keep the narrow closed-end band separate from the open-position tolerance.
constexpr double gripper_closed_empty_tolerance_m = 0.001;
bool gripper_measured_closed_empty(const GripperState &, const SystemSettings &);
bool gripper_measured_open_empty(const GripperState &, const SystemSettings &);
bool rgbd_frame_valid(const RgbdFrame &, const SettingsSnapshot &, const TimeContext &);
struct FrameBuffer
{
  std::deque<RgbdFrame> frames;
  DecisionStatus push(const RgbdFrame &, const SettingsSnapshot &, const TimeContext &);
  std::optional<RgbdFrame> take(std::uint64_t frame_id);
};
bool inside_workspace(const Pose3d &, const MotionConfig &);
double vector_norm(const Vector3d & vector);
}  // namespace ptc_core
