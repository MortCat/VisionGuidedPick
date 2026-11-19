#pragma once
#include "ptc_core/state/track_state.hpp"
#include "ptc_core/pose_math.hpp"

namespace ptc_core
{
enum class GraspPhase {kEmpty, kApproaching, kClosing, kVerifying, kHolding, kReleasing, kFailed, kUnknown, kCount};
enum class Possession {kUnknown, kEmpty, kHolding};
enum class GraspEventKind
{kApproachRequested, kApproachCompleted, kCloseAccepted, kCloseFailed, kEvidenceUpdated,
  kSceneAttached, kLiftCompleted, kLiftFailed, kReleaseRequested, kReleaseCompleted,
  kReleaseFailed, kTick, kCancelRequested, kDeviceUnavailable, kStopConfirmed,
  kResetRequested, kClockReset, kCount};
struct GraspState
{
  GraspPhase phase{GraspPhase::kUnknown}; Possession possession{Possession::kUnknown};
  OperationContext context; bool awaiting_stop{false}, faulted{false}, scene_attached{false}, lifted{false};
  std::size_t confirmations{0}; std::uint64_t evidence_sequence{0}; Nanoseconds evidence_timestamp{0};
  Nanoseconds close_source_deadline{0};
  Pose3d lift_target_world;
  double bin_height_offset_m{0};
  std::optional<double> probe_origin_world_z_m;
  Pose3d grasp_tcp_in_object;
  // Measured robot_base TCP orientation at confirmed lift; cleared per grasp.
  std::optional<Quaterniond> held_tcp_orientation_base;
};
// A command-model prediction, never a new visual observation or Holding evidence.
struct GraspMotionPrediction
{
  EventKey key; StampedPose3d object_world; Nanoseconds reference_start{0};
  double measured_lift_distance_m{0}; // Measured TCP rise; never possession evidence.
};
struct GraspEvent
{
  GraspEventKind kind{GraspEventKind::kTick}; EventKey key; ArmSnapshot arm; GripperState gripper;
  TrackedObjectState track; double relative_speed_mps{kUnset}; bool grasp_window_valid{false}, scene_confirmed{false};
  StampedPose3d tcp_world; // TF-converted measured TCP, not the robot_base pose relabeled as world.
  std::optional<Pose3d> grasp_tcp_in_object;
  std::optional<GraspMotionPrediction> motion_prediction;
  // Optional measured-geometry context for the moving semantic grasp probe.
  double expected_contact_span_m{kUnset};
};
struct GraspDecision {GraspState state; DecisionStatus status; std::vector<Command> commands; TransitionRecord record;};
std::optional<Pose3d> committed_grasp_object_pose(const GraspEvent &, const SettingsSnapshot &, const TimeContext &);
// Configured robot_base bin target. The moving demo keeps the held TCP
// orientation during transfer; formal mode keeps the configured orientation.
Pose3d bin_target_pose(const Pose3d & configured, const GraspState &, const SettingsSnapshot &);
bool aperture_below_contact_span(double width_m,double expected_span_m);
double close_wall_timeout_sec(const SystemSettings &);
bool grasp_stage_expired(const GraspState &,const TimeContext &);
GraspDecision advance_grasp(const GraspState &, const GraspEvent &, const SettingsSnapshot &, const TimeContext &);
}  // namespace ptc_core
