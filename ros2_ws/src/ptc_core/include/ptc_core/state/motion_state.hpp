#pragma once
#include "ptc_core/move_to_pose.hpp"
#include "ptc_core/state/device_snapshot.hpp"
#include "ptc_core/state/scene_state.hpp"
#include "ptc_core/pose_math.hpp"

namespace ptc_core
{
enum class MotionEventKind
{
  kDeviceReady, kDeviceUnavailable, kMoveRequested, kPlanSucceeded, kPlanFailed,
  kExecutionSucceeded, kExecutionFailed, kFollowRequested, kTargetUpdated,
  kCancelRequested, kTick, kStopConfirmed, kStopFailed, kResetRequested, kClockReset, kCount
};
struct MotionState
{
  ArmMotionState phase{ArmMotionState::kNotReady}; OperationContext context;
  ArmSnapshot arm; MoveToPoseRequest request; Nanoseconds operation_deadline{0}, update_deadline{0}, target_timestamp{0};
  MoveToPoseError pending_result{MoveToPoseError::kSuccess}; bool active{false};
  // Planner output is separate from the admitted moving-target request.
  std::optional<StampedPose3d> planned_target_pose;
};
struct MotionEvent
{
  MotionEventKind kind{MotionEventKind::kTick}; EventKey key; ArmSnapshot arm;
  MoveToPoseRequest request; Pose3d target_world; Nanoseconds target_timestamp{0};
  Vector3d linear_velocity, angular_velocity; Nanoseconds follow_timeout{0};
  StampedPose3d base_in_world; SceneState scene;
};
struct MotionDecision
{
  MotionState state; DecisionStatus status; std::vector<Command> commands; TransitionRecord record;
  std::optional<MoveToPoseResult> result;
  bool request_accepted{false}; // Applied may instead describe a local timeout/stop transition.
};
MotionDecision advance_motion(const MotionState &, const MotionEvent &, const SettingsSnapshot &, const TimeContext &);
MoveToPosePhase m1_action_phase(ArmMotionState);
}  // namespace ptc_core
