#pragma once
#include "ptc_core/state/grasp_state.hpp"
#include "ptc_core/state/scene_state.hpp"

namespace ptc_core
{
enum class TaskPhase
{kWaitingReady, kObserving, kPreparingPick = 3, kPicking, kTransporting, kPlacing, kReporting, kRecovering, kFaulted, kCount};
enum class TaskEventKind
{kStartRequested, kReadinessChanged, kTargetLost, kMotionFinished,
  kGraspUpdated, kPlaceConfirmed, kReportAcknowledged, kCancelRequested, kTick,
  kFaultRaised, kRecoveryConfirmed, kResetRequested, kClockReset, kGraspPrepared, kGraspTrialRequested, kCount};
enum class CycleOutcome {kNone, kDefectPicked, kNoDefectObserved, kInconclusive, kCanceled, kFailed};
struct ReadinessStatus
{bool vision_ready{false}, inference_ready{false}; std::string vision_revision, inference_revision, motion_revision;};
struct TaskState
{
  TaskPhase phase{TaskPhase::kWaitingReady}; OperationContext context;
  GraspState grasp; TrackedObjectState selected_track;
  Nanoseconds cycle_deadline{0}; std::uint64_t expected_operation_id{0}, retired_track_id{0};
  std::uint64_t recovery_home_command_id{0};
  std::size_t retries{0}; bool observation_arrived{false}, report_emitted{false}, terminal_fault{false};
  CycleOutcome outcome{CycleOutcome::kNone};
};
struct TaskEvent
{
  TaskEventKind kind{TaskEventKind::kTick}; EventKey key; ReadinessStatus readiness;
  ArmSnapshot arm; GripperState gripper; SceneState scene; TrackedObjectState track;
  GraspEvent grasp_event; bool motion_succeeded{false};
  std::optional<double> bin_height_offset_m;
};
struct TaskDecision {TaskState state; DecisionStatus status; std::vector<Command> commands; TransitionRecord record;};
bool system_ready(const TaskEvent &, const SettingsSnapshot &, const TimeContext &);
// Fixed motions and unassigned semantic wait use phase-specific perception requirements.
bool system_ready(const TaskEvent &, const SettingsSnapshot &, const TimeContext &, TaskPhase);
// Stopped at measured Home without a target: perception, scene and motion gaps
// wait for target selection, which checks system_ready again.
bool unassigned_observation_wait(const TaskState &, const TaskEvent &, const SettingsSnapshot &, const TimeContext &);
TaskDecision advance_task(const TaskState &, const TaskEvent &, const SettingsSnapshot &, const TimeContext &);
TaskDecision dispatch_task_batch(const TaskState &, std::vector<TaskEvent>, const SettingsSnapshot &, const TimeContext &);
}  // namespace ptc_core
