#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "ptc_core/pose3d.hpp"

namespace ptc_core
{
using Nanoseconds = std::chrono::nanoseconds;
struct TimeContext {Nanoseconds source_now{0}, monotonic_now{0}; std::uint64_t epoch{0}; bool paused{false};};
struct EventKey
{
  std::uint64_t epoch{0}, cycle_id{0}, track_id{0}, session_id{0}, operation_id{0}, sequence{0};
  std::string settings_revision;
};
enum class Disposition {kApplied, kRejected, kIgnored};
enum class CommandKind
{
  kPlan, kExecuteTrajectory, kStartFollow, kUpdateTarget, kStopArm, kStopGripper,
  kCloseGripper, kOpenGripper, kAttachObject, kDetachObject, kLift,
  kEvaluateView, kCancelInference, kMoveObservation, kMoveBinApproach, kMoveBinRelease,
  kReport, kRetireTrack, kSyncScene, kStartInspection, kStartGrasp,
  kAnalyzeGeometry, kSaveInspectionReport
};
struct Command
{
  CommandKind kind{CommandKind::kStopArm}; EventKey key;
  std::uint64_t command_id{0}, related_command_id{0}, frame_id{0}, scene_revision{0}; std::string producer;
  Pose3d target; std::string reference_frame, view_id;
  Nanoseconds source_timestamp{0}, deadline{0};
  Vector3d linear_velocity, angular_velocity;
  double width_m{0}, effort_n{0};
};
struct TransitionRecord
{
  std::string machine, from, to, reason; EventKey key;
  Nanoseconds source_time{0}, monotonic_time{0}; Disposition disposition{Disposition::kIgnored};
};
struct DecisionStatus {Disposition disposition{Disposition::kIgnored}; std::string reason{"event not applicable"};};
struct OperationContext
{
  EventKey key; Nanoseconds deadline{0}, last_monotonic{0}; std::uint64_t next_command_id{1};
  bool inhibited{false}; std::deque<TransitionRecord> history;
};
std::optional<Nanoseconds> seconds_to_duration(double seconds);
std::optional<Nanoseconds> deadline_after(Nanoseconds now, double seconds);
bool valid_time(const TimeContext & time);
bool source_is_fresh(Nanoseconds stamp, const TimeContext & time, double max_age, double future_tolerance);
bool same_operation(const EventKey & event, const EventKey & active);
bool same_cycle(const EventKey & event, const EventKey & active);
bool same_track_identity(const EventKey &, const EventKey &);
bool valid_observation_key(const EventKey &, const TimeContext &, const std::string & revision);
int event_priority(bool fault, bool cancel, bool deadline);
bool expired(Nanoseconds deadline, const TimeContext & time);
bool append_history(std::deque<TransitionRecord> & history, TransitionRecord record, std::size_t capacity);
bool begin_operation(OperationContext &, const EventKey &, const TimeContext &, const std::string & revision, double timeout);
bool valid_event(const EventKey &, const TimeContext &, const std::string & revision);
bool emit_command(OperationContext &, std::vector<Command> &, CommandKind, const std::string & producer);
TransitionRecord record_decision(OperationContext &, const std::string & machine, int from, int to,
  const EventKey &, const TimeContext &, const DecisionStatus &, std::size_t capacity);

// Overflow is latched; a caller must deliver a fault/stop event, not silently drop it.
struct EventQueue
{
  std::size_t capacity{0}; bool overflowed{false};
  std::deque<EventKey> events;
  bool push(const EventKey & event);
  std::optional<EventKey> pop();
};
}  // namespace ptc_core
