#pragma once

#include "ptc_core/state/device_snapshot.hpp"

namespace ptc_core
{
enum class TrackPhase {kCandidate, kTracking, kTemporarilyLost, kLost, kRetired, kCount};
enum class TrackEventKind {kObservation, kMissing, kTick, kRetire, kClockReset, kCount};
struct ObjectObservation
{
  Pose3d pose_world; Nanoseconds timestamp{0}; std::string reference_frame;
  double position_stddev_m{kUnset}; bool valid{false};
  double yaw_stddev_rad{kUnset}; bool yaw_valid{false};
  Vector3d envelope_center_object_m; double envelope_radius_xy_m{kUnset},envelope_half_height_m{kUnset};
  bool envelope_valid{false};
};
struct TrackedObjectState
{
  TrackPhase phase{TrackPhase::kCandidate}; OperationContext context;
  ObjectObservation observation; Vector3d velocity_mps;
  bool velocity_valid{false}; std::size_t confirmations{0};
  double velocity_fit_residual_m{kUnset},velocity_uncertainty_mps{kUnset};
  std::vector<ObjectObservation> velocity_samples; // Bounded measured history, never predicted samples.
};
struct TrackEvent {TrackEventKind kind{TrackEventKind::kTick}; EventKey key; ObjectObservation observation;};
struct TrackDecision {TrackedObjectState state; DecisionStatus status; TransitionRecord record;};
TrackDecision advance_track(const TrackedObjectState &, const TrackEvent &, const SettingsSnapshot &, const TimeContext &);
bool track_is_usable(const TrackedObjectState &, const SettingsSnapshot &, const TimeContext &, bool prediction = false);
std::optional<double> predicted_position_uncertainty_m(const TrackedObjectState &, const SettingsSnapshot &, const TimeContext &, double horizon);
std::optional<Pose3d> predict_object_pose(const TrackedObjectState &, const SettingsSnapshot &, const TimeContext &, double horizon);
std::optional<Pose3d> track_pose_at_observation(const TrackedObjectState &, const SettingsSnapshot &,
  const TimeContext &, Nanoseconds capture_timestamp);
}  // namespace ptc_core
