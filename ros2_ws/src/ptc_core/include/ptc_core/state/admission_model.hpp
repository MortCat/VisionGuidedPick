#pragma once

#include "ptc_core/state/track_state.hpp"

namespace ptc_core
{
struct InspectionWindowEstimate
{
  bool feasible{false};
  bool enough_time{false};
  double available_sec{kUnset};
  double required_sec{kUnset};
  std::string reason;
};

// Intersect the calibrated observer zone with the translated TCP-view reach
// bounds, then estimate a bounded conservative time-to-exit from track motion.
InspectionWindowEstimate estimate_inspection_window(const TrackedObjectState &,
  const Pose3d & planned_object_pose,const std::vector<Pose3d> & tcp_view_poses,
  const SettingsSnapshot &,const TimeContext &,double required_sec);
}  // namespace ptc_core
