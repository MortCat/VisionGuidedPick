#pragma once
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <optional>
#include <vector>

namespace ptc_core
{
inline bool bin_recovery_arrived(const Pose3d & measured,const Pose3d & preferred,
  const Pose3d & configured,double position_tolerance_m,double orientation_tolerance_rad)
{
  return pose_near(measured,preferred,position_tolerance_m,orientation_tolerance_rad) ||
    pose_near(measured,configured,position_tolerance_m,orientation_tolerance_rad);
}
// Try measured carrying orientation, then the configured bin orientation.
// A failed pose is never silently resubmitted within the same recovery.
inline std::optional<Pose3d> select_bin_recovery_target(const Pose3d & preferred,
  const Pose3d & configured,const std::vector<Pose3d> & failed,
  double position_tolerance_m,double orientation_tolerance_rad)
{
  for(const auto & candidate:{preferred,configured}){
    if(!is_valid_pose(candidate)){continue;}
    if(std::none_of(failed.begin(),failed.end(),[&](const Pose3d & pose){
      return pose_near(candidate,pose,position_tolerance_m,orientation_tolerance_rad);
    })){return candidate;}
  }
  return std::nullopt;
}
}
