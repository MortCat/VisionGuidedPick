#pragma once
#include <optional>
#include "ptc_core/pose3d.hpp"
namespace ptc_core
{
std::optional<Pose3d> interpolate_pose(const Pose3d &,const Pose3d &,double fraction);
bool pose_near(const Pose3d &, const Pose3d &, double position_tolerance, double orientation_tolerance);
std::optional<Pose3d> compose_pose(const Pose3d & parent, const Pose3d & child);
std::optional<Pose3d> inverse_pose(const Pose3d &);
std::optional<Pose3d> camera_view_to_tcp(const Pose3d & camera_world, const Pose3d & camera_in_tcp);
}  // namespace ptc_core
