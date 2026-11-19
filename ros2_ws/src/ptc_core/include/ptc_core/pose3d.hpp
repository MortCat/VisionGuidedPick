#pragma once

#include <chrono>
#include <cmath>
#include <string>

namespace ptc_core
{

struct Vector3d
{
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
};

struct Quaterniond
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose3d
{
  Vector3d position;
  Quaterniond orientation;
};

struct StampedPose3d
{
  std::chrono::nanoseconds timestamp{0};
  std::string reference_frame;
  Pose3d pose;
};

inline bool is_valid_pose(const Pose3d & pose)
{
  const bool position_is_finite =
    std::isfinite(pose.position.x_m) &&
    std::isfinite(pose.position.y_m) &&
    std::isfinite(pose.position.z_m);

  const Quaterniond & orientation = pose.orientation;
  const bool orientation_is_finite =
    std::isfinite(orientation.x) &&
    std::isfinite(orientation.y) &&
    std::isfinite(orientation.z) &&
    std::isfinite(orientation.w);

  if (!position_is_finite || !orientation_is_finite) {
    return false;
  }

  const double norm_squared =
    orientation.x * orientation.x +
    orientation.y * orientation.y +
    orientation.z * orientation.z +
    orientation.w * orientation.w;

  constexpr double quaternion_norm_tolerance = 1.0e-3;
  return std::abs(norm_squared - 1.0) <= quaternion_norm_tolerance;
}

}  // namespace ptc_core
