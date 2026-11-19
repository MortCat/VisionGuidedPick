#pragma once
#include "ptc_core/vision/upright_pose_model.hpp"
namespace ptc_core
{
struct CloudRegistrationResult
{
  Pose3d object_world;
  double error_m{kUnset},inlier_fraction{0};
  bool valid{false},yaw_valid{false};
  std::string reason;
};
struct CloudRegistrationModel
{
  PointIndex coarse,fine;
  double radius_m{0},half_height_m{0};
  Vector3d center_object;
  double support_z_m{0};
  CloudRegistrationModel(const std::vector<Vector3d> &,const CloudRegistrationConfig &);
};
CloudRegistrationResult register_cloud(const std::vector<Vector3d> &,const CloudRegistrationModel &,
  const GeometryConfig &,const TrackingConfig &,const std::optional<Pose3d> & initial={});
}
