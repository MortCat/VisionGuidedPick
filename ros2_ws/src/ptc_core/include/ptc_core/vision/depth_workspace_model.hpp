#pragma once

#include "ptc_core/vision/rgbd_image_frame.hpp"

namespace ptc_core
{
// Static authored workspace geometry, never conveyor speed or simulator poses.
struct ForegroundObjectObservation
{
  ImageRegion region;
  std::vector<std::uint8_t> mask;
  Vector3d center_world, bounds_min_world, bounds_max_world;
  std::size_t measured_points{0};
};
// Segments measured registered RGB-D above a calibrated support plane. Does
// not infer semantic orientation, classify defects, or consult simulator truth.
std::vector<ForegroundObjectObservation> detect_workspace_objects(
  const RgbdImageFrame & frame, const DepthWorkspaceConfig & config);
}  // namespace ptc_core
