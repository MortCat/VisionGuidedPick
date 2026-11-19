#pragma once
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include <optional>

namespace ptc_core
{
struct BinHeightEstimate
{
  double surface_z_m{0};
  std::size_t points{0}, cells{0};
};
// Points and exclusions use robot_base coordinates. Empty/occluded views return no estimate.
std::optional<BinHeightEstimate> estimate_bin_height(const std::vector<Vector3d> &,
  const BinHeightConfig &,const std::vector<std::array<Vector3d,2>> & exclusions);
}
