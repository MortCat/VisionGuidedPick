#pragma once
#include "ptc_core/state/device_snapshot.hpp"
#include <array>
#include <cstdint>
#include <memory>

namespace ptc_core
{
struct CameraModel
{
  std::uint32_t width{0},height{0};
  double fx{0},fy{0},cx{0},cy{0};
  std::array<double,8> distortion{}; // OpenCV plumb_bob/rational, zero for rectified.
};
struct ImageRegion {std::uint32_t x{0},y{0},width{0},height{0};};
struct FrameQuality
{double brightness{0},laplacian_variance{0}; std::size_t valid_depth_points{0}; bool valid{false};};
struct RgbdImageFrame
{
  RgbdFrame metadata; CameraModel camera;
  std::vector<std::uint8_t> rgb; // tightly packed RGB8
  std::vector<float> depth_m; // tightly packed and registered to RGB
};
ImageRegion image_region(const std::array<double,4> & bbox,const CameraModel &,double margin);
std::vector<float> decode_depth(std::uint32_t width,std::uint32_t height,std::uint32_t step,
  bool big_endian,bool millimetres,const std::vector<std::uint8_t> &,const VisionConfig &);
std::vector<float> register_depth(const std::vector<float> &,const CameraModel & depth,const CameraModel & color,
  const Pose3d & depth_in_color);
FrameQuality frame_quality(const RgbdImageFrame &,const ImageRegion &,const VisionConfig &);
std::vector<Vector3d> region_points_world(const RgbdImageFrame &,const ImageRegion &,
  std::size_t max_points,const std::vector<std::uint8_t> & mask = {});
inline constexpr std::size_t kGraspNeighborLimit = 384;
inline constexpr double kGraspNeighborVoxelM = 0.005;
// Offsets from the selected object's captured position, expressed in world axes.
std::vector<Vector3d> grasp_neighbor_offsets(const RgbdImageFrame &,const ImageRegion &,
  const std::vector<std::uint8_t> & target_mask,const Vector3d & target_world,double tool_margin_m,double belt_z_m);
Vector3d transform_point(const Pose3d &,const Vector3d &);
std::optional<std::array<double,2>> project_point(const Vector3d & point_camera,const CameraModel &);
}  // namespace ptc_core
