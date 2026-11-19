#pragma once
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_core/state/device_snapshot.hpp"
#include "ptc_core/state/grasp_state.hpp"
#include <limits>
#include <vector>
namespace ptc_core {
struct GraspMaskBox {Pose3d box_in_tcp;Vector3d half_extent;};
struct GraspPatchObservation {
  ImageRegion roi;std::vector<Vector3d> points_tcp;std::vector<std::uint8_t> pixel_labels;
  std::size_t valid_depth_pixels{0},self_pixels{0},support_pixels{0},candidate_pixels{0};
  bool roi_clipped{false};std::string reason;
};
// Measured selected-target surface (foreground mask, self-masked) expressed in TCP at its capture.
struct GraspTargetSeed {std::array<Vector3d,2> bounds_tcp;std::size_t points{0};std::uint64_t frame_id{0};Nanoseconds stamp{0};
  Pose3d tcp_in_world;double median_world_z_m{0};};
// Diagnostic only, never a control input. surface_rise_m is NaN when the surface was not measured.
struct GraspLiftPossession {Possession state{Possession::kUnknown};double tcp_rise_m{0},surface_rise_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t surface_points{0};std::string reason;};
struct GraspSupportPlaneEstimate {double z_m{0};std::size_t pixels{0};};
// Diagnostic region only. Candidate points are NOT classified object/holding evidence.
// Pixels whose world height is at or below support_reject_z_world are labelled support, not candidate.
GraspPatchObservation extract_grasp_patch_tcp(const RgbdImageFrame &,
  const std::array<Vector3d,2> & region_tcp,const std::vector<GraspMaskBox> &,std::size_t max_points,
  std::optional<double> support_reject_z_world=std::nullopt);
// Median height of visible depth within the workspace XY bounds and one foreground band of the
// configured plane. Nullopt when fewer than config.min_points support pixels are visible.
std::optional<GraspSupportPlaneEstimate> measure_support_plane(const RgbdImageFrame &,const DepthWorkspaceConfig &);
// Nullopt with reason when fewer than min_points unmasked target pixels have valid depth.
std::optional<GraspTargetSeed> grasp_target_seed_tcp(const RgbdImageFrame &,const std::vector<std::uint8_t> & target_mask,
  const std::vector<GraspMaskBox> &,std::size_t min_points,std::string & reason);
// Union of the model pad-gap region and the seed bounds expanded by margin.
std::array<Vector3d,2> grasp_observation_region(const std::array<Vector3d,2> & pad_region_tcp,const GraspTargetSeed &,double margin_m);
// After close, compares the visible seed surface with two hypotheses over the measured TCP rise since the seed:
// moved with the TCP or stayed at its Closing world height. Holding needs "moved" plus post-close gripper
// contact; Empty needs "stayed" plus no contact. Too little rise, too little visible surface, no post-close
// gripper sample or disagreement give Unknown. Widths and point counts alone never decide.
GraspLiftPossession evaluate_lift_possession(const RgbdImageFrame &,const GraspTargetSeed &,const std::vector<GraspMaskBox> &,
  std::optional<double> support_reject_z_world,const std::optional<GripperState> & gripper,double resolution_m,std::size_t min_points);
}
