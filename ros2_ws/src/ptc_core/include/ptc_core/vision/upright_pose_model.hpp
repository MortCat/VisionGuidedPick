#pragma once
#include "ptc_core/vision/point_index.hpp"
#include "ptc_core/state/track_state.hpp"
namespace ptc_core
{
struct UprightPoseResult
{
  Pose3d pose_world;
  double error_m{kUnset},position_stddev_m{kUnset},yaw_stddev_rad{kUnset};
  bool valid{false},yaw_valid{false};
};
UprightPoseResult fit_upright_object(const std::vector<Vector3d> & world_points,const PointIndex & reference,
  const GeometryConfig &,const TrackingConfig &);
std::vector<Vector3d> voxel_sample(const std::vector<Vector3d> &,double voxel_size,std::size_t max_points);
// Association uses only fixed-frame visual observations; no belt/ground-truth input.
struct VisualTrackModel
{
  std::vector<TrackedObjectState> tracks;
  std::uint64_t next_track_id{1},next_sequence{1};
};
// Candidate IDs are the capture-time assignments made alongside the detections;
// zero starts a new track. Returns zero for any rejected observation.
std::vector<std::uint64_t> update_visual_tracks(VisualTrackModel &,const std::vector<ObjectObservation> &,
  const std::vector<std::uint64_t> & candidate_track_ids,const SettingsSnapshot &,const TimeContext &);
}  // namespace ptc_core
