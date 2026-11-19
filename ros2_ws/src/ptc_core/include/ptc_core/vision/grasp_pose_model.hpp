#pragma once
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_core/state/track_state.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
namespace ptc_core
{
struct GraspSurfacePoint
{
  Vector3d position_object_m;
  std::optional<Vector3d> view_direction_object;
  Nanoseconds capture_timestamp{0};
};
struct GraspSurfaceSnapshot
{
  EventKey key;
  std::uint64_t revision{0};
  std::string object_frame,settings_revision;
  std::vector<GraspSurfacePoint> points;
  std::vector<Vector3d> neighbor_offsets_world;
  Nanoseconds neighbor_stamp{0};
  double neighbor_capture_ms{0};
};
struct GripperGeometryModel
{
  std::string model_id,description_id,collision_geometry_digest,calibration_id,tcp_frame;
  Vector3d closing_axis_tcp,approach_axis_tcp,finger_center_tcp;
  double minimum_opening_m{0},maximum_opening_m{0};
  std::optional<std::array<Vector3d,2>> fingertip_box_tcp;
};
// Robot calibration geometry, independent of object class or simulator truth.
std::optional<double> fingertip_support_clearance(const Pose3d & tcp_world,const GripperGeometryModel &,double plane_z_m);
std::optional<Pose3d> grasp_recovery_retreat(const Pose3d & tcp_world,const GripperGeometryModel &,double plane_z_m,double margin_m,double retreat_m);
struct GraspContactPatch
{
  Vector3d center_object_m,normal_object;
  std::size_t support_point_count{0};
  double support_radius_m{0},normal_quality{0};
  bool normal_polarity_known{false};
};
struct GraspCandidate
{
  std::uint64_t candidate_id{0},surface_revision{0};
  Pose3d tcp_in_object,pregrasp_in_object,tcp_world;
  Vector3d approach_direction_object;
  GraspContactPatch left_contact,right_contact;
  double opening_width_m{0},geometry_score{0};
  std::optional<double> model_score;
  std::string source,gripper_model_id,collision_geometry_digest,reason;
  bool valid{false};
  double slice_normal_alignment{0};
  // An empty side has insufficient surface evidence. A measured zero has
  // enough evidence to assess the pad but no aligned support at its plane.
  std::array<std::optional<double>,2> pad_contact_coverage;
};
enum class GraspSearchStatus {kComplete,kBudgetLimited,kInvalidInput,kNoCandidates};
struct GraspProposalResult
{
  EventKey key;
  std::uint64_t surface_revision{0};
  std::string object_frame,gripper_model_id,collision_geometry_digest,reason;
  GraspSearchStatus status{GraspSearchStatus::kInvalidInput};
  std::vector<GraspCandidate> candidates;
  std::size_t input_point_count{0},finite_point_count{0},sampled_point_count{0},seed_count{0};
  std::vector<std::uint8_t> approach_direction_coverage;
  double elapsed_ms{0};
};
// A short object-relative approach sampled by the existing follow owner.
struct GraspApproachReference
{Pose3d start_in_object,goal_in_object;Nanoseconds source_start{0};double duration_sec{0};};
struct GraspApproachSample
{Pose3d tcp_world;Vector3d relative_linear_world,relative_angular_world;double progress{0};};
// One immutable Cartesian recipe, sampled against actual object time. The
// path clock may slow while the object clock always continues at its own speed.
// Command boundary and twist share one source timestamp and world frame.
struct GraspEntryState
{Pose3d tcp_world;Vector3d linear_world,angular_world;Nanoseconds timestamp{0};};
struct GraspRelativeApproach
{
  Pose3d initial_in_object;
  std::array<Vector3d,6> position_coefficients,rotation_coefficients;
  double linear_bound_mps{0},angular_bound_radps{0};
  std::array<Vector3d,2> position_bounds_object;
};
struct GraspMotionReference
{
  Pose3d initial_tcp_world,object_world,pregrasp_in_object,contact_in_object,lift_in_object;
  Vector3d object_velocity;
  Nanoseconds source_start{0};
  double approach_sec{0},follow_sec{0},descent_sec{0},lift_sec{0};
  std::optional<GraspRelativeApproach> relative_approach;
  double descent_start()const{return approach_sec+follow_sec;}
  double contact_start()const{return descent_start()+descent_sec;}
  double lift_end()const{return contact_start()+lift_sec;}
};
enum class GraspMotionSegment {kApproach,kFollow,kDescend,kLift,kComplete};
struct GraspMotionSample
{
  Pose3d tcp_world;Vector3d linear_world,angular_world;
  GraspMotionSegment segment{GraspMotionSegment::kApproach};
};
std::optional<GraspMotionReference> make_grasp_motion_reference(const Pose3d & initial_tcp_world,
  const Pose3d & object_world,const Vector3d & object_velocity,Nanoseconds source_start,
  const Pose3d & pregrasp_in_object,const Pose3d & contact_in_object,double lift_m,
  double max_linear_speed,double max_angular_speed,double minimum_segment_sec,double follow_sec,
  std::optional<double> descent_max_linear_speed=std::nullopt,
  std::optional<double> probe_max_linear_speed=std::nullopt);
// Object-only lead is used for the entry boundary; source/path clocks remain separate.
std::optional<GraspMotionReference> make_relative_grasp_motion_reference(
  const GraspEntryState &,const Pose3d & object_world,const Vector3d & object_velocity,
  const Pose3d & pregrasp,const Pose3d & contact,double lift_m,double world_linear_cap,
  double angular_cap,double minimum_segment_sec,double follow_sec,double descent_linear,
  double probe_linear,double minimum_approach_sec=0.,double object_lead_sec=0.,
  std::string * failure_reason=nullptr);
std::optional<GraspEntryState> extrapolate_grasp_entry(const GraspEntryState &,Nanoseconds source);
std::optional<GraspMotionSample> sample_grasp_motion_reference(const GraspMotionReference &,
  Nanoseconds source_time,double path_time,double path_rate=1.,
  const Vector3d & object_offset={},const Vector3d & offset_velocity={});

// Compare the physical finger midpoint, not the camera or TCP origin.
bool grasp_center_aligned(const Pose3d & measured_tcp,const Pose3d & desired_tcp,
  const Vector3d & finger_center_tcp,double relative_speed_mps,double max_relative_speed_mps,
  double position_tolerance_m,double orientation_tolerance_rad);
// Measured completion of downward travel; XY mismatch remains diagnostic.
bool grasp_descent_height_reached(const Pose3d & measured_tcp,const Pose3d & desired_tcp,
  const Vector3d & finger_center_tcp,double height_tolerance_m);
// Reach this pose by a joint-space route before starting the final Servo descent.
std::optional<Pose3d> make_grasp_preapproach_pose(const Pose3d & contact_in_object,
  const Vector3d & approach_direction_object,double clearance_m);
std::optional<GraspApproachReference> make_grasp_approach_reference(const Pose3d & start_in_object,
  const Pose3d & goal_in_object,Nanoseconds source_start,double max_linear_speed,double max_angular_speed,const Vector3d & object_velocity,double minimum_duration_sec=0);
std::optional<GraspApproachSample> sample_grasp_approach(const GraspApproachReference &,const Pose3d & object_world,Nanoseconds source_time);
GripperGeometryModel configured_gripper_geometry(const SettingsSnapshot &,const std::string & collision_geometry_digest);
GraspProposalResult propose_grasp_candidates(const GraspSurfaceSnapshot &,const TrackedObjectState &,
  const GripperGeometryModel &,const SettingsSnapshot &,const TimeContext &,double pregrasp_clearance_m);
// Commissioning only: measured top-view slice bounds when opposed normals are
// unavailable. Contact normals remain explicitly unknown; physics proves grip.
GraspProposalResult propose_top_view_grasp_candidates(const GraspSurfaceSnapshot &,const TrackedObjectState &,
  const GripperGeometryModel &,const SettingsSnapshot &,const TimeContext &,double pregrasp_clearance_m);
// Keep the proposal's contact order. Wrist proximity only chooses between
// poses sharing one contact pair and approach direction; pairs with measured
// bilateral slice normals precede pairs without them.
void order_grasp_candidates_for_wrist(std::vector<GraspCandidate> &,const Quaterniond & measured_tcp_world);

}
