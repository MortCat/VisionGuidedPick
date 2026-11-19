#pragma once

#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <utility>

#include "ptc_core/pose3d.hpp"

namespace ptc_core
{
inline constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();
struct RuntimeConfig {std::string workstation_namespace; bool use_sim_time{true}; std::string motion_backend; std::string operation_mode{"production"};};
struct FrameConfig {std::string world, robot_base, tcp, color_optical, depth_optical;};
struct CalibrationConfig {std::string robot_description_path, expected_calibration_id, hand_eye_path;};
struct TimeConfig {double sensor_future_tolerance_sec{kUnset};};
struct DepthWorkspaceConfig
{
  Vector3d bounds_min_m, bounds_max_m;
  double support_plane_z_m{0}, min_height_m{.008}, max_height_m{.13};
  double cell_size_m{.008}, min_span_m{.02}, max_span_m{.14};
  std::size_t min_points{24}, max_objects{16};
};
struct TrackingCameraConfig
{
  std::string role{"disabled"},color_optical,depth_optical,calibration_id;
  Pose3d camera_world;
};
struct VisionConfig
{
  double rgb_depth_sync_tolerance_sec{kUnset}, frame_max_age_sec{kUnset}, tf_lookup_timeout_sec{kUnset};
  std::size_t max_buffered_frames{0};
  double roi_margin_ratio{kUnset}, depth_min_m{kUnset}, depth_max_m{kUnset};
  std::string depth_registration;
  std::string foreground_source{"model"}; DepthWorkspaceConfig depth_workspace; TrackingCameraConfig tracking_camera;
  std::size_t min_valid_depth_points{0};
  double min_laplacian_variance{kUnset},min_brightness{kUnset},max_brightness{kUnset},max_frame_megabytes{kUnset};
};
struct TrackingConfig
{
  std::size_t min_confirmed_observations{0};
  double observation_max_age_sec{kUnset}, temporary_loss_timeout_sec{kUnset};
  double max_prediction_horizon_sec{kUnset}, association_distance_m{kUnset}, max_position_stddev_m{kUnset};
  std::size_t max_tracked_objects{0};
  double max_yaw_stddev_rad{kUnset},yaw_ambiguity_margin_m{kUnset};
  std::size_t velocity_min_samples{0},velocity_max_samples{0};
  double velocity_window_sec{kUnset},velocity_min_span_sec{kUnset};
  double max_velocity_fit_residual_m{kUnset},max_velocity_uncertainty_mps{kUnset};
};
struct TargetSelectionRoiConfig {Vector3d bounds_min_m,bounds_max_m;};
struct TaskConfig
{
  double cycle_timeout_sec{kUnset}, preparing_pick_timeout_sec{kUnset}, transport_timeout_sec{kUnset};
  double place_timeout_sec{kUnset}, recovery_timeout_sec{kUnset};
  std::size_t max_cycle_retries{0};
  double candidate_retry_interval_sec{kUnset};
  std::size_t max_candidate_admission_attempts_per_observation{0};
  std::optional<TargetSelectionRoiConfig> target_selection_roi;
};
struct InferenceConfig
{
  std::string backend, model_path, model_version;
  double request_timeout_sec{kUnset};
  std::size_t max_pending_requests{0};
  double defect_confidence_threshold{kUnset}, anomaly_score_threshold{kUnset};
  std::size_t rf_detr_min_positive_frames{0}, efficient_ad_min_positive_frames{0};
  std::vector<std::string> model_class_names,object_class_ids,defect_class_ids;
  double object_confidence_threshold{kUnset};
  double candidate_confidence_threshold{kUnset};
  std::string device;
};
struct CloudRegistrationConfig
{
  double coarse_voxel_size_m{kUnset},fine_voxel_size_m{kUnset};
  std::size_t max_points{0},max_iterations{0};
  double max_correspondence_distance_m{kUnset},min_inlier_fraction{kUnset};
  double convergence_translation_m{kUnset},convergence_yaw_rad{kUnset};
  double local_translation_limit_m{kUnset},local_yaw_limit_rad{kUnset};
};
struct CloudFusionConfig
{
  double evidence_voxel_size_m{kUnset};
  std::size_t max_points{0};
};
struct GeometryConfig
{
  bool enabled{false}; std::string healthy_reference_path;
  double registration_max_error_m{kUnset}, defect_distance_threshold_m{kUnset};
  double yaw_search_step_rad{kUnset};
  CloudRegistrationConfig registration;
  CloudFusionConfig fusion;
  std::size_t min_registration_points{0},min_defect_points{0};
  double min_observed_reference_fraction{kUnset};
  double defect_merge_distance_m{kUnset};
};
struct ReportConfig
{
  std::string output_dir;
  double save_timeout_sec{kUnset};
  std::size_t max_frames{0}, max_detections_per_frame{0};
  double max_total_megabytes{kUnset};
};
struct InspectionConfig
{
  double acquisition_timeout_sec{kUnset}, evaluation_timeout_sec{kUnset};
  double view_target_position_tolerance_m{kUnset}, view_target_orientation_tolerance_rad{kUnset};
  InferenceConfig inference;
  GeometryConfig geometry;
  ReportConfig report;
};
struct MotionConfig
{
  double observation_update_period_sec{.05},observation_preview_horizon_sec{.20};
  std::string planning_group,srdf_path,moveit_config_path,gripper_joint;
  std::vector<std::string> touch_links;
  double servo_position_gain{kUnset},servo_orientation_gain{kUnset},joint_stop_velocity{kUnset},gripper_width_factor{kUnset};
  double joint_position_tolerance_rad{kUnset};
  double joint_state_max_age_sec{kUnset}, tcp_state_max_age_sec{kUnset};
  double planning_timeout_sec{kUnset}, execution_timeout_sec{kUnset}, stop_confirmation_timeout_sec{kUnset};
  double target_update_timeout_sec{kUnset}, max_velocity_scale{kUnset}, max_acceleration_scale{kUnset};
  double position_tolerance_m{kUnset}, orientation_tolerance_rad{kUnset};
  double max_follow_linear_speed_mps{kUnset}, max_follow_angular_speed_radps{kUnset};
  Vector3d bounds_min_m{kUnset, kUnset, kUnset}, bounds_max_m{kUnset, kUnset, kUnset};
  Pose3d observation_pose{{kUnset, kUnset, kUnset}, {}};
};
struct BinHeightConfig
{
  Vector3d bounds_min_m, bounds_max_m;
  double release_clearance_m{0.04};
};
struct GraspConfig
{
  bool moving_intercept{false};
  double approach_timeout_sec{kUnset}, close_timeout_sec{kUnset}, verification_timeout_sec{kUnset};
  double release_timeout_sec{kUnset}, max_relative_linear_speed_mps{kUnset};
  Pose3d approach_tcp_in_object{{kUnset, kUnset, kUnset}, {}};
  Vector3d closing_axis_tcp{kUnset,kUnset,kUnset},finger_center_tcp{kUnset,kUnset,kUnset};
  double intercept_lead_sec{kUnset},contact_band_fraction{kUnset};
  double verification_probe_m{kUnset};
  double pregrasp_clearance_m{0.04};
  std::optional<std::array<Vector3d,2>> fingertip_box_tcp;
  std::optional<std::array<double,2>> pad_contact_size_m;
  std::optional<std::array<double,2>> finger_effort_thresholds_n;
  std::optional<double> minimum_holding_width_m;
  double support_plane_clearance_m{0.01};
  double descent_speed_mps{0.05}, probe_lift_speed_mps{0.05};
  std::size_t min_contact_points{0};
  double open_width_m{kUnset}, close_width_m{kUnset}, max_effort_n{kUnset};
  std::string verification_method;
  std::string neighbor_clearance_mode{"shadow"};
  std::optional<BinHeightConfig> bin_height;
  std::size_t min_confirmations{0};
  double lift_distance_m{kUnset};
  Pose3d bin_approach_pose{{kUnset, kUnset, kUnset}, {}}, bin_release_pose{{kUnset, kUnset, kUnset}, {}};
};
struct SceneConfig
{
  std::string static_geometry_path; double dynamic_object_max_age_sec{kUnset};
  bool object_cloud_enabled{false}, occupancy_map_enabled{false};
  double voxel_size_m{kUnset}, resolution_m{kUnset};
  std::size_t max_points_per_object{0};
};
struct DiagnosticsConfig
{
  std::size_t event_queue_capacity{256}, transition_history_capacity{512};
  bool record_rgbd{false};
  std::string recording_output_dir;
};
struct SystemSettings
{
  int schema_version{1}; std::string configuration_state{"draft"};
  RuntimeConfig runtime; FrameConfig frames; CalibrationConfig calibration; TimeConfig time;
  VisionConfig vision; TrackingConfig tracking; TaskConfig task; InspectionConfig inspection;
  MotionConfig motion; GraspConfig grasp; SceneConfig scene; DiagnosticsConfig diagnostics;
};
struct ConfigError {std::string path, message;};
std::vector<ConfigError> validate_settings(const SystemSettings & settings);

// Only a validated copy is published. No caller can mutate a frozen snapshot.
class SettingsSnapshot
{
public:
  const SystemSettings & get() const {return *settings_;}
  const std::string & revision() const {return revision_;}
  static std::optional<SettingsSnapshot> create(
    const SystemSettings & settings, const std::string & revision, std::vector<ConfigError> & errors);
private:
  SettingsSnapshot(const SystemSettings & settings, std::string revision)
  : settings_(std::make_shared<const SystemSettings>(settings)), revision_(std::move(revision)) {}
  std::shared_ptr<const SystemSettings> settings_;
  std::string revision_;
};
}  // namespace ptc_core
