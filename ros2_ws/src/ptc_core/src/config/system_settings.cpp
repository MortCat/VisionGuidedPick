#include "ptc_core/config/system_settings.hpp"

#include <cmath>
#include <set>
#include <utility>
#include <algorithm>

#include "ptc_core/move_to_pose.hpp"
#include "ptc_core/pose_math.hpp"

namespace ptc_core
{
std::vector<ConfigError> validate_settings(const SystemSettings & s)
{
  std::vector<ConfigError> errors;
  const auto check = [&](bool ok, const std::string & path, const char * message) {
      if (!ok) {errors.push_back({path, message});}
    };
  const auto positive = [&](double value, const std::string & path) {
      check(std::isfinite(value) && value > 0.0, path, "must be finite and positive");
    };
  const auto duration = [&](double value, const std::string & path, bool zero = false) {
      check(std::isfinite(value) && (zero ? value == 0.0 || value >= 1e-9 : value >= 1e-9) &&
        value < 9223372036.0, path, "invalid duration or nanosecond overflow");
    };
  const auto count = [&](std::size_t value, const std::string & path, std::size_t min = 1) {
      check(value >= min && value <= 1000000, path, "count outside supported [minimum, 1000000]");
    };
  const auto text = [&](const std::string & value, const std::string & path) {
      check(!value.empty(), path, "must be non-empty");
    };
  check(s.schema_version == 1 || s.schema_version == 2, "schema_version", "supported schemas: 1 (legacy), 2");
  check(s.configuration_state == "ready", "configuration_state", "draft cannot run operations");
  const bool demo=s.runtime.operation_mode=="assumed_ng_demo";
  check(demo || s.runtime.operation_mode=="production","runtime.operation_mode","select production or assumed_ng_demo");
  check((demo && s.inspection.inference.backend=="assumed_ng") || (!demo && s.inspection.inference.backend!="assumed_ng"),
    "inspection.inference.backend","assumed_ng requires explicit assumed_ng_demo mode");
  check(s.vision.foreground_source=="model" || (demo && s.vision.foreground_source=="depth_workspace"),
    "vision.foreground.source","depth workspace segmentation is enabled explicitly for this demo");
  if(s.vision.foreground_source=="depth_workspace") {
    const auto & w=s.vision.depth_workspace;
    positive(w.cell_size_m,"vision.foreground.cell_size_m");count(w.min_points,"vision.foreground.min_points",3);count(w.max_objects,"vision.foreground.max_objects");
    check(std::isfinite(w.support_plane_z_m) && std::isfinite(std::hypot(w.bounds_min_m.x_m,w.bounds_min_m.y_m,w.bounds_min_m.z_m)) && std::isfinite(std::hypot(w.bounds_max_m.x_m,w.bounds_max_m.y_m,w.bounds_max_m.z_m)) &&
      w.bounds_min_m.x_m<w.bounds_max_m.x_m && w.bounds_min_m.y_m<w.bounds_max_m.y_m && w.bounds_min_m.z_m<w.bounds_max_m.z_m,
      "vision.foreground","invalid calibrated support/workspace");
    check(w.min_height_m>0 && w.max_height_m>w.min_height_m && w.min_span_m>0 && w.max_span_m>w.min_span_m,
      "vision.foreground","invalid foreground height/span bounds");
  }
  text(s.runtime.workstation_namespace, "runtime.namespace");
  const auto valid_namespace = [](const std::string & name) {
      bool segment_start = true;
      for (const char c : name) {
        if (c == '/') {if (segment_start) {return false;} segment_start = true; continue;}
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
        if (!letter && (segment_start || c < '0' || c > '9')) {return false;} segment_start = false;
      } return !segment_start;
    };
  check(!s.runtime.workstation_namespace.empty() && s.runtime.workstation_namespace.front() != '/' &&
    s.runtime.workstation_namespace.find("//") == std::string::npos,
    "runtime.namespace", "must be a relative namespace");
  check(valid_namespace(s.runtime.workstation_namespace),"runtime.namespace","invalid ROS namespace token");
  check(s.runtime.motion_backend == "mock" || s.runtime.motion_backend == "moveit",
    "runtime.motion_backend", "select mock or moveit");
  text(s.frames.world, "frames.world"); text(s.frames.robot_base, "frames.robot_base");
  text(s.frames.tcp, "frames.tcp"); text(s.frames.color_optical, "frames.color_optical");
  text(s.frames.depth_optical, "frames.depth_optical");
  const auto frame = [&](const std::string & id, const std::string & path) {
      check(!id.empty() && id.front() != '/' && id.find_first_of(" \t\r\n") == std::string::npos,
        path,"frame id must be nonempty, relative and contain no whitespace");
    };
  frame(s.frames.world,"frames.world"); frame(s.frames.robot_base,"frames.robot_base"); frame(s.frames.tcp,"frames.tcp");
  frame(s.frames.color_optical,"frames.color_optical"); frame(s.frames.depth_optical,"frames.depth_optical");
  text(s.calibration.robot_description_path, "calibration.robot_description_path");
  text(s.calibration.expected_calibration_id, "calibration.expected_calibration_id");
  duration(s.time.sensor_future_tolerance_sec, "time.sensor_future_tolerance_sec", true);
#define P(section, field) positive(s.section.field, #section "." #field)
#define D(section, field) duration(s.section.field, #section "." #field)
#define C(section, field) count(s.section.field, #section "." #field)
  D(vision, rgb_depth_sync_tolerance_sec); D(vision, frame_max_age_sec); D(vision, tf_lookup_timeout_sec);
  C(vision, max_buffered_frames); P(vision, depth_min_m); P(vision, depth_max_m);
  check(s.vision.depth_min_m < s.vision.depth_max_m, "vision.depth_max_m", "must exceed minimum depth");
  check(std::isfinite(s.vision.roi_margin_ratio) && s.vision.roi_margin_ratio >= 0.0 &&
    s.vision.roi_margin_ratio <= 1.0, "vision.roi_margin_ratio", "must be in [0,1]");
  check(s.vision.depth_registration == "required", "vision.depth_registration", "registration is required");
  count(s.tracking.min_confirmed_observations, "tracking.min_confirmed_observations", 2);
  D(tracking, observation_max_age_sec); D(tracking, temporary_loss_timeout_sec);
  duration(s.tracking.max_prediction_horizon_sec, "tracking.max_prediction_horizon_sec", true);
  check(s.tracking.max_prediction_horizon_sec <= s.tracking.temporary_loss_timeout_sec,
    "tracking.max_prediction_horizon_sec", "cannot exceed loss timeout");
  P(tracking, association_distance_m); P(tracking, max_position_stddev_m); C(tracking, max_tracked_objects);
  count(s.tracking.velocity_min_samples, "tracking.velocity_min_samples", 3);
  count(s.tracking.velocity_max_samples, "tracking.velocity_max_samples", s.tracking.velocity_min_samples);
  check(s.tracking.velocity_max_samples<=64,"tracking.velocity_max_samples","bounded history cannot exceed 64 samples");
  D(tracking, velocity_window_sec); D(tracking, velocity_min_span_sec);
  check(s.tracking.velocity_min_span_sec<=s.tracking.velocity_window_sec,
    "tracking.velocity_min_span_sec","cannot exceed the velocity window");
  P(tracking, max_velocity_fit_residual_m); P(tracking, max_velocity_uncertainty_mps);
  D(task, cycle_timeout_sec); D(task, preparing_pick_timeout_sec); D(task, transport_timeout_sec);
  D(task, place_timeout_sec); D(task, recovery_timeout_sec);
  count(s.task.max_cycle_retries, "task.max_cycle_retries", 0);
  D(task, candidate_retry_interval_sec);
  if(s.task.target_selection_roi){
    const auto & lo=s.task.target_selection_roi->bounds_min_m;const auto & hi=s.task.target_selection_roi->bounds_max_m;
    check(std::isfinite(std::hypot(lo.x_m,lo.y_m,lo.z_m))&&std::isfinite(std::hypot(hi.x_m,hi.y_m,hi.z_m))&&
      lo.x_m<hi.x_m&&lo.y_m<hi.y_m&&lo.z_m<hi.z_m,"task.target_selection_roi","invalid world-coordinate bounds");
  }
  check(s.task.candidate_retry_interval_sec <= s.task.cycle_timeout_sec,
    "task.candidate_retry_interval_sec", "cannot exceed the cycle timeout");
  count(s.task.max_candidate_admission_attempts_per_observation,
    "task.max_candidate_admission_attempts_per_observation", 1);
  check(s.task.max_candidate_admission_attempts_per_observation <= 5,
    "task.max_candidate_admission_attempts_per_observation", "cannot exceed five attempts");
  D(inspection, acquisition_timeout_sec); D(inspection, evaluation_timeout_sec);
  P(inspection, view_target_position_tolerance_m); P(inspection, view_target_orientation_tolerance_rad);
  check(s.inspection.view_target_orientation_tolerance_rad <= 3.141592653589793,
    "inspection.view_target_orientation_tolerance_rad", "cannot exceed pi");
  const auto & i = s.inspection.inference;
  check(i.backend == "rf_detr" || i.backend == "efficient_ad" || (demo && i.backend=="assumed_ng"), "inspection.inference.backend", "invalid backend");
  if(!demo){text(i.model_path, "inspection.inference.model_path");} text(i.model_version, "inspection.inference.model_version");
  duration(i.request_timeout_sec, "inspection.inference.request_timeout_sec");
  count(i.max_pending_requests, "inspection.inference.max_pending_requests");
  if (i.backend == "rf_detr") {
    check(std::isfinite(i.defect_confidence_threshold) && i.defect_confidence_threshold >= 0.0 &&
      i.defect_confidence_threshold <= 1.0, "inspection.inference.rf_detr.defect_confidence_threshold", "must be in [0,1]");
    count(i.rf_detr_min_positive_frames, "inspection.inference.rf_detr.min_positive_frames");
  } else if (i.backend == "efficient_ad") {
    check(std::isfinite(i.anomaly_score_threshold), "inspection.inference.efficient_ad.anomaly_score_threshold", "must be finite");
    count(i.efficient_ad_min_positive_frames, "inspection.inference.efficient_ad.min_positive_frames");
  }
  const auto & observer=s.vision.tracking_camera;
  check(observer.role=="disabled"||observer.role=="source"||observer.role=="external","vision.tracking_camera.role","choose disabled, source or external");
  if(observer.role=="source") {text(observer.color_optical,"vision.tracking_camera.color_optical");text(observer.depth_optical,"vision.tracking_camera.depth_optical");text(observer.calibration_id,"vision.tracking_camera.calibration_id");check(is_valid_pose(observer.camera_world),"vision.tracking_camera.camera_world","known fixed camera pose required");}
  if (s.inspection.geometry.enabled) {
    if (!(s.vision.tracking_camera.role=="external" && s.vision.foreground_source=="model")) {
      text(s.inspection.geometry.healthy_reference_path, "inspection.geometry.healthy_reference_path");
    }
    positive(s.inspection.geometry.registration_max_error_m, "inspection.geometry.registration_max_error_m");
    positive(s.inspection.geometry.defect_distance_threshold_m, "inspection.geometry.defect_distance_threshold_m");
  }
  if (s.schema_version == 2) {
    text(s.calibration.hand_eye_path,"calibration.hand_eye_path");
    C(vision,min_valid_depth_points); P(vision,min_laplacian_variance); P(vision,max_frame_megabytes);
    check(s.vision.max_frame_megabytes<=256,"vision.max_frame_megabytes","per-frame limit cannot exceed 256 MiB");
    check(std::isfinite(s.vision.min_brightness) && std::isfinite(s.vision.max_brightness) && s.vision.min_brightness>=0 &&
      s.vision.max_brightness<=255 && s.vision.min_brightness<s.vision.max_brightness,"vision","invalid brightness bounds");
    P(tracking,max_yaw_stddev_rad); P(tracking,yaw_ambiguity_margin_m);
    check(s.tracking.max_yaw_stddev_rad<3.141592653589793,"tracking.max_yaw_stddev_rad","yaw must be identifiable within pi");
    check(std::isfinite(i.object_confidence_threshold) && i.object_confidence_threshold>0 && i.object_confidence_threshold<=1,
      "inspection.inference.object_confidence_threshold","must be in (0,1]");
    check(std::isfinite(i.candidate_confidence_threshold) && i.candidate_confidence_threshold>=0 &&
      i.candidate_confidence_threshold<=(demo?i.object_confidence_threshold:std::min(i.object_confidence_threshold,i.defect_confidence_threshold)),
      "inspection.inference.candidate_confidence_threshold","candidate threshold must not hide required positive evidence");
    check(i.device=="cpu" || i.device=="cuda", "inspection.inference.device", "select cpu or cuda");
    std::set<std::string> class_names,roles;
    for (const auto & name : i.model_class_names) {check(!name.empty() && class_names.insert(name).second,
      "inspection.inference.model_class_names","class labels must be nonempty and unique");}
    check(!i.object_class_ids.empty() && (demo || !i.defect_class_ids.empty()),"inspection.inference","object and defect classes are required");
    for (const auto & name : i.object_class_ids) {check(class_names.count(name) && roles.insert(name).second,"inspection.inference.object_class_ids","unknown/duplicate label");}
    for (const auto & name : i.defect_class_ids) {check(class_names.count(name) && roles.insert(name).second,"inspection.inference.defect_class_ids","unknown/duplicate or object/defect overlap");}
    check(s.inspection.geometry.enabled, "inspection.geometry.enabled", "pick geometry settings are required");
    positive(s.inspection.geometry.registration.coarse_voxel_size_m, "inspection.geometry.registration.coarse_voxel_size_m");
    positive(s.inspection.geometry.registration.fine_voxel_size_m, "inspection.geometry.registration.fine_voxel_size_m");
    count(s.inspection.geometry.registration.max_points, "inspection.geometry.registration.max_points");
    count(s.inspection.geometry.registration.max_iterations, "inspection.geometry.registration.max_iterations");
    positive(s.inspection.geometry.registration.max_correspondence_distance_m, "inspection.geometry.registration.max_correspondence_distance_m");
    positive(s.inspection.geometry.registration.min_inlier_fraction, "inspection.geometry.registration.min_inlier_fraction");
    positive(s.inspection.geometry.registration.convergence_translation_m, "inspection.geometry.registration.convergence_translation_m");
    positive(s.inspection.geometry.registration.convergence_yaw_rad, "inspection.geometry.registration.convergence_yaw_rad");
    positive(s.inspection.geometry.registration.local_translation_limit_m, "inspection.geometry.registration.local_translation_limit_m");
    positive(s.inspection.geometry.registration.local_yaw_limit_rad, "inspection.geometry.registration.local_yaw_limit_rad");
    positive(s.inspection.geometry.fusion.evidence_voxel_size_m, "inspection.geometry.fusion.evidence_voxel_size_m");
    count(s.inspection.geometry.fusion.max_points, "inspection.geometry.fusion.max_points");
    const auto & registration=s.inspection.geometry.registration;
    check(registration.coarse_voxel_size_m>=registration.fine_voxel_size_m && registration.min_inlier_fraction<=1 &&
      registration.local_yaw_limit_rad<=3.141592653589793 && registration.max_points>=s.inspection.geometry.min_registration_points,
      "inspection.geometry.registration","invalid scale/correspondence/local limits");
    positive(s.inspection.geometry.yaw_search_step_rad, "inspection.geometry.yaw_search_step_rad");
    check(s.inspection.geometry.yaw_search_step_rad <= 3.141592653589793,
      "inspection.geometry.yaw_search_step_rad", "cannot exceed pi");

    count(s.inspection.geometry.min_registration_points,"inspection.geometry.min_registration_points",3);
    count(s.inspection.geometry.min_defect_points,"inspection.geometry.min_defect_points");
    positive(s.inspection.geometry.defect_merge_distance_m,"inspection.geometry.defect_merge_distance_m");
    positive(s.inspection.report.max_total_megabytes,"inspection.report.max_total_megabytes");
    check(s.inspection.geometry.min_registration_points<=s.inspection.geometry.fusion.max_points &&
      std::isfinite(s.inspection.geometry.min_observed_reference_fraction) && s.inspection.geometry.min_observed_reference_fraction>0 &&
      s.inspection.geometry.min_observed_reference_fraction<=1,"inspection.geometry","invalid geometric sample/coverage limits");
    text(s.inspection.report.output_dir, "inspection.report.output_dir");
    duration(s.inspection.report.save_timeout_sec, "inspection.report.save_timeout_sec");
    count(s.inspection.report.max_frames, "inspection.report.max_frames");
    count(s.inspection.report.max_detections_per_frame, "inspection.report.max_detections_per_frame");
    check(s.inspection.report.max_frames <= 4096 && s.inspection.report.max_detections_per_frame <= 4096 &&
      s.tracking.max_tracked_objects <= 256,"inspection.report", "exceeds ROS wire capacity");
  }
  D(motion, joint_state_max_age_sec); D(motion, tcp_state_max_age_sec); D(motion, planning_timeout_sec);
  D(motion, execution_timeout_sec); D(motion, stop_confirmation_timeout_sec); D(motion, target_update_timeout_sec);
  check(is_valid_motion_scale(s.motion.max_velocity_scale), "motion.max_velocity_scale", "must be in (0,1]");
  check(is_valid_motion_scale(s.motion.max_acceleration_scale), "motion.max_acceleration_scale", "must be in (0,1]");
  P(motion, position_tolerance_m); P(motion, orientation_tolerance_rad);
  check(s.motion.orientation_tolerance_rad <= 3.141592653589793, "motion.orientation_tolerance_rad", "cannot exceed pi");
  P(motion, max_follow_linear_speed_mps); P(motion, max_follow_angular_speed_radps);
  check(std::isfinite(s.motion.observation_update_period_sec)&&s.motion.observation_update_period_sec>0&&
    std::isfinite(s.motion.observation_preview_horizon_sec)&&s.motion.observation_preview_horizon_sec>=s.motion.observation_update_period_sec&&s.motion.observation_preview_horizon_sec<=.5,
    "motion.observation_stream","invalid period or existing bridge horizon exceeded");
  const auto & lo = s.motion.bounds_min_m; const auto & hi = s.motion.bounds_max_m;
  check(std::isfinite(lo.x_m) && std::isfinite(lo.y_m) && std::isfinite(lo.z_m) &&
    std::isfinite(hi.x_m) && std::isfinite(hi.y_m) && std::isfinite(hi.z_m) &&
    lo.x_m < hi.x_m && lo.y_m < hi.y_m && lo.z_m < hi.z_m,
    "motion.workspace", "invalid bounds");
  check(is_valid_pose(s.motion.observation_pose), "motion.observation_pose", "invalid pose");
  D(grasp, approach_timeout_sec); D(grasp, close_timeout_sec); D(grasp, verification_timeout_sec); D(grasp, release_timeout_sec);
  check(std::isfinite(s.grasp.max_relative_linear_speed_mps) && s.grasp.max_relative_linear_speed_mps >= 0.0,
    "grasp.max_relative_linear_speed_mps", "must be finite and non-negative");
  P(grasp,descent_speed_mps); P(grasp,probe_lift_speed_mps); P(grasp,support_plane_clearance_m);
  check(s.grasp.neighbor_clearance_mode=="shadow" || s.grasp.neighbor_clearance_mode=="rank",
    "grasp.neighbor_clearance_mode","require shadow or rank");
  if(const auto & size=s.grasp.pad_contact_size_m){
    check(std::isfinite((*size)[0]) && std::isfinite((*size)[1]) && (*size)[0]>0 && (*size)[1]>0,
      "grasp.pad_contact_size_m","require positive finite contact width and height");
  }
  if(const auto width=s.grasp.minimum_holding_width_m){
    check(std::isfinite(*width) && *width>s.grasp.close_width_m && *width<s.grasp.open_width_m,
      "grasp.minimum_holding_width_m","require a finite aperture between closed and fully open");
  }
  if(const auto & force=s.grasp.finger_effort_thresholds_n){
    check(std::isfinite((*force)[0]) && std::isfinite((*force)[1]) && (*force)[0]>=0 && (*force)[1]>(*force)[0],
      "grasp.finger_effort_thresholds_n","require finite 0 <= empty threshold < holding threshold");
  }
  if(s.grasp.fingertip_box_tcp){
    const auto & lo=(*s.grasp.fingertip_box_tcp)[0];const auto & hi=(*s.grasp.fingertip_box_tcp)[1];
    check(std::isfinite(lo.x_m)&&std::isfinite(lo.y_m)&&std::isfinite(lo.z_m)&&std::isfinite(hi.x_m)&&std::isfinite(hi.y_m)&&std::isfinite(hi.z_m)&&
      lo.x_m<hi.x_m&&lo.y_m<hi.y_m&&lo.z_m<hi.z_m,"grasp.fingertip_box_tcp","finite ordered TCP-frame bounds required");
  }
  check(is_valid_pose(s.grasp.approach_tcp_in_object), "grasp.approach_tcp_in_object", "invalid pose");
  if(s.schema_version==2) {
    const auto & a=s.grasp.closing_axis_tcp; const auto & p=s.grasp.finger_center_tcp;
    check(std::isfinite(a.x_m+a.y_m+a.z_m) && std::abs(std::sqrt(a.x_m*a.x_m+a.y_m*a.y_m+a.z_m*a.z_m)-1)<1e-6,"grasp.closing_axis_tcp","unit vector required");
    check(std::isfinite(p.x_m+p.y_m+p.z_m),"grasp.finger_center_tcp_m","finite fingertip offset required");
    D(grasp,intercept_lead_sec); C(grasp,min_contact_points);
    P(grasp,verification_probe_m);
    P(grasp,pregrasp_clearance_m);
    check(std::isfinite(s.grasp.contact_band_fraction) && s.grasp.contact_band_fraction>0 && s.grasp.contact_band_fraction<=1,"grasp.contact_band_fraction","require (0,1]");
  }
  if(s.schema_version==2 && s.runtime.motion_backend=="moveit") {
    text(s.motion.planning_group,"motion.moveit.planning_group");text(s.motion.srdf_path,"motion.moveit.srdf_path");text(s.motion.gripper_joint,"motion.moveit.gripper_joint");
    text(s.motion.moveit_config_path,"motion.moveit.moveit_config_path");
    check(!s.motion.touch_links.empty(),"motion.moveit.touch_links","gripper contact links required");
    P(motion,servo_position_gain);P(motion,servo_orientation_gain);P(motion,joint_stop_velocity);P(motion,gripper_width_factor);P(motion,joint_position_tolerance_rad);
  }
  check(std::isfinite(s.grasp.open_width_m) && std::isfinite(s.grasp.close_width_m) &&
    s.grasp.close_width_m >= 0 && s.grasp.close_width_m < s.grasp.open_width_m,
    "grasp.gripper", "require 0 <= close < open");
  if(demo){check(std::isfinite(s.grasp.max_effort_n) && s.grasp.max_effort_n>=0,"grasp.max_effort_n","demo effort must be nonnegative; zero selects position-only adapter");}
  else{P(grasp,max_effort_n);} C(grasp, min_confirmations); P(grasp, lift_distance_m);
  check(s.grasp.verification_method == "joint_and_visual" || s.grasp.verification_method == "contact_and_visual",
    "grasp.verification.method", "verification evidence method is required");
  check(is_valid_pose(s.grasp.bin_approach_pose), "grasp.bin_approach_pose", "invalid pose");
  check(is_valid_pose(s.grasp.bin_release_pose), "grasp.bin_release_pose", "invalid pose");
  if(s.grasp.bin_height){
    const auto & bin=*s.grasp.bin_height;
    check(std::isfinite(std::hypot(bin.bounds_min_m.x_m,bin.bounds_min_m.y_m,bin.bounds_min_m.z_m)) && std::isfinite(std::hypot(bin.bounds_max_m.x_m,bin.bounds_max_m.y_m,bin.bounds_max_m.z_m)) &&
      bin.bounds_min_m.x_m<bin.bounds_max_m.x_m && bin.bounds_min_m.y_m<bin.bounds_max_m.y_m &&
      bin.bounds_min_m.z_m<bin.bounds_max_m.z_m &&
      std::isfinite(bin.release_clearance_m) && bin.release_clearance_m>0,
      "grasp.bin_height","finite ordered interior bounds and positive clearance required");
  }
  text(s.scene.static_geometry_path, "scene.static_geometry_path"); D(scene, dynamic_object_max_age_sec);
  if (s.scene.object_cloud_enabled) {P(scene, voxel_size_m); C(scene, max_points_per_object);}
  if (s.scene.occupancy_map_enabled) {P(scene, resolution_m);}
  C(diagnostics, event_queue_capacity); C(diagnostics, transition_history_capacity);
  if (s.diagnostics.record_rgbd) {text(s.diagnostics.recording_output_dir, "diagnostics.recording_output_dir");}
#undef P
#undef D
#undef C
  return errors;
}

std::optional<SettingsSnapshot> SettingsSnapshot::create(
  const SystemSettings & settings, const std::string & revision, std::vector<ConfigError> & errors)
{
  errors = validate_settings(settings);
  if (revision.empty()) {errors.push_back({"settings_revision", "must be non-empty"});}
  if (!errors.empty()) {return std::nullopt;}
  return SettingsSnapshot(settings, revision);
}
}  // namespace ptc_core
