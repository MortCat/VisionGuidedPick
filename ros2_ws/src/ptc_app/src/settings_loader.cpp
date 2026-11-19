#include "ptc_app/settings_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <openssl/evp.h>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace ptc_app
{
namespace
{
using namespace ptc_core;
using DigestContext = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
DigestContext digest_context()
{
  DigestContext context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr) != 1) {throw std::runtime_error("SHA256 initialization failed");}
  return context;
}
std::string digest_final(EVP_MD_CTX * context)
{
  std::array<unsigned char,EVP_MAX_MD_SIZE> bytes{}; unsigned int length = 0;
  if (EVP_DigestFinal_ex(context,bytes.data(),&length) != 1 || length != 32) {throw std::runtime_error("SHA256 finalization failed");}
  std::ostringstream out; out << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < length; ++i) {out << std::setw(2) << static_cast<unsigned int>(bytes[i]);} return out.str();
}
std::string read_yaml_text(const std::filesystem::path & path)
{
  if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > 1024*1024) {throw std::runtime_error("YAML must be a readable regular file <= 1 MiB");}
  std::ifstream in(path,std::ios::binary); if (!in) {throw std::runtime_error("cannot read YAML");}
  std::string text((std::istreambuf_iterator<char>(in)),{}); if (in.bad()) {throw std::runtime_error("YAML read failed");} return text;
}
const std::map<std::string,std::set<std::string>> & schema()
{
  static const std::map<std::string,std::set<std::string>> keys{
    {"",{"schema_version","configuration_state","runtime","frames","calibration","time","vision","tracking","task","inspection","motion","grasp","scene","diagnostics","ros"}},
    {"ros",{"endpoints"}},
    {"runtime",{"namespace","use_sim_time","motion_backend","operation_mode"}},
    {"frames",{"world","robot_base","tcp","color_optical","depth_optical"}},
    {"calibration",{"robot_description_path","expected_calibration_id","hand_eye_path"}}, {"time",{"sensor_future_tolerance_sec"}},
    {"vision",{"rgb_depth_sync_tolerance_sec","frame_max_age_sec","tf_lookup_timeout_sec","max_buffered_frames","roi_margin_ratio","depth_min_m","depth_max_m","depth_registration","min_valid_depth_points","min_laplacian_variance","min_brightness","max_brightness","max_frame_megabytes","foreground","tracking_camera"}},
    {"vision.tracking_camera",{"role","color_optical","depth_optical","calibration_id","camera_world"}},
    {"vision.tracking_camera.camera_world",{"position_m","orientation_xyzw"}},
    {"vision.foreground",{"source","bounds_min_m","bounds_max_m","support_plane_z_m","min_height_m","max_height_m","cell_size_m","min_span_m","max_span_m","min_points","max_objects"}},
    {"tracking",{"min_confirmed_observations","observation_max_age_sec","temporary_loss_timeout_sec","max_prediction_horizon_sec","association_distance_m","max_position_stddev_m","max_tracked_objects","max_yaw_stddev_rad","yaw_ambiguity_margin_m","velocity_min_samples","velocity_max_samples","velocity_window_sec","velocity_min_span_sec","max_velocity_fit_residual_m","max_velocity_uncertainty_mps"}},
    {"task",{"cycle_timeout_sec","preparing_pick_timeout_sec","transport_timeout_sec","place_timeout_sec","recovery_timeout_sec","max_cycle_retries","candidate_retry_interval_sec","max_candidate_admission_attempts_per_observation","target_selection_roi"}},
    {"task.target_selection_roi",{"bounds_min_m","bounds_max_m"}},
    {"inspection",{"acquisition_timeout_sec","evaluation_timeout_sec","view_target_position_tolerance_m","view_target_orientation_tolerance_rad","views","coverage","inference","geometry","report","view_transition_budget_sec","capture_budget_sec"}},
    {"inspection.report",{"output_dir","save_timeout_sec","max_frames","max_detections_per_frame","max_total_megabytes"}},
    {"inspection.views[]",{"id","direction_object_xyz","standoff_m","center_offset_object_m","elevation_m","roll_rad"}}, {"inspection.coverage",{"required_view_ids","min_valid_frames_per_view"}},
    {"inspection.inference",{"backend","model_path","model_version","request_timeout_sec","max_pending_requests","rf_detr","efficient_ad","model_class_names","object_class_ids","defect_class_ids","object_confidence_threshold","candidate_confidence_threshold","device"}},
    {"inspection.inference.rf_detr",{"defect_confidence_threshold","min_positive_frames"}},
    {"inspection.inference.efficient_ad",{"anomaly_score_threshold","min_positive_frames"}},
    {"inspection.geometry",{"enabled","healthy_reference_path","registration_max_error_m","defect_distance_threshold_m","voxel_size_m","yaw_search_step_rad","max_points","registration_iterations","min_registration_points","min_defect_points","min_observed_reference_fraction","defect_merge_distance_m","registration","fusion"}},
    {"inspection.geometry.registration",{"coarse_voxel_size_m","fine_voxel_size_m","max_points","max_iterations","max_correspondence_distance_m","min_inlier_fraction","convergence_translation_m","convergence_yaw_rad","local_translation_limit_m","local_yaw_limit_rad"}},
    {"inspection.geometry.fusion",{"evidence_voxel_size_m","max_points","max_keyframes","min_view_change_rad","min_new_surface_fraction"}},
    {"motion.observation_stream",{"update_period_sec","preview_horizon_sec"}},
    {"motion",{"observation_stream","moveit","joint_state_max_age_sec","tcp_state_max_age_sec","planning_timeout_sec","execution_timeout_sec","stop_confirmation_timeout_sec","target_update_timeout_sec","max_velocity_scale","max_acceleration_scale","position_tolerance_m","orientation_tolerance_rad","max_follow_linear_speed_mps","max_follow_angular_speed_radps","workspace","observation_pose"}},
    {"motion.moveit",{"planning_group","srdf_path","moveit_config_path","gripper_joint","touch_links","servo_position_gain","servo_orientation_gain","joint_stop_velocity","gripper_width_factor","joint_position_tolerance_rad"}},
    {"motion.workspace",{"bounds_min_m","bounds_max_m"}}, {"motion.observation_pose",{"position_m","orientation_xyzw"}},
    {"grasp",{"moving_intercept","approach_timeout_sec","close_timeout_sec","verification_timeout_sec","release_timeout_sec","max_relative_linear_speed_mps","approach_tcp_in_object","closing_axis_tcp","finger_center_tcp_m","intercept_lead_sec","contact_band_fraction","min_contact_points","verification_probe_m","pregrasp_clearance_m","neighbor_clearance_mode","pad_contact_size_m","finger_effort_thresholds_n","minimum_holding_width_m","descent_speed_mps","probe_lift_speed_mps","fingertip_box_tcp","support_plane_clearance_m","gripper","verification","lift_distance_m","bin_approach_pose","bin_release_pose","bin_height"}},
    {"grasp.approach_tcp_in_object",{"position_m","orientation_xyzw"}}, {"grasp.bin_approach_pose",{"position_m","orientation_xyzw"}},
    {"grasp.bin_release_pose",{"position_m","orientation_xyzw"}}, {"grasp.gripper",{"open_width_m","close_width_m","max_effort_n"}},
    {"grasp.bin_height",{"bounds_min_m","bounds_max_m","release_clearance_m"}},
    {"grasp.fingertip_box_tcp",{"bounds_min_m","bounds_max_m"}},
    {"grasp.verification",{"method","min_confirmations"}}, {"scene",{"static_geometry_path","dynamic_object_max_age_sec","object_cloud","occupancy_map"}},
    {"scene.object_cloud",{"enabled","voxel_size_m","max_points_per_object"}}, {"scene.occupancy_map",{"enabled","resolution_m"}},
    {"diagnostics",{"event_queue_capacity","transition_history_capacity","record_rgbd","recording_output_dir"}}
  }; return keys;
}
struct Reader
{
  YAML::Node root; std::vector<ConfigError> & errors; std::map<std::string,std::string> canonical;
  void error(const std::string & p, const std::string & message) {if (errors.size() < 1000) {errors.push_back({p,message});}}
  void check_tree(const YAML::Node & n, const std::string & p, unsigned depth, std::size_t & visits)
  {
    if (++visits > 10000 || depth > 32) {error(p,"YAML structure/alias expansion limit exceeded"); return;}
    if (n.Tag() != "?" && n.Tag() != "!" && !n.Tag().empty() && n.Tag().find("tag:yaml.org,2002:") != 0) {error(p,"custom YAML tags are forbidden");}
    if (n.IsMap()) {
      if (p == "ros.endpoints") {
        std::set<std::string> seen;
        for (const auto & pair : n) {
          if (!pair.first.IsScalar()) {error(p,"endpoint key must be text"); continue;}
          const auto key = pair.first.Scalar();
          if (retired_endpoints().count(key)) {error(p+"."+key,"retired five-view endpoint; remove this key");}
          else if (!seen.insert(key).second || !default_endpoints().count(key)) {error(p+"."+key,"unknown/duplicate logical endpoint");}
          if (!pair.second.IsScalar()) {error(p+"."+key,"endpoint must be relative text");}
        }
        return;
      }
      const auto allowed = schema().find(p); std::set<std::string> seen;
      if (allowed == schema().end()) {error(p,"unexpected mapping"); return;}
      for (const auto & pair : n) {
        if (!pair.first.IsScalar()) {error(p,"mapping key must be scalar text"); continue;}
        const auto key = pair.first.Scalar(); const auto child = p.empty() ? key : p+"."+key;
        if (!seen.insert(key).second) {error(child,"duplicate YAML key");}
        if (!allowed->second.count(key)) {error(child,"unknown YAML key"); continue;}
        check_tree(pair.second,child,depth+1,visits);
      }
    } else if (n.IsSequence()) {
      for (const auto & child : n) {check_tree(child,p+"[]",depth+1,visits); if (visits > 10000) {break;}}
    }
  }
  YAML::Node at(const std::string & p) const
  {
    YAML::Node n = root; std::size_t start = 0;
    while (start < p.size()) {
      auto end = p.find('.',start); if (end == std::string::npos) {end = p.size();}
      if (!n.IsMap()) {return YAML::Node(YAML::NodeType::Undefined);}
      const YAML::Node next = static_cast<const YAML::Node &>(n)[p.substr(start,end-start)];
      if(!next.IsDefined()){return YAML::Node(YAML::NodeType::Undefined);}
      n.reset(next); start = end+1;
    } return n;
  }
  bool scalar(const YAML::Node & n, const std::string & p, bool required)
  {
    if (!n.IsDefined() || n.IsNull()) {if (required) {error(p,"required value is missing/null");} return false;}
    if (!n.IsScalar()) {error(p,"expected scalar"); return false;} return true;
  }
  std::string string_value(const YAML::Node & n, const std::string & p, bool required = true)
  {
    if (!scalar(n,p,required)) {return {};}
    const auto text = n.Scalar(); const auto tag = n.Tag();
    static const std::regex numeric("[+-]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?");
    if ((tag == "?" && (text == "true" || text == "false" || std::regex_match(text,numeric))) ||
      (tag != "?" && tag != "!" && tag != "tag:yaml.org,2002:str")) {error(p,"expected text, not implicit numeric/bool"); return {};}
    canonical[p] = "s:"+text; return text;
  }
  double number_value(const YAML::Node & n, const std::string & p, bool required = true)
  {
    if (!scalar(n,p,required)) {return kUnset;}
    static const std::regex numeric("[+-]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?");
    if ((n.Tag() != "?" && n.Tag() != "tag:yaml.org,2002:float" && n.Tag() != "tag:yaml.org,2002:int") || !std::regex_match(n.Scalar(),numeric)) {error(p,"expected finite numeric scalar, not string/bool"); return kUnset;}
    std::istringstream in(n.Scalar()); in.imbue(std::locale::classic()); double value = kUnset; in >> value;
    if (!in || !in.eof() || !std::isfinite(value)) {error(p,"invalid/non-finite numeric scalar"); return kUnset;}
    std::ostringstream out; out.imbue(std::locale::classic()); out << std::scientific << std::setprecision(17) << (value == 0 ? 0 : value);
    canonical[p] = "f:"+out.str(); return value;
  }
  std::size_t integer(const std::string & p, bool required = true)
  {
    const auto n = at(p); if (!scalar(n,p,required)) {return 0;}
    static const std::regex decimal("[0-9]+"); std::uint64_t value = 0;
    if ((n.Tag() != "?" && n.Tag() != "tag:yaml.org,2002:int") || !std::regex_match(n.Scalar(),decimal)) {error(p,"expected nonnegative decimal integer, not bool/float/string"); return 0;}
    const auto text = n.Scalar(); const auto parsed = std::from_chars(text.data(),text.data()+text.size(),value);
    if (parsed.ec != std::errc{} || value > 1000000) {error(p,"integer overflow/supported count limit"); return 0;}
    canonical[p] = "i:"+std::to_string(value); return static_cast<std::size_t>(value);
  }
  bool boolean(const std::string & p)
  {
    const auto n = at(p); if (!scalar(n,p,true)) {return false;}
    if ((n.Tag() != "?" && n.Tag() != "tag:yaml.org,2002:bool") || (n.Scalar() != "true" && n.Scalar() != "false")) {error(p,"expected true/false boolean"); return false;}
    canonical[p] = "b:"+n.Scalar(); return n.Scalar() == "true";
  }
  std::string text(const std::string & p, bool required = true) {return string_value(at(p),p,required);}
  double number(const std::string & p, bool required = true) {return number_value(at(p),p,required);}
  std::vector<double> array(const YAML::Node & n, const std::string & p, std::size_t length)
  {
    if (!n.IsSequence() || n.size() != length) {error(p,"expected numeric sequence of length "+std::to_string(length)); return std::vector<double>(length,kUnset);}
    std::vector<double> values;
    for (std::size_t i = 0; i < length; ++i) {values.push_back(number_value(n[i],p+"["+std::to_string(i)+"]"));} return values;
  }
  std::vector<std::string> texts(const std::string & p)
  {
    const auto n=at(p); std::vector<std::string> out;
    if (!n.IsSequence()) {error(p,"expected text list"); return out;}
    for (std::size_t i=0;i<n.size();++i) {out.push_back(string_value(n[i],p+"["+std::to_string(i)+"]"));} return out;
  }
  Vector3d xyz(const std::string & p) {const auto v = array(at(p),p,3); return {v[0],v[1],v[2]};}
  Pose3d pose(const std::string & p)
  {const auto v = xyz(p+".position_m"); const auto q = array(at(p+".orientation_xyzw"),p+".orientation_xyzw",4); return {v,{q[0],q[1],q[2],q[3]}};}
};
SystemSettings read_settings(Reader & r)
{
  SystemSettings s;
  s.schema_version = static_cast<int>(r.integer("schema_version")); s.configuration_state = r.text("configuration_state");
  s.runtime = {r.text("runtime.namespace"),r.boolean("runtime.use_sim_time"),r.text("runtime.motion_backend")};
  if(r.at("runtime.operation_mode").IsDefined()){s.runtime.operation_mode=r.text("runtime.operation_mode");}
  const bool demo=s.runtime.operation_mode=="assumed_ng_demo";
  if(r.at("vision.tracking_camera").IsDefined()){
    auto & camera=s.vision.tracking_camera;camera.role=r.text("vision.tracking_camera.role");
    if(camera.role=="source") {camera.color_optical=r.text("vision.tracking_camera.color_optical");camera.depth_optical=r.text("vision.tracking_camera.depth_optical");camera.calibration_id=r.text("vision.tracking_camera.calibration_id");camera.camera_world=r.pose("vision.tracking_camera.camera_world");}
  }
  if(r.at("vision.foreground").IsDefined()) {
    s.vision.foreground_source=r.text("vision.foreground.source");auto & w=s.vision.depth_workspace;
    w.bounds_min_m=r.xyz("vision.foreground.bounds_min_m");w.bounds_max_m=r.xyz("vision.foreground.bounds_max_m");
    w.support_plane_z_m=r.number("vision.foreground.support_plane_z_m");w.min_height_m=r.number("vision.foreground.min_height_m");w.max_height_m=r.number("vision.foreground.max_height_m");
    w.cell_size_m=r.number("vision.foreground.cell_size_m");w.min_span_m=r.number("vision.foreground.min_span_m");w.max_span_m=r.number("vision.foreground.max_span_m");
    w.min_points=r.integer("vision.foreground.min_points");w.max_objects=r.integer("vision.foreground.max_objects");
  }
  s.frames = {r.text("frames.world"),r.text("frames.robot_base"),r.text("frames.tcp"),r.text("frames.color_optical"),r.text("frames.depth_optical")};
  s.calibration = {r.text("calibration.robot_description_path"),r.text("calibration.expected_calibration_id")};
#define N(section, field) s.section.field = r.number(#section "." #field)
#define I(section, field) s.section.field = r.integer(#section "." #field)
  N(time,sensor_future_tolerance_sec);
  N(vision,rgb_depth_sync_tolerance_sec); N(vision,frame_max_age_sec); N(vision,tf_lookup_timeout_sec); I(vision,max_buffered_frames);
  N(vision,roi_margin_ratio); N(vision,depth_min_m); N(vision,depth_max_m); s.vision.depth_registration = r.text("vision.depth_registration");
  I(tracking,min_confirmed_observations); N(tracking,observation_max_age_sec); N(tracking,temporary_loss_timeout_sec); N(tracking,max_prediction_horizon_sec);
  N(tracking,association_distance_m); N(tracking,max_position_stddev_m); I(tracking,max_tracked_objects);
  I(tracking,velocity_min_samples); I(tracking,velocity_max_samples);
  N(tracking,velocity_window_sec); N(tracking,velocity_min_span_sec);
  N(tracking,max_velocity_fit_residual_m); N(tracking,max_velocity_uncertainty_mps);
  N(task,cycle_timeout_sec); N(task,preparing_pick_timeout_sec); N(task,transport_timeout_sec); N(task,place_timeout_sec); N(task,recovery_timeout_sec); I(task,max_cycle_retries);
  N(task,candidate_retry_interval_sec); I(task,max_candidate_admission_attempts_per_observation);
  if(r.at("task.target_selection_roi").IsDefined()){
    s.task.target_selection_roi=TargetSelectionRoiConfig{r.xyz("task.target_selection_roi.bounds_min_m"),r.xyz("task.target_selection_roi.bounds_max_m")};
  }
  N(inspection,acquisition_timeout_sec); N(inspection,evaluation_timeout_sec); N(inspection,view_target_position_tolerance_m); N(inspection,view_target_orientation_tolerance_rad);
  auto & model = s.inspection.inference; model.backend = r.text("inspection.inference.backend"); model.model_path = r.text("inspection.inference.model_path",!demo);
  model.model_version = r.text("inspection.inference.model_version"); model.request_timeout_sec = r.number("inspection.inference.request_timeout_sec");
  model.max_pending_requests = r.integer("inspection.inference.max_pending_requests");
  model.defect_confidence_threshold = r.number("inspection.inference.rf_detr.defect_confidence_threshold",model.backend == "rf_detr");
  model.rf_detr_min_positive_frames = r.integer("inspection.inference.rf_detr.min_positive_frames",model.backend == "rf_detr");
  model.anomaly_score_threshold = r.number("inspection.inference.efficient_ad.anomaly_score_threshold",model.backend == "efficient_ad");
  model.efficient_ad_min_positive_frames = r.integer("inspection.inference.efficient_ad.min_positive_frames",model.backend == "efficient_ad");
  auto & geometry = s.inspection.geometry; geometry.enabled = r.boolean("inspection.geometry.enabled");
  const bool measured_geometry=r.at("vision.tracking_camera.role").as<std::string>("")=="external" &&
    r.at("vision.foreground.source").as<std::string>("")=="model";
  geometry.healthy_reference_path = r.text("inspection.geometry.healthy_reference_path",geometry.enabled&&!measured_geometry);
  geometry.registration_max_error_m = r.number("inspection.geometry.registration_max_error_m",geometry.enabled);
  geometry.defect_distance_threshold_m = r.number("inspection.geometry.defect_distance_threshold_m",geometry.enabled);
  N(motion,joint_state_max_age_sec); N(motion,tcp_state_max_age_sec); N(motion,planning_timeout_sec); N(motion,execution_timeout_sec);
  N(motion,stop_confirmation_timeout_sec); N(motion,target_update_timeout_sec); N(motion,max_velocity_scale); N(motion,max_acceleration_scale);
  N(motion,position_tolerance_m); N(motion,orientation_tolerance_rad); N(motion,max_follow_linear_speed_mps); N(motion,max_follow_angular_speed_radps);
  const auto observation_period=r.at("motion.observation_stream.update_period_sec");
  const auto observation_horizon=r.at("motion.observation_stream.preview_horizon_sec");
  s.motion.observation_update_period_sec=r.number_value(observation_period.IsDefined()?observation_period:YAML::Load("0.05"),"motion.observation_stream.update_period_sec");
  s.motion.observation_preview_horizon_sec=r.number_value(observation_horizon.IsDefined()?observation_horizon:YAML::Load("0.20"),"motion.observation_stream.preview_horizon_sec");
  s.motion.bounds_min_m = r.xyz("motion.workspace.bounds_min_m"); s.motion.bounds_max_m = r.xyz("motion.workspace.bounds_max_m"); s.motion.observation_pose = r.pose("motion.observation_pose");
  if(r.at("grasp.moving_intercept").IsDefined()){s.grasp.moving_intercept=r.boolean("grasp.moving_intercept");}
  N(grasp,approach_timeout_sec); N(grasp,close_timeout_sec); N(grasp,verification_timeout_sec); N(grasp,release_timeout_sec); N(grasp,max_relative_linear_speed_mps);
  s.grasp.approach_tcp_in_object = r.pose("grasp.approach_tcp_in_object");
  s.grasp.open_width_m = r.number("grasp.gripper.open_width_m"); s.grasp.close_width_m = r.number("grasp.gripper.close_width_m"); s.grasp.max_effort_n = r.number("grasp.gripper.max_effort_n");
  s.grasp.verification_method = r.text("grasp.verification.method"); s.grasp.min_confirmations = r.integer("grasp.verification.min_confirmations"); N(grasp,lift_distance_m);
  s.grasp.bin_approach_pose = r.pose("grasp.bin_approach_pose"); s.grasp.bin_release_pose = r.pose("grasp.bin_release_pose");
  if(r.at("grasp.bin_height").IsDefined()){
    s.grasp.bin_height=BinHeightConfig{r.xyz("grasp.bin_height.bounds_min_m"),
      r.xyz("grasp.bin_height.bounds_max_m"),r.number("grasp.bin_height.release_clearance_m")};
  }
  s.scene.static_geometry_path = r.text("scene.static_geometry_path"); N(scene,dynamic_object_max_age_sec);
  s.scene.object_cloud_enabled = r.boolean("scene.object_cloud.enabled"); s.scene.occupancy_map_enabled = r.boolean("scene.occupancy_map.enabled");
  s.scene.voxel_size_m = r.number("scene.object_cloud.voxel_size_m",s.scene.object_cloud_enabled);
  s.scene.max_points_per_object = r.integer("scene.object_cloud.max_points_per_object",s.scene.object_cloud_enabled);
  s.scene.resolution_m = r.number("scene.occupancy_map.resolution_m",s.scene.occupancy_map_enabled);
  I(diagnostics,event_queue_capacity); I(diagnostics,transition_history_capacity);
  s.diagnostics.record_rgbd = r.boolean("diagnostics.record_rgbd");
  s.diagnostics.recording_output_dir = r.text("diagnostics.recording_output_dir",s.diagnostics.record_rgbd);
  if (s.schema_version == 2) {
    s.calibration.hand_eye_path = r.text("calibration.hand_eye_path");
    I(vision,min_valid_depth_points); N(vision,min_laplacian_variance);
    N(vision,min_brightness); N(vision,max_brightness); N(vision,max_frame_megabytes);
    N(tracking,max_yaw_stddev_rad); N(tracking,yaw_ambiguity_margin_m);
    model.model_class_names=r.texts("inspection.inference.model_class_names");
    model.object_class_ids=r.texts("inspection.inference.object_class_ids");
    model.defect_class_ids=r.texts("inspection.inference.defect_class_ids");
    model.object_confidence_threshold=r.number("inspection.inference.object_confidence_threshold");
    model.candidate_confidence_threshold=r.number("inspection.inference.candidate_confidence_threshold");
    model.device=r.text("inspection.inference.device");
    for (const auto & legacy : {"voxel_size_m","max_points","registration_iterations"}) {if(r.at(std::string("inspection.geometry.")+legacy).IsDefined()) {r.error(std::string("inspection.geometry.")+legacy,"migrate to registration/fusion settings");}}
    geometry.yaw_search_step_rad = r.number("inspection.geometry.yaw_search_step_rad");
    geometry.registration.coarse_voxel_size_m = r.number("inspection.geometry.registration.coarse_voxel_size_m");
    geometry.registration.fine_voxel_size_m = r.number("inspection.geometry.registration.fine_voxel_size_m");
    geometry.registration.max_points = r.integer("inspection.geometry.registration.max_points");
    geometry.registration.max_iterations = r.integer("inspection.geometry.registration.max_iterations");
    geometry.registration.max_correspondence_distance_m = r.number("inspection.geometry.registration.max_correspondence_distance_m");
    geometry.registration.min_inlier_fraction = r.number("inspection.geometry.registration.min_inlier_fraction");
    geometry.registration.convergence_translation_m = r.number("inspection.geometry.registration.convergence_translation_m");
    geometry.registration.convergence_yaw_rad = r.number("inspection.geometry.registration.convergence_yaw_rad");
    geometry.registration.local_translation_limit_m = r.number("inspection.geometry.registration.local_translation_limit_m");
    geometry.registration.local_yaw_limit_rad = r.number("inspection.geometry.registration.local_yaw_limit_rad");
    geometry.fusion.evidence_voxel_size_m = r.number("inspection.geometry.fusion.evidence_voxel_size_m");
    geometry.fusion.max_points = r.integer("inspection.geometry.fusion.max_points");
    geometry.min_registration_points=r.integer("inspection.geometry.min_registration_points");
    geometry.min_defect_points=r.integer("inspection.geometry.min_defect_points");
    geometry.defect_merge_distance_m=r.number("inspection.geometry.defect_merge_distance_m");
    s.grasp.closing_axis_tcp=r.xyz("grasp.closing_axis_tcp");
    s.grasp.finger_center_tcp=r.xyz("grasp.finger_center_tcp_m");
    s.grasp.intercept_lead_sec=r.number("grasp.intercept_lead_sec");
    s.grasp.contact_band_fraction=r.number("grasp.contact_band_fraction");
    s.grasp.min_contact_points=r.integer("grasp.min_contact_points");
    s.grasp.verification_probe_m=r.number("grasp.verification_probe_m");
    if(r.at("grasp.pregrasp_clearance_m").IsDefined()){s.grasp.pregrasp_clearance_m=r.number("grasp.pregrasp_clearance_m");}
    if(r.at("grasp.neighbor_clearance_mode").IsDefined()){
      s.grasp.neighbor_clearance_mode=r.text("grasp.neighbor_clearance_mode");
    }
    if(r.at("grasp.fingertip_box_tcp").IsDefined()){s.grasp.fingertip_box_tcp=std::array<Vector3d,2>{r.xyz("grasp.fingertip_box_tcp.bounds_min_m"),r.xyz("grasp.fingertip_box_tcp.bounds_max_m")};}
    for(const auto & entry : {std::make_pair("grasp.pad_contact_size_m",&s.grasp.pad_contact_size_m),
                             std::make_pair("grasp.finger_effort_thresholds_n",&s.grasp.finger_effort_thresholds_n)}){
      const auto values=r.at(entry.first);
      if(!values.IsDefined())continue;
      if(!values.IsSequence()||values.size()!=2)throw std::invalid_argument(std::string(entry.first)+" requires two values");
      *entry.second=std::array<double,2>{values[0].as<double>(),values[1].as<double>()};
    }
    if(r.at("grasp.minimum_holding_width_m").IsDefined()){
      s.grasp.minimum_holding_width_m=r.number("grasp.minimum_holding_width_m");
    }
    if(r.at("grasp.support_plane_clearance_m").IsDefined()){s.grasp.support_plane_clearance_m=r.number("grasp.support_plane_clearance_m");}
    if(r.at("grasp.descent_speed_mps").IsDefined()){s.grasp.descent_speed_mps=r.number("grasp.descent_speed_mps");}
    else{s.grasp.descent_speed_mps=s.grasp.max_relative_linear_speed_mps>0?s.grasp.max_relative_linear_speed_mps:.05;}
    if(r.at("grasp.probe_lift_speed_mps").IsDefined()){s.grasp.probe_lift_speed_mps=r.number("grasp.probe_lift_speed_mps");}
    else{s.grasp.probe_lift_speed_mps=s.grasp.max_relative_linear_speed_mps>0?s.grasp.max_relative_linear_speed_mps:.05;}
    if(s.runtime.motion_backend=="moveit") {
      s.motion.planning_group=r.text("motion.moveit.planning_group");s.motion.srdf_path=r.text("motion.moveit.srdf_path");
      s.motion.moveit_config_path=r.text("motion.moveit.moveit_config_path");
      s.motion.gripper_joint=r.text("motion.moveit.gripper_joint");s.motion.touch_links=r.texts("motion.moveit.touch_links");
      s.motion.servo_position_gain=r.number("motion.moveit.servo_position_gain");s.motion.servo_orientation_gain=r.number("motion.moveit.servo_orientation_gain");
      s.motion.joint_stop_velocity=r.number("motion.moveit.joint_stop_velocity");s.motion.gripper_width_factor=r.number("motion.moveit.gripper_width_factor");
      s.motion.joint_position_tolerance_rad=r.number("motion.moveit.joint_position_tolerance_rad");
    }
    geometry.min_observed_reference_fraction=r.number("inspection.geometry.min_observed_reference_fraction");
    auto & report = s.inspection.report;
    report.output_dir = r.text("inspection.report.output_dir");
    report.save_timeout_sec = r.number("inspection.report.save_timeout_sec");
    report.max_frames = r.integer("inspection.report.max_frames");
    report.max_detections_per_frame = r.integer("inspection.report.max_detections_per_frame");
    report.max_total_megabytes=r.number("inspection.report.max_total_megabytes");
  }
#undef N
#undef I
  return s;
}
std::filesystem::path resolve(const std::filesystem::path & base, const std::string & value)
{return std::filesystem::absolute(base/std::filesystem::u8path(value)).lexically_normal();}
void resource(SettingsLoadResult & result, Reader & r, const std::filesystem::path & base,
  std::string & value, const std::string & field, const std::string & kind, const std::string & expected_id, const std::string & backend = {})
{
  try {
    const auto path = resolve(base,value); const auto digest = sha256_file(path);
    const auto documents = YAML::LoadAll(read_yaml_text(std::filesystem::u8path(path.u8string()+".identity.yaml")));
    if (documents.size() != 1 || !documents[0].IsMap()) {throw std::runtime_error("identity manifest must be one mapping");}
    const auto manifest = documents[0]; std::set<std::string> keys;
    if (manifest.Tag() != "?" && !manifest.Tag().empty() && manifest.Tag() != "tag:yaml.org,2002:map") {throw std::runtime_error("custom identity manifest tags are forbidden");}
    for (const auto & pair : manifest) {
      if (!pair.first.IsScalar() || !keys.insert(pair.first.Scalar()).second ||
        (pair.first.Scalar() != "kind" && pair.first.Scalar() != "id" && pair.first.Scalar() != "sha256" && pair.first.Scalar() != "backend" && pair.first.Scalar() != "dependencies")) {throw std::runtime_error("unknown/duplicate identity manifest key");}
    }
    const auto manifest_kind = r.string_value(manifest["kind"],field+".identity.kind");
    const auto id = r.string_value(manifest["id"],field+".identity.id");
    const auto declared = r.string_value(manifest["sha256"],field+".identity.sha256");
    if (manifest_kind != kind || id.empty() || (!expected_id.empty() && id != expected_id) || declared != digest ||
      (!backend.empty() && r.string_value(manifest["backend"],field+".identity.backend") != backend)) {throw std::runtime_error("asset digest/kind/version/calibration/backend mismatch");}
    result.resources.push_back({kind,id,digest,path}); value = path.u8string();
    r.canonical["asset."+kind] = id+":"+digest+":"+backend;
    const auto dependencies=manifest["dependencies"];
    if (dependencies.IsDefined()) {
      if (!dependencies.IsSequence()) {throw std::runtime_error("asset dependencies must be a list");}
      std::set<std::string> dependency_names;
      for (std::size_t i=0;i<dependencies.size();++i) {
        const auto dependency=dependencies[i];
        if (!dependency.IsMap() || dependency.size()!=2 || !dependency["path"] || !dependency["sha256"]) {
          throw std::runtime_error("dependency requires path and sha256");
        }
        const auto name=dependency["path"].as<std::string>(); const auto expected=dependency["sha256"].as<std::string>();
        if (!dependency_names.insert(name).second) {throw std::runtime_error("duplicate asset dependency");}
        const auto dep_path=resolve(path.parent_path(),name); const auto dep_digest=sha256_file(dep_path);
        if (dep_digest!=expected) {throw std::runtime_error("asset dependency digest mismatch");}
        result.resources.push_back({kind+"_dependency",name,dep_digest,dep_path});
        r.canonical["asset."+kind+".dependency["+std::to_string(i)+"]"]=dep_digest;
      }
    }
  } catch (const std::exception & e) {r.error(field,e.what());}
}
std::string semantic_revision(Reader & r, const SystemSettings & s)
{
  const auto excluded = [&](const std::string & p) {
      if (p == "runtime.namespace" || p == "runtime.motion_backend" || p.find("ros.endpoints.") == 0 ||
        p == "inspection.report.output_dir" || p.find("_path") != std::string::npos || p.find("_output_dir") != std::string::npos) {return true;}
      if (p.rfind("inspection.views",0)==0 || p.rfind("inspection.coverage",0)==0 ||
        p=="inspection.view_transition_budget_sec" || p=="inspection.capture_budget_sec" ||
        p=="inspection.geometry.fusion.max_keyframes" || p=="inspection.geometry.fusion.min_view_change_rad" ||
        p=="inspection.geometry.fusion.min_new_surface_fraction") {return true;}
      if (s.inspection.inference.backend != "rf_detr" && p.find("inspection.inference.rf_detr.") == 0) {return true;}
      if (s.inspection.inference.backend != "efficient_ad" && p.find("inspection.inference.efficient_ad.") == 0) {return true;}
      if (!s.inspection.geometry.enabled && p.find("inspection.geometry.") == 0 && p != "inspection.geometry.enabled") {return true;}
      if (!s.scene.object_cloud_enabled && p.find("scene.object_cloud.") == 0 && p != "scene.object_cloud.enabled") {return true;}
      if (!s.scene.occupancy_map_enabled && p.find("scene.occupancy_map.") == 0 && p != "scene.occupancy_map.enabled") {return true;}
      return false;
    };
  std::string canonical = "ptc-semantic-schema-"+std::to_string(s.schema_version)+"\n";
  for (const auto & pair : r.canonical) {if (!excluded(pair.first)) {
      canonical += std::to_string(pair.first.size())+":"+pair.first+std::to_string(pair.second.size())+":"+pair.second+"\n";
    }} return "sha256:"+sha256_text(canonical);
}
}  // namespace
std::string sha256_text(const std::string & text)
{
  auto context = digest_context();
  if (EVP_DigestUpdate(context.get(),text.data(),text.size()) != 1) {throw std::runtime_error("SHA256 update failed");}
  return digest_final(context.get());
}
std::string sha256_file(const std::filesystem::path & path)
{
  if (!std::filesystem::is_regular_file(path)) {throw std::runtime_error("asset must be a readable regular file");}
  const auto size = std::filesystem::file_size(path); const auto modified = std::filesystem::last_write_time(path);
  std::ifstream in(path,std::ios::binary); if (!in) {throw std::runtime_error("cannot read asset");}
  auto context = digest_context(); std::array<char,65536> buffer{}; std::uintmax_t read = 0;
  while (in) {in.read(buffer.data(),buffer.size()); const auto count = in.gcount(); read += static_cast<std::uintmax_t>(count);
    if (EVP_DigestUpdate(context.get(),buffer.data(),static_cast<std::size_t>(count)) != 1) {throw std::runtime_error("SHA256 update failed");}}
  if (in.bad() || read != size || std::filesystem::file_size(path) != size || std::filesystem::last_write_time(path) != modified) {throw std::runtime_error("asset read failed/changed during snapshot");}
  return digest_final(context.get());
}
SettingsLoadResult SettingsLoader::load(const std::filesystem::path & yaml_path) const
{
  SettingsLoadResult result;
  try {
    const auto documents = YAML::LoadAll(read_yaml_text(yaml_path));
    if (documents.size() != 1 || !documents[0].IsMap()) {throw std::runtime_error("settings must be exactly one YAML mapping");}
    Reader r{documents[0],result.errors,{}}; std::size_t visits = 0; r.check_tree(r.root,"",0,visits);
    if (!result.errors.empty()) {return result;}
    auto s = read_settings(r); const auto semantic_errors = validate_settings(s);
    result.endpoints.names = default_endpoints();
    std::string binding = s.runtime.workstation_namespace+"\n"+s.runtime.motion_backend+"\n";
    std::set<std::string> endpoint_names;
    for (auto & pair : result.endpoints.names) {
      if (s.schema_version == 2 && ((pair.first!="planning_scene" && pair.first!="object_cloud_preview" && pair.first!="grasp_neighborhood" && pair.first!="inspection_view_status" && pair.first!="controller_owner_enable" && pair.first.rfind("observer_",0)!=0 && pair.first!="shared_tracks") || r.at("ros.endpoints."+pair.first).IsDefined())) {pair.second = r.text("ros.endpoints."+pair.first);}
      if (!valid_relative_endpoint(pair.second)) {r.error("ros.endpoints."+pair.first,"must be a relative ROS name");}
      if (!endpoint_names.insert(pair.second).second) {r.error("ros.endpoints."+pair.first,"logical endpoints must not collide");}
      // Optional display telemetry does not change the device/control binding identity.
      if(pair.first!="inspection_view_status"){binding += pair.first+"="+pair.second+"\n";}
    }
    result.binding_revision = "sha256:"+sha256_text(binding);
    result.errors.insert(result.errors.end(),semantic_errors.begin(),semantic_errors.end());
    if (!result.errors.empty()) {return result;}
    const auto base = std::filesystem::absolute(yaml_path).parent_path();
    resource(result,r,base,s.calibration.robot_description_path,"calibration.robot_description_path","robot_description",s.calibration.expected_calibration_id);
    if (s.schema_version == 2) {resource(result,r,base,s.calibration.hand_eye_path,"calibration.hand_eye_path","hand_eye",s.calibration.expected_calibration_id);}
    if(s.schema_version==2 && s.runtime.motion_backend=="moveit") {resource(result,r,base,s.motion.srdf_path,"motion.moveit.srdf_path","robot_semantic",s.calibration.expected_calibration_id);}
    if(s.schema_version==2 && s.runtime.motion_backend=="moveit") {resource(result,r,base,s.motion.moveit_config_path,"motion.moveit.moveit_config_path","moveit_config",{});}
    if(s.runtime.operation_mode!="assumed_ng_demo"){resource(result,r,base,s.inspection.inference.model_path,"inspection.inference.model_path","model",s.inspection.inference.model_version,s.inspection.inference.backend);}
    resource(result,r,base,s.scene.static_geometry_path,"scene.static_geometry_path","static_geometry",{});
    if (s.inspection.geometry.enabled && !(s.vision.tracking_camera.role=="external" && s.vision.foreground_source=="model")) {
      resource(result,r,base,s.inspection.geometry.healthy_reference_path,"inspection.geometry.healthy_reference_path","healthy_reference",{});
    }
    const auto output = [&](std::string & value, const std::string & field, bool enabled) {
        if (value.empty()) {return;} const auto path = resolve(base,value); value = path.u8string();
        if (enabled && (!std::filesystem::is_directory(path) ||
          (std::filesystem::status(path).permissions() & (std::filesystem::perms::owner_write|std::filesystem::perms::group_write|std::filesystem::perms::others_write)) == std::filesystem::perms::none)) {r.error(field,"output directory must exist and permit writes; runtime writer also checks ACL/I/O errors");}
      };
    output(s.diagnostics.recording_output_dir,"diagnostics.recording_output_dir",s.diagnostics.record_rgbd);
    if (s.schema_version == 2) {s.inspection.report.output_dir = resolve(base,s.inspection.report.output_dir).u8string();}
    if (result.errors.empty()) {result.snapshot = SettingsSnapshot::create(s,semantic_revision(r,s),result.errors);}
  } catch (const std::exception & e) {result.errors.push_back({"settings",e.what()});}
  return result;
}
}  // namespace ptc_app
