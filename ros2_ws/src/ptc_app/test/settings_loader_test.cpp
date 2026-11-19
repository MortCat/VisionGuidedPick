#include "ptc_app/settings_loader.hpp"
#include <cmath>
#include "test_support.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <fstream>
#include <filesystem>
using namespace ptc_app;
using namespace ptc_test;
namespace fs = std::filesystem;

struct TestDirectory
{
  fs::path path;
  explicit TestDirectory(const fs::path & base) : path(base/("loader-fixture-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
  {CHECK(fs::create_directory(path));}
  ~TestDirectory() {std::error_code error; fs::remove_all(path,error);} // Only this newly created, narrowly scoped fixture directory.
};
void write_text(const fs::path & path, const std::string & text)
{std::ofstream out(path,std::ios::binary); out << text; CHECK(out.good());}
void write_yaml(const fs::path & path, const YAML::Node & node)
{YAML::Emitter out; out << node; CHECK(out.good()); write_text(path,out.c_str());}

void tests(const fs::path & fixtures, const fs::path & output, const fs::path & draft)
{
  CHECK(sha256_text("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  SettingsLoader loader; auto valid = loader.load(fixtures/"settings.yaml");
  if (!valid.snapshot) {for (const auto & e : valid.errors) {std::cerr << e.path << ": " << e.message << '\n';}}
  CHECK(valid.snapshot); CHECK(valid.resources.size() == 3); CHECK(fs::path(valid.snapshot->get().calibration.robot_description_path).is_absolute());
  CHECK(!loader.load(draft).snapshot); CHECK(!loader.load(fixtures/"missing.yaml").snapshot);
  TestDirectory tmp(output);
  for (const auto & file : fs::directory_iterator(fixtures)) {if (file.is_regular_file()) {fs::copy_file(file.path(),tmp.path/file.path().filename());}}
  auto node = YAML::LoadFile((fixtures/"settings.yaml").string());
  const auto check = [&](const YAML::Node & n) {write_yaml(tmp.path/"settings.yaml",n); return loader.load(tmp.path/"settings.yaml");};
  auto relocated = check(node); CHECK(relocated.snapshot); CHECK(relocated.snapshot->revision() == valid.snapshot->revision());
  node["runtime"]["namespace"] = "other_cell"; node["runtime"]["motion_backend"] = "moveit";
  node["diagnostics"]["recording_output_dir"] = "different_log";
  auto deployed = check(node); CHECK(deployed.snapshot); CHECK(deployed.snapshot->revision() == valid.snapshot->revision());
  node["task"]["candidate_retry_interval_sec"] = 0.8; CHECK(check(node).snapshot->revision() != valid.snapshot->revision());
  node = YAML::LoadFile((fixtures/"settings.yaml").string());
  node["tracking"]["velocity_min_span_sec"] = 0.3; CHECK(check(node).snapshot->revision() != valid.snapshot->revision());
  node = YAML::LoadFile((fixtures/"settings.yaml").string());
  node["inspection"]["inference"]["rf_detr"]["defect_confidence_threshold"] = 0.9;
  CHECK(check(node).snapshot->revision() != valid.snapshot->revision());
  const auto original = YAML::LoadFile((fixtures/"settings.yaml").string());
  const auto bad = [&](const std::string & field, const YAML::Node & value) {
      auto n = YAML::Clone(original); auto split = field.find('.'); n[field.substr(0,split)][field.substr(split+1)] = value;
      if (check(n).snapshot) {throw std::runtime_error("invalid fixture accepted: "+field+" = "+YAML::Dump(value));}
    };
  bad("vision.max_buffered_frames",YAML::Node(true)); bad("vision.max_buffered_frames",YAML::Node(-1));
  bad("vision.max_buffered_frames",YAML::Node(1.5));
  bad("task.candidate_retry_interval_sec",YAML::Node(0));
  bad("task.max_candidate_admission_attempts_per_observation",YAML::Node(0));
  bad("task.max_candidate_admission_attempts_per_observation",YAML::Node(6));
  bad("tracking.velocity_min_span_sec",YAML::Node(0.6));
  bad("tracking.velocity_max_samples",YAML::Node(65));
  bad("tracking.max_velocity_uncertainty_mps",YAML::Node(0));
  auto numeric_text = YAML::Node("0.1"); numeric_text.SetTag("tag:yaml.org,2002:str"); bad("vision.depth_min_m",numeric_text);
  bad("vision.depth_min_m",YAML::Load(".nan")); bad("vision.depth_min_m",YAML::Load(".inf"));
  bad("vision.depth_min_m",YAML::Node(YAML::NodeType::Null)); bad("vision.unrecognized",YAML::Node(1));
  bad("frames.world",YAML::Node(true)); bad("calibration.expected_calibration_id",YAML::Node("other_installation"));
  bad("calibration.robot_description_path",YAML::Node("absent.urdf"));
  auto invalid = YAML::Clone(original); invalid["schema_version"] = 2; CHECK(!check(invalid).snapshot);
  invalid = YAML::Clone(original); invalid["motion"]["observation_pose"]["orientation_xyzw"] = YAML::Load("[0,0,0,0]"); CHECK(!check(invalid).snapshot);
  invalid = YAML::Clone(original); invalid["scene"]["occupancy_map"]["enabled"] = true; CHECK(!check(invalid).snapshot);
  YAML::Emitter emitted; emitted << original;
  write_text(tmp.path/"settings.yaml",std::string(emitted.c_str())+"\nschema_version: 1\n"); CHECK(!loader.load(tmp.path/"settings.yaml").snapshot);
  write_text(tmp.path/"settings.yaml",std::string(emitted.c_str())+"\n---\n{}"); CHECK(!loader.load(tmp.path/"settings.yaml").snapshot);
  write_text(tmp.path/"settings.yaml","runtime: &recursive {namespace: *recursive}\n"); CHECK(!loader.load(tmp.path/"settings.yaml").snapshot);
  invalid = YAML::Clone(original); invalid["inspection"]["inference"]["model_version"] = "model_v2"; CHECK(!check(invalid).snapshot);
  write_text(tmp.path/"synthetic_model.fixture","changed bytes\n"); CHECK(!check(original).snapshot);
  auto identity = YAML::LoadFile((fixtures/"synthetic_model.fixture.identity.yaml").string()); identity["sha256"] = sha256_file(tmp.path/"synthetic_model.fixture");
  write_yaml(tmp.path/"synthetic_model.fixture.identity.yaml",identity); auto changed = check(original); CHECK(changed.snapshot); CHECK(changed.snapshot->revision() != valid.snapshot->revision());
  identity["id"] = "wrong_version"; write_yaml(tmp.path/"synthetic_model.fixture.identity.yaml",identity); CHECK(!check(original).snapshot);

  // Synthetic v2 migration exercises settings semantics, not model/device readiness.
  fs::copy_file(fixtures/"synthetic_model.fixture",tmp.path/"synthetic_model.fixture",fs::copy_options::overwrite_existing);
  fs::copy_file(fixtures/"synthetic_model.fixture.identity.yaml",tmp.path/"synthetic_model.fixture.identity.yaml",fs::copy_options::overwrite_existing);
  write_text(tmp.path/"healthy.fixture","0 0 0\n");
  YAML::Node healthy; healthy["kind"] = "healthy_reference"; healthy["id"] = "synthetic_healthy";
  healthy["sha256"] = sha256_file(tmp.path/"healthy.fixture"); write_yaml(tmp.path/"healthy.fixture.identity.yaml",healthy);
  auto v2 = YAML::Clone(original); v2["schema_version"] = 2;
  v2["vision"]["min_valid_depth_points"]=10; v2["vision"]["min_laplacian_variance"]=1;
  v2["vision"]["min_brightness"]=1; v2["vision"]["max_brightness"]=254; v2["vision"]["max_frame_megabytes"]=8;
  v2["tracking"]["max_yaw_stddev_rad"]=0.2; v2["tracking"]["yaw_ambiguity_margin_m"]=0.001;
  v2["inspection"]["inference"]["model_class_names"]=YAML::Load("[bunny,defect]");
  v2["inspection"]["inference"]["object_class_ids"]=YAML::Load("[bunny]");
  v2["inspection"]["inference"]["defect_class_ids"]=YAML::Load("[defect]");
  v2["inspection"]["inference"]["object_confidence_threshold"]=0.5;
  v2["inspection"]["inference"]["candidate_confidence_threshold"]=0.1; v2["inspection"]["inference"]["device"]="cpu";
  write_text(tmp.path/"hand_eye.fixture","synthetic boundary asset\n");
  YAML::Node hand_eye; hand_eye["kind"]="hand_eye"; hand_eye["id"]="synthetic_calibration";
  hand_eye["sha256"]=sha256_file(tmp.path/"hand_eye.fixture"); write_yaml(tmp.path/"hand_eye.fixture.identity.yaml",hand_eye);
  v2["calibration"]["hand_eye_path"]="hand_eye.fixture";
  v2["inspection"]["geometry"] = YAML::Load("{enabled: true, healthy_reference_path: healthy.fixture, registration_max_error_m: 0.01, defect_distance_threshold_m: 0.02, yaw_search_step_rad: 0.1, registration: {coarse_voxel_size_m: 0.01, fine_voxel_size_m: 0.002, max_points: 1000, max_iterations: 10, max_correspondence_distance_m: 0.03, min_inlier_fraction: 0.5, convergence_translation_m: 0.0001, convergence_yaw_rad: 0.001, local_translation_limit_m: 0.02, local_yaw_limit_rad: 0.1}, fusion: {evidence_voxel_size_m: 0.002, max_points: 1000}}");
  v2["inspection"]["geometry"]["min_registration_points"]=10; v2["inspection"]["geometry"]["min_defect_points"]=3;
  v2["inspection"]["geometry"]["min_observed_reference_fraction"]=0.1;
  v2["inspection"]["geometry"]["defect_merge_distance_m"]=0.02;
  v2["grasp"]["closing_axis_tcp"]=YAML::Load("[1,0,0]");v2["grasp"]["finger_center_tcp_m"]=YAML::Load("[0,0,0.1]");
  v2["grasp"]["intercept_lead_sec"]=0.2;v2["grasp"]["contact_band_fraction"]=0.3;v2["grasp"]["min_contact_points"]=3;
  v2["grasp"]["verification_probe_m"]=0.01;
  v2["inspection"]["report"] = YAML::Load("{output_dir: reports, save_timeout_sec: 1, max_frames: 10, max_detections_per_frame: 50}");
  v2["inspection"]["report"]["max_total_megabytes"]=64;
  for (const auto & endpoint : default_endpoints()) {v2["ros"]["endpoints"][endpoint.first] = endpoint.second;}
  auto migrated = check(v2);
  if(!migrated.snapshot){for(const auto & e:migrated.errors){std::cerr<<e.path<<": "<<e.message<<'\n';}}
  CHECK(migrated.snapshot); CHECK(migrated.snapshot->get().schema_version == 2);
  CHECK(std::abs(migrated.snapshot->get().grasp.pregrasp_clearance_m-.04)<1e-9);
  auto bad_clearance=YAML::Clone(v2);bad_clearance["grasp"]["pregrasp_clearance_m"]=0;CHECK(!check(bad_clearance).snapshot);
  bad_clearance["grasp"]["pregrasp_clearance_m"]=-.04;CHECK(!check(bad_clearance).snapshot);
  CHECK(migrated.snapshot->get().grasp.neighbor_clearance_mode=="shadow");
  auto neighbor_ranking=YAML::Clone(v2);
  neighbor_ranking["grasp"]["neighbor_clearance_mode"]="rank";
  const auto ranked=check(neighbor_ranking);
  CHECK(ranked.snapshot && ranked.snapshot->get().grasp.neighbor_clearance_mode=="rank");
  neighbor_ranking["grasp"]["neighbor_clearance_mode"]="unknown";
  CHECK(!check(neighbor_ranking).snapshot);
  v2["ros"]["endpoints"]["rgb"] = "another/image"; auto remapped = check(v2);
  CHECK(remapped.snapshot); CHECK(remapped.snapshot->revision() == migrated.snapshot->revision());
  CHECK(remapped.binding_revision != migrated.binding_revision);
  // New motion speeds are optional and do not replace alignment acceptance.
  const auto legacy_speeds=check(v2);CHECK(legacy_speeds.snapshot);
  const double legacy_speed=legacy_speeds.snapshot->get().grasp.max_relative_linear_speed_mps>0?legacy_speeds.snapshot->get().grasp.max_relative_linear_speed_mps:.05;
  CHECK(legacy_speeds.snapshot->get().grasp.descent_speed_mps==legacy_speed);
  CHECK(legacy_speeds.snapshot->get().grasp.probe_lift_speed_mps==legacy_speed);
  auto legacy_custom=YAML::Clone(v2);legacy_custom["grasp"]["max_relative_linear_speed_mps"]=.12;
  const auto custom_loaded=check(legacy_custom);CHECK(custom_loaded.snapshot);
  CHECK(custom_loaded.snapshot->get().grasp.descent_speed_mps==.12);
  CHECK(custom_loaded.snapshot->get().grasp.probe_lift_speed_mps==.12);
  auto speeds=YAML::Clone(v2);speeds["grasp"]["descent_speed_mps"]=.20;speeds["grasp"]["probe_lift_speed_mps"]=.18;
  const auto speed_loaded=check(speeds);CHECK(speed_loaded.snapshot);
  CHECK(speed_loaded.snapshot->get().grasp.descent_speed_mps==.20);
  CHECK(speed_loaded.snapshot->get().grasp.probe_lift_speed_mps==.18);
  CHECK(speed_loaded.snapshot->get().grasp.max_relative_linear_speed_mps==legacy_speeds.snapshot->get().grasp.max_relative_linear_speed_mps);
  CHECK(speed_loaded.snapshot->revision()!=legacy_speeds.snapshot->revision());
  for(const auto * field:{"descent_speed_mps","probe_lift_speed_mps"}){
    for(const auto * invalid:{"0","-0.1",".nan",".inf"}){
      auto bad_speed=YAML::Clone(speeds);bad_speed["grasp"][field]=YAML::Load(invalid);CHECK(!check(bad_speed).snapshot);
    }
  }
  auto finger_settings=YAML::Clone(v2);
  finger_settings["grasp"]["fingertip_box_tcp"]["bounds_min_m"]=YAML::Load("[-0.01,-0.06,-0.04]");
  finger_settings["grasp"]["fingertip_box_tcp"]["bounds_max_m"]=YAML::Load("[0.01,0.06,0.009]");
  finger_settings["grasp"]["support_plane_clearance_m"]=.01;
  const auto calibrated_fingers=check(finger_settings);CHECK(calibrated_fingers.snapshot);
  CHECK(calibrated_fingers.snapshot->get().grasp.fingertip_box_tcp);
  CHECK(!check(v2).snapshot->get().grasp.fingertip_box_tcp);
  CHECK(calibrated_fingers.snapshot->revision()!=check(v2).snapshot->revision());
  auto inverted_fingers=YAML::Clone(finger_settings);inverted_fingers["grasp"]["fingertip_box_tcp"]["bounds_max_m"]=YAML::Load("[-0.02,0.06,0.009]");CHECK(!check(inverted_fingers).snapshot);
  finger_settings["grasp"]["support_plane_clearance_m"]=0;CHECK(!check(finger_settings).snapshot);
  auto roi=YAML::Clone(v2);roi["task"]["target_selection_roi"]["bounds_min_m"]=YAML::Load("[0.45,-1.10,0.75]");
  roi["task"]["target_selection_roi"]["bounds_max_m"]=YAML::Load("[0.775,-0.80,1.10]");
  auto roi_loaded=check(roi);CHECK(roi_loaded.snapshot);CHECK(roi_loaded.snapshot->get().task.target_selection_roi);
  CHECK(roi_loaded.snapshot->revision()!=check(v2).snapshot->revision());
  roi["task"]["target_selection_roi"]["bounds_max_m"]=YAML::Load("[0.40,-0.80,1.10]");CHECK(!check(roi).snapshot);
  auto disabled = YAML::Clone(v2); disabled["inspection"]["geometry"]["enabled"] = false; CHECK(!check(disabled).snapshot);
  auto duplicate = YAML::Clone(v2); duplicate["ros"]["endpoints"]["depth"] = "another/image"; CHECK(!check(duplicate).snapshot);
  // Active minimal endpoint contract: retired five-view keys are absent and rejected;
  // current grasp capture, view status and the pick-only control endpoints remain.
  for (const auto * active : {"rgb","depth","frame","tracks","shared_tracks","capture_grasp_surface","grasp_neighborhood",
         "grasp_evaluate","motion_check","arm_move","arm_follow","arm_stop","gripper_command","gripper_status",
         "task_command","task_status","inspection_view_status"}) {CHECK(migrated.endpoints.names.count(active) == 1);}
  for (const auto & retired : retired_endpoints()) {
    CHECK(!default_endpoints().count(retired)); CHECK(!migrated.endpoints.names.count(retired));
    auto stale = YAML::Clone(v2); stale["ros"]["endpoints"][retired] = "legacy/"+retired; const auto rejected = check(stale);
    CHECK(!rejected.snapshot);
    bool reported = false;
    for (const auto & e : rejected.errors) {reported = reported || (e.path == "ros.endpoints."+retired && e.message.find("retired") != std::string::npos);}
    CHECK(reported);
  }
  CHECK(migrated.endpoints.at("capture_grasp_surface") == "vision/capture_grasp_surface");
}
int main(int argc, char ** argv)
{if (argc != 4) {return 2;} return run([&] {tests(fs::u8path(argv[1]),fs::u8path(argv[2]),fs::u8path(argv[3]));});}
