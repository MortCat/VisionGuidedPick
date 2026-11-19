#include "ptc_app/settings_check_runner.hpp"
#include "ptc_app/settings_loader.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>
namespace ptc_app
{
int SettingsCheckRunner::run(int argc, char ** argv) const
{
  const bool manifest=argc==3 && std::string(argv[2])=="--export-json";
  if (argc != 2 && !manifest) {std::cerr << "Usage: settings_check <settings.yaml> [--export-json]\n"; return 2;}
  const auto result = SettingsLoader{}.load(std::filesystem::u8path(argv[1]));
  for (const auto & error : result.errors) {std::cerr << error.path << ": " << error.message << '\n';}
  if (!result.snapshot) {return 1;}
  if (manifest) {
    if (result.snapshot->get().schema_version!=2) {std::cerr<<"runtime manifest requires schema v2\n"; return 1;}
    const auto & s=result.snapshot->get(); const auto & model=s.inspection.inference;
    YAML::Emitter emitter; emitter.SetMapFormat(YAML::Flow); emitter.SetSeqFormat(YAML::Flow); emitter.SetStringFormat(YAML::DoubleQuoted);
    emitter.SetDoublePrecision(17);emitter<<YAML::BeginMap;
    const auto add=[&](const char * key,const auto & value){emitter<<YAML::Key<<key<<YAML::Value<<value;};
    add("schema_version",s.schema_version); add("settings_revision",result.snapshot->revision()); add("binding_revision",result.binding_revision);
    add("operation_mode",s.runtime.operation_mode); add("namespace",s.runtime.workstation_namespace); add("use_sim_time",s.runtime.use_sim_time);
    add("calibration_id",s.calibration.expected_calibration_id); add("world_frame",s.frames.world);
    add("motion_backend",s.runtime.motion_backend);add("robot_description_path",s.calibration.robot_description_path);
    add("srdf_path",s.motion.srdf_path);add("moveit_config_path",s.motion.moveit_config_path);
    add("joint_position_tolerance_rad",s.motion.joint_position_tolerance_rad);
    emitter<<YAML::Key<<"endpoints"<<YAML::Value<<YAML::BeginMap;
    for (const auto & pair : result.endpoints.names) {add(pair.first.c_str(),pair.second);}
    emitter<<YAML::EndMap;
    add("model_path",model.model_path); add("model_version",model.model_version); add("backend",model.backend);
    add("model_class_names",model.model_class_names); add("object_class_ids",model.object_class_ids); add("defect_class_ids",model.defect_class_ids);
    add("candidate_confidence_threshold",model.candidate_confidence_threshold); add("device",model.device);
    add("max_pending_requests",model.max_pending_requests); add("request_timeout_sec",model.request_timeout_sec);
    add("max_detections_per_frame",s.inspection.report.max_detections_per_frame);
    add("max_cache_frames",s.inspection.report.max_frames+s.vision.max_buffered_frames);
    add("max_frame_megabytes",s.vision.max_frame_megabytes); add("frame_max_age_sec",s.vision.frame_max_age_sec);
    add("sensor_future_tolerance_sec",s.time.sensor_future_tolerance_sec);
    emitter<<YAML::EndMap; std::cout<<emitter.c_str()<<'\n'; return 0;
  }
  std::cout << "Validated immutable settings " << result.snapshot->revision() << '\n'; return 0;
}
}  // namespace ptc_app
