#pragma once
#include <iostream>
#include <stdexcept>
#include <string>
#include "ptc_core/config/system_settings.hpp"
#include "ptc_core/state/device_snapshot.hpp"

#define CHECK(expression) do {if (!(expression)) {throw std::runtime_error( \
  std::string(__FILE__) + ":" + std::to_string(__LINE__) + " " + #expression);}} while (false)

namespace ptc_test
{
inline ptc_core::SystemSettings settings_data()
{
  using namespace ptc_core;
  SystemSettings s;
  s.configuration_state = "ready"; s.runtime = {"cell_1", true, "mock"};
  s.frames = {"world", "base", "tcp", "rgb_optical", "depth_optical"};
  s.calibration = {"synthetic_robot.urdf", "synthetic_calibration"};
  s.time.sensor_future_tolerance_sec = 0.01;
  s.vision = {0.01, 0.5, 0.1, 4, 0.1, 0.1, 5.0, "required"};
  s.tracking = {2, 0.5, 1.0, 0.2, 0.5, 0.05, 8};
  s.tracking.velocity_min_samples=3; s.tracking.velocity_max_samples=8;
  s.tracking.velocity_window_sec=0.5; s.tracking.velocity_min_span_sec=0.25;
  s.tracking.max_velocity_fit_residual_m=0.02; s.tracking.max_velocity_uncertainty_mps=0.08;
  s.task = {100.0, 10.0, 10.0, 10.0, 5.0, 1, 0.75, 2};
  s.inspection.acquisition_timeout_sec = 20; s.inspection.evaluation_timeout_sec = 5;
  s.inspection.view_target_position_tolerance_m = 0.01;
  s.inspection.view_target_orientation_tolerance_rad = 0.05;
  s.inspection.inference = {"rf_detr", "synthetic_model.bin", "model_v1", 2, 2, 0.8, kUnset, 1, 0};
  s.motion.joint_state_max_age_sec = 0.5; s.motion.tcp_state_max_age_sec = 0.5;
  s.motion.planning_timeout_sec = 2; s.motion.execution_timeout_sec = 5;
  s.motion.stop_confirmation_timeout_sec = 1; s.motion.target_update_timeout_sec = 0.2;
  s.motion.max_velocity_scale = 0.5; s.motion.max_acceleration_scale = 0.5;
  s.motion.position_tolerance_m = 0.01; s.motion.orientation_tolerance_rad = 0.05;
  s.motion.max_follow_linear_speed_mps = 1; s.motion.max_follow_angular_speed_radps = 1;
  s.motion.bounds_min_m = {-2,-2,-2}; s.motion.bounds_max_m = {2,2,2};
  s.motion.observation_pose = {};
  s.grasp.approach_timeout_sec = 3; s.grasp.close_timeout_sec = 1;
  s.grasp.verification_timeout_sec = 2; s.grasp.release_timeout_sec = 1;
  s.grasp.max_relative_linear_speed_mps = 0.1; s.grasp.approach_tcp_in_object = {};
  s.grasp.open_width_m = 0.08; s.grasp.close_width_m = 0.02; s.grasp.max_effort_n = 10;
  s.grasp.verification_method = "joint_and_visual"; s.grasp.min_confirmations = 2;
  s.grasp.lift_distance_m = 0.1; s.grasp.bin_approach_pose = {}; s.grasp.bin_release_pose = {};
  s.scene.static_geometry_path = "synthetic_scene.json"; s.scene.dynamic_object_max_age_sec = 0.5;
  return s;
}
inline ptc_core::SettingsSnapshot settings(const ptc_core::SystemSettings & data = settings_data())
{
  std::vector<ptc_core::ConfigError> errors;
  auto snapshot = ptc_core::SettingsSnapshot::create(data, "synthetic_revision", errors);
  CHECK(snapshot); return *snapshot;
}
inline ptc_core::TimeContext time(double source = 1, double monotonic = -1)
{return {*ptc_core::seconds_to_duration(source), *ptc_core::seconds_to_duration(monotonic == -1 ? source : monotonic), 1, false};}
inline ptc_core::EventKey key(std::uint64_t operation = 1, std::uint64_t sequence = 1)
{return {1, 1, 1, 1, operation, sequence, "synthetic_revision"};}
inline ptc_core::DeviceStatus healthy()
{return {ptc_core::DeviceConnectionState::kConnected, ptc_core::DeviceHealthState::kHealthy, true, false, 0, {}};}
inline ptc_core::ArmSnapshot arm()
{
  ptc_core::ArmSnapshot a;
  a.epoch = 1; a.joint_timestamp = time().source_now; a.tcp_timestamp = a.joint_timestamp;
  a.state.timestamp = a.tcp_timestamp; a.state.status = healthy(); a.state.base_frame = "base"; a.state.tcp_frame = "tcp";
  a.state.joints.push_back({"joint_1", 0, 0, 0}); a.tf_valid = a.controller_ready = a.stopped = true;
  return a;
}
inline ptc_core::GripperState gripper()
{
  ptc_core::GripperState g;
  g.epoch = 1; g.timestamp = time().source_now; g.status = healthy(); g.width_m = 0.08;
  g.stopped = g.empty_confirmed = true; g.evidence_sequence = 1; return g;
}
inline ptc_core::RgbdFrame frame(std::uint64_t id = 1)
{
  return {id, 1, time().source_now, time().source_now, "rgb_optical", "depth_optical",
    "synthetic_calibration", true, true, true, {}};
}
template<typename Function>
int run(Function function)
{
  try {function(); std::cout << "PASS\n"; return 0;}
  catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
}  // namespace ptc_test
