#pragma once
#include "ptc_core/state/motion_state.hpp"
#include "ptc_interfaces/action/sync_scene.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <cstdint>

namespace ptc_motion
{
struct GraspContactTarget
{
  ptc_core::StampedPose3d center;
  double radius_m{0};
};
struct MotionCheckRequest
{
  std::vector<ptc_core::StampedPose3d> targets;
  std::vector<GraspContactTarget> contact_patches;
  ptc_core::EventKey key;
  std::uint64_t track_id{0},surface_revision{0};
  double timeout_sec{0},closure_width_m{0};
  bool grasp{false};
};
struct GraspCandidateCheck
{
  std::uint32_t candidate_id{0};
  MotionCheckRequest motion;
};
struct GraspCandidatesRequest
{
  ptc_core::EventKey key;
  std::uint64_t scene_revision{0},surface_revision{0};
  double timeout_sec{0};
  std::vector<GraspCandidateCheck> candidates;
  std::vector<ptc_core::Vector3d> neighbor_points_world;
  double neighbor_capture_ms{0};
};
struct GraspCandidateFeasibility
{
  std::uint32_t candidate_id{0};
  bool evaluated{false},feasible{false};
  std::string failure_reason;
  double minimum_joint_margin_rad{0},pregrasp_duration_sec{0};
};
struct GraspCandidatesResult
{
  ptc_core::Nanoseconds measured_joint_stamp{0};
  std::vector<GraspCandidateFeasibility> candidates;
};
// Composition adapter, not a new state machine. Gateway owns admission and mode.
struct MotionBackend
{
  std::function<ptc_core::ArmSnapshot(const ptc_core::TimeContext &)> arm_snapshot;
  std::function<ptc_core::GripperState(const ptc_core::TimeContext &)> gripper_snapshot;
  std::function<bool(const ptc_core::MoveToPoseRequest &)> plan;
  std::function<bool(const MotionCheckRequest &)> check_motion;
  std::function<GraspCandidatesResult(const GraspCandidatesRequest &)> evaluate_grasp_candidates;
  std::function<std::optional<ptc_core::StampedPose3d>()> planned_target;
  std::function<bool()> execute;
  std::function<bool(const ptc_core::Command &)> follow;
  std::function<void()> stop_arm,stop_gripper;
  std::function<bool(double,double,bool,double)> gripper;
  std::function<bool(const ptc_interfaces::action::SyncScene::Goal &)> sync_scene;
};
}  // namespace ptc_motion
