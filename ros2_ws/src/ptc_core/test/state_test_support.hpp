#pragma once
#include "test_support.hpp"
#include "ptc_core/state/task_state.hpp"
#include "ptc_core/state/motion_state.hpp"

namespace ptc_test
{
inline ptc_core::ArmSnapshot arm_at(double stamp)
{auto a = arm(); a.state.timestamp = a.joint_timestamp = a.tcp_timestamp = time(stamp).source_now; return a;}
inline ptc_core::GripperState gripper_at(double stamp, std::uint64_t sequence = 1)
{auto g = gripper(); g.timestamp = time(stamp).source_now; g.evidence_sequence = sequence; return g;}
inline ptc_core::TrackedObjectState track(double stamp = 1, std::uint64_t id = 1)
{
  ptc_core::TrackedObjectState s; s.phase = ptc_core::TrackPhase::kTracking; s.context.key = key(11); s.context.key.track_id = id;
  s.context.key.cycle_id = s.context.key.operation_id = s.context.key.session_id = 0; // Continuous vision is not owned by a task cycle.
  s.observation = {{},time(stamp).source_now,"world",0.01,true};s.observation.envelope_valid=true;
  s.observation.envelope_radius_xy_m=0.05;s.observation.envelope_half_height_m=0.04;
  s.confirmations = 2; s.velocity_valid = true; s.velocity_mps = {0.1,0,0};
  s.velocity_fit_residual_m=0.001;s.velocity_uncertainty_mps=0.01;return s;
}
inline ptc_core::SceneState scene(double stamp = 1, bool attached = false)
{
  ptc_core::SceneState s; s.context.key = key(10); s.revision = 1; s.coherent = true;
  s.static_geometry.push_back({"floor","world",ptc_core::GeometryKind::kBox,{}, {3,3,0.1},{}});
  s.dynamic_objects.push_back({key(11),track(stamp).observation,{}});
  if (attached) {s.attachment = ptc_core::AttachmentPhase::kAttached; s.attached_track_id = 1;} return s;
}
inline ptc_core::TaskEvent task_event(ptc_core::TaskEventKind kind, const ptc_core::TaskState & s,
  std::uint64_t seq, double stamp = 1)
{
  ptc_core::TaskEvent e; e.kind = kind; e.key = s.context.key;
  if (!e.key.epoch) {e.key = key(); e.key.track_id = 0;}
  e.key.sequence = seq; e.key.operation_id = s.expected_operation_id ? s.expected_operation_id : 1;
  e.arm = arm_at(stamp); e.gripper = gripper_at(stamp); e.track = track(stamp);
  e.scene = scene(stamp,s.grasp.possession == ptc_core::Possession::kHolding);
  e.readiness = {true,true,"synthetic_revision","synthetic_revision","synthetic_revision"}; return e;
}
inline ptc_core::MotionEvent move_event(std::uint64_t operation = 2)
{
  ptc_core::MotionEvent e; e.kind = ptc_core::MotionEventKind::kMoveRequested; e.key = key(operation); e.arm = arm();
  e.request = {{time().source_now,"base",{}},0.5,0.5,*ptc_core::seconds_to_duration(10)};
  e.base_in_world = {time().source_now,"world",{}}; e.scene = scene(); return e;
}
inline void classified(const ptc_core::DecisionStatus & s)
{CHECK(!s.reason.empty()); CHECK(s.disposition == ptc_core::Disposition::kApplied || s.disposition == ptc_core::Disposition::kRejected || s.disposition == ptc_core::Disposition::kIgnored);}
}  // namespace ptc_test
