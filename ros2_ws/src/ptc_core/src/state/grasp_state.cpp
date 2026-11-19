#include "ptc_core/state/grasp_state.hpp"
#include "ptc_core/vision/grasp_pose_model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ptc_core
{
std::optional<Pose3d> committed_grasp_object_pose(const GraspEvent & e,
  const SettingsSnapshot & snapshot,const TimeContext & t)
{
  const auto & c=snapshot.get();
  if(c.runtime.operation_mode!="assumed_ng_demo" || !c.grasp.moving_intercept || !e.motion_prediction)return std::nullopt;
  const auto & p=*e.motion_prediction;
  const double age=std::chrono::duration<double>(t.source_now-p.reference_start).count();
  if(!valid_event(p.key,t,snapshot.revision()) || !same_track_identity(p.key,e.key) ||
    p.key.cycle_id!=e.key.cycle_id || p.reference_start.count()<0 || age<0 ||
    age>c.task.preparing_pick_timeout_sec+c.grasp.approach_timeout_sec ||
    p.object_world.reference_frame!=c.frames.world || !is_valid_pose(p.object_world.pose) ||
    !source_is_fresh(p.object_world.timestamp,t,c.motion.tcp_state_max_age_sec,c.time.sensor_future_tolerance_sec))return std::nullopt;
  return p.object_world.pose;
}
Pose3d bin_target_pose(const Pose3d & configured,const GraspState & grasp,const SettingsSnapshot & snapshot)
{
  const auto & c=snapshot.get();
  auto target=configured;
  if(c.grasp.bin_height && std::isfinite(grasp.bin_height_offset_m) && grasp.bin_height_offset_m>0){
    target.position.z_m+=grasp.bin_height_offset_m;
  }
  if(c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept && grasp.held_tcp_orientation_base){
    target.orientation=*grasp.held_tcp_orientation_base;
  }
  return target;
}
bool aperture_below_contact_span(double width_m,double expected_span_m)
{
  return std::isfinite(width_m) && width_m>=0. && std::isfinite(expected_span_m) &&
    expected_span_m>0. && width_m<expected_span_m/3.;
}
double close_wall_timeout_sec(const SystemSettings & settings)
{
  // The existing close and verification window bounds a slow simulation's
  // close action in elapsed real time. The parent task retains its deadline.
  return settings.grasp.close_timeout_sec +
    (settings.runtime.use_sim_time ? settings.grasp.verification_timeout_sec : 0.);
}
bool grasp_stage_expired(const GraspState & grasp,const TimeContext & time)
{
  return expired(grasp.context.deadline,time) || (grasp.phase==GraspPhase::kClosing &&
    grasp.close_source_deadline.count()>0 && time.source_now>=grasp.close_source_deadline);
}
GraspDecision advance_grasp(const GraspState & original, const GraspEvent & e,
  const SettingsSnapshot & snapshot, const TimeContext & t)
{
  GraspDecision d{original, {}, {}, {}}; auto & s = d.state; const auto & c = snapshot.get();
  // Bounded prediction bridges approach/closure. In the moving demo, a fresh
  // measured gripper contact is the possession evidence because the object may
  // occlude its own camera during lift; formal verification still uses vision.
  const auto committed_object=committed_grasp_object_pose(e,snapshot,t);
  const bool predicted_demo_approach=c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept;
  const auto finish = [&]() {
      if (s.context.inhibited) {s.faulted = true; if (s.possession != Possession::kHolding) {s.phase = GraspPhase::kUnknown; s.possession = Possession::kUnknown;}}
      d.record = record_decision(s.context, "grasp", static_cast<int>(original.phase), static_cast<int>(s.phase),
        e.key, t, d.status, c.diagnostics.transition_history_capacity); return d;
    };
  const auto reject = [&](const char * reason) {d.status = {Disposition::kRejected, reason}; return finish();};
  const auto stop = [&](const char * reason) {
      if (!s.awaiting_stop) {
        s.awaiting_stop = true; s.faulted = true;
        s.context.deadline = deadline_after(t.monotonic_now, c.motion.stop_confirmation_timeout_sec).value_or(t.monotonic_now);
        emit_command(s.context, d.commands, CommandKind::kStopArm, "grasp");
        emit_command(s.context, d.commands, CommandKind::kStopGripper, "grasp");
      }
      // A failed close may have left the fingers fully open. Only a newer,
      // fresh measured Empty distinguishes that miss from unknown possession.
      const bool empty_close_failure=e.kind==GraspEventKind::kCloseFailed && s.phase==GraspPhase::kClosing &&
        gripper_snapshot_ready(e.gripper,snapshot,t) && e.gripper.stopped && e.gripper.empty_confirmed &&
        e.gripper.evidence_sequence>s.evidence_sequence && e.gripper.timestamp>s.evidence_timestamp;
      const bool closed_empty = predicted_demo_approach && e.kind==GraspEventKind::kEvidenceUpdated &&
        s.phase==GraspPhase::kVerifying && gripper_snapshot_ready(e.gripper,snapshot,t) &&
        e.gripper.empty_confirmed && gripper_measured_closed_empty(e.gripper,c) &&
        e.gripper.evidence_sequence>s.evidence_sequence && e.gripper.timestamp>s.evidence_timestamp;
      if (empty_close_failure || closed_empty) {s.possession=Possession::kEmpty;s.evidence_sequence=e.gripper.evidence_sequence;s.evidence_timestamp=e.gripper.timestamp;}
      else if (s.phase == GraspPhase::kClosing || s.phase == GraspPhase::kVerifying || s.phase == GraspPhase::kReleasing ||
        (predicted_demo_approach && s.phase==GraspPhase::kHolding && !s.lifted && !s.scene_attached)) {s.possession = Possession::kUnknown;}
      s.phase = s.possession == Possession::kHolding ? GraspPhase::kHolding :
        (s.possession == Possession::kEmpty ? GraspPhase::kFailed : GraspPhase::kUnknown);
      d.status = {Disposition::kApplied, reason};
    };
  const bool operating = s.phase == GraspPhase::kApproaching || s.phase == GraspPhase::kClosing ||
    s.phase == GraspPhase::kVerifying || s.phase == GraspPhase::kReleasing ||
    (s.phase == GraspPhase::kHolding && !s.lifted && !s.faulted);
  if (static_cast<int>(s.phase) < 0 || static_cast<int>(e.kind) < 0 || s.phase >= GraspPhase::kCount || e.kind >= GraspEventKind::kCount ||
    s.possession < Possession::kUnknown || s.possession > Possession::kHolding) {return reject("invalid phase/event/possession");}
  if (!valid_time(t) || t.monotonic_now < s.context.last_monotonic) {return reject("invalid/backward time");}
  if (e.kind != GraspEventKind::kResetRequested && s.awaiting_stop && expired(s.context.deadline, t)) {
    s.context.inhibited = true; s.faulted = true; d.status = {Disposition::kApplied, "arm/gripper stop deadline unconfirmed"}; return finish();
  }
  if (e.kind != GraspEventKind::kResetRequested && (operating || s.possession == Possession::kHolding) && !s.awaiting_stop &&
    (t.paused || e.kind == GraspEventKind::kClockReset || s.context.key.epoch != t.epoch)) {stop("clock invalidated grasp"); return finish();}
  if (e.kind == GraspEventKind::kDeviceUnavailable && s.context.key.operation_id) {stop("device unavailable; possession retained or unknown"); return finish();}
  if (e.kind == GraspEventKind::kCancelRequested && (operating || s.possession == Possession::kHolding) && !s.awaiting_stop &&
    valid_event(e.key,t,snapshot.revision()) && same_operation(e.key,s.context.key) && e.key.sequence > s.context.key.sequence) {
    stop("cancel handled before ordinary completion/deadline; no automatic open");
    s.context.key.sequence = e.key.sequence; s.context.last_monotonic = t.monotonic_now; return finish();
  }
  if (e.kind != GraspEventKind::kResetRequested && operating && grasp_stage_expired(s,t)) {stop("grasp stage deadline reached"); return finish();}
  if (!valid_event(e.key, t, snapshot.revision()) || e.key.cycle_id == 0) {return reject("invalid grasp identity/revision");}
  if (e.kind == GraspEventKind::kResetRequested &&
    (s.phase == GraspPhase::kUnknown || s.phase == GraspPhase::kFailed || s.phase == GraspPhase::kEmpty || s.phase == GraspPhase::kHolding)) {
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped || !gripper_snapshot_ready(e.gripper, snapshot, t) ||
      !e.gripper.stopped || !e.gripper.empty_confirmed || (s.possession == Possession::kHolding &&
      (e.gripper.evidence_sequence <= s.evidence_sequence || e.gripper.timestamp <= s.evidence_timestamp))) {return reject("reset requires both stops and newer confirmed Empty, not an assumed release");}
    s.phase = GraspPhase::kEmpty; s.possession = Possession::kEmpty; s.awaiting_stop = s.faulted = s.context.inhibited = false;
    s.context.key = e.key; s.context.deadline = Nanoseconds{0}; s.context.last_monotonic = t.monotonic_now;
    s.scene_attached = s.lifted = false; s.confirmations = 0; s.held_tcp_orientation_base.reset();
    s.probe_origin_world_z_m.reset();s.bin_height_offset_m=0;
    d.status = {Disposition::kApplied, "manual reset with measured Empty; no command replay"}; return finish();
  }
  const bool new_approach = e.kind == GraspEventKind::kApproachRequested && s.phase == GraspPhase::kEmpty;
  if (!new_approach && s.context.key.operation_id &&
    (!same_operation(e.key, s.context.key) || e.key.sequence <= s.context.key.sequence)) {return reject("mismatched/duplicate/out-of-order grasp event");}
  s.context.last_monotonic = t.monotonic_now;
  if (s.awaiting_stop) {
    if (e.kind != GraspEventKind::kStopConfirmed) {return reject("awaiting both stop confirmations");}
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped || !gripper_snapshot_ready(e.gripper, snapshot, t) || !e.gripper.stopped) {
      return reject("arm and gripper must both confirm stopped");
    }
    s.awaiting_stop = false; d.status = {Disposition::kApplied, "both stopped; manual recovery required"};
  } else if (new_approach) {
    if (s.faulted || s.possession != Possession::kEmpty || !e.grasp_window_valid || e.key.session_id == 0 ||
      !same_track_identity(e.key, e.track.context.key) || (!committed_object && !track_is_usable(e.track, snapshot, t, predicted_demo_approach)) ||
      !inside_workspace(e.track.observation.pose_world, c.motion) || !arm_snapshot_ready(e.arm, snapshot, t) ||
      !gripper_snapshot_ready(e.gripper, snapshot, t) || !e.gripper.empty_confirmed || !e.gripper.stopped) {return reject("invalid grasp admission/window/device/track");}
    if(c.schema_version==2 && (!e.grasp_tcp_in_object || !is_valid_pose(*e.grasp_tcp_in_object))) {return reject("geometry-derived grasp pose required");}
    s.grasp_tcp_in_object=c.schema_version==2 ? *e.grasp_tcp_in_object:c.grasp.approach_tcp_in_object;
    const auto current_object=committed_object?committed_object:c.schema_version==2?predict_object_pose(e.track,snapshot,t,0):std::optional<Pose3d>(e.track.observation.pose_world);
    const auto target = current_object?compose_pose(*current_object,s.grasp_tcp_in_object):std::nullopt;
    if (!target || !inside_workspace(*target, c.motion)) {return reject("approach TCP outside workspace");}
    if (!begin_operation(s.context, e.key, t, snapshot.revision(), c.grasp.approach_timeout_sec)) {return reject("cannot start grasp");}
    s.phase = GraspPhase::kApproaching; s.scene_attached = s.lifted = false; s.confirmations = 0; s.held_tcp_orientation_base.reset();
    s.probe_origin_world_z_m.reset();s.bin_height_offset_m=0;
    s.evidence_sequence = e.gripper.evidence_sequence; s.evidence_timestamp = e.gripper.timestamp;
    emit_command(s.context, d.commands, CommandKind::kStartFollow, "grasp");
    if (!d.commands.empty()) {d.commands.back().target = *target; d.commands.back().reference_frame = c.frames.world;
      d.commands.back().source_timestamp = e.track.observation.timestamp; d.commands.back().linear_velocity = e.track.velocity_mps;}
    d.status = {Disposition::kApplied, "approach admitted"};
  } else if (e.kind == GraspEventKind::kCancelRequested && (operating || s.possession == Possession::kHolding)) {stop("cancel requested; no automatic open");
  } else if (e.kind == GraspEventKind::kApproachCompleted && s.phase == GraspPhase::kApproaching) {
    const auto current_object=committed_object?committed_object:c.schema_version==2?track_pose_at_observation(e.track,snapshot,t,e.tcp_world.timestamp):std::optional<Pose3d>(e.track.observation.pose_world);
    const auto desired_tcp = current_object?compose_pose(*current_object,c.schema_version==2?s.grasp_tcp_in_object:c.grasp.approach_tcp_in_object):std::nullopt;
    // The existing bounded commissioning reference exercises physical motion;
    // speed is measured/reported, not an additional contact veto in this trial.
    const double maximum_speed=committed_object?std::numeric_limits<double>::max():c.grasp.max_relative_linear_speed_mps;
    const char * close_blocker=nullptr;
    if (!e.grasp_window_valid) {close_blocker="grasp window invalid";}
    else if (!same_track_identity(e.key,e.track.context.key)) {close_blocker="target identity changed";}
    else if (!committed_object && !track_is_usable(e.track,snapshot,t,predicted_demo_approach)) {close_blocker="target track unusable";}
    else if (!std::isfinite(e.relative_speed_mps) || e.relative_speed_mps<0 || e.relative_speed_mps>maximum_speed) {close_blocker="relative speed invalid or above limit";}
    else if (predicted_demo_approach && !committed_object && e.arm.state.motion_state!=ArmMotionState::kFollowing) {close_blocker="arm is not following";}
    else if (!arm_snapshot_ready(e.arm,snapshot,t)) {close_blocker="arm feedback unavailable";}
    else if (!gripper_snapshot_ready(e.gripper,snapshot,t)) {close_blocker="gripper feedback unavailable";}
    else if (!e.gripper.empty_confirmed) {close_blocker="gripper Empty unconfirmed";}
    else if (!desired_tcp || !is_valid_pose(e.tcp_world.pose) || e.tcp_world.reference_frame!=c.frames.world) {close_blocker="TCP pose or frame invalid";}
    else if (!source_is_fresh(e.tcp_world.timestamp,t,c.motion.tcp_state_max_age_sec,c.time.sensor_future_tolerance_sec)) {close_blocker="TCP timestamp outside freshness window";}
    else if (!committed_object && (c.schema_version==2 ?
      !grasp_center_aligned(e.tcp_world.pose,*desired_tcp,c.grasp.finger_center_tcp,e.relative_speed_mps,maximum_speed,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad) :
      !pose_near(e.tcp_world.pose,*desired_tcp,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad))) {close_blocker="TCP alignment outside tolerance";}
    if (close_blocker) {stop(close_blocker);return finish();}
    s.phase = GraspPhase::kClosing;
    s.context.deadline = deadline_after(t.monotonic_now,close_wall_timeout_sec(c)).value_or(t.monotonic_now);
    s.close_source_deadline = c.runtime.use_sim_time ?
      deadline_after(t.source_now,c.grasp.close_timeout_sec).value_or(t.source_now) : Nanoseconds{0};
    emit_command(s.context, d.commands, CommandKind::kCloseGripper, "grasp");
    if (!d.commands.empty()) {d.commands.back().width_m = c.grasp.close_width_m; d.commands.back().effort_n = c.grasp.max_effort_n;}
    d.status = {Disposition::kApplied, committed_object?"timed commissioning descent completed; pose/speed diagnostic, close requested":"velocity matched; close requested"};
  } else if (e.kind == GraspEventKind::kCloseAccepted && s.phase == GraspPhase::kClosing) {
    s.phase = GraspPhase::kVerifying; s.possession = Possession::kUnknown;
    s.context.deadline = deadline_after(t.monotonic_now, c.grasp.verification_timeout_sec).value_or(t.monotonic_now);
    d.status = {Disposition::kApplied, "close acknowledgment is not Holding"};
  } else if (e.kind == GraspEventKind::kEvidenceUpdated && s.phase == GraspPhase::kVerifying) {
    const auto & g = e.gripper;
    if (!gripper_snapshot_ready(g, snapshot, t) || !g.stopped || g.evidence_sequence <= s.evidence_sequence || g.timestamp <= s.evidence_timestamp) {return reject("stale/duplicate/incomplete grasp evidence");}
    if (predicted_demo_approach && g.empty_confirmed && gripper_measured_closed_empty(g,c)) {
      stop("measured fully closed without contact; empty grasp");
      return finish();
    }
    const bool demo_contact_evidence=predicted_demo_approach&&committed_object.has_value();
    if (demo_contact_evidence && g.contact_confirmed &&
      aperture_below_contact_span(g.width_m,e.expected_contact_span_m)) {
      stop("measured aperture too small for selected contact span; possession unknown");
      return finish();
    }
    s.evidence_sequence = g.evidence_sequence; s.evidence_timestamp = g.timestamp;
    const bool device_evidence=demo_contact_evidence ? g.contact_confirmed :
      (c.grasp.verification_method=="contact_and_visual" ? g.contact_confirmed :
      (g.width_m>c.grasp.close_width_m && g.width_m<c.grasp.open_width_m));
    const bool visual_evidence=demo_contact_evidence || g.visual_follow_confirmed;
    if (!visual_evidence || !device_evidence || g.empty_confirmed) {s.confirmations = 0; d.status = {Disposition::kApplied, demo_contact_evidence?"waiting for measured gripper contact":"insufficient independent evidence"};}
    else {
      ++s.confirmations;
      if (s.confirmations >= c.grasp.min_confirmations) {s.phase = GraspPhase::kHolding; s.possession = Possession::kHolding;
        // Retain the measured probe origin through the controller stop. The
        // fresh stopped TCP pose will account for settling before attachment.
        if(demo_contact_evidence && e.motion_prediction &&
          std::isfinite(e.motion_prediction->measured_lift_distance_m) && e.motion_prediction->measured_lift_distance_m>0. &&
          e.tcp_world.reference_frame==c.frames.world && is_valid_pose(e.tcp_world.pose) &&
          source_is_fresh(e.tcp_world.timestamp,t,c.motion.tcp_state_max_age_sec,c.time.sensor_future_tolerance_sec)){
          s.probe_origin_world_z_m=e.tcp_world.pose.position.z_m-e.motion_prediction->measured_lift_distance_m;
        }
        s.context.deadline = deadline_after(t.monotonic_now,c.motion.stop_confirmation_timeout_sec+c.motion.planning_timeout_sec).value_or(t.monotonic_now);
        emit_command(s.context, d.commands, CommandKind::kAttachObject, "grasp");}
      d.status = {Disposition::kApplied, "grasp evidence accepted"};
    }
  } else if (e.kind == GraspEventKind::kEvidenceUpdated && s.phase == GraspPhase::kHolding) {
    if (!gripper_snapshot_ready(e.gripper,snapshot,t) || e.gripper.evidence_sequence <= s.evidence_sequence || e.gripper.timestamp <= s.evidence_timestamp) {return reject("stale/duplicate Holding evidence");}
    s.evidence_sequence = e.gripper.evidence_sequence; s.evidence_timestamp = e.gripper.timestamp;
    if (e.gripper.empty_confirmed) {s.possession = Possession::kUnknown; s.phase = GraspPhase::kUnknown; stop("Holding contradicted by measured Empty; object location unknown");}
    else if(c.grasp.finger_effort_thresholds_n && e.gripper.stopped && !e.gripper.contact_confirmed){
      s.possession=Possession::kUnknown;s.phase=GraspPhase::kUnknown;
      stop("Holding no longer supported by current finger effort; object location unknown");
    }else {d.status = {Disposition::kApplied,"Holding retained; lost visual evidence is not an automatic release"};}
  } else if (e.kind == GraspEventKind::kSceneAttached && s.phase == GraspPhase::kHolding && !s.scene_attached) {
    if (!e.scene_confirmed || !arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped || e.tcp_world.reference_frame != c.frames.world ||
      !is_valid_pose(e.tcp_world.pose) || !source_is_fresh(e.tcp_world.timestamp,t,c.motion.tcp_state_max_age_sec,c.time.sensor_future_tolerance_sec)) {
      stop("scene attachment/TF-converted TCP not confirmed"); return finish();}
    const double already_lifted=s.probe_origin_world_z_m?
      std::max(0.,e.tcp_world.pose.position.z_m-*s.probe_origin_world_z_m):
      committed_object && e.motion_prediction?e.motion_prediction->measured_lift_distance_m:0.;
    if(!std::isfinite(already_lifted)||already_lifted<0){stop("invalid measured lift distance");return finish();}
    s.lift_target_world = e.tcp_world.pose; s.lift_target_world.position.z_m += std::max(0.,c.grasp.lift_distance_m-already_lifted);
    if (!inside_workspace(s.lift_target_world,c.motion)) {stop("lift target outside workspace"); return finish();}
    s.scene_attached = true; s.context.deadline = deadline_after(t.monotonic_now,c.motion.execution_timeout_sec).value_or(t.monotonic_now);
    emit_command(s.context, d.commands, CommandKind::kLift, "grasp");
    if (!d.commands.empty()) {d.commands.back().target = s.lift_target_world; d.commands.back().reference_frame = c.frames.world;}
    d.status = {Disposition::kApplied, "attached scene confirmed; lift requested"};
  } else if (e.kind == GraspEventKind::kLiftCompleted && s.phase == GraspPhase::kHolding && s.scene_attached) {
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped || e.tcp_world.reference_frame != c.frames.world ||
      !source_is_fresh(e.tcp_world.timestamp,t,c.motion.tcp_state_max_age_sec,c.time.sensor_future_tolerance_sec) ||
      !pose_near(e.tcp_world.pose,s.lift_target_world,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad)) {
      return reject("lift completion requires measured world TCP arrival/stopped");}
    // Bin transfer keeps this measured robot_base orientation in the moving demo.
    const auto & q=e.arm.state.tcp_pose.orientation;const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    s.held_tcp_orientation_base=Quaterniond{q.x/n,q.y/n,q.z/n,q.w/n};
    s.lifted = true;
    // Transport is timed by the parent task. A completed lift must not leave
    // its old deadline looking like an active grasp stage; release sets its own.
    s.context.deadline = Nanoseconds{0};
    d.status = {Disposition::kApplied, "lift confirmed"};
  } else if (e.kind == GraspEventKind::kReleaseRequested && s.phase == GraspPhase::kHolding) {
    if (s.faulted || !s.lifted || !s.scene_attached || !e.scene_confirmed || !arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped ||
      !pose_near(e.arm.state.tcp_pose, bin_target_pose(c.grasp.bin_release_pose,s,snapshot), c.motion.position_tolerance_m, c.motion.orientation_tolerance_rad)) {return reject("release requires confirmed bin arrival and coherent scene");}
    s.phase = GraspPhase::kReleasing; s.context.deadline = deadline_after(t.monotonic_now, c.grasp.release_timeout_sec).value_or(t.monotonic_now);
    emit_command(s.context, d.commands, CommandKind::kOpenGripper, "grasp");
    if (!d.commands.empty()) {d.commands.back().width_m = c.grasp.open_width_m; d.commands.back().effort_n = c.grasp.max_effort_n;}
    d.status = {Disposition::kApplied, "explicit release at bin"};
  } else if (e.kind == GraspEventKind::kReleaseCompleted && s.phase == GraspPhase::kReleasing) {
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped || !gripper_snapshot_ready(e.gripper, snapshot, t) ||
      !gripper_measured_open_empty(e.gripper,c)) {return reject("release requires measured open and Empty");}
    s.phase = GraspPhase::kEmpty; s.possession = Possession::kEmpty; emit_command(s.context, d.commands, CommandKind::kDetachObject, "grasp");
    d.status = {Disposition::kApplied, "measured Empty; scene detach still requires acknowledgment"};
  } else if ((e.kind == GraspEventKind::kCloseFailed && s.phase == GraspPhase::kClosing) ||
    (e.kind == GraspEventKind::kLiftFailed && s.phase == GraspPhase::kHolding) ||
    (e.kind == GraspEventKind::kReleaseFailed && s.phase == GraspPhase::kReleasing)) {stop("grasp operation failed");
  } else if (operating && e.kind == GraspEventKind::kTick) {d.status = {Disposition::kApplied, "grasp deadline checked"};}
  if (d.status.disposition == Disposition::kApplied) {s.context.key.sequence = e.key.sequence;}
  return finish();
}
}  // namespace ptc_core
