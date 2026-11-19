#include "ptc_core/state/task_state.hpp"
#include <algorithm>
#include <limits>

namespace ptc_core
{
bool system_ready(const TaskEvent & e, const SettingsSnapshot & snapshot, const TimeContext & t)
{
  return system_ready(e,snapshot,t,TaskPhase::kWaitingReady);
}
bool system_ready(const TaskEvent & e, const SettingsSnapshot & snapshot, const TimeContext & t, TaskPhase phase)
{
  const auto & c=snapshot.get();
  const bool perception_required=phase!=TaskPhase::kTransporting && phase!=TaskPhase::kPlacing;
  // The fixed observation move and the unassigned wait do not use a target.
  // START and target selection still check live perception in their own events.
  const bool waiting_for_semantic_target=phase==TaskPhase::kObserving &&
    e.kind==TaskEventKind::kReadinessChanged && e.key.track_id==0 &&
    c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept &&
    c.vision.tracking_camera.role=="external" && c.vision.foreground_source=="model";
  auto prediction_event=e.grasp_event;prediction_event.key=e.key;
  const auto committed_object=committed_grasp_object_pose(prediction_event,snapshot,t);
  const bool predicted_grasp_preparation=(e.kind==TaskEventKind::kGraspPrepared ||
    (e.kind==TaskEventKind::kMotionFinished && e.motion_succeeded && e.grasp_event.grasp_window_valid)) && c.runtime.operation_mode=="assumed_ng_demo" &&
    c.grasp.moving_intercept && e.key.track_id &&
    same_track_identity(e.key,e.track.context.key) && (committed_object || track_is_usable(e.track,snapshot,t,true));
  const bool perception_ready=waiting_for_semantic_target || (e.readiness.vision_ready && e.readiness.inference_ready);
  return !t.paused && (!perception_required || perception_ready || predicted_grasp_preparation) &&
    e.readiness.vision_revision == snapshot.revision() && e.readiness.inference_revision == snapshot.revision() &&
    e.readiness.motion_revision == snapshot.revision() && arm_snapshot_ready(e.arm,snapshot,t) &&
    gripper_snapshot_ready(e.gripper,snapshot,t) && scene_ready(e.scene,snapshot,t,
      snapshot.get().runtime.operation_mode=="assumed_ng_demo" && e.key.track_id &&
      same_track_identity(e.key,e.track.context.key) && (committed_object || track_is_usable(e.track,snapshot,t,true))?e.key.track_id:0);
}
bool unassigned_observation_wait(const TaskState & s, const TaskEvent & e, const SettingsSnapshot & snapshot, const TimeContext & t)
{
  const auto & c=snapshot.get();
  // Nothing moves while waiting at Home. Background scene expiry, inference
  // freshness and motion-owner availability are rechecked at target selection.
  return s.phase==TaskPhase::kObserving && s.observation_arrived && s.context.key.track_id==0 && e.key.track_id==0 &&
    c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept &&
    c.vision.tracking_camera.role=="external" && c.vision.foreground_source=="model" && !t.paused &&
    e.arm.stopped && arm_snapshot_ready(e.arm,snapshot,t) && gripper_snapshot_ready(e.gripper,snapshot,t) &&
    e.readiness.vision_revision==snapshot.revision() && e.readiness.inference_revision==snapshot.revision();
}
TaskDecision advance_task(const TaskState & original, const TaskEvent & e,
  const SettingsSnapshot & snapshot, const TimeContext & t)
{
  TaskDecision d{original, {}, {}, {}}; auto & s = d.state; const auto & c = snapshot.get();
  const auto finish = [&]() {
      if (s.context.inhibited) {s.phase = TaskPhase::kFaulted;}
      if (s.cycle_deadline.count() > 0 && s.phase != TaskPhase::kRecovering && s.phase != TaskPhase::kReporting) {
        for (auto & command : d.commands) {if (command.kind != CommandKind::kStopArm && command.kind != CommandKind::kStopGripper && command.kind != CommandKind::kCancelInference) {
            command.deadline = std::min(command.deadline,s.cycle_deadline);
          }}
      }
      d.record = record_decision(s.context,"task",static_cast<int>(original.phase),static_cast<int>(s.phase),
        e.key,t,d.status,c.diagnostics.transition_history_capacity); return d;
    };
  const auto reject = [&](const char * reason) {d.status = {Disposition::kRejected,reason}; return finish();};
  const auto request = [&](CommandKind kind) {
      emit_command(s.context,d.commands,kind,"task");
      if (!d.commands.empty()) {
        auto & cmd = d.commands.back(); cmd.key.operation_id = cmd.command_id;
        s.expected_operation_id = cmd.command_id;
        if (kind == CommandKind::kMoveObservation) {cmd.target = c.motion.observation_pose; cmd.reference_frame = c.frames.robot_base;}
        if (kind == CommandKind::kMoveBinApproach) {cmd.target = bin_target_pose(c.grasp.bin_approach_pose,s.grasp,snapshot); cmd.reference_frame = c.frames.robot_base;}
        if (kind == CommandKind::kMoveBinRelease) {cmd.target = bin_target_pose(c.grasp.bin_release_pose,s.grasp,snapshot); cmd.reference_frame = c.frames.robot_base;}
      }
    };
  const auto stage = [&](TaskPhase phase, double timeout) {
      s.phase = phase; s.context.deadline = std::min(s.cycle_deadline,
        deadline_after(t.monotonic_now,timeout).value_or(t.monotonic_now));
    };
  const auto report = [&]() {
      s.phase = TaskPhase::kReporting;
      s.context.deadline = deadline_after(t.monotonic_now,c.task.recovery_timeout_sec).value_or(t.monotonic_now);
      if (!s.report_emitted) {request(CommandKind::kReport); s.report_emitted = true;
        emit_command(s.context,d.commands,CommandKind::kRetireTrack,"task");}
    };
  const auto recover = [&](CycleOutcome outcome, bool fault, const char * reason) {
      if (s.phase != TaskPhase::kRecovering && s.phase != TaskPhase::kFaulted) {
        s.phase = TaskPhase::kRecovering; s.outcome = outcome; s.terminal_fault = fault;s.recovery_home_command_id=0;
        s.context.deadline = deadline_after(t.monotonic_now,c.task.recovery_timeout_sec).value_or(t.monotonic_now);
        request(CommandKind::kStopArm); emit_command(s.context,d.commands,CommandKind::kStopGripper,"task");
        emit_command(s.context,d.commands,CommandKind::kCancelInference,"task");
        // Pending results are no longer admissible. Do not overwrite possession after cancellation.
        if (s.grasp.phase == GraspPhase::kClosing || s.grasp.phase == GraspPhase::kVerifying || s.grasp.phase == GraspPhase::kReleasing) {
          s.grasp.phase = GraspPhase::kUnknown; s.grasp.possession = Possession::kUnknown;
        }
        if (s.grasp.phase == GraspPhase::kApproaching) {s.grasp.phase = GraspPhase::kFailed; s.grasp.faulted = true;}
      }
      if (fault) {s.terminal_fault = true;}
      else if(outcome==CycleOutcome::kCanceled && s.phase==TaskPhase::kRecovering && s.outcome!=CycleOutcome::kCanceled){
        s.outcome=outcome;s.recovery_home_command_id=0;
        request(CommandKind::kStopArm);emit_command(s.context,d.commands,CommandKind::kStopGripper,"task");
      }
      d.status = {Disposition::kApplied,reason};
    };
  const bool active = s.phase != TaskPhase::kWaitingReady && s.phase != TaskPhase::kFaulted && s.phase != TaskPhase::kReporting;
  if (static_cast<int>(e.kind) < 0 || static_cast<int>(s.phase) < 0 || e.kind >= TaskEventKind::kCount || s.phase >= TaskPhase::kCount) {return reject("invalid task phase/event");}
  if (!valid_time(t) || t.monotonic_now < s.context.last_monotonic) {return reject("invalid/backward time");}
  // Automatic demo recovery must also honor operator Cancel while a
  // transient Faulted task owns an opening/reset operation.
  if(s.phase==TaskPhase::kFaulted && e.kind==TaskEventKind::kCancelRequested &&
    valid_event(e.key,t,snapshot.revision()) && same_cycle(e.key,s.context.key) && e.key.sequence>s.context.key.sequence){
    s.outcome=CycleOutcome::kCanceled;s.recovery_home_command_id=0;
    emit_command(s.context,d.commands,CommandKind::kStopArm,"task");
    emit_command(s.context,d.commands,CommandKind::kStopGripper,"task");
    s.context.key.sequence=e.key.sequence;s.context.last_monotonic=t.monotonic_now;
    d.status={Disposition::kApplied,"operator cancel supersedes fault recovery"};return finish();
  }
  if (s.phase == TaskPhase::kReporting && expired(s.context.deadline,t)) {
    s.phase = TaskPhase::kWaitingReady; s.retired_track_id=s.context.key.track_id; s.context.inhibited=false;
    d.status = {Disposition::kApplied,"report acknowledgment deadline; continue after safe task completion"}; return finish();
  }
  if (s.phase == TaskPhase::kRecovering && expired(s.context.deadline,t)) {
    // A timed-out Home is still an active device operation until canceled.
    // Do not leave it running after Task has stopped processing completions.
    emit_command(s.context,d.commands,CommandKind::kStopArm,"task");
    emit_command(s.context,d.commands,CommandKind::kStopGripper,"task");
    s.context.inhibited = true; s.phase = TaskPhase::kFaulted;s.recovery_home_command_id=0;
    d.status = {Disposition::kApplied,"recovery stop deadline unconfirmed"}; return finish();
  }
  if (active && s.phase != TaskPhase::kRecovering &&
    (e.kind == TaskEventKind::kClockReset || s.context.key.epoch != t.epoch || t.paused)) {
    recover(CycleOutcome::kFailed,true,"clock invalidated cycle; stop before manual reset"); return finish();
  }
  if (active && valid_event(e.key,t,snapshot.revision()) && same_cycle(e.key,s.context.key) && e.key.sequence > s.context.key.sequence) {
    if (e.kind == TaskEventKind::kFaultRaised || e.kind == TaskEventKind::kCancelRequested ||
      e.kind == TaskEventKind::kTargetLost || (e.kind == TaskEventKind::kReadinessChanged && !system_ready(e,snapshot,t,s.phase) &&
      !unassigned_observation_wait(s,e,snapshot,t))) {
      const bool target_lost = e.kind == TaskEventKind::kTargetLost;
      if (!target_lost || ((s.phase == TaskPhase::kPreparingPick || s.phase == TaskPhase::kPicking) && s.grasp.possession != Possession::kHolding)) {
        const auto outcome = e.kind == TaskEventKind::kCancelRequested ? CycleOutcome::kCanceled : target_lost ? CycleOutcome::kInconclusive : CycleOutcome::kFailed;
        recover(outcome,e.kind == TaskEventKind::kFaultRaised,
          target_lost?"selected visual track lost":e.kind==TaskEventKind::kReadinessChanged?"readiness lost":
          e.kind==TaskEventKind::kCancelRequested?"cancel requested":"device/task fault raised");
        s.context.key.sequence = e.key.sequence; s.context.last_monotonic = t.monotonic_now; return finish();
      }
    }
  }
  if (active && s.phase != TaskPhase::kRecovering && (expired(s.cycle_deadline,t) || expired(s.context.deadline,t))) {
    recover(CycleOutcome::kFailed,false,"cycle/stage deadline reached"); return finish();
  }
  if (!valid_event(e.key,t,snapshot.revision()) || e.key.cycle_id == 0) {return reject("invalid task identity/revision");}
  const bool invalidated_recovery = s.phase == TaskPhase::kRecovering && s.context.key.epoch != t.epoch;
  if (e.kind == TaskEventKind::kResetRequested &&
    (s.phase == TaskPhase::kFaulted || s.phase == TaskPhase::kWaitingReady || invalidated_recovery)) {
    if (!system_ready(e,snapshot,t,s.phase) || !e.arm.stopped || !e.gripper.stopped || !e.gripper.empty_confirmed ||
      e.scene.attachment != AttachmentPhase::kDetached) {return reject("manual reset requires coherent scene, both stopped, known Empty");}
    GraspEvent reset; reset.kind = GraspEventKind::kResetRequested; reset.key = e.key; reset.arm = e.arm; reset.gripper = e.gripper;
    auto child = advance_grasp(s.grasp,reset,snapshot,t);
    if (child.status.disposition != Disposition::kApplied) {return reject("grasp reset rejected");}
    s.grasp = child.state; s.context.key = e.key; s.context.last_monotonic = t.monotonic_now;
    s.context.inhibited = false; s.phase = TaskPhase::kWaitingReady; s.expected_operation_id = 0; s.selected_track = {};
    s.terminal_fault = false; s.recovery_home_command_id = 0;
    s.retries = 0; s.outcome = CycleOutcome::kNone; s.report_emitted = false;
    d.status = {Disposition::kApplied,"manual reset; active work not resumed"}; return finish();
  }
  const bool starting = e.kind == TaskEventKind::kStartRequested && s.phase == TaskPhase::kWaitingReady;
  if (!starting && s.context.key.cycle_id &&
    (!same_cycle(e.key,s.context.key) || e.key.sequence <= s.context.key.sequence)) {return reject("wrong cycle/track or duplicate/out-of-order task event");}
  if (!starting && e.kind != TaskEventKind::kTick && e.kind != TaskEventKind::kCancelRequested &&
    e.kind != TaskEventKind::kFaultRaised && e.kind != TaskEventKind::kReadinessChanged && e.kind != TaskEventKind::kTargetLost &&
    e.kind != TaskEventKind::kGraspUpdated &&
    s.expected_operation_id && e.key.operation_id != s.expected_operation_id) {return reject("wrong child operation");}
  if (e.kind == TaskEventKind::kGraspUpdated && (s.phase == TaskPhase::kPicking || s.phase == TaskPhase::kTransporting || s.phase == TaskPhase::kPlacing) &&
    e.key.operation_id != s.grasp.context.key.operation_id) {return reject("wrong grasp operation");}
  s.context.last_monotonic = t.monotonic_now;
  const auto & grip = e.gripper;
  const bool carrying = (s.phase == TaskPhase::kPicking || s.phase == TaskPhase::kTransporting ||
    s.phase == TaskPhase::kPlacing) && s.grasp.phase == GraspPhase::kHolding;
  if (carrying && (e.kind == TaskEventKind::kTick || e.kind == TaskEventKind::kMotionFinished ||
    e.kind == TaskEventKind::kReadinessChanged) && grip.stopped &&
    grip.evidence_sequence > s.grasp.evidence_sequence && grip.timestamp > s.grasp.evidence_timestamp &&
    gripper_snapshot_ready(grip,snapshot,t) && !grip.contact_confirmed &&
    grip.empty_confirmed) {
    s.grasp.possession = Possession::kEmpty;
    s.grasp.phase = GraspPhase::kFailed;
    s.grasp.evidence_sequence = grip.evidence_sequence;
    s.grasp.evidence_timestamp = grip.timestamp;
    recover(CycleOutcome::kFailed,false,"grasp lost: measured empty gripper before bin release; stop and recover Home");
    s.context.key.sequence = e.key.sequence;
    return finish();
  }
  if (starting) {
    if (e.key.cycle_id <= s.context.key.cycle_id || e.key.track_id != 0 || !system_ready(e,snapshot,t,s.phase) ||
      !e.arm.stopped || !e.gripper.stopped || !e.gripper.empty_confirmed || e.scene.attachment != AttachmentPhase::kDetached ||
      s.grasp.possession == Possession::kHolding) {return reject("new cycle requires ready, stopped, detached, measured Empty");}
    // Startup Unknown becomes Empty only from fresh measured evidence, never from defaults.
    GraspEvent reset; reset.kind = GraspEventKind::kResetRequested; reset.key = e.key; reset.arm = e.arm; reset.gripper = e.gripper;
    const auto child = advance_grasp(s.grasp,reset,snapshot,t);
    if (child.status.disposition != Disposition::kApplied || child.state.possession != Possession::kEmpty) {return reject("grasp cannot become ready");}
    if (!begin_operation(s.context,e.key,t,snapshot.revision(),c.task.cycle_timeout_sec)) {return reject("cannot begin cycle");}
    const bool recovered_at_home = c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept &&
      (s.outcome==CycleOutcome::kFailed || s.outcome==CycleOutcome::kInconclusive) &&
      e.arm.state.motion_state==ArmMotionState::kIdle &&
      pose_near(e.arm.state.tcp_pose,c.motion.observation_pose,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad);
    s.grasp = child.state; s.selected_track = {}; s.cycle_deadline = s.context.deadline;
    s.retries = 0; s.report_emitted = false; s.observation_arrived = recovered_at_home; s.outcome = CycleOutcome::kNone;
    s.phase = TaskPhase::kObserving;
    if(recovered_at_home){
      s.expected_operation_id=0;
      d.status={Disposition::kApplied,"cycle started at measured Home; observation ready"};
    }else{
      request(CommandKind::kMoveObservation);
      d.status={Disposition::kApplied,"cycle started; observation move requested"};
    }
  } else if (active && e.kind == TaskEventKind::kFaultRaised) {recover(CycleOutcome::kFailed,true,"fault raised; stop both devices");
  } else if (active && e.kind == TaskEventKind::kCancelRequested) {recover(CycleOutcome::kCanceled,false,"cycle canceled; stop both devices");
  } else if (active && e.kind == TaskEventKind::kReadinessChanged && !system_ready(e,snapshot,t,s.phase) &&
    !unassigned_observation_wait(s,e,snapshot,t)) {recover(CycleOutcome::kFailed,false,"readiness lost");
  } else if (e.kind == TaskEventKind::kTargetLost && (s.phase == TaskPhase::kPreparingPick || s.phase == TaskPhase::kPicking) &&
    s.grasp.possession != Possession::kHolding) {recover(CycleOutcome::kInconclusive,false,"selected visual track lost");
  } else if (s.phase == TaskPhase::kObserving && e.kind == TaskEventKind::kMotionFinished) {
    if (!e.motion_succeeded || !arm_snapshot_ready(e.arm,snapshot,t) || !e.arm.stopped ||
      !pose_near(e.arm.state.tcp_pose,c.motion.observation_pose,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad)) {
      recover(CycleOutcome::kFailed,false,"observation arrival failed"); return finish();}
    s.observation_arrived = true; d.status = {Disposition::kApplied,"observation pose measured"};
  } else if (s.phase == TaskPhase::kObserving && e.kind == TaskEventKind::kGraspTrialRequested) {
    if(c.runtime.operation_mode!="assumed_ng_demo" || !c.grasp.moving_intercept ||
      !s.observation_arrived || !system_ready(e,snapshot,t,s.phase) || !arm_snapshot_ready(e.arm,snapshot,t) || !e.arm.stopped ||
      !e.gripper.empty_confirmed || !track_is_usable(e.track,snapshot,t) || !e.track.context.key.track_id ||
      e.track.context.key.track_id==s.retired_track_id || !inside_workspace(e.track.observation.pose_world,c.motion)){
      return reject("pick-only commissioning requires ready Empty Home and an observed target in demo mode");
    }
    s.cycle_deadline=deadline_after(t.monotonic_now,c.task.cycle_timeout_sec).value_or(t.monotonic_now);
    s.selected_track=e.track;s.context.key.track_id=e.track.context.key.track_id;
    stage(TaskPhase::kPreparingPick,c.task.preparing_pick_timeout_sec);
    d.status={Disposition::kApplied,"pick-only commissioning selected; no inspection result claimed"};
  } else if (s.phase == TaskPhase::kPreparingPick && (e.kind == TaskEventKind::kMotionFinished || e.kind == TaskEventKind::kGraspPrepared)) {
    auto prediction_event=e.grasp_event;prediction_event.key=e.key;
    const auto committed_object=committed_grasp_object_pose(prediction_event,snapshot,t);
    const bool demo_following=c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept && e.arm.state.motion_state==ArmMotionState::kFollowing;
    if ((e.kind==TaskEventKind::kMotionFinished ? !e.motion_succeeded : !demo_following) || !system_ready(e,snapshot,t,s.phase) || (!e.arm.stopped && !demo_following) || !e.gripper.stopped || !e.gripper.empty_confirmed ||
      !same_track_identity(e.track.context.key,s.context.key) || (!committed_object && !track_is_usable(e.track,snapshot,t,c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept))) {recover(CycleOutcome::kFailed,false,"pick preparation not ready"); return finish();}
    s.selected_track = e.track; stage(TaskPhase::kPicking,c.grasp.approach_timeout_sec+c.grasp.close_timeout_sec+c.grasp.verification_timeout_sec+c.motion.execution_timeout_sec);
    request(CommandKind::kStartGrasp); GraspEvent begin = e.grasp_event; begin.kind = GraspEventKind::kApproachRequested;
    begin.key = s.context.key; begin.key.operation_id = s.expected_operation_id; begin.key.sequence = e.key.sequence;
    begin.arm = e.arm; begin.gripper = e.gripper; begin.track = e.track;
    const auto child = advance_grasp(s.grasp,begin,snapshot,t); s.grasp = child.state;
    d.commands.insert(d.commands.end(),child.commands.begin(),child.commands.end());
    if (child.status.disposition != Disposition::kApplied) {recover(CycleOutcome::kFailed,false,"grasp admission rejected");}
    else {s.grasp.context.deadline = std::min(s.grasp.context.deadline,s.context.deadline); d.status = {Disposition::kApplied,"pick started"};}
  } else if ((s.phase == TaskPhase::kPicking || s.phase == TaskPhase::kTransporting || s.phase == TaskPhase::kPlacing) && e.kind == TaskEventKind::kGraspUpdated) {
    if (!same_operation(e.key,e.grasp_event.key) || e.key.sequence != e.grasp_event.key.sequence) {return reject("grasp event identity mismatch");}
    if (e.grasp_event.kind == GraspEventKind::kSceneAttached &&
      (!scene_ready(e.scene,snapshot,t,(c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept)?s.context.key.track_id:0) || e.scene.attachment != AttachmentPhase::kAttached || e.scene.attached_track_id != s.context.key.track_id)) {
      recover(CycleOutcome::kFailed,true,"scene attachment inconsistent"); return finish();}
    const auto child = advance_grasp(s.grasp,e.grasp_event,snapshot,t); s.grasp = child.state;
    s.grasp.context.deadline = std::min(s.grasp.context.deadline,s.context.deadline);
    d.commands.insert(d.commands.end(),child.commands.begin(),child.commands.end()); d.status = child.status;
    if (s.grasp.faulted || s.grasp.context.inhibited) {
      const bool empty_failure=s.grasp.phase==GraspPhase::kFailed && s.grasp.possession==Possession::kEmpty && !s.grasp.context.inhibited;
      recover(CycleOutcome::kFailed,!empty_failure,empty_failure?"empty grasp attempt failed; stop before recovery":"grasp fault/unknown possession");
    }
    else if (s.phase == TaskPhase::kPicking && s.grasp.phase == GraspPhase::kHolding && s.grasp.lifted) {
      if(c.grasp.bin_height && e.bin_height_offset_m && std::isfinite(*e.bin_height_offset_m) && *e.bin_height_offset_m>=0){
        s.grasp.bin_height_offset_m=*e.bin_height_offset_m;
      }
      stage(TaskPhase::kTransporting,c.task.transport_timeout_sec); request(CommandKind::kMoveBinApproach);
      d.status = {Disposition::kApplied,"Holding and lift confirmed; transport requested"};
    } else if (s.grasp.phase == GraspPhase::kHolding && original.grasp.phase != GraspPhase::kHolding) {
      emit_command(s.context,d.commands,CommandKind::kStopArm,"task"); // Stop following before lift, through motion owner.
    }
  } else if (s.phase == TaskPhase::kTransporting && e.kind == TaskEventKind::kMotionFinished) {
    if (!e.motion_succeeded || !arm_snapshot_ready(e.arm,snapshot,t) || !e.arm.stopped || s.grasp.possession != Possession::kHolding ||
      !scene_ready(e.scene,snapshot,t) || e.scene.attachment != AttachmentPhase::kAttached ||
      !pose_near(e.arm.state.tcp_pose,bin_target_pose(c.grasp.bin_approach_pose,s.grasp,snapshot),c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad)) {
      recover(CycleOutcome::kFailed,true,"transport arrival/holding/scene invalid"); return finish();}
    // Freeze the release target after bin arrival. Feedback during descent cannot move it.
    if(c.grasp.bin_height && e.bin_height_offset_m && std::isfinite(*e.bin_height_offset_m) && *e.bin_height_offset_m>=0){
      s.grasp.bin_height_offset_m=std::max(s.grasp.bin_height_offset_m,*e.bin_height_offset_m);
    }
    stage(TaskPhase::kPlacing,c.task.place_timeout_sec); request(CommandKind::kMoveBinRelease);
    d.status = {Disposition::kApplied,"bin approach confirmed; release pose requested"};
  } else if (s.phase == TaskPhase::kPlacing && e.kind == TaskEventKind::kMotionFinished) {
    if (!e.motion_succeeded || !scene_ready(e.scene,snapshot,t) || e.scene.attachment != AttachmentPhase::kAttached) {recover(CycleOutcome::kFailed,true,"release pose failed"); return finish();}
    GraspEvent release = e.grasp_event; release.kind = GraspEventKind::kReleaseRequested; release.key = s.grasp.context.key;
    release.key.sequence = e.key.sequence; release.arm = e.arm; release.gripper = e.gripper; release.scene_confirmed = true;
    const auto child = advance_grasp(s.grasp,release,snapshot,t); s.grasp = child.state;
    s.expected_operation_id = s.grasp.context.key.operation_id;
    d.commands.insert(d.commands.end(),child.commands.begin(),child.commands.end()); d.status = child.status;
    if (child.status.disposition != Disposition::kApplied) {recover(CycleOutcome::kFailed,true,"release rejected");}
    else if (gripper_snapshot_ready(e.gripper,snapshot,t) && e.gripper.stopped &&
      e.gripper.evidence_sequence>original.grasp.evidence_sequence &&
      e.gripper.timestamp>original.grasp.evidence_timestamp &&
      aperture_below_contact_span(e.gripper.width_m,e.grasp_event.expected_contact_span_m)) {
      s.outcome = CycleOutcome::kFailed;
      d.status = {Disposition::kApplied,"aperture contracted during transfer; release at bin before reporting"};
    }
  } else if (s.phase == TaskPhase::kPlacing && e.kind == TaskEventKind::kPlaceConfirmed) {
    if (s.grasp.phase != GraspPhase::kEmpty || s.grasp.possession != Possession::kEmpty || !system_ready(e,snapshot,t,s.phase) ||
      !e.arm.stopped || !e.gripper.stopped || !e.gripper.empty_confirmed || e.scene.attachment != AttachmentPhase::kDetached) {return reject("place requires measured Empty and acknowledged scene detach");}
    const bool lost_in_transfer=s.outcome==CycleOutcome::kFailed;
    if(!lost_in_transfer){s.outcome=CycleOutcome::kDefectPicked;}
    report(); d.status={Disposition::kApplied,lost_in_transfer?
      "transfer loss; bin release and Empty confirmed; report failed placement":
      "place confirmed; one report requested"};
  } else if (s.phase == TaskPhase::kRecovering && s.recovery_home_command_id && e.kind == TaskEventKind::kMotionFinished && e.key.operation_id==s.recovery_home_command_id) {
    const bool home=e.motion_succeeded && arm_snapshot_ready(e.arm,snapshot,t) && e.arm.stopped &&
      gripper_snapshot_ready(e.gripper,snapshot,t) && e.gripper.stopped && e.gripper.empty_confirmed &&
      s.grasp.possession==Possession::kEmpty && scene_ready(e.scene,snapshot,t) &&
      e.scene.attachment==AttachmentPhase::kDetached &&
      pose_near(e.arm.state.tcp_pose,c.motion.observation_pose,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad);
    if(home){s.recovery_home_command_id=0;report();d.status={Disposition::kApplied,"recovery Home measured; report skipped target before next cycle"};}
    else if(!s.terminal_fault && s.grasp.possession==Possession::kEmpty && e.gripper.empty_confirmed){
      s.recovery_home_command_id=0;
      d.status={Disposition::kApplied,"recovery Home interrupted; wait for fresh idle scene and retry within original recovery deadline"};
    }else{s.phase=TaskPhase::kFaulted;s.context.inhibited=true;s.recovery_home_command_id=0;d.status={Disposition::kApplied,"recovery Home failed with unknown possession/fault; no arrival assumed"};}
  } else if ((s.phase == TaskPhase::kRecovering || s.phase == TaskPhase::kFaulted) && e.kind == TaskEventKind::kRecoveryConfirmed) {
    if(s.recovery_home_command_id){return reject("recovery Home command already pending");}
    if (!arm_snapshot_ready(e.arm,snapshot,t) || !e.arm.stopped || !gripper_snapshot_ready(e.gripper,snapshot,t) || !e.gripper.stopped) {return reject("recovery requires both stops");}
    // Commissioning may reconcile a failed grasp only after actual release,
    // gateway Idle and coherent detach. A task fault is not a permanent latch;
    // operator Cancel and unresolved device/possession faults retain their policy.
    const bool demo_recovery=c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept && (s.outcome==CycleOutcome::kFailed || s.outcome==CycleOutcome::kInconclusive) &&
      e.arm.state.motion_state==ArmMotionState::kIdle && e.gripper.empty_confirmed &&
      e.scene.attachment==AttachmentPhase::kDetached && scene_ready(e.scene,snapshot,t);
    if(demo_recovery && !s.recovery_home_command_id){
      GraspEvent reset;reset.kind=GraspEventKind::kResetRequested;reset.key=e.key;reset.arm=e.arm;reset.gripper=e.gripper;
      const auto child=advance_grasp(s.grasp,reset,snapshot,t);
      if(child.status.disposition!=Disposition::kApplied){return reject("recovery requires measured Empty grasp reconciliation");}
      s.grasp=child.state;s.terminal_fault=false;s.context.inhibited=false;
      if(s.phase==TaskPhase::kFaulted){s.context.deadline=deadline_after(t.monotonic_now,c.task.recovery_timeout_sec).value_or(t.monotonic_now);}
      s.phase=TaskPhase::kRecovering;
    }
    if (s.terminal_fault || s.grasp.possession != Possession::kEmpty || !e.gripper.empty_confirmed || e.scene.attachment != AttachmentPhase::kDetached) {
      s.phase = TaskPhase::kFaulted; s.context.inhibited = true; d.status = {Disposition::kApplied,"stopped but possession/scene/fault requires manual intervention"};
    } else if (!scene_ready(e.scene,snapshot,t)) {return reject("recovery waiting for current scene acknowledgment");
    } else if (!demo_recovery && s.outcome == CycleOutcome::kFailed && s.retries < c.task.max_cycle_retries && !expired(s.cycle_deadline,t) && system_ready(e,snapshot,t,s.phase)) {
      if (s.grasp.phase == GraspPhase::kFailed) {
        GraspEvent reset; reset.kind = GraspEventKind::kResetRequested; reset.key = e.key; reset.arm = e.arm; reset.gripper = e.gripper;
        const auto child = advance_grasp(s.grasp,reset,snapshot,t); s.grasp = child.state;
        if (child.status.disposition != Disposition::kApplied) {s.context.inhibited = true; d.status = {Disposition::kApplied,"recovery grasp reset inconsistent"}; return finish();}
      }
      ++s.retries; s.retired_track_id = s.context.key.track_id; emit_command(s.context,d.commands,CommandKind::kRetireTrack,"task");
      s.context.key.track_id = 0; s.selected_track = {}; s.phase = TaskPhase::kObserving; s.context.deadline = s.cycle_deadline;
      s.observation_arrived = false; request(CommandKind::kMoveObservation); d.status = {Disposition::kApplied,"bounded retry; reacquire a new visual track"};
    } else {
      if (s.grasp.phase == GraspPhase::kFailed) {
        GraspEvent reset; reset.kind = GraspEventKind::kResetRequested; reset.key = e.key; reset.arm = e.arm; reset.gripper = e.gripper;
        const auto child = advance_grasp(s.grasp,reset,snapshot,t); s.grasp = child.state;
        if (child.status.disposition != Disposition::kApplied) {s.context.inhibited = true; d.status = {Disposition::kApplied,"recovery grasp reset inconsistent"}; return finish();}
      }
      const bool skipped=s.outcome==CycleOutcome::kFailed || s.outcome==CycleOutcome::kInconclusive;
      if(skipped && !pose_near(e.arm.state.tcp_pose,c.motion.observation_pose,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad)){
        // Reuse the original recovery budget; an interrupted Home may retry
        // after a fresh scene ACK, but never renews its deadline indefinitely.
        request(CommandKind::kMoveObservation);s.recovery_home_command_id=s.expected_operation_id;
        d.status={Disposition::kApplied,"stopped and Empty; return Home before reporting skipped target"};
      }else{report(); d.status = {Disposition::kApplied,"stopped and Empty; report terminal cycle outcome"};}
    }
  } else if (s.phase == TaskPhase::kReporting && e.kind == TaskEventKind::kReportAcknowledged) {
    s.phase = TaskPhase::kWaitingReady; s.retired_track_id = s.context.key.track_id; d.status = {Disposition::kApplied,"report acknowledged; explicit new cycle required"};
  } else if (active && e.kind == TaskEventKind::kTick) {d.status = {Disposition::kApplied,"task deadline checked"};}
  if (d.status.disposition == Disposition::kApplied) {s.context.key.sequence = e.key.sequence;}
  return finish();
}
TaskDecision dispatch_task_batch(const TaskState & initial, std::vector<TaskEvent> events,
  const SettingsSnapshot & snapshot, const TimeContext & t)
{
  TaskDecision combined{initial,{Disposition::kIgnored,"empty event batch"},{},{}};
  if (events.size() > snapshot.get().diagnostics.event_queue_capacity) {
    auto & context = combined.state.context;
    emit_command(context,combined.commands,CommandKind::kStopArm,"task");
    emit_command(context,combined.commands,CommandKind::kStopGripper,"task");
    emit_command(context,combined.commands,CommandKind::kCancelInference,"task");
    context.inhibited = true; combined.state.phase = TaskPhase::kFaulted;
    if (combined.state.grasp.phase == GraspPhase::kClosing || combined.state.grasp.phase == GraspPhase::kVerifying || combined.state.grasp.phase == GraspPhase::kReleasing) {
      combined.state.grasp.phase = GraspPhase::kUnknown; combined.state.grasp.possession = Possession::kUnknown;
    }
    combined.status = {Disposition::kApplied,"event queue overflow; inhibit and independent adapter stop path required"};
    combined.record = record_decision(context,"task",static_cast<int>(initial.phase),static_cast<int>(TaskPhase::kFaulted),
      context.key,t,combined.status,snapshot.get().diagnostics.transition_history_capacity); return combined;
  }
  const auto priority = [&](const TaskEvent & e) {
      const bool invalidated = e.kind == TaskEventKind::kFaultRaised || e.kind == TaskEventKind::kClockReset ||
        e.kind == TaskEventKind::kTargetLost || (e.kind == TaskEventKind::kReadinessChanged && !system_ready(e,snapshot,t,initial.phase));
      return event_priority(invalidated,e.kind == TaskEventKind::kCancelRequested,e.kind == TaskEventKind::kTick);
    };
  std::stable_sort(events.begin(),events.end(),[&](const auto & a,const auto & b) {return priority(a) < priority(b);});
  for (const auto & e : events) {
    const auto decision = advance_task(combined.state,e,snapshot,t); combined.state = decision.state;
    combined.commands.insert(combined.commands.end(),decision.commands.begin(),decision.commands.end());
    combined.record = decision.record; combined.status = decision.status;
  }
  return combined;
}
}  // namespace ptc_core
