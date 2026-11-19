#include "ptc_core/state/motion_state.hpp"
#include <algorithm>
#include <cmath>

namespace ptc_core
{
MoveToPosePhase m1_action_phase(ArmMotionState phase)
{
  switch (phase) {
    case ArmMotionState::kValidating: return MoveToPosePhase::kValidating;
    case ArmMotionState::kPlanning: return MoveToPosePhase::kPlanning;
    case ArmMotionState::kMoving: return MoveToPosePhase::kExecuting;
    case ArmMotionState::kStopping: return MoveToPosePhase::kCanceling;
    default: return MoveToPosePhase::kUnknown; // Following belongs to a different operation/Action.
  }
}
MotionDecision advance_motion(const MotionState & original, const MotionEvent & e,
  const SettingsSnapshot & snapshot, const TimeContext & t)
{
  MotionDecision d{original, {}, {}, {}, {}}; auto & s = d.state; const auto & c = snapshot.get();
  const auto finish = [&]() {
      if (s.context.inhibited) {
        s.phase = ArmMotionState::kFault;
        if (d.request_accepted) {
          d.status.reason = "command emission inhibited; request not accepted";
          if (e.kind == MotionEventKind::kMoveRequested) {
            d.result = MoveToPoseResult{false,MoveToPoseError::kInternalError,d.status.reason,{}};
          }
        }
        d.request_accepted = false;
      }
      d.record = record_decision(s.context, "motion", static_cast<int>(original.phase), static_cast<int>(s.phase),
        e.key, t, d.status, c.diagnostics.transition_history_capacity); return d;
    };
  const auto reject = [&](const char * reason, MoveToPoseError error = MoveToPoseError::kInternalError) {
      d.status = {Disposition::kRejected, reason};
      if (e.kind == MotionEventKind::kMoveRequested) {d.result = MoveToPoseResult{false,error,reason,{}};}
      return finish();
    };
  const auto stop = [&](MoveToPoseError reason) {
      if (s.phase != ArmMotionState::kStopping && s.phase != ArmMotionState::kFault) {
        s.phase = ArmMotionState::kStopping; s.pending_result = reason;
        s.context.deadline = deadline_after(t.monotonic_now, c.motion.stop_confirmation_timeout_sec).value_or(t.monotonic_now);
        emit_command(s.context, d.commands, CommandKind::kStopArm, "motion");
      }
      d.status = {Disposition::kApplied, "stop requested; ownership retained until confirmation"};
    };
  const auto execute_ready_plan = [&]() {
      s.phase = ArmMotionState::kMoving;
      s.context.deadline = std::min(s.operation_deadline, deadline_after(t.monotonic_now, c.motion.execution_timeout_sec).value_or(t.monotonic_now));
      emit_command(s.context, d.commands, CommandKind::kExecuteTrajectory, "motion");
      if (!d.commands.empty()) {
        d.commands.back().target = s.planned_target_pose->pose;
        d.commands.back().reference_frame = c.frames.robot_base;
      }
      d.status = {Disposition::kApplied, "plan ready; measured stop confirmed"};
    };
  if (static_cast<int>(e.kind) < 0 || e.kind >= MotionEventKind::kCount || static_cast<int>(s.phase) > static_cast<int>(ArmMotionState::kFollowing)) {
    return reject("invalid event or phase");
  }
  if (!valid_time(t) || t.monotonic_now < s.context.last_monotonic) {return reject("invalid/backward monotonic time");}
  if (s.active && s.phase == ArmMotionState::kStopping && expired(s.context.deadline, t)) {
    s.phase = ArmMotionState::kFault; s.context.inhibited = true;
    d.status = {Disposition::kApplied, "stop not confirmed before deadline"}; return finish();
  }
  if (s.active && s.phase != ArmMotionState::kStopping && s.phase != ArmMotionState::kFault &&
    (t.paused || e.kind == MotionEventKind::kClockReset || s.context.key.epoch != t.epoch)) {
    stop(MoveToPoseError::kNotReady); return finish();
  }
  // Local readiness loss is processed before asynchronous successes, regardless of event sequence.
  if (e.kind == MotionEventKind::kDeviceUnavailable) {
    if (s.active) {stop(MoveToPoseError::kNotReady);} else {s.phase = ArmMotionState::kNotReady; d.status = {Disposition::kApplied, "device unavailable"};}
    s.arm = {}; return finish();
  }
  if (s.active && s.phase != ArmMotionState::kStopping && s.phase != ArmMotionState::kFault &&
    e.kind == MotionEventKind::kCancelRequested && valid_event(e.key,t,snapshot.revision()) &&
    same_operation(e.key,s.context.key) && e.key.sequence > s.context.key.sequence) {
    stop(MoveToPoseError::kCanceled); s.context.key.sequence = e.key.sequence;
    s.context.last_monotonic = t.monotonic_now; return finish();
  }
  if (s.active && s.phase != ArmMotionState::kStopping && s.phase != ArmMotionState::kFault &&
    (expired(s.operation_deadline, t) ||
    (s.phase != ArmMotionState::kFollowing && expired(s.context.deadline, t)) ||
    (s.phase == ArmMotionState::kFollowing && expired(s.update_deadline, t)))) {
    stop(MoveToPoseError::kTimedOut); return finish();
  }
  if (!valid_event(e.key, t, snapshot.revision())) {return reject("invalid identity/revision");}
  if (e.kind == MotionEventKind::kResetRequested && s.phase == ArmMotionState::kFault) {
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped) {return reject("fault reset requires fresh stop confirmation");}
    s.context.inhibited = false; s.active = false; s.arm = e.arm; s.phase = ArmMotionState::kIdle;
    s.context.key = e.key; s.context.last_monotonic = t.monotonic_now;
    d.status = {Disposition::kApplied, "manual reset confirmed; no previous command replayed"}; return finish();
  }
  if (s.active && (e.kind == MotionEventKind::kMoveRequested || e.kind == MotionEventKind::kFollowRequested) &&
    !same_operation(e.key,s.context.key)) {return reject("motion owner busy",MoveToPoseError::kBusy);}
  if (s.active && (!same_operation(e.key, s.context.key) || e.key.sequence <= s.context.key.sequence)) {
    return reject("mismatched/duplicate/out-of-order operation");
  }
  s.context.last_monotonic = t.monotonic_now;
  if (e.kind == MotionEventKind::kDeviceReady && !s.active && s.phase != ArmMotionState::kFault) {
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped) {return reject("device not fresh/healthy/stopped");}
    s.arm = e.arm; s.phase = ArmMotionState::kIdle; d.status = {Disposition::kApplied, "device ready"};
  } else if (e.kind == MotionEventKind::kMoveRequested || e.kind == MotionEventKind::kFollowRequested) {
    if (s.phase != ArmMotionState::kIdle || s.active) {return reject("busy or not ready",s.active ? MoveToPoseError::kBusy : MoveToPoseError::kNotReady);}
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped) {return reject("stale/not-ready/not-stopped arm",MoveToPoseError::kNotReady);}
    const auto predicted_track=c.runtime.operation_mode=="assumed_ng_demo" &&
      (e.kind==MotionEventKind::kFollowRequested || (e.kind==MotionEventKind::kMoveRequested && e.request.prediction_horizon.count()>0))?e.key.track_id:0;
    if (!scene_ready(e.scene,snapshot,t,predicted_track)) {return reject("scene is not fresh/coherent/acknowledged",MoveToPoseError::kNotReady);}
    if (e.kind == MotionEventKind::kMoveRequested &&
      (!is_valid_move_to_pose_request(e.request) || e.request.target_pose.reference_frame != c.frames.robot_base ||
      e.request.target_pose.timestamp.count() < 0 || e.request.max_velocity_scale > c.motion.max_velocity_scale ||
      e.request.max_acceleration_scale > c.motion.max_acceleration_scale)) {
      return reject("M1 goal must be validated/frozen in robot_base by adapter",MoveToPoseError::kInvalidGoal);
    }
    if (e.kind == MotionEventKind::kMoveRequested) {
      const auto current_target = predict_move_target(e.request,t.source_now);
      if (!current_target || (e.request.prediction_horizon.count()>0 && (!seconds_to_duration(c.tracking.max_prediction_horizon_sec) || e.request.prediction_horizon > *seconds_to_duration(c.tracking.max_prediction_horizon_sec))) ||
        vector_norm(e.request.target_linear_velocity) > c.motion.max_follow_linear_speed_mps) {
        return reject("invalid or expired moving-pose prediction",MoveToPoseError::kInvalidGoal);
      }
      const auto world_goal = compose_pose(e.base_in_world.pose,current_target->pose);
      if (e.base_in_world.reference_frame != c.frames.world || !world_goal ||
        !source_is_fresh(e.base_in_world.timestamp,t,c.motion.tcp_state_max_age_sec,c.time.sensor_future_tolerance_sec)) {return reject("M1 requires measured base/world TF",MoveToPoseError::kTfUnavailable);}
      if (!inside_workspace(*world_goal,c.motion)) {return reject("M1 target outside world workspace",MoveToPoseError::kInvalidGoal);}
    }
    if (e.kind == MotionEventKind::kFollowRequested && (e.key.session_id == 0 || e.key.track_id == 0 ||
        !inside_workspace(e.target_world, c.motion) ||
        !source_is_fresh(e.target_timestamp, t, c.motion.target_update_timeout_sec, c.time.sensor_future_tolerance_sec) ||
        !std::isfinite(vector_norm(e.linear_velocity)) || vector_norm(e.linear_velocity) > c.motion.max_follow_linear_speed_mps ||
        !std::isfinite(vector_norm(e.angular_velocity)) || vector_norm(e.angular_velocity) > c.motion.max_follow_angular_speed_radps)) {
      return reject("invalid dynamic target/session/speed");
    }
    const double timeout = e.kind == MotionEventKind::kMoveRequested ?
      std::chrono::duration<double>(e.request.timeout).count() :
      e.follow_timeout.count()>0?std::chrono::duration<double>(e.follow_timeout).count():c.motion.execution_timeout_sec;
    if (!begin_operation(s.context, e.key, t, snapshot.revision(), timeout)) {return reject("cannot begin operation");}
    d.request_accepted = true;
    s.arm = e.arm; s.active = true; s.request = e.request; s.operation_deadline = s.context.deadline;
    s.planned_target_pose.reset();
    if (e.kind == MotionEventKind::kMoveRequested) {
      s.context.deadline = std::min(s.operation_deadline, deadline_after(t.monotonic_now, c.motion.planning_timeout_sec).value_or(t.monotonic_now));
      s.phase = ArmMotionState::kPlanning;
      emit_command(s.context, d.commands, CommandKind::kPlan, "motion");
      if (!d.commands.empty()) {d.commands.back().target = e.request.target_pose.pose; d.commands.back().reference_frame = c.frames.robot_base;}
    } else {
      s.phase = ArmMotionState::kFollowing; s.target_timestamp = e.target_timestamp;
      s.update_deadline = deadline_after(t.monotonic_now, c.motion.target_update_timeout_sec).value_or(t.monotonic_now);
      emit_command(s.context, d.commands, CommandKind::kStartFollow, "motion");
      if (!d.commands.empty()) {d.commands.back().target = e.target_world; d.commands.back().reference_frame = c.frames.world;
        d.commands.back().source_timestamp = e.target_timestamp; d.commands.back().linear_velocity = e.linear_velocity;
        d.commands.back().angular_velocity = e.angular_velocity;}
    }
    d.status = {Disposition::kApplied, "motion accepted"};
  } else if (e.kind == MotionEventKind::kCancelRequested && s.active) {
    stop(MoveToPoseError::kCanceled);
  } else if (e.kind == MotionEventKind::kPlanSucceeded && s.phase == ArmMotionState::kPlanning) {
    if (!scene_ready(e.scene,snapshot,t,s.request.prediction_horizon.count()>0?s.context.key.track_id:0) || !arm_snapshot_ready(e.arm,snapshot,t)) {stop(MoveToPoseError::kNotReady); return finish();}
    if (s.request.prediction_horizon.count() > 0) {
      const auto expected = predict_move_target(s.request,e.request.target_pose.timestamp);
      if (!expected || e.request.target_pose.reference_frame != c.frames.robot_base ||
        !pose_near(expected->pose,e.request.target_pose.pose,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad)) {
        stop(MoveToPoseError::kInvalidGoal); return finish();
      }
    }
    // Freeze only the arrival endpoint. The original request still identifies
    // an admitted moving target during execution and retains its source time.
    s.planned_target_pose = s.request.prediction_horizon.count()>0 ? e.request.target_pose : s.request.target_pose;
    if (expired(s.context.deadline, t)) {stop(MoveToPoseError::kTimedOut);}
    else if (e.arm.stopped) {execute_ready_plan();}
    else {
      // Keep the completed plan while HOLD settles. The existing planning
      // deadline, measured stop and backend start-state checks still apply.
      s.arm = e.arm;
      d.status = {Disposition::kApplied, "plan ready; awaiting measured stop within original deadline"};
    }
  } else if (e.kind == MotionEventKind::kPlanFailed && s.phase == ArmMotionState::kPlanning) {
    stop(MoveToPoseError::kPlanningFailed);
  } else if (e.kind == MotionEventKind::kExecutionFailed && s.phase == ArmMotionState::kMoving) {
    stop(MoveToPoseError::kExecutionFailed);
  } else if (e.kind == MotionEventKind::kExecutionSucceeded && s.phase == ArmMotionState::kMoving) {
    if (!arm_snapshot_ready(e.arm, snapshot, t) || !e.arm.stopped) {return reject("success requires actual fresh stopped state");}
    const auto & p = e.arm.state.tcp_pose;
    const auto & goal = s.planned_target_pose ? s.planned_target_pose->pose : s.request.target_pose.pose;
    if (!pose_near(p,goal,c.motion.position_tolerance_m,c.motion.orientation_tolerance_rad)) {return reject("measured TCP outside tolerance");}
    s.active = false; s.phase = ArmMotionState::kIdle; s.arm = e.arm;
    d.result = MoveToPoseResult{true, MoveToPoseError::kSuccess, "arrived", {e.arm.tcp_timestamp, c.frames.robot_base, p}};
    d.status = {Disposition::kApplied, "measured arrival confirmed"};
  } else if (e.kind == MotionEventKind::kTargetUpdated && s.phase == ArmMotionState::kFollowing) {
    if (!arm_snapshot_ready(e.arm,snapshot,t) || !scene_ready(e.scene,snapshot,t,s.context.key.track_id)) {stop(MoveToPoseError::kNotReady); return finish();}
    if (e.target_timestamp <= s.target_timestamp) {return reject("non-increasing target timestamp");}
    if (!inside_workspace(e.target_world, c.motion) ||
      !source_is_fresh(e.target_timestamp, t, c.motion.target_update_timeout_sec, c.time.sensor_future_tolerance_sec) ||
      !std::isfinite(vector_norm(e.linear_velocity)) || vector_norm(e.linear_velocity) > c.motion.max_follow_linear_speed_mps ||
      !std::isfinite(vector_norm(e.angular_velocity)) || vector_norm(e.angular_velocity) > c.motion.max_follow_angular_speed_radps) {
      stop(MoveToPoseError::kInvalidGoal);
    } else {
      s.arm = e.arm;
      s.target_timestamp = e.target_timestamp;
      s.update_deadline = deadline_after(t.monotonic_now, c.motion.target_update_timeout_sec).value_or(t.monotonic_now);
      emit_command(s.context, d.commands, CommandKind::kUpdateTarget, "motion");
      if (!d.commands.empty()) {d.commands.back().target = e.target_world; d.commands.back().reference_frame = c.frames.world;
        d.commands.back().source_timestamp = e.target_timestamp; d.commands.back().linear_velocity = e.linear_velocity;
        d.commands.back().angular_velocity = e.angular_velocity;}
      d.status = {Disposition::kApplied, "target updated"};
    }
  } else if (e.kind == MotionEventKind::kStopConfirmed && s.phase == ArmMotionState::kStopping) {
    if (!e.arm.stopped || !arm_snapshot_ready(e.arm, snapshot, t)) {return reject("stop confirmation invalid");}
    s.active = false; s.arm = e.arm; s.phase = ArmMotionState::kIdle;
    d.result = MoveToPoseResult{false, s.pending_result, "stopped", {e.arm.tcp_timestamp, c.frames.robot_base, e.arm.state.tcp_pose}};
    d.status = {Disposition::kApplied, "stopped; ownership released"};
  } else if (e.kind == MotionEventKind::kStopFailed && s.phase == ArmMotionState::kStopping) {
    s.phase = ArmMotionState::kFault; s.context.inhibited = true; d.status = {Disposition::kApplied, "stop failed; ownership inhibited"};
  } else if (s.active && (e.kind == MotionEventKind::kTick || e.kind == MotionEventKind::kDeviceReady)) {
    const auto predicted_track=(s.request.prediction_horizon.count()>0||s.phase==ArmMotionState::kFollowing?s.context.key.track_id:0);
    if (s.phase != ArmMotionState::kStopping && s.phase != ArmMotionState::kFault &&
      (!arm_snapshot_ready(e.arm, snapshot, t) || !scene_ready(e.scene,snapshot,t,predicted_track))) {stop(MoveToPoseError::kNotReady); return finish();}
    s.arm = e.arm;
    if (expired(s.context.deadline, t)) {stop(MoveToPoseError::kTimedOut);}
    else if (s.phase == ArmMotionState::kPlanning && s.planned_target_pose && e.arm.stopped) {execute_ready_plan();}
    else {d.status = {Disposition::kApplied, "deadline checked"};}
  }
  if (d.status.disposition == Disposition::kApplied) {s.context.key.sequence = e.key.sequence;}
  return finish();
}
}  // namespace ptc_core
