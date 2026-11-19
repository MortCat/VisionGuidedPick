#include "state_test_support.hpp"
#include "ptc_core/state/admission_model.hpp"
#include "ptc_core/vision/upright_pose_model.hpp"
#include <limits>
using namespace ptc_core;
using namespace ptc_test;

void tests()
{
  const auto cfg = settings();
  TrackedObjectState s; TrackEvent e; e.kind = TrackEventKind::kObservation; e.key = key(11);
  e.observation = {{},time(0.9).source_now,"world",0.01,true}; e.observation.pose_world.position.x_m = -0.03;
  auto d = advance_track(s,e,cfg,time(0.9)); CHECK(d.state.phase == TrackPhase::kCandidate); s = d.state;
  e.key.sequence = 2; e.observation.timestamp = time(1.0).source_now; e.observation.pose_world.position.x_m = -0.02;
  d = advance_track(s,e,cfg,time(1.0)); CHECK(d.state.phase == TrackPhase::kCandidate); CHECK(!d.state.velocity_valid); s = d.state;
  e.key.sequence = 3; e.observation.timestamp = time(1.15).source_now; e.observation.pose_world.position.x_m = -0.005;
  d = advance_track(s,e,cfg,time(1.15)); CHECK(d.state.phase == TrackPhase::kCandidate); CHECK(!d.state.velocity_valid); s = d.state;
  e.key.sequence = 4; e.observation.timestamp = time(1.2).source_now; e.observation.pose_world.position.x_m = 0;
  d = advance_track(s,e,cfg,time(1.2)); CHECK(d.state.phase == TrackPhase::kTracking); CHECK(std::abs(d.state.velocity_mps.x_m-0.1) < 1e-10); s = d.state;
  const auto earlier_capture=track_pose_at_observation(s,cfg,time(1.2),time(1.1).source_now);
  CHECK(earlier_capture);CHECK(std::abs(earlier_capture->position.x_m+.01)<1e-9);
  CHECK(!track_pose_at_observation(s,cfg,time(1.2),time(.2).source_now));
  auto yaw_update=e;yaw_update.key.sequence=50;yaw_update.observation.timestamp=time(1.25).source_now;
  yaw_update.observation.pose_world.position.x_m=.005;
  yaw_update.observation.pose_world.orientation={0,0,std::sin(.2),std::cos(.2)};
  const auto continuous=advance_track(s,yaw_update,cfg,time(1.25));CHECK(continuous.status.disposition==Disposition::kApplied);
  CHECK(std::abs(continuous.state.observation.pose_world.orientation.z)<1e-9);
  CHECK(std::abs(continuous.state.velocity_samples.back().pose_world.orientation.z-std::sin(.2))<1e-9);
  auto reestimating=continuous.state;reestimating.phase=TrackPhase::kCandidate;
  yaw_update.key.sequence=51;yaw_update.observation.timestamp=time(1.3).source_now;
  yaw_update.observation.pose_world.position.x_m=.01;
  const auto same_axes=advance_track(reestimating,yaw_update,cfg,time(1.3));
  CHECK(same_axes.status.disposition==Disposition::kApplied);
  CHECK(std::abs(same_axes.state.observation.pose_world.orientation.z)<1e-9);

  CHECK(s.velocity_fit_residual_m < 1e-8); CHECK(s.velocity_uncertainty_mps > 0);
  CHECK(advance_track(s,e,cfg,time(1.2)).status.disposition == Disposition::kRejected);
  e.key.sequence = 5; e.observation.timestamp = time(1.15).source_now; e.observation.position_stddev_m = 0.01;
  CHECK(advance_track(s,e,cfg,time(1.2)).status.disposition == Disposition::kRejected);
  e.key.sequence = 6; e.kind = TrackEventKind::kMissing; d = advance_track(s,e,cfg,time(1.3)); CHECK(d.state.phase == TrackPhase::kTemporarilyLost);
  CHECK(predict_object_pose(d.state,cfg,time(1.3),0.05)); CHECK(!predict_object_pose(d.state,cfg,time(1.41),0.01));
  auto dropout_config = settings_data();
  dropout_config.tracking.observation_max_age_sec = 0.05;
  dropout_config.tracking.max_prediction_horizon_sec = 0.2;
  const auto dropout = settings(dropout_config);
  CHECK(!track_is_usable(s,dropout,time(1.3)));
  CHECK(track_is_usable(s,dropout,time(1.3),true));
  CHECK(predict_object_pose(s,dropout,time(1.3),0.05));
  CHECK(!predict_object_pose(s,dropout,time(1.41),0));
  CHECK(!predict_object_pose(s,dropout,time(1.35),0.1));
  auto zero = settings_data(); zero.tracking.max_prediction_horizon_sec = 0;
  CHECK(!predict_object_pose(d.state,settings(zero),time(1.3),0));
  e.key.sequence = 7; e.kind = TrackEventKind::kObservation; e.observation = track(2).observation;
  d = advance_track(d.state,e,cfg,time(2.3)); CHECK(d.state.phase == TrackPhase::kLost);
  e.key.sequence = 8; CHECK(advance_track(d.state,e,cfg,time(2.3)).state.phase == TrackPhase::kLost);
  e.kind = TrackEventKind::kRetire; CHECK(advance_track(s,e,cfg,time(1.2)).state.phase == TrackPhase::kRetired);
  auto paused = time(1.2); paused.paused = true; CHECK(advance_track(s,e,cfg,paused).state.phase == TrackPhase::kLost);
  auto reset = time(1.2); reset.epoch = 2; CHECK(advance_track(s,e,cfg,reset).state.phase == TrackPhase::kLost);
  e.key.track_id = 2; CHECK(advance_track(s,e,cfg,time(1.2)).status.disposition == Disposition::kRejected);

  // Alternating depth jitter must not become a large vertical feed-forward speed.
  TrackedObjectState noisy;
  for(unsigned i=0;i<8;++i){TrackEvent observed;observed.kind=TrackEventKind::kObservation;observed.key=key(12,i+1);const double t=.5+.05*i;observed.observation={{},time(t).source_now,"world",.01,true};observed.observation.pose_world.position={.01*i,0,i%2?.005:-.005};noisy=advance_track(noisy,observed,cfg,time(t)).state;}
  CHECK(noisy.velocity_valid);CHECK(std::abs(noisy.velocity_mps.x_m-.2)<1e-10);CHECK(std::abs(noisy.velocity_mps.z_m)<.02);CHECK(noisy.velocity_samples.size()<=8);
  // Use the line fit at capture time, not the final noisy depth sample.
  CHECK(std::abs(noisy.observation.pose_world.position.z_m)<.005);
  CHECK(std::abs(noisy.velocity_samples.back().pose_world.position.z_m-.005)<1e-9);
  CHECK(noisy.velocity_fit_residual_m<=cfg.get().tracking.max_velocity_fit_residual_m);
  CHECK(noisy.velocity_uncertainty_mps>0);
  CHECK(noisy.velocity_uncertainty_mps<=cfg.get().tracking.max_velocity_uncertainty_mps);
  const auto near_uncertainty=predicted_position_uncertainty_m(noisy,cfg,time(.85),0.01);
  const auto far_uncertainty=predicted_position_uncertainty_m(noisy,cfg,time(.9),0.01);
  CHECK(near_uncertainty&&far_uncertainty&&*far_uncertainty>*near_uncertainty);

  // Position noise must invalidate velocity when the configured quality limit is too strict.
  auto strict_velocity_data=settings_data();strict_velocity_data.tracking.max_velocity_uncertainty_mps=0.001;
  const auto strict_velocity=settings(strict_velocity_data);
  TrackedObjectState uncertain;
  for(unsigned i=0;i<3;++i){TrackEvent observed;observed.kind=TrackEventKind::kObservation;observed.key=key(13,i+1);const double t=.5+.15*i;observed.observation={{},time(t).source_now,"world",.01,true};observed.observation.pose_world.position.x_m=.01*i;uncertain=advance_track(uncertain,observed,strict_velocity,time(t)).state;}
  CHECK(!uncertain.velocity_valid);CHECK(uncertain.phase==TrackPhase::kCandidate);

  TrackedObjectState jumped;
  for(unsigned i=0;i<4;++i){TrackEvent observed;observed.kind=TrackEventKind::kObservation;observed.key=key(14,i+1);const double t=.5+.1*i;observed.observation={{},time(t).source_now,"world",.01,true};observed.observation.pose_world.position.x_m=i==3?.13:.01*i;jumped=advance_track(jumped,observed,cfg,time(t)).state;}
  CHECK(!jumped.velocity_valid);CHECK(jumped.phase==TrackPhase::kCandidate);

  auto association_data=settings_data();association_data.tracking.association_distance_m=0.075;
  const auto association_settings=settings(association_data);
  VisualTrackModel visual_tracks;
  ObjectObservation left{{},time(1.0).source_now,"world",0.01,true};left.pose_world.position.x_m=0;
  ObjectObservation right=left;right.pose_world.position.x_m=0.15;
  auto assigned=update_visual_tracks(visual_tracks,{left,right},{0,0},association_settings,time(1.0));
  CHECK(assigned.size()==2&&assigned[0]==1&&assigned[1]==2);
  ObjectObservation moved=right;moved.timestamp=time(1.1).source_now;moved.pose_world.position.x_m=0.14;
  assigned=update_visual_tracks(visual_tracks,{moved},{1},association_settings,time(1.1));
  CHECK(assigned.size()==1&&assigned[0]==0);CHECK(visual_tracks.tracks.size()==2);
  moved.timestamp=time(1.2).source_now;
  assigned=update_visual_tracks(visual_tracks,{moved},{2},association_settings,time(1.2));
  CHECK(assigned.size()==1&&assigned[0]==2);
  const auto prediction=track_pose_at_observation(track(1),cfg,time(1.1),time(1.1).source_now);
  CHECK(prediction&&std::abs(prediction->position.x_m-0.01)<1e-10);

  MotionState m; MotionEvent me; me.kind = MotionEventKind::kDeviceReady; me.key = key(); me.arm = arm();
  me.scene = scene(); me.base_in_world = {time().source_now,"world",{}};
  auto md = advance_motion(m,me,cfg,time()); CHECK(md.state.phase == ArmMotionState::kIdle); m = md.state;
  me.kind = MotionEventKind::kMoveRequested; me.key = key(2); me.request = {{time().source_now,"base",{}},0.5,0.5,*seconds_to_duration(10)};
  md = advance_motion(m,me,cfg,time()); CHECK(md.state.phase == ArmMotionState::kPlanning); CHECK(md.commands.size() == 1); CHECK(md.commands[0].kind == CommandKind::kPlan); m = md.state;
  me.key = key(3); CHECK(advance_motion(m,me,cfg,time()).status.disposition == Disposition::kRejected);
  me.key = key(2,2); me.kind = MotionEventKind::kPlanSucceeded;
  md = advance_motion(m,me,cfg,time(1.1)); CHECK(md.state.phase == ArmMotionState::kMoving); CHECK(md.commands[0].kind == CommandKind::kExecuteTrajectory); m = md.state;
  CHECK(advance_motion(m,me,cfg,time(1.1)).commands.empty());
  // A post-grasp HOLD can briefly lose its stopped sample while planning.
  // Retain the completed plan, never execute until fresh stopped feedback.
  auto settling_plan=me;settling_plan.arm=arm_at(1.1);settling_plan.arm.stopped=false;
  const auto settling_start=advance_motion(MotionState{ArmMotionState::kIdle},move_event(),cfg,time()).state;
  const auto waiting_plan=advance_motion(settling_start,settling_plan,cfg,time(1.1));
  CHECK(waiting_plan.state.phase==ArmMotionState::kPlanning);
  CHECK(waiting_plan.state.planned_target_pose);CHECK(waiting_plan.commands.empty());
  const auto original_plan_deadline=waiting_plan.state.context.deadline;
  auto settled_tick=settling_plan;settled_tick.kind=MotionEventKind::kTick;settled_tick.key.sequence=3;
  settled_tick.arm=arm_at(1.2);settled_tick.arm.stopped=false;
  const auto still_settling=advance_motion(waiting_plan.state,settled_tick,cfg,time(1.2));
  CHECK(still_settling.state.phase==ArmMotionState::kPlanning);CHECK(still_settling.commands.empty());
  CHECK(still_settling.state.context.deadline==original_plan_deadline);
  settled_tick.key.sequence=4;settled_tick.arm=arm_at(1.3);
  const auto settled=advance_motion(still_settling.state,settled_tick,cfg,time(1.3));
  CHECK(settled.state.phase==ArmMotionState::kMoving);CHECK(settled.commands.size()==1);
  CHECK(settled.commands.front().kind==CommandKind::kExecuteTrajectory);
  settled_tick.key.sequence=5;settled_tick.arm=arm_at(1.4);
  CHECK(advance_motion(settled.state,settled_tick,cfg,time(1.4)).commands.empty());
  auto settling_cancel=settling_plan;settling_cancel.kind=MotionEventKind::kCancelRequested;settling_cancel.key.sequence=3;
  const auto canceled_plan=advance_motion(waiting_plan.state,settling_cancel,cfg,time(1.2));
  CHECK(canceled_plan.state.phase==ArmMotionState::kStopping);
  CHECK(canceled_plan.state.pending_result==MoveToPoseError::kCanceled);
  auto late_settle=settled_tick;late_settle.arm=arm_at(3);
  const auto expired_plan=advance_motion(waiting_plan.state,late_settle,cfg,time(3));
  CHECK(expired_plan.state.phase==ArmMotionState::kStopping);
  CHECK(expired_plan.state.pending_result==MoveToPoseError::kTimedOut);
  // Delayed results never bypass fresh feedback or leak into a new move.
  auto stale_settle=settled_tick;stale_settle.key.sequence=3;stale_settle.arm=arm_at(.1);
  const auto stale_plan_feedback=advance_motion(waiting_plan.state,stale_settle,cfg,time(1.2));
  CHECK(stale_plan_feedback.state.phase==ArmMotionState::kStopping);
  CHECK(stale_plan_feedback.state.pending_result==MoveToPoseError::kNotReady);
  auto complete_settled=settled_tick;complete_settled.kind=MotionEventKind::kExecutionSucceeded;
  complete_settled.key.sequence=6;complete_settled.arm=arm_at(1.4);
  const auto complete_plan=advance_motion(settled.state,complete_settled,cfg,time(1.4));
  CHECK(complete_plan.result && complete_plan.result->success);
  auto next_request=move_event(3);next_request.arm=arm_at(1.4);next_request.scene=scene(1.4);
  const auto next_plan=advance_motion(complete_plan.state,next_request,cfg,time(1.4));
  CHECK(next_plan.request_accepted);CHECK(next_plan.state.phase==ArmMotionState::kPlanning);CHECK(!next_plan.state.planned_target_pose);
  auto next_tick=next_request;next_tick.kind=MotionEventKind::kTick;next_tick.key.sequence=2;next_tick.arm=arm_at(1.5);
  CHECK(advance_motion(next_plan.state,next_tick,cfg,time(1.5)).commands.empty());
  auto moving_state=m;moving_state.phase=ArmMotionState::kPlanning;
  moving_state.request.target_pose.timestamp=time(1).source_now;
  moving_state.request.target_linear_velocity={.06,0,0};moving_state.request.prediction_horizon=*seconds_to_duration(2);
  auto endpoint_event=me;endpoint_event.key.sequence=3;
  endpoint_event.request.target_pose={time(2).source_now,"base",{}};endpoint_event.request.target_pose.pose.position.x_m=.06;
  const auto corrected=advance_motion(moving_state,endpoint_event,cfg,time(1.1));
  CHECK(corrected.state.phase==ArmMotionState::kMoving);CHECK(corrected.state.planned_target_pose);
  CHECK(std::abs(corrected.state.planned_target_pose->pose.position.x_m-.06)<1e-9);
  CHECK(corrected.state.request.target_pose.timestamp==time(1).source_now);
  CHECK(corrected.state.request.prediction_horizon==*seconds_to_duration(2));
  endpoint_event.request.target_pose.pose.position.x_m=.5;
  CHECK(advance_motion(moving_state,endpoint_event,cfg,time(1.1)).state.phase==ArmMotionState::kStopping);

  // Moving M1 keeps its admitted target identity after the planner freezes an
  // arrival endpoint. Reproduce the WSL stop at a 0.766667 s observation age.
  auto moving_demo_data=settings_data();moving_demo_data.runtime.operation_mode="assumed_ng_demo";
  moving_demo_data.inspection.inference.backend="assumed_ng";
  moving_demo_data.grasp.moving_intercept=true;
  moving_demo_data.tracking.max_prediction_horizon_sec=3.;
  moving_demo_data.tracking.temporary_loss_timeout_sec=3.5;
  moving_demo_data.scene.dynamic_object_max_age_sec=.7;
  const auto moving_demo=settings(moving_demo_data);
  auto admitted_move=move_event();admitted_move.request.target_linear_velocity={.06,0,0};
  admitted_move.request.prediction_horizon=*seconds_to_duration(2.);
  const auto admitted=advance_motion(MotionState{ArmMotionState::kIdle},admitted_move,moving_demo,time());
  CHECK(admitted.request_accepted);
  auto planned=admitted_move;planned.kind=MotionEventKind::kPlanSucceeded;planned.key.sequence=2;
  planned.arm=arm_at(1.1);planned.request.target_pose={time(2).source_now,"base",{}};
  planned.request.target_pose.pose.position.x_m=.06;
  const auto executing=advance_motion(admitted.state,planned,moving_demo,time(1.1));
  CHECK(executing.state.phase==ArmMotionState::kMoving);
  CHECK(executing.state.request.prediction_horizon==admitted_move.request.prediction_horizon);
  CHECK(executing.commands.front().target.position.x_m==.06);
  auto stale_tick=planned;stale_tick.kind=MotionEventKind::kTick;stale_tick.key.sequence=3;
  stale_tick.arm=arm_at(1.766667);
  const auto continuing=advance_motion(executing.state,stale_tick,moving_demo,time(1.766667));
  CHECK(continuing.state.phase==ArmMotionState::kMoving);CHECK(continuing.commands.empty());
  // The actual planned endpoint remains the arrival target, not a new prediction.
  auto arrival=stale_tick;arrival.kind=MotionEventKind::kExecutionSucceeded;arrival.key.sequence=4;
  arrival.arm=arm_at(2.4);arrival.arm.state.tcp_pose.position.x_m=.06;
  const auto arrived=advance_motion(continuing.state,arrival,moving_demo,time(2.4));
  CHECK(arrived.result && arrived.result->success);
  // Anonymous/static moves retain normal scene expiry, even in demo mode.
  auto static_move=admitted_move;static_move.request.prediction_horizon=Nanoseconds{0};
  static_move.request.target_linear_velocity={};
  auto static_planning=advance_motion(MotionState{ArmMotionState::kIdle},static_move,moving_demo,time());
  auto static_plan=planned;static_plan.request=static_move.request;
  const auto static_executing=advance_motion(static_planning.state,static_plan,moving_demo,time(1.1));
  const auto stale_static=advance_motion(static_executing.state,stale_tick,moving_demo,time(1.766667));
  CHECK(stale_static.state.phase==ArmMotionState::kStopping);
  CHECK(stale_static.state.pending_result==MoveToPoseError::kNotReady);
  // Operation deadlines and explicit cancellation still stop an admitted move.
  auto late_tick=stale_tick;late_tick.key.sequence=4;late_tick.arm=arm_at(7);
  CHECK(advance_motion(continuing.state,late_tick,moving_demo,time(7)).state.pending_result==MoveToPoseError::kTimedOut);
  auto cancel_move=stale_tick;cancel_move.kind=MotionEventKind::kCancelRequested;cancel_move.key.sequence=4;
  const auto stopping_move=advance_motion(continuing.state,cancel_move,moving_demo,time(1.8));
  CHECK(stopping_move.state.phase==ArmMotionState::kStopping);
  CHECK(stopping_move.state.pending_result==MoveToPoseError::kCanceled);
  auto stopped_move=cancel_move;stopped_move.kind=MotionEventKind::kStopConfirmed;stopped_move.key.sequence=5;
  stopped_move.arm=arm_at(1.81);
  CHECK(!advance_motion(stopping_move.state,stopped_move,moving_demo,time(1.81)).state.active);
  me.key.sequence = 3; me.kind = MotionEventKind::kExecutionSucceeded; me.arm = arm_at(1.2); me.arm.state.tcp_pose.position.x_m = 0.5;
  CHECK(advance_motion(m,me,cfg,time(1.2)).status.disposition == Disposition::kRejected);
  me.arm.state.tcp_pose = {}; md = advance_motion(m,me,cfg,time(1.2)); CHECK(md.result && md.result->success); CHECK(!md.state.active);
  // A worker completion rejected for transient feedback remains eligible for
  // a later measured snapshot, but never survives cancel or the old deadline.
  auto completion=me;completion.key.sequence=4;completion.arm.tf_valid=false;
  auto deferred=advance_motion(m,completion,cfg,time(1.2));
  CHECK(!deferred.result && deferred.state.phase==ArmMotionState::kMoving);
  completion.key.sequence=5;completion.arm=arm_at(1.21);completion.arm.state.tcp_pose={};
  auto accepted_completion=advance_motion(deferred.state,completion,cfg,time(1.21));
  CHECK(accepted_completion.result && accepted_completion.result->success);
  auto canceled_completion=completion;canceled_completion.kind=MotionEventKind::kCancelRequested;
  const auto canceled_state=advance_motion(deferred.state,canceled_completion,cfg,time(1.21)).state;
  completion.key.sequence=6;
  const auto late_completion=advance_motion(canceled_state,completion,cfg,time(1.22));
  CHECK(!late_completion.result || !late_completion.result->success);
  CHECK(late_completion.state.phase!=ArmMotionState::kIdle);
  CHECK(advance_motion(deferred.state,completion,cfg,time(11)).state.phase==ArmMotionState::kStopping);
  me.kind = MotionEventKind::kPlanSucceeded; me.key = key(2,3);
  auto planning = advance_motion(MotionState{ArmMotionState::kIdle},move_event(),cfg,time()).state;
  md = advance_motion(planning,me,cfg,time(3)); CHECK(md.state.phase == ArmMotionState::kStopping); CHECK(md.state.pending_result == MoveToPoseError::kTimedOut);
  me.kind = MotionEventKind::kStopConfirmed; me.arm = arm_at(4); me.key.sequence = 4;
  md = advance_motion(md.state,me,cfg,time(4)); CHECK(md.state.phase == ArmMotionState::kFault); CHECK(md.state.active);
  me.kind = MotionEventKind::kResetRequested; me.key = key(5); md = advance_motion(md.state,me,cfg,time(4)); CHECK(md.state.phase == ArmMotionState::kIdle); CHECK(md.commands.empty());
  me.kind = MotionEventKind::kTick; me.key = key(2,2); me.arm = arm();
  CHECK(advance_motion(planning,me,cfg,time(1.5)).state.pending_result == MoveToPoseError::kNotReady);
  me.kind = MotionEventKind::kCancelRequested; md = advance_motion(planning,me,cfg,time()); CHECK(md.commands.size() == 1); CHECK(md.state.active);
  me.key.sequence = 3; CHECK(advance_motion(md.state,me,cfg,time()).commands.empty());
  me.kind = MotionEventKind::kStopConfirmed; me.arm = arm(); md = advance_motion(md.state,me,cfg,time()); CHECK(!md.state.active); CHECK(md.result->error_code == MoveToPoseError::kCanceled);
  me.kind = MotionEventKind::kFollowRequested; me.key = key(6); me.target_timestamp = time().source_now; me.linear_velocity = {0.1,0,0};
  md = advance_motion(md.state,me,cfg,time()); CHECK(md.state.phase == ArmMotionState::kFollowing); CHECK(md.commands.size() == 1);
  auto longer_follow=me;longer_follow.follow_timeout=*seconds_to_duration(20);
  const auto extended=advance_motion(MotionState{ArmMotionState::kIdle},longer_follow,cfg,time());
  CHECK(extended.request_accepted);
  CHECK(extended.state.operation_deadline==time().monotonic_now+*seconds_to_duration(20));
  CHECK(extended.state.context.deadline==extended.state.operation_deadline);
  CHECK(extended.state.update_deadline==time().monotonic_now+*seconds_to_duration(cfg.get().motion.target_update_timeout_sec));
  m = md.state; me.kind = MotionEventKind::kTargetUpdated; me.key.sequence = 2;
  CHECK(advance_motion(m,me,cfg,time(1.1)).status.disposition == Disposition::kRejected);
  me.target_timestamp = time(1.1).source_now; md = advance_motion(m,me,cfg,time(1.1)); CHECK(md.commands.size() == 1);
  me.key.sequence = 3; me.target_timestamp = time(1.3).source_now;
  CHECK(advance_motion(md.state,me,cfg,time(1.3)).state.phase == ArmMotionState::kStopping);
  me.linear_velocity.x_m = std::numeric_limits<double>::infinity(); me.kind = MotionEventKind::kFollowRequested; me.key = key(7);
  CHECK(advance_motion(MotionState{ArmMotionState::kIdle},me,cfg,time(1.3)).status.disposition == Disposition::kRejected);
  m.context.next_command_id = std::numeric_limits<std::uint64_t>::max(); me.kind = MotionEventKind::kCancelRequested; me.key = key(6,2);
  CHECK(advance_motion(m,me,cfg,time()).state.phase == ArmMotionState::kFault);
}
int main() {return run(tests);}
