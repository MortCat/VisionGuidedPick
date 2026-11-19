#include "state_test_support.hpp"
#include <limits>
using namespace ptc_core;
using namespace ptc_test;

void tests()
{
  const auto cfg = settings();
  CHECK(m1_action_phase(ArmMotionState::kMoving) == MoveToPosePhase::kExecuting);
  CHECK(m1_action_phase(ArmMotionState::kStopping) == MoveToPosePhase::kCanceling);
  CHECK(m1_action_phase(ArmMotionState::kFollowing) == MoveToPosePhase::kUnknown);
  auto k = key(); auto t = time();
  for (int which = 0; which < 6; ++which) {
    auto invalid = k;
    if (which == 0) {invalid.epoch = 0;} if (which == 1) {invalid.settings_revision.clear();} if (which == 2) {invalid.operation_id = 0;}
    if (which == 3) {invalid.sequence = 0;} if (which == 4) {invalid.settings_revision = "wrong";} if (which == 5) {invalid.epoch = 2;}
    CHECK(!valid_event(invalid,t,cfg.revision()));
  }
  auto data = settings_data(); data.motion.planning_timeout_sec = 1e-12; CHECK(!validate_settings(data).empty());
  data = settings_data(); data.runtime.workstation_namespace = "1bad"; CHECK(!validate_settings(data).empty());
  data.runtime.workstation_namespace = "valid/sub_cell"; CHECK(validate_settings(data).empty());
  data.frames.world = "/world"; CHECK(!validate_settings(data).empty());
  CHECK(!seconds_to_duration(1e-12)); CHECK(!seconds_to_duration(std::numeric_limits<double>::infinity()));
  CHECK(!deadline_after(Nanoseconds::max(),1)); CHECK(source_is_fresh(time(1.01).source_now,t,1,0.01)); CHECK(!source_is_fresh(time(1.011).source_now,t,1,0.01));
  CHECK(!source_is_fresh(Nanoseconds::min(),t,1,0));
  auto a = arm(); a.state.timestamp -= Nanoseconds{1}; CHECK(!arm_snapshot_ready(a,cfg,t));
  a = arm(); a.state.joints.push_back(a.state.joints[0]); CHECK(!arm_snapshot_ready(a,cfg,t));
  auto f = frame(); f.depth_timestamp += *seconds_to_duration(0.011); CHECK(!rgbd_frame_valid(f,cfg,t));
  f = frame(); f.color_timestamp = Nanoseconds::min(); f.depth_timestamp = Nanoseconds::max(); CHECK(!rgbd_frame_valid(f,cfg,t));
  auto p = Pose3d{}; p.orientation.w = std::sqrt(0.9995); CHECK(is_valid_pose(p)); CHECK(pose_near(p,p,0.001,0.001)); CHECK(compose_pose(p,p));

  TrackedObjectState track_state = track(); TrackEvent te; te.key = key(11,2); te.kind = static_cast<TrackEventKind>(-1);
  CHECK(advance_track(track_state,te,cfg,t).status.disposition == Disposition::kRejected);
  MotionEvent me; me.key = key(2); me.kind = static_cast<MotionEventKind>(-1); CHECK(advance_motion({},me,cfg,t).status.disposition == Disposition::kRejected);
  GraspEvent ge; ge.key = key(4); ge.kind = static_cast<GraspEventKind>(-1); CHECK(advance_grasp({},ge,cfg,t).status.disposition == Disposition::kRejected);
  TaskEvent task; task.key = key(); task.kind = static_cast<TaskEventKind>(-1); CHECK(advance_task({},task,cfg,t).status.disposition == Disposition::kRejected);
  SceneEvent se; se.key = key(); se.kind = static_cast<SceneEventKind>(-1); CHECK(advance_scene({},se,cfg,t).status.disposition == Disposition::kRejected);
  me.kind = MotionEventKind::kMoveRequested; me.arm = arm(); me.request = {{time().source_now,"base",{}},0.9,0.5,*seconds_to_duration(5)};
  CHECK(advance_motion(MotionState{ArmMotionState::kIdle},me,cfg,t).status.disposition == Disposition::kRejected);
  auto fixed_goal = move_event(); fixed_goal.base_in_world.pose.position.x_m = 3;
  CHECK(advance_motion(MotionState{ArmMotionState::kIdle},fixed_goal,cfg,t).result->error_code == MoveToPoseError::kInvalidGoal);
  fixed_goal = move_event(); fixed_goal.base_in_world.reference_frame.clear();
  CHECK(advance_motion(MotionState{ArmMotionState::kIdle},fixed_goal,cfg,t).result->error_code == MoveToPoseError::kTfUnavailable);
  fixed_goal = move_event(); fixed_goal.key.cycle_id = fixed_goal.key.track_id = fixed_goal.key.session_id = 0;
  auto independent = advance_motion(MotionState{ArmMotionState::kIdle},fixed_goal,cfg,t); CHECK(independent.state.phase == ArmMotionState::kPlanning);
  CHECK(independent.request_accepted);
  MotionState exhausted; exhausted.phase = ArmMotionState::kIdle;
  exhausted.context.next_command_id = std::numeric_limits<std::uint64_t>::max();
  const auto overflow = advance_motion(exhausted,fixed_goal,cfg,t);
  CHECK(!overflow.request_accepted); CHECK(overflow.state.phase == ArmMotionState::kFault); CHECK(overflow.commands.empty());
  CHECK(overflow.result && overflow.result->error_code == MoveToPoseError::kInternalError);
  auto busy_request = move_event(50); CHECK(advance_motion(independent.state,busy_request,cfg,t).result->error_code == MoveToPoseError::kBusy);
  auto timed = independent.state; timed.context.deadline = timed.operation_deadline = time().monotonic_now;
  CHECK(!advance_motion(timed,busy_request,cfg,t).request_accepted);
  fixed_goal.kind = MotionEventKind::kCancelRequested; fixed_goal.key.sequence = 2;
  CHECK(advance_motion(independent.state,fixed_goal,cfg,t).state.pending_result == MoveToPoseError::kCanceled);
  TrackedObjectState independent_track; TrackEvent vision; vision.kind = TrackEventKind::kObservation; vision.key = key();
  vision.key.cycle_id = vision.key.operation_id = vision.key.session_id = 0; vision.observation = track().observation;
  independent_track = advance_track(independent_track,vision,cfg,t).state; CHECK(independent_track.phase == TrackPhase::kCandidate);
  vision.key.sequence = 2; vision.observation.timestamp = time(1.1).source_now; vision.observation.pose_world.position.x_m = 0.01;
  independent_track = advance_track(independent_track,vision,cfg,time(1.1)).state;
  CHECK(independent_track.phase == TrackPhase::kCandidate); CHECK(!independent_track.velocity_valid);
  vision.key.sequence = 3; vision.observation.timestamp = time(1.2).source_now; vision.observation.pose_world.position.x_m = 0.02;
  independent_track = advance_track(independent_track,vision,cfg,time(1.2)).state;
  vision.key.sequence = 4; vision.observation.timestamp = time(1.3).source_now; vision.observation.pose_world.position.x_m = 0.03;
  independent_track = advance_track(independent_track,vision,cfg,time(1.3)).state;
  CHECK(independent_track.phase == TrackPhase::kTracking); CHECK(independent_track.velocity_valid);
  fixed_goal = move_event(); fixed_goal.scene.coherent = false;
  CHECK(advance_motion(MotionState{ArmMotionState::kIdle},fixed_goal,cfg,t).status.disposition == Disposition::kRejected);
  MotionState owned; owned.phase = ArmMotionState::kPlanning; owned.active = true; owned.context.key = key(2); owned.context.last_monotonic = time(2).monotonic_now;
  me.key.sequence = 2; me.kind = MotionEventKind::kPlanSucceeded; CHECK(advance_motion(owned,me,cfg,t).status.disposition == Disposition::kRejected);
  me.kind = MotionEventKind::kCancelRequested; owned.context.last_monotonic = t.monotonic_now; owned.operation_deadline = owned.context.deadline = *seconds_to_duration(10);
  auto stopped = advance_motion(owned,me,cfg,t).state;
  CHECK(advance_motion(owned,me,cfg,time(1,10)).state.pending_result == MoveToPoseError::kCanceled);
  auto epoch = time(); epoch.epoch = 2; epoch.source_now = Nanoseconds{0};
  me.kind = MotionEventKind::kTick; me.key.sequence = 3; CHECK(advance_motion(stopped,me,cfg,epoch).state.active);
  epoch.monotonic_now = *seconds_to_duration(2); CHECK(advance_motion(stopped,me,cfg,epoch).state.phase == ArmMotionState::kFault);

  GraspState hold; hold.phase = GraspPhase::kHolding; hold.possession = Possession::kHolding; hold.scene_attached = hold.lifted = true;
  hold.context.key = key(4); hold.evidence_sequence = 1; hold.evidence_timestamp = t.source_now;
  ge.kind = GraspEventKind::kEvidenceUpdated; ge.key = key(4,2); ge.gripper = gripper_at(1.1,2); ge.gripper.empty_confirmed = false; ge.gripper.visual_follow_confirmed = false;
  auto gd = advance_grasp(hold,ge,cfg,time(1.1)); CHECK(gd.state.possession == Possession::kHolding); CHECK(gd.commands.empty());
  ge.gripper.empty_confirmed = true; gd = advance_grasp(hold,ge,cfg,time(1.1)); CHECK(gd.state.possession == Possession::kUnknown); CHECK(gd.state.awaiting_stop);
  for (const auto & command : gd.commands) {CHECK(command.kind != CommandKind::kOpenGripper);}
  ge.kind = GraspEventKind::kTick; ge.key.sequence = 3; gd = advance_grasp(gd.state,ge,cfg,time(1.1,2.1)); CHECK(gd.state.context.inhibited);
  ge.kind = GraspEventKind::kResetRequested; ge.key = key(5); ge.arm = arm_at(2.2); ge.gripper = gripper_at(2.2,3);
  gd = advance_grasp(gd.state,ge,cfg,time(2.2)); CHECK(gd.state.phase == GraspPhase::kEmpty); CHECK(!gd.state.context.inhibited); CHECK(gd.commands.empty());
  TaskState carrying; carrying.phase = TaskPhase::kTransporting; carrying.context.key = key(8); carrying.context.deadline = carrying.cycle_deadline = *seconds_to_duration(10);
  carrying.expected_operation_id = 8; carrying.grasp = hold;
  task = task_event(TaskEventKind::kFaultRaised,carrying,2); CHECK(advance_task(carrying,task,cfg,time(1,10)).state.terminal_fault);
  task = task_event(TaskEventKind::kMotionFinished,carrying,2,1.1); task.gripper = gripper_at(1.1,2);
  const auto dropped = advance_task(carrying,task,cfg,time(1.1));
  CHECK(dropped.state.phase == TaskPhase::kRecovering);
  CHECK(dropped.state.grasp.possession == Possession::kEmpty && !dropped.state.terminal_fault);
  task = task_event(TaskEventKind::kGraspUpdated,carrying,2,1.1); task.key.operation_id = 4; task.grasp_event = ge;
  task.grasp_event.kind = GraspEventKind::kEvidenceUpdated; task.grasp_event.key = task.key; task.grasp_event.gripper = gripper_at(1.1,2);
  CHECK(advance_task(carrying,task,cfg,time(1.1)).state.phase == TaskPhase::kRecovering);
  se.kind = SceneEventKind::kClockReset; auto sc = advance_scene(scene(),se,cfg,t).state; CHECK(sc.context.inhibited);
  se.kind = SceneEventKind::kResetRequested; se.arm = arm(); se.gripper = gripper(); sc = advance_scene(sc,se,cfg,t).state; CHECK(!sc.context.inhibited); CHECK(!sc.coherent); CHECK(sc.dynamic_objects.empty());
}
int main() {return run(tests);}
