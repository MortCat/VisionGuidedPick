#include "state_test_support.hpp"
#include <algorithm>
#include <set>
#include <tuple>
using namespace ptc_core;
using namespace ptc_test;

void tests()
{
  auto data = settings_data(); data.inspection.geometry = {true,"reference.fixture",0.001,0.01};
  const auto cfg = settings(data); TaskState s; MotionState motion; SceneState collision = scene();
  std::vector<Command> all; std::uint64_t sequence = 0, motion_operation = 100;
  double now = 1;
  const auto event = [&](TaskEventKind kind) {return task_event(kind,s,++sequence,now);};
  const auto step = [&](const TaskEvent & e) {
      const auto d = advance_task(s,e,cfg,time(now)); classified(d.status);
      if (d.status.disposition != Disposition::kApplied) {throw std::runtime_error("full cycle: "+d.status.reason);}
      s = d.state; all.insert(all.end(),d.commands.begin(),d.commands.end()); return d;
    };
  // Synthetic adapter: identity world/base TF. Fresh measured arrival is fed to both machines.
  // Real adapters must resolve/freeze TF and correlate their separate operation IDs.
  const auto execute = [&](Pose3d target) {
      MotionEvent e; e.kind = MotionEventKind::kDeviceReady; e.key = key(motion_operation); e.arm = arm_at(now);
      e.scene = collision; e.base_in_world = {time(now).source_now,"world",{}};
      auto d = advance_motion(motion,e,cfg,time(now)); CHECK(d.state.phase == ArmMotionState::kIdle); motion = d.state;
      e.kind = MotionEventKind::kMoveRequested; e.key = key(++motion_operation); e.request = {{time(now).source_now,"base",target},0.5,0.5,*seconds_to_duration(5)};
      d = advance_motion(motion,e,cfg,time(now)); CHECK(d.state.phase == ArmMotionState::kPlanning); motion = d.state;
      e.kind = MotionEventKind::kFollowRequested; e.key.sequence = 2; e.target_timestamp = time(now).source_now;
      CHECK(advance_motion(motion,e,cfg,time(now)).status.disposition == Disposition::kRejected); // mode exclusion
      e.kind = MotionEventKind::kPlanSucceeded; d = advance_motion(motion,e,cfg,time(now)); CHECK(d.state.phase == ArmMotionState::kMoving); motion = d.state;
      e.kind = MotionEventKind::kExecutionSucceeded; e.key.sequence = 3; e.arm.state.tcp_pose = target;
      d = advance_motion(motion,e,cfg,time(now)); CHECK(d.result && d.result->success); CHECK(d.state.phase == ArmMotionState::kIdle); motion = d.state;
      return e.arm;
    };
  step(event(TaskEventKind::kStartRequested)); CHECK(s.phase == TaskPhase::kObserving);
  auto e = event(TaskEventKind::kMotionFinished); e.motion_succeeded = true; e.arm = execute(cfg.get().motion.observation_pose); step(e);
  s.phase=TaskPhase::kPreparingPick;
  s.context.key.track_id=track(now).context.key.track_id;
  s.selected_track=track(now);
  e = event(TaskEventKind::kMotionFinished); e.motion_succeeded = true; e.grasp_event.grasp_window_valid = true; step(e); CHECK(s.phase == TaskPhase::kPicking);
  // An empty approach failure must stop and close its report before another
  // Home/observation move. Unknown possession still requires intervention.
  {
    auto failure=event(TaskEventKind::kGraspUpdated);failure.grasp_event.kind=GraspEventKind::kTick;
    failure.grasp_event.key=failure.key;failure.grasp_event.arm=arm_at(now);failure.grasp_event.gripper=gripper_at(now);
    auto expired_approach=s;expired_approach.grasp.context.deadline=time(now).monotonic_now-Nanoseconds{1};
    expired_approach.retries=data.task.max_cycle_retries; // Exercise terminal report, after bounded retries.
    const auto failed=advance_task(expired_approach,failure,cfg,time(now));
    CHECK(failed.state.phase==TaskPhase::kRecovering);CHECK(!failed.state.terminal_fault);
    CHECK(std::none_of(failed.commands.begin(),failed.commands.end(),[](const auto & command){return command.kind==CommandKind::kMoveObservation;}));
    auto stopped=task_event(TaskEventKind::kRecoveryConfirmed,failed.state,++sequence,now);
    stopped.arm.stopped=false;CHECK(advance_task(failed.state,stopped,cfg,time(now)).status.disposition==Disposition::kRejected);
    stopped.arm.stopped=true;stopped.gripper.empty_confirmed=false;
    CHECK(advance_task(failed.state,stopped,cfg,time(now)).state.phase==TaskPhase::kFaulted);
    stopped.gripper.empty_confirmed=true;const auto closed=advance_task(failed.state,stopped,cfg,time(now));
    CHECK(closed.state.phase==TaskPhase::kReporting);
    auto ack=task_event(TaskEventKind::kReportAcknowledged,closed.state,++sequence,now);
    const auto idle=advance_task(closed.state,ack,cfg,time(now));CHECK(idle.state.phase==TaskPhase::kWaitingReady);
    auto restart=task_event(TaskEventKind::kStartRequested,idle.state,++sequence,now);++restart.key.cycle_id;restart.key.track_id=0;
    const auto next=advance_task(idle.state,restart,cfg,time(now));CHECK(next.state.phase==TaskPhase::kObserving);
    CHECK(std::any_of(next.commands.begin(),next.commands.end(),[](const auto & command){return command.kind==CommandKind::kMoveObservation;}));
    auto unknown=expired_approach;unknown.grasp.possession=Possession::kUnknown;
    CHECK(advance_task(unknown,failure,cfg,time(now)).state.terminal_fault);
  }
  // A canceled close with newer measured open fingers is a missed grasp,
  // not unknown possession. Stop, measure Home, report, then take another cycle.
  {
    auto closing=s;closing.grasp.phase=GraspPhase::kClosing;closing.retries=data.task.max_cycle_retries;
    closing.grasp.evidence_timestamp=time(now).source_now-Nanoseconds{100000000};
    auto miss=event(TaskEventKind::kGraspUpdated);miss.grasp_event.key=miss.key;
    miss.grasp_event.kind=GraspEventKind::kCloseFailed;miss.grasp_event.arm=arm_at(now);
    miss.grasp_event.gripper=gripper_at(now);
    miss.grasp_event.gripper.evidence_sequence=closing.grasp.evidence_sequence+1;
    const auto missed=advance_task(closing,miss,cfg,time(now));
    CHECK(missed.state.phase==TaskPhase::kRecovering);CHECK(!missed.state.terminal_fault);
    CHECK(missed.state.grasp.possession==Possession::kEmpty);
    auto stopped=task_event(TaskEventKind::kRecoveryConfirmed,missed.state,++sequence,now);
    stopped.arm.state.tcp_pose=cfg.get().motion.observation_pose;stopped.arm.state.tcp_pose.position.x_m+=.1;
    const auto home=advance_task(missed.state,stopped,cfg,time(now));
    CHECK(home.state.phase==TaskPhase::kRecovering);CHECK(home.state.recovery_home_command_id!=0);
    auto arrived=task_event(TaskEventKind::kMotionFinished,home.state,++sequence,now);
    arrived.motion_succeeded=true;arrived.arm.state.tcp_pose=cfg.get().motion.observation_pose;
    const auto reported=advance_task(home.state,arrived,cfg,time(now));CHECK(reported.state.phase==TaskPhase::kReporting);
    auto ack=task_event(TaskEventKind::kReportAcknowledged,reported.state,++sequence,now);
    const auto idle=advance_task(reported.state,ack,cfg,time(now));CHECK(idle.state.phase==TaskPhase::kWaitingReady);
    auto next=task_event(TaskEventKind::kStartRequested,idle.state,++sequence,now);++next.key.cycle_id;next.key.track_id=0;
    CHECK(advance_task(idle.state,next,cfg,time(now)).state.phase==TaskPhase::kObserving);
    miss.grasp_event.gripper.evidence_sequence=closing.grasp.evidence_sequence;
    CHECK(advance_task(closing,miss,cfg,time(now)).state.terminal_fault);
    ++miss.grasp_event.gripper.evidence_sequence;miss.grasp_event.gripper.empty_confirmed=false;
    CHECK(advance_task(closing,miss,cfg,time(now)).state.terminal_fault);
  }
  MotionEvent follow; follow.kind = MotionEventKind::kFollowRequested; follow.key = key(++motion_operation);
  follow.arm = arm(); follow.target_timestamp = time().source_now; follow.linear_velocity = track().velocity_mps;
  follow.scene = collision;
  auto md = advance_motion(motion,follow,cfg,time()); CHECK(md.state.phase == ArmMotionState::kFollowing); motion = md.state;
  const auto child_event = [&](GraspEventKind kind) {
      auto child = event(TaskEventKind::kGraspUpdated); child.grasp_event.kind = kind; child.grasp_event.key = child.key;
      child.grasp_event.arm = arm_at(now); child.grasp_event.gripper = gripper_at(now);
      child.grasp_event.track = track(now); child.grasp_event.grasp_window_valid = true; child.grasp_event.relative_speed_mps = 0;
      child.grasp_event.tcp_world = {time(now).source_now,"world",{}}; return child;
    };
  e = child_event(GraspEventKind::kApproachCompleted); e.grasp_event.arm.stopped = false; step(e); CHECK(s.grasp.phase == GraspPhase::kClosing);
  step(child_event(GraspEventKind::kCloseAccepted)); CHECK(s.grasp.possession == Possession::kUnknown);
  for (std::uint64_t i = 2; i <= 3; ++i) {
    now = 1+static_cast<double>(i)*0.01; e = child_event(GraspEventKind::kEvidenceUpdated);
    e.grasp_event.gripper = gripper_at(now,i); e.grasp_event.gripper.empty_confirmed = false;
    e.grasp_event.gripper.width_m = 0.03; e.grasp_event.gripper.visual_follow_confirmed = true; step(e);
  }
  CHECK(s.grasp.phase == GraspPhase::kHolding); CHECK(!s.grasp.lifted);
  follow.kind = MotionEventKind::kCancelRequested; follow.key.sequence = 2;
  md = advance_motion(motion,follow,cfg,time(now)); CHECK(md.state.active); CHECK(md.state.phase == ArmMotionState::kStopping); motion = md.state;
  follow.kind = MotionEventKind::kStopConfirmed; follow.key.sequence = 3; follow.arm = arm_at(now);
  md = advance_motion(motion,follow,cfg,time(now)); CHECK(!md.state.active); motion = md.state;
  SceneEvent se; se.kind = SceneEventKind::kAttachRequested; se.key = key(10,2);
  auto sd = advance_scene(collision,se,cfg,time(now)); CHECK(sd.state.attachment == AttachmentPhase::kAttachPending); collision = sd.state;
  se.kind = SceneEventKind::kAttachConfirmed; se.key.sequence = 3; se.command_id = collision.pending_command_id; se.scene_revision = collision.revision;
  collision = advance_scene(collision,se,cfg,time(now)).state; CHECK(scene_ready(collision,cfg,time(now)));
  e = child_event(GraspEventKind::kSceneAttached); e.scene = collision; e.grasp_event.scene_confirmed = true;
  auto lift = step(e); CHECK(lift.commands.size() == 1); CHECK(lift.commands[0].kind == CommandKind::kLift);
  e = child_event(GraspEventKind::kLiftCompleted); e.scene = collision; e.grasp_event.arm = execute(lift.commands[0].target);
  e.grasp_event.tcp_world.pose = e.grasp_event.arm.state.tcp_pose; const auto before_lift = s; const auto lift_done = e;
  const auto has_target = [](const TaskDecision & d, CommandKind kind, const Pose3d & target) {
      return std::any_of(d.commands.begin(),d.commands.end(),[&](const auto & cmd) {return cmd.kind == kind && cmd.reference_frame == "base" &&
        pose_near(cmd.target,target,1e-12,1e-9);});
    };
  {
    // Moving-demo transfer: configured bin positions with the measured
    // robot_base held orientation, for both commands and arrival checks.
    auto demo_data = data; demo_data.runtime.operation_mode = "assumed_ng_demo"; demo_data.inspection.inference.backend = "assumed_ng";
    demo_data.grasp.moving_intercept = true;
    const auto demo_cfg = settings(demo_data);
    const Quaterniond held{0,0,0.3826834323650898,0.9238795325112867}; // 45 deg yaw
    const auto & approach = cfg.get().grasp.bin_approach_pose; const auto & release = cfg.get().grasp.bin_release_pose;
    auto tilted = lift_done; tilted.grasp_event.arm.state.tcp_pose.orientation = held;
    const auto demo_lift = advance_task(before_lift,tilted,demo_cfg,time(now));
    CHECK(demo_lift.state.phase == TaskPhase::kTransporting);
    CHECK(has_target(demo_lift,CommandKind::kMoveBinApproach,{approach.position,held}));
    auto arrival = task_event(TaskEventKind::kMotionFinished,demo_lift.state,++sequence,now); arrival.motion_succeeded = true; arrival.scene = collision; arrival.gripper.empty_confirmed = false;
    arrival.arm.state.tcp_pose = approach; // Configured orientation is not the held one.
    CHECK(advance_task(demo_lift.state,arrival,demo_cfg,time(now)).state.phase == TaskPhase::kRecovering);
    arrival.arm.state.tcp_pose = {approach.position,{-held.x,-held.y,-held.z,-held.w}}; // q and -q
    const auto placing = advance_task(demo_lift.state,arrival,demo_cfg,time(now));
    CHECK(placing.state.phase == TaskPhase::kPlacing);
    CHECK(has_target(placing,CommandKind::kMoveBinRelease,{release.position,held}));
    // Formal mode ignores the held orientation and keeps the configured pose.
    const auto formal_lift = advance_task(before_lift,tilted,cfg,time(now));
    CHECK(formal_lift.state.phase == TaskPhase::kTransporting);
    CHECK(has_target(formal_lift,CommandKind::kMoveBinApproach,approach));
    auto formal_arrival = task_event(TaskEventKind::kMotionFinished,formal_lift.state,++sequence,now); formal_arrival.motion_succeeded = true;
    formal_arrival.scene = collision; formal_arrival.gripper.empty_confirmed = false; formal_arrival.arm.state.tcp_pose = {approach.position,held};
    CHECK(advance_task(formal_lift.state,formal_arrival,cfg,time(now)).state.phase == TaskPhase::kRecovering);
    formal_arrival.arm.state.tcp_pose = approach;
    CHECK(advance_task(formal_lift.state,formal_arrival,cfg,time(now)).state.phase == TaskPhase::kPlacing);
  }
  CHECK(has_target(step(e),CommandKind::kMoveBinApproach,cfg.get().grasp.bin_approach_pose)); CHECK(s.phase == TaskPhase::kTransporting);
  e = event(TaskEventKind::kMotionFinished); e.motion_succeeded = true; e.arm = execute(cfg.get().grasp.bin_approach_pose); e.scene = collision; e.gripper.empty_confirmed = false; step(e); CHECK(s.phase == TaskPhase::kPlacing);
  e = event(TaskEventKind::kMotionFinished); e.motion_succeeded = true; e.arm = execute(cfg.get().grasp.bin_release_pose); e.scene = collision; e.gripper.empty_confirmed = false;
  auto slipped=e; slipped.gripper=gripper_at(now,s.grasp.evidence_sequence+1);
  slipped.gripper.empty_confirmed=false; slipped.gripper.contact_confirmed=true; slipped.gripper.width_m=.005;
  slipped.gripper.timestamp=s.grasp.evidence_timestamp+Nanoseconds{1};
  slipped.grasp_event.expected_contact_span_m=.04;
  const auto lost_release=advance_task(s,slipped,cfg,time(now));
  CHECK(lost_release.state.outcome==CycleOutcome::kFailed);
  CHECK(lost_release.commands.size()==1 && lost_release.commands.front().kind==CommandKind::kOpenGripper);
  step(e); CHECK(s.grasp.phase == GraspPhase::kReleasing);
  e = child_event(GraspEventKind::kReleaseCompleted); e.scene = collision;
  const auto lost_empty=advance_task(lost_release.state,e,cfg,time(now));
  CHECK(lost_empty.state.grasp.phase==GraspPhase::kEmpty);
  step(e); CHECK(s.grasp.phase == GraspPhase::kEmpty);
  se.kind = SceneEventKind::kDetachRequested; se.key.sequence = 4; collision = advance_scene(collision,se,cfg,time(now)).state;
  se.kind = SceneEventKind::kDetachConfirmed; se.key.sequence = 5; se.command_id = collision.pending_command_id; se.scene_revision = collision.revision;
  collision = advance_scene(collision,se,cfg,time(now)).state; CHECK(collision.attachment == AttachmentPhase::kDetached);
  e = event(TaskEventKind::kPlaceConfirmed); e.scene = collision;
  const auto lost_report=advance_task(lost_empty.state,e,cfg,time(now));
  CHECK(lost_report.state.phase==TaskPhase::kReporting);
  CHECK(lost_report.state.outcome==CycleOutcome::kFailed);
  step(e); CHECK(s.phase == TaskPhase::kReporting); CHECK(s.outcome == CycleOutcome::kDefectPicked);
  step(event(TaskEventKind::kReportAcknowledged)); CHECK(s.phase == TaskPhase::kWaitingReady);
  const auto duplicate = advance_task(s,e,cfg,time(now)); CHECK(duplicate.commands.empty());
  const auto count = [&](CommandKind kind) {return std::count_if(all.begin(),all.end(),[&](const auto & cmd) {return cmd.kind == kind;});};
  CHECK(count(CommandKind::kCloseGripper) == 1); CHECK(count(CommandKind::kOpenGripper) == 1); CHECK(count(CommandKind::kReport) == 1);
  std::set<std::tuple<std::string,std::uint64_t,std::uint64_t,std::uint64_t,std::uint64_t>> ids;
  for (const auto & cmd : all) {CHECK(ids.emplace(cmd.producer,cmd.key.epoch,cmd.key.cycle_id,cmd.key.operation_id,cmd.command_id).second); CHECK(cmd.deadline <= s.cycle_deadline);}
  std::cout << "Full logical cycle verified: tracking input, inspection, follow/stop arbitration, evidence, scene attach, lift, transport, release, one report.\n";
}
int main() {return run(tests);}
