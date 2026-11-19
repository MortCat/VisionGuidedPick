#include "state_test_support.hpp"
using namespace ptc_core;
using namespace ptc_test;

void tests()
{
  const auto cfg = settings(); GraspState s; CHECK(s.possession == Possession::kUnknown);
  GraspEvent e; e.kind = GraspEventKind::kApproachRequested; e.key = key(4); e.arm = arm(); e.gripper = gripper(); e.track = track(); e.grasp_window_valid = true;
  CHECK(advance_grasp(s,e,cfg,time()).commands.empty());
  e.kind = GraspEventKind::kResetRequested; e.gripper.stopped = false; CHECK(advance_grasp(s,e,cfg,time()).status.disposition == Disposition::kRejected);
  e.gripper.stopped = true; auto d = advance_grasp(s,e,cfg,time()); CHECK(d.state.phase == GraspPhase::kEmpty); CHECK(d.commands.empty()); s = d.state;
  e.kind = GraspEventKind::kApproachRequested; e.key = key(5); d = advance_grasp(s,e,cfg,time()); CHECK(d.state.phase == GraspPhase::kApproaching); CHECK(d.commands[0].kind == CommandKind::kStartFollow); s = d.state;
  e.kind = GraspEventKind::kApproachCompleted; e.key.sequence = 2; e.relative_speed_mps = 0; e.tcp_world = {time().source_now,"world",{}};
  auto fast = e; fast.relative_speed_mps = 1;
  const auto rejected_close=advance_grasp(s,fast,cfg,time());
  CHECK(rejected_close.state.phase == GraspPhase::kFailed);
  CHECK(rejected_close.status.reason == "relative speed invalid or above limit");
  CHECK(rejected_close.commands.size() == 2);
  CHECK(rejected_close.commands.front().kind == CommandKind::kStopArm);
  CHECK(rejected_close.commands.back().kind == CommandKind::kStopGripper);
  auto demo_data=settings_data();demo_data.runtime.operation_mode="assumed_ng_demo";demo_data.inspection.inference.backend="assumed_ng";demo_data.grasp.moving_intercept=true;
  demo_data.inspection.view_target_position_tolerance_m=.03;demo_data.inspection.view_target_orientation_tolerance_rad=.35;demo_data.grasp.min_confirmations=3;
  demo_data.grasp.verification_probe_m=.04;
  const auto demo_cfg=settings(demo_data);auto demo_contact=e;demo_contact.arm.state.motion_state=ArmMotionState::kFollowing;demo_contact.arm.stopped=false;
  demo_contact.relative_speed_mps=.2;demo_contact.tcp_world.pose.position.x_m=.03;
  CHECK(advance_grasp(s,demo_contact,demo_cfg,time()).state.phase==GraspPhase::kFailed);
  demo_contact.relative_speed_mps=0;demo_contact.tcp_world=e.tcp_world;
  const auto attempted=advance_grasp(s,demo_contact,demo_cfg,time());CHECK(attempted.state.phase==GraspPhase::kClosing);
  CHECK(attempted.commands.front().kind==CommandKind::kCloseGripper);CHECK(attempted.state.possession!=Possession::kHolding);
  auto close_reply=demo_contact;
  close_reply.kind=GraspEventKind::kCloseAccepted;close_reply.key.sequence++;
  CHECK(advance_grasp(attempted.state,close_reply,demo_cfg,time(1.6,2.3)).state.phase==GraspPhase::kVerifying);
  CHECK(advance_grasp(attempted.state,close_reply,demo_cfg,time(2.0,2.3)).state.awaiting_stop);
  CHECK(advance_grasp(attempted.state,close_reply,demo_cfg,time(1.6,4.0)).state.awaiting_stop);
  auto paused=time(1.6,2.3);paused.paused=true;
  CHECK(advance_grasp(attempted.state,close_reply,demo_cfg,paused).status.reason=="clock invalidated grasp");
  auto hardware_data=demo_data;hardware_data.runtime.use_sim_time=false;
  const auto hardware_cfg=settings(hardware_data);
  const auto hardware_attempt=advance_grasp(s,demo_contact,hardware_cfg,time());
  CHECK(hardware_attempt.state.phase==GraspPhase::kClosing);
  CHECK(advance_grasp(hardware_attempt.state,close_reply,hardware_cfg,time(1.6,2.3)).state.awaiting_stop);
  // Relax only an existing valid, bounded demo recipe. Ordinary/formal
  // contact keeps its original speed/pose gates; no Holding is manufactured.
  auto trial=demo_contact;trial.relative_speed_mps=.2;
  trial.motion_prediction=GraspMotionPrediction{trial.key,{time().source_now,"world",{}},time().source_now};
  trial.tcp_world.pose.position.x_m=.02;
  trial.tcp_world.pose.orientation={0,0,std::sin(.1),std::cos(.1)};
  auto relaxed=advance_grasp(s,trial,demo_cfg,time());
  CHECK(relaxed.state.phase==GraspPhase::kClosing);CHECK(relaxed.state.possession!=Possession::kHolding);
  // In the moving demo, occlusion during lift is expected. Fresh controller
  // contact plus partial aperture can confirm a grasp without a new camera frame.
  auto demo_closed=trial;demo_closed.kind=GraspEventKind::kCloseAccepted;demo_closed.key.sequence=3;
  demo_closed.gripper=gripper_at(1.0,2);demo_closed.gripper.empty_confirmed=false;
  auto demo_verifying=advance_grasp(relaxed.state,demo_closed,demo_cfg,time(1.0));
  CHECK(demo_verifying.state.phase==GraspPhase::kVerifying);
  auto contact=demo_closed;contact.kind=GraspEventKind::kEvidenceUpdated;contact.key.sequence=4;
  contact.gripper=gripper_at(1.1,3);contact.gripper.width_m=.03;contact.gripper.empty_confirmed=false;
  contact.gripper.contact_confirmed=true; contact.motion_prediction->object_world.timestamp=time(1.1).source_now;contact.gripper.visual_follow_confirmed=false;
  contact.expected_contact_span_m=.04;
  auto near_closed=contact;
  near_closed.gripper.width_m=.006;
  const auto mismatched=advance_grasp(demo_verifying.state,near_closed,demo_cfg,time(1.1));
  CHECK(mismatched.state.possession==Possession::kUnknown);
  CHECK(mismatched.state.awaiting_stop);
  CHECK(mismatched.commands.size()==2);
  CHECK(mismatched.commands.front().kind==CommandKind::kStopArm);
  auto contact_check=advance_grasp(demo_verifying.state,contact,demo_cfg,time(1.1));
  CHECK(contact_check.state.confirmations==1);CHECK(contact_check.state.phase==GraspPhase::kVerifying);
  contact.key.sequence=5;contact.gripper=gripper_at(1.2,4);contact.gripper.width_m=.03;
  contact.gripper.empty_confirmed=false;contact.gripper.contact_confirmed=true; contact.motion_prediction->object_world.timestamp=time(1.2).source_now;
  contact_check=advance_grasp(contact_check.state,contact,demo_cfg,time(1.2));
  CHECK(contact_check.state.confirmations==2);
  contact.key.sequence=6;contact.gripper=gripper_at(1.3,5);contact.gripper.width_m=.03;
  contact.gripper.empty_confirmed=false;contact.gripper.contact_confirmed=true; contact.motion_prediction->object_world.timestamp=time(1.3).source_now;
  contact_check=advance_grasp(contact_check.state,contact,demo_cfg,time(1.3));
  CHECK(contact_check.state.phase==GraspPhase::kHolding);CHECK(contact_check.state.possession==Possession::kHolding);
  CHECK(contact_check.commands.size()==1);if(!contact_check.commands.empty())CHECK(contact_check.commands.front().kind==CommandKind::kAttachObject);
  // A measured full recipe lift must not request the same lift a second time.
  auto lifted_contact=contact;lifted_contact.kind=GraspEventKind::kSceneAttached;lifted_contact.key.sequence=7;
  lifted_contact.arm=arm_at(1.3);lifted_contact.scene_confirmed=true;
  lifted_contact.tcp_world.timestamp=time(1.3).source_now;
  lifted_contact.motion_prediction->measured_lift_distance_m=demo_data.grasp.lift_distance_m;
  const auto no_extra_lift=advance_grasp(contact_check.state,lifted_contact,demo_cfg,time(1.3));
  CHECK(no_extra_lift.state.scene_attached);
  CHECK(no_extra_lift.commands.size()==1);
  CHECK(no_extra_lift.commands.front().kind==CommandKind::kLift);
  CHECK(pose_near(no_extra_lift.commands.front().target,lifted_contact.tcp_world.pose,1e-8,1e-8));
  CHECK(pose_near(no_extra_lift.state.lift_target_world,lifted_contact.tcp_world.pose,1e-8,1e-8));
  // Verification performs only the configured probe. Holding then requests
  // the remaining full lift, without repeating the already measured travel.
  lifted_contact.motion_prediction->measured_lift_distance_m=demo_data.grasp.verification_probe_m;
  const auto remaining_lift=advance_grasp(contact_check.state,lifted_contact,demo_cfg,time(1.3));
  CHECK(remaining_lift.state.scene_attached);
  CHECK(remaining_lift.commands.size()==1);
  CHECK(remaining_lift.commands.front().kind==CommandKind::kLift);
  CHECK(std::abs(remaining_lift.state.lift_target_world.position.z_m-lifted_contact.tcp_world.pose.position.z_m-
    (demo_data.grasp.lift_distance_m-demo_data.grasp.verification_probe_m))<1e-8);
  lifted_contact.motion_prediction->measured_lift_distance_m=0.;
  const auto full_lift=advance_grasp(contact_check.state,lifted_contact,demo_cfg,time(1.3));
  CHECK(std::abs(full_lift.state.lift_target_world.position.z_m-lifted_contact.tcp_world.pose.position.z_m-demo_data.grasp.lift_distance_m)<1e-8);
  // The live Follow stop clears the motion prediction before scene attachment.
  auto verified=contact;
  verified.tcp_world.timestamp=time(1.3).source_now;
  verified.tcp_world.pose.position.z_m=.10;
  verified.motion_prediction->measured_lift_distance_m=demo_data.grasp.lift_distance_m;
  auto one_confirmation=demo_data;
  one_confirmation.grasp.min_confirmations=1;
  const auto one_confirmation_cfg=settings(one_confirmation);
  const auto holding=advance_grasp(demo_verifying.state,verified,one_confirmation_cfg,time(1.3));
  CHECK(holding.state.phase==GraspPhase::kHolding);
  CHECK(holding.state.probe_origin_world_z_m.has_value());
  lifted_contact.motion_prediction.reset();
  lifted_contact.tcp_world=verified.tcp_world;
  const auto retained_lift=advance_grasp(holding.state,lifted_contact,one_confirmation_cfg,time(1.3));
  CHECK(retained_lift.state.scene_attached);
  CHECK(pose_near(retained_lift.state.lift_target_world,lifted_contact.tcp_world.pose,1e-8,1e-8));
  lifted_contact.tcp_world.pose.position.z_m-=.009;
  const auto settled_lift=advance_grasp(holding.state,lifted_contact,one_confirmation_cfg,time(1.3));
  CHECK(std::abs(settled_lift.state.lift_target_world.position.z_m-.10)<1e-8);
  // Unattached commissioning contact is not irrevocable physical possession.
  const double handoff_deadline=std::chrono::duration<double>(contact_check.state.context.deadline).count();
  CHECK(handoff_deadline>=1.3+demo_data.motion.stop_confirmation_timeout_sec+demo_data.motion.planning_timeout_sec-1e-8);
  auto timeout_contact=contact;timeout_contact.kind=GraspEventKind::kTick;timeout_contact.key.sequence++;
  const auto failed_contact=advance_grasp(contact_check.state,timeout_contact,demo_cfg,time(handoff_deadline+.01));
  CHECK(failed_contact.state.faulted);CHECK(failed_contact.state.possession==Possession::kUnknown);
  auto attached_contact=contact_check.state;attached_contact.scene_attached=true;
  CHECK(advance_grasp(attached_contact,timeout_contact,demo_cfg,time(handoff_deadline+.01)).state.possession==Possession::kHolding);
  CHECK(advance_grasp(contact_check.state,timeout_contact,cfg,time(handoff_deadline+.01)).state.possession==Possession::kHolding);
  trial.motion_prediction.reset();CHECK(advance_grasp(s,trial,demo_cfg,time()).state.phase==GraspPhase::kFailed);
  trial.motion_prediction=GraspMotionPrediction{trial.key,{time().source_now,"world",{}},time().source_now};
  trial.tcp_world.pose.position.x_m=.2;
  // Timed simulated recipe records even large alignment errors, but does not
  // manufacture Holding. Ordinary/formal operation still rejects this pose.
  const auto off_center=advance_grasp(s,trial,demo_cfg,time());
  CHECK(off_center.state.phase==GraspPhase::kClosing);CHECK(off_center.state.possession!=Possession::kHolding);
  CHECK(advance_grasp(s,trial,cfg,time()).state.phase==GraspPhase::kFailed);
  auto invalid_tcp=trial;invalid_tcp.tcp_world.pose.orientation={0,0,0,0};
  CHECK(advance_grasp(s,invalid_tcp,demo_cfg,time()).state.phase==GraspPhase::kFailed);
  demo_contact.arm.state.motion_state=ArmMotionState::kIdle;CHECK(advance_grasp(s,demo_contact,demo_cfg,time()).state.phase==GraspPhase::kFailed);
  d = advance_grasp(s,e,cfg,time()); CHECK(d.state.phase == GraspPhase::kClosing); CHECK(d.commands[0].kind == CommandKind::kCloseGripper); s = d.state;
  e.kind = GraspEventKind::kCloseAccepted; e.key.sequence = 3; d = advance_grasp(s,e,cfg,time()); CHECK(d.state.phase == GraspPhase::kVerifying); CHECK(d.state.possession == Possession::kUnknown); s = d.state;
  // A fresh, stopped, fully closed simulated gripper with no contact is a miss.
  auto empty_close=e;empty_close.kind=GraspEventKind::kEvidenceUpdated;empty_close.key.sequence=4;
  empty_close.gripper=gripper_at(1.01,2);empty_close.gripper.empty_confirmed=true;
  empty_close.gripper.contact_confirmed=false;empty_close.gripper.width_m=0.0002;
  const auto missed=advance_grasp(s,empty_close,demo_cfg,time(1.01));
  CHECK(missed.state.phase==GraspPhase::kFailed);CHECK(missed.state.possession==Possession::kEmpty);
  CHECK(missed.state.awaiting_stop);
  // Contact evidence must not be turned into an empty miss.
  empty_close.gripper.contact_confirmed=true;
  CHECK(advance_grasp(s,empty_close,demo_cfg,time(1.01)).state.possession!=Possession::kEmpty);
  // No new camera frame: the existing verification deadline must still stop
  // the attempt, without inventing Holding/Empty or attaching the object.
  auto no_image=e;no_image.kind=GraspEventKind::kTick;no_image.key.sequence=4;
  auto waiting=advance_grasp(s,no_image,cfg,time(1.01));
  CHECK(waiting.state.phase==GraspPhase::kVerifying);CHECK(waiting.state.confirmations==0);
  no_image.key.sequence++;
  const double overdue=std::chrono::duration<double>(s.context.deadline).count()+.01;
  const auto timed_out=advance_grasp(waiting.state,no_image,cfg,time(overdue));
  CHECK(timed_out.state.awaiting_stop);CHECK(timed_out.state.faulted);
  CHECK(timed_out.state.possession==Possession::kUnknown);CHECK(timed_out.commands.size()==2);
  CHECK(timed_out.commands[0].kind==CommandKind::kStopArm);
  CHECK(timed_out.commands[1].kind==CommandKind::kStopGripper);
  e.kind = GraspEventKind::kEvidenceUpdated; e.key.sequence = 4; e.gripper = gripper_at(1.01,2); e.gripper.empty_confirmed = false; e.gripper.width_m = 0.03;
  CHECK(advance_grasp(s,e,cfg,time(1.01)).state.confirmations == 0); e.gripper.visual_follow_confirmed = true;
  d = advance_grasp(s,e,cfg,time(1.01)); CHECK(d.state.confirmations == 1); s = d.state;
  e.key.sequence = 5; CHECK(advance_grasp(s,e,cfg,time(1.01)).status.disposition == Disposition::kRejected);
  e.gripper = gripper_at(1.02,3); e.gripper.width_m = 0.03; e.gripper.empty_confirmed = false; e.gripper.visual_follow_confirmed = true;
  d = advance_grasp(s,e,cfg,time(1.02)); CHECK(d.state.phase == GraspPhase::kHolding); CHECK(d.commands[0].kind == CommandKind::kAttachObject); s = d.state;
  e.kind = GraspEventKind::kSceneAttached; e.key.sequence = 6; e.scene_confirmed = true; e.arm = arm_at(1.03); e.tcp_world.timestamp = time(1.03).source_now;
  d = advance_grasp(s,e,cfg,time(1.03)); CHECK(d.commands[0].kind == CommandKind::kLift); CHECK(d.commands[0].target.position.z_m == 0.1); s = d.state;
  e.kind = GraspEventKind::kLiftCompleted; e.key.sequence = 7; CHECK(advance_grasp(s,e,cfg,time(1.03)).status.disposition == Disposition::kRejected);
  e.tcp_world.pose.position.z_m = 0.1; const auto lifting = s; d = advance_grasp(s,e,cfg,time(1.03)); CHECK(d.state.lifted); s = d.state;
  CHECK(s.held_tcp_orientation_base);
  CHECK(s.context.deadline == Nanoseconds{0});
  {
    auto tick=e;tick.kind=GraspEventKind::kTick;tick.key.sequence++;
    const double former_deadline=std::chrono::duration<double>(lifting.context.deadline).count();
    const auto held=advance_grasp(s,tick,cfg,time(former_deadline+.01));
    CHECK(held.state.phase==GraspPhase::kHolding);CHECK(!held.state.faulted);
    CHECK(held.commands.empty());
  }
  {
    // The moving demo keeps the measured robot_base held orientation for bin
    // transfer; position and tolerances stay configured. Formal keeps config.
    const double k=1.0004,qz=0.3826834323650898,qw=0.9238795325112867; // 45 deg yaw, unnormalized sample.
    auto tilted=e;tilted.arm.state.tcp_pose.orientation={0,0,k*qz,k*qw};
    const auto held=advance_grasp(lifting,tilted,cfg,time(1.03)).state;
    CHECK(held.lifted);CHECK(held.held_tcp_orientation_base);
    const auto q=*held.held_tcp_orientation_base;
    CHECK(std::abs(q.x)<1e-12 && std::abs(q.y)<1e-12 && std::abs(q.z-qz)<1e-12 && std::abs(q.w-qw)<1e-12);
    const auto & bin=cfg.get().grasp.bin_release_pose;const auto & m=cfg.get().motion;
    const auto demo_target=bin_target_pose(bin,held,demo_cfg);
    CHECK(demo_target.position.x_m==bin.position.x_m && demo_target.position.y_m==bin.position.y_m && demo_target.position.z_m==bin.position.z_m);
    CHECK(demo_target.orientation.z==q.z && demo_target.orientation.w==q.w);
    CHECK(!pose_near(demo_target,bin,m.position_tolerance_m,m.orientation_tolerance_rad));
    const auto formal_target=bin_target_pose(bin,held,cfg);
    CHECK(formal_target.orientation.z==bin.orientation.z && formal_target.orientation.w==bin.orientation.w);
    // Release validation reuses the same target; q and -q are one orientation.
    auto release=tilted;release.kind=GraspEventKind::kReleaseRequested;release.key=key(5,8);release.arm=arm_at(1.04);release.scene_confirmed=true;
    release.arm.state.tcp_pose={bin.position,{-q.x,-q.y,-q.z,-q.w}};
    CHECK(advance_grasp(held,release,demo_cfg,time(1.04)).state.phase==GraspPhase::kReleasing);
    CHECK(advance_grasp(held,release,cfg,time(1.04)).status.disposition==Disposition::kRejected);
    release.arm.state.tcp_pose=bin;
    CHECK(advance_grasp(held,release,demo_cfg,time(1.04)).status.disposition==Disposition::kRejected);
    CHECK(advance_grasp(held,release,cfg,time(1.04)).state.phase==GraspPhase::kReleasing);
  }
  e.kind = GraspEventKind::kCancelRequested; e.key.sequence = 8; d = advance_grasp(s,e,cfg,time(1.03)); CHECK(d.state.possession == Possession::kHolding); CHECK(d.commands.size() == 2);
  for (const auto & cmd : d.commands) {CHECK(cmd.kind != CommandKind::kOpenGripper);}
  auto canceled = d.state; e.kind = GraspEventKind::kStopConfirmed; e.key.sequence = 9; e.gripper.stopped = false;
  CHECK(advance_grasp(canceled,e,cfg,time(1.03)).state.awaiting_stop); e.gripper.stopped = true;
  d = advance_grasp(canceled,e,cfg,time(1.03)); CHECK(!d.state.awaiting_stop); CHECK(d.state.possession == Possession::kHolding);
  e.kind = GraspEventKind::kResetRequested; e.key = key(6); CHECK(advance_grasp(d.state,e,cfg,time(1.03)).status.disposition == Disposition::kRejected);
  // Explicit manual removal, newer measured Empty, and both stops may reconcile possession. It sends no open command.
  e.gripper = gripper_at(1.04,4); d = advance_grasp(d.state,e,cfg,time(1.04)); CHECK(d.state.possession == Possession::kEmpty); CHECK(d.commands.empty());
  CHECK(!d.state.held_tcp_orientation_base); // Manual reset leaves no held pose for a later cycle.
  e.kind = GraspEventKind::kReleaseRequested; e.key = key(5,8); e.arm = arm_at(1.04); e.scene_confirmed = true;
  d = advance_grasp(s,e,cfg,time(1.04)); CHECK(d.state.phase == GraspPhase::kReleasing); CHECK(d.commands[0].kind == CommandKind::kOpenGripper);
  auto releasing = d.state; e.kind = GraspEventKind::kReleaseCompleted; e.key.sequence = 9; e.gripper.empty_confirmed = false;
  CHECK(advance_grasp(releasing,e,cfg,time(1.04)).status.disposition == Disposition::kRejected); e.gripper.empty_confirmed = true;
  auto release_closed=e;release_closed.gripper.width_m=cfg.get().grasp.close_width_m;
  CHECK(advance_grasp(releasing,release_closed,cfg,time(1.04)).status.disposition==Disposition::kRejected);
  auto release_in_motion=e;release_in_motion.arm.stopped=false;
  const auto release_waiting=advance_grasp(releasing,release_in_motion,cfg,time(1.04));
  CHECK(release_waiting.status.disposition==Disposition::kRejected);
  CHECK(release_waiting.state.phase==GraspPhase::kReleasing);
  CHECK(release_waiting.state.context.deadline==releasing.context.deadline);
  release_in_motion.arm=arm_at(1.05);release_in_motion.gripper=gripper_at(1.05,5);
  release_in_motion.key.sequence=10;
  const auto release_after_stop=advance_grasp(release_waiting.state,release_in_motion,cfg,time(1.05));
  CHECK(release_after_stop.state.phase==GraspPhase::kEmpty);
  CHECK(release_after_stop.commands.size()==1 && release_after_stop.commands.front().kind==CommandKind::kDetachObject);
  // A new grasp starts without the previous cycle's held orientation.
  CHECK(release_after_stop.state.held_tcp_orientation_base);
  GraspEvent next_grasp;next_grasp.kind=GraspEventKind::kApproachRequested;next_grasp.key=key(7);next_grasp.arm=arm_at(1.05);
  next_grasp.gripper=gripper_at(1.05,6);next_grasp.track=track(1.05);next_grasp.grasp_window_valid=true;
  const auto next_cycle=advance_grasp(release_after_stop.state,next_grasp,cfg,time(1.05));
  CHECK(next_cycle.state.phase==GraspPhase::kApproaching);CHECK(!next_cycle.state.held_tcp_orientation_base);
  d = advance_grasp(releasing,e,cfg,time(1.04)); CHECK(d.state.phase == GraspPhase::kEmpty); CHECK(d.commands[0].kind == CommandKind::kDetachObject);
  e.kind = GraspEventKind::kReleaseFailed; CHECK(advance_grasp(releasing,e,cfg,time(1.04)).state.possession == Possession::kUnknown);
  e.kind = GraspEventKind::kReleaseCompleted; CHECK(advance_grasp(releasing,e,cfg,time(2.04)).state.possession == Possession::kUnknown);
  CHECK(GraspState{}.possession == Possession::kUnknown); CHECK(MotionState{}.phase == ArmMotionState::kNotReady); // process restart defaults

  SceneState sc; SceneEvent se; se.kind = SceneEventKind::kStaticLoaded; se.key = key(10);
  se.static_geometry = scene().static_geometry; auto sd = advance_scene(sc,se,cfg,time()); CHECK(!sd.state.coherent);
  CHECK(sd.commands.size() == 1); CHECK(sd.commands[0].deadline == sd.state.context.deadline);
  CHECK(sd.commands[0].deadline > time().monotonic_now); sc = sd.state;
  se.kind = SceneEventKind::kUpdateConfirmed; se.key.sequence = 2; se.command_id = sc.pending_command_id; se.scene_revision = sc.revision;
  sd = advance_scene(sc,se,cfg,time()); CHECK(scene_ready(sd.state,cfg,time())); sc = sd.state;
  se.kind = SceneEventKind::kObservation; se.key.sequence = 3; se.object = scene().dynamic_objects[0]; se.source = ObservationSource::kSimulationTruth;
  CHECK(advance_scene(sc,se,cfg,time()).status.disposition == Disposition::kRejected); se.source = ObservationSource::kVision;
  sd = advance_scene(sc,se,cfg,time()); CHECK(sd.state.dynamic_objects.size() == 1); sc = sd.state;
  se.kind = SceneEventKind::kUpdateConfirmed; se.key.sequence = 4; se.command_id = sc.pending_command_id; se.scene_revision = sc.revision;
  sc = advance_scene(sc,se,cfg,time()).state;
  se.kind = SceneEventKind::kAttachRequested; se.key.sequence = 5; sc = advance_scene(sc,se,cfg,time()).state; CHECK(sc.attachment == AttachmentPhase::kAttachPending); CHECK(!scene_ready(sc,cfg,time()));
  se.kind = SceneEventKind::kAttachConfirmed; se.key.sequence = 6; se.command_id = sc.pending_command_id; se.scene_revision = sc.revision;
  auto wrong = se; ++wrong.scene_revision; CHECK(advance_scene(sc,wrong,cfg,time()).status.disposition == Disposition::kRejected);
  sc = advance_scene(sc,se,cfg,time()).state; CHECK(sc.attachment == AttachmentPhase::kAttached);
  se.kind = SceneEventKind::kExpire; se.key.sequence = 7; CHECK(advance_scene(sc,se,cfg,time(2)).state.dynamic_objects.size() == 1); // attached geometry does not expire as a free object
  se.kind = SceneEventKind::kDetachRequested; sc = advance_scene(sc,se,cfg,time(2)).state; CHECK(sc.attachment == AttachmentPhase::kDetachPending);
  se.kind = SceneEventKind::kDetachConfirmed; se.key.sequence = 8; se.command_id = sc.pending_command_id; se.scene_revision = sc.revision;
  sc = advance_scene(sc,se,cfg,time(2)).state; CHECK(sc.attachment == AttachmentPhase::kDetached); CHECK(sc.dynamic_objects.empty()); CHECK(sc.static_geometry.size() == 1);
  // A failed initial upload can be reset only with measured stopped/Empty
  // feedback, then must obtain a new backend acknowledgment before readiness.
  auto initial=scene();initial.dynamic_objects.clear();
  SceneEvent initial_event;initial_event.kind=SceneEventKind::kStaticLoaded;initial_event.key=key(10,20);initial_event.static_geometry=initial.static_geometry;
  initial=advance_scene(initial,initial_event,cfg,time()).state;
  initial_event.kind=SceneEventKind::kUpdateFailed;initial_event.key.sequence++;
  initial_event.command_id=initial.pending_command_id;initial_event.scene_revision=initial.revision;
  initial=advance_scene(initial,initial_event,cfg,time()).state;CHECK(initial.context.inhibited);
  initial_event.kind=SceneEventKind::kResetRequested;initial_event.key.sequence++;initial_event.arm=arm_at(1.);initial_event.gripper=gripper_at(1.);
  initial=advance_scene(initial,initial_event,cfg,time()).state;CHECK(!initial.context.inhibited);CHECK(!scene_ready(initial,cfg,time()));
  initial_event.kind=SceneEventKind::kStaticLoaded;initial_event.key.sequence++;
  initial=advance_scene(initial,initial_event,cfg,time()).state;
  initial_event.kind=SceneEventKind::kUpdateConfirmed;initial_event.key.sequence++;initial_event.command_id=initial.pending_command_id;initial_event.scene_revision=initial.revision;
  CHECK(scene_ready(advance_scene(initial,initial_event,cfg,time()).state,cfg,time()));
  auto ordered=scene();ordered.dynamic_objects.clear();ordered.context.last_monotonic=*seconds_to_duration(1.1);
  SceneEvent current;current.kind=SceneEventKind::kObservation;current.key=key(10,2);current.object=scene().dynamic_objects.front();
  CHECK(advance_scene(ordered,current,cfg,time(1.,1.)).status.disposition==Disposition::kRejected);
  const auto fresh_event=advance_scene(ordered,current,cfg,time(1.,1.2));
  CHECK(fresh_event.status.disposition==Disposition::kApplied);CHECK(fresh_event.state.dynamic_objects.size()==1);
  CHECK(fresh_event.state.dynamic_objects.front().observation.timestamp==current.object.observation.timestamp);
  // One owner may fold all tracks from a camera frame into one scene
  // write. Only the final revision is acknowledged; all observations survive.
  auto batch=scene();batch.dynamic_objects.clear();batch.context.key.sequence=1;
  SceneEvent update;update.kind=SceneEventKind::kObservation;update.key=key(10,2);
  update.object=scene().dynamic_objects.front();update.object.key.track_id=11;
  auto first=advance_scene(batch,update,cfg,time());CHECK(first.status.disposition==Disposition::kApplied);
  update.key.sequence=3;update.object.key.track_id=12;
  auto second=advance_scene(first.state,update,cfg,time());CHECK(second.status.disposition==Disposition::kApplied);
  CHECK(second.state.dynamic_objects.size()==2);CHECK(second.state.revision==first.state.revision+1);
  SceneEvent ack;ack.kind=SceneEventKind::kUpdateConfirmed;ack.key=key(10,4);
  ack.scene_revision=second.state.revision;ack.command_id=second.state.pending_command_id;
  auto committed=advance_scene(second.state,ack,cfg,time());CHECK(scene_ready(committed.state,cfg,time()));
  CHECK(committed.state.dynamic_objects[0].key.track_id==11);CHECK(committed.state.dynamic_objects[1].key.track_id==12);
  SceneEvent refresh;refresh.kind=SceneEventKind::kRefresh;refresh.key=key(10,5);refresh.key.track_id=11;
  auto refreshed=advance_scene(committed.state,refresh,cfg,time());
  CHECK(refreshed.commands.size()==1 && refreshed.commands.front().kind==CommandKind::kSyncScene);
  CHECK(refreshed.state.revision==committed.state.revision+1 && !refreshed.state.coherent);
  CHECK(refreshed.state.dynamic_objects.front().observation.timestamp==time().source_now);
  CHECK(advance_scene(committed.state,refresh,cfg,time(2)).status.disposition==Disposition::kRejected);
  refresh.key.track_id=13;
  CHECK(advance_scene(committed.state,refresh,cfg,time()).status.disposition==Disposition::kRejected);
  // A short target occlusion does not abort an already-running finite
  // observation. Other obstacles and new commands retain normal freshness.
  auto prediction_data=settings_data();prediction_data.tracking.max_prediction_horizon_sec=3.;prediction_data.tracking.temporary_loss_timeout_sec=3.5;
  const auto prediction_cfg=settings(prediction_data);auto occluded=scene();
  CHECK(!scene_ready(occluded,prediction_cfg,time(1.8)));
  CHECK(scene_ready(occluded,prediction_cfg,time(1.8),1));
  CHECK(occluded.dynamic_objects.front().observation.timestamp==time().source_now);
  CHECK(!scene_ready(occluded,prediction_cfg,time(4.1),1));
  auto committed_data=prediction_data;committed_data.runtime.operation_mode="assumed_ng_demo";
  committed_data.inspection.inference.backend="assumed_ng";committed_data.grasp.moving_intercept=true;
  const auto committed_cfg=settings(committed_data);
  CHECK(scene_ready(occluded,committed_cfg,time(4.1),1));
  CHECK(!scene_ready(occluded,committed_cfg,time(4.1))); // New admission remains fresh-scene only.
  // A measured commissioning close may be occluded during its probe.
  // Reuse the admitted prediction policy for attachment, never for formal mode
  // or an unconfirmed empty close.
  SceneEvent held_attach;held_attach.kind=SceneEventKind::kAttachRequested;held_attach.key=key(10,2);
  held_attach.arm=arm_at(4.1);held_attach.gripper=gripper_at(4.1,2);
  held_attach.gripper.contact_confirmed=true;held_attach.gripper.empty_confirmed=false;held_attach.gripper.width_m=.04;
  CHECK(advance_scene(occluded,held_attach,committed_cfg,time(4.1)).state.attachment==AttachmentPhase::kAttachPending);
  CHECK(advance_scene(occluded,held_attach,prediction_cfg,time(4.1)).status.disposition==Disposition::kRejected);
  held_attach.gripper.contact_confirmed=false;
  // Unknown/non-empty is conservatively represented for recovery planning;
  // this is not a Holding or physical grasp confirmation.
  const auto possible_carry=advance_scene(occluded,held_attach,committed_cfg,time(4.1));
  CHECK(possible_carry.state.attachment==AttachmentPhase::kAttachPending);
  held_attach.gripper.empty_confirmed=true;
  CHECK(advance_scene(occluded,held_attach,committed_cfg,time(4.1)).status.disposition==Disposition::kRejected);
  held_attach.gripper.empty_confirmed=false;
  // Failed scene synchronization can be retried with the original measured
  // object, without inventing a fresh timestamp or confirming Holding.
  auto failed_scene=occluded;failed_scene.context.inhibited=true;failed_scene.coherent=false;
  failed_scene.attachment=AttachmentPhase::kUnknown;failed_scene.attached_track_id=1;failed_scene.dynamic_objects.clear();
  held_attach.object=occluded.dynamic_objects.front();
  const auto rebuilt=advance_scene(failed_scene,held_attach,committed_cfg,time(4.1));
  CHECK(rebuilt.status.disposition==Disposition::kApplied);CHECK(rebuilt.state.attachment==AttachmentPhase::kAttachPending);
  CHECK(!rebuilt.state.context.inhibited);CHECK(!rebuilt.state.coherent);CHECK(rebuilt.commands.size()==1);
  CHECK(rebuilt.state.dynamic_objects.front().observation.timestamp==occluded.dynamic_objects.front().observation.timestamp);
  // A stopped physical Empty can clear an uncertain planning attachment;
  // a close/contact proxy, moving arm, or stale Empty cannot authorize cleanup.
  auto stale_attachment=occluded;stale_attachment.attachment=AttachmentPhase::kUnknown;
  stale_attachment.attached_track_id=1;stale_attachment.context.inhibited=true;
  auto cleanup=held_attach;cleanup.kind=SceneEventKind::kDetachRequested;
  cleanup.gripper=gripper_at(4.1,8);cleanup.arm=arm_at(4.1);
  const auto detaching=advance_scene(stale_attachment,cleanup,committed_cfg,time(4.1));
  CHECK(detaching.status.disposition==Disposition::kApplied);CHECK(!detaching.state.context.inhibited);
  CHECK(detaching.state.attachment==AttachmentPhase::kDetachPending);
  CHECK(detaching.state.dynamic_objects.empty());CHECK(!detaching.state.static_geometry.empty());
  CHECK(detaching.commands.size()==1);CHECK(detaching.commands.front().kind==CommandKind::kDetachObject);
  cleanup.gripper.empty_confirmed=false;
  CHECK(advance_scene(stale_attachment,cleanup,committed_cfg,time(4.1)).commands.empty());
  cleanup.gripper.empty_confirmed=true;cleanup.arm.stopped=false;
  CHECK(advance_scene(stale_attachment,cleanup,committed_cfg,time(4.1)).commands.empty());
  const double budget=committed_data.task.preparing_pick_timeout_sec+committed_data.grasp.approach_timeout_sec+
    committed_data.grasp.close_timeout_sec+committed_data.grasp.verification_timeout_sec;
  CHECK(scene_ready(occluded,committed_cfg,time(1.+budget+.1),1)); // Motion owns its finite execution deadline.
  auto nearby_stale=occluded;auto neighbor=nearby_stale.dynamic_objects.front();neighbor.key.track_id=2;nearby_stale.dynamic_objects.push_back(neighbor);
  CHECK(scene_ready(nearby_stale,committed_cfg,time(4.1),1));
  CHECK(!scene_ready(nearby_stale,committed_cfg,time(4.1)));
  CHECK(!scene_ready(nearby_stale,prediction_cfg,time(4.1),1));
  // Attached commissioning transport does not require images of free
  // conveyor objects after the wrist camera turns toward the bin.
  auto carried_scene=nearby_stale;carried_scene.attachment=AttachmentPhase::kAttached;carried_scene.attached_track_id=1;
  CHECK(scene_ready(carried_scene,committed_cfg,time(4.1)));
  CHECK(!scene_ready(carried_scene,cfg,time(4.1))); // Formal mode remains strict.
  auto released_scene=carried_scene;released_scene.attachment=AttachmentPhase::kDetached;released_scene.attached_track_id=0;
  CHECK(!scene_ready(released_scene,committed_cfg,time(4.1)));
  auto unacknowledged=carried_scene;unacknowledged.coherent=false;
  CHECK(!scene_ready(unacknowledged,committed_cfg,time(4.1)));
  auto uncertain_attachment=carried_scene;uncertain_attachment.attachment=AttachmentPhase::kUnknown;
  CHECK(!scene_ready(uncertain_attachment,committed_cfg,time(4.1)));
  MotionState active;active.active=true;active.phase=ArmMotionState::kMoving;
  active.context.key=key(30);active.context.last_monotonic=time().monotonic_now;
  active.operation_deadline=active.context.deadline=time(10.).monotonic_now;
  MotionEvent tick;tick.kind=MotionEventKind::kTick;tick.key=key(30,2);tick.arm=arm_at(1.8);tick.scene=occluded;
  auto moving_pick=active;moving_pick.request.prediction_horizon=*seconds_to_duration(2.);
  CHECK(advance_motion(moving_pick,tick,prediction_cfg,time(1.8)).state.phase==ArmMotionState::kMoving);
  auto ordinary=active;
  CHECK(advance_motion(ordinary,tick,prediction_cfg,time(1.8)).state.phase==ArmMotionState::kStopping);
  auto other=occluded.dynamic_objects.front();other.key.track_id=2;occluded.dynamic_objects.push_back(other);
  CHECK(!scene_ready(occluded,prediction_cfg,time(1.8),1));
  // Occlusion must not delete the current demo grasp target before attach.
  auto retained_scene=scene();
  auto retain_event=se;retain_event.kind=SceneEventKind::kExpire;retain_event.key=key(10,2);
  const auto retained=advance_scene(retained_scene,retain_event,demo_cfg,time(5.));
  CHECK(retained.state.dynamic_objects.size()==1);
  CHECK(retained.state.dynamic_objects.front().observation.timestamp==track().observation.timestamp);
  retain_event.key.track_id=0;
  const auto recovery_expiry=advance_scene(retained_scene,retain_event,demo_cfg,time(5.));
  CHECK(recovery_expiry.state.dynamic_objects.empty());
  CHECK(!recovery_expiry.state.coherent); // No motion before the backend ACK.
  CHECK(recovery_expiry.state.static_geometry.size()==retained_scene.static_geometry.size());
  CHECK(recovery_expiry.commands.size()==1);
  auto expiry_ack=retain_event;expiry_ack.kind=SceneEventKind::kUpdateConfirmed;
  expiry_ack.key.sequence++;expiry_ack.command_id=recovery_expiry.state.pending_command_id;
  expiry_ack.scene_revision=recovery_expiry.state.revision;
  const auto recovery_scene=advance_scene(recovery_expiry.state,expiry_ack,demo_cfg,time(5.));
  CHECK(scene_ready(recovery_scene.state,demo_cfg,time(5.)));
  auto recovery_attached=retained_scene;recovery_attached.attachment=AttachmentPhase::kAttached;
  recovery_attached.attached_track_id=retained_scene.dynamic_objects.front().key.track_id;
  CHECK(advance_scene(recovery_attached,retain_event,demo_cfg,time(5.)).state.dynamic_objects.size()==1);
  CHECK(advance_scene(retained_scene,retain_event,cfg,time(5.)).state.dynamic_objects.empty());
  auto free = scene(); se.kind = SceneEventKind::kExpire; se.key = key(10,2);
  sd = advance_scene(free,se,cfg,time(1.5)); CHECK(sd.state.dynamic_objects.empty()); CHECK(sd.state.static_geometry.size() == 1); CHECK(!sd.state.coherent); CHECK(sd.state.expired_track_ids == std::vector<std::uint64_t>{1});
  se.kind = SceneEventKind::kUpdateConfirmed; se.key.sequence = 3; se.scene_revision = sd.state.revision; se.command_id = sd.state.pending_command_id;
  CHECK(advance_scene(sd.state,se,cfg,time(3.5)).state.context.inhibited);
}
int main() {return run(tests);}
