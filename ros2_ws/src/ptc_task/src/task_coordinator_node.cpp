#include "ptc_core/grasp_offset_ramp.hpp"
#include <limits>
#include "ptc_core/target_selection.hpp"
#include "ptc_task/task_coordinator_node.hpp"
#include "ptc_app/node_runtime.hpp"
#include "ptc_app/bounded_worker.hpp"
#include "ptc_core/bin_recovery_policy.hpp"
#include "ptc_app/scene_loader.hpp"
#include "ptc_app/robot_description_loader.hpp"
#include "ptc_app/rgbd_converter.hpp"
#include "ptc_app/ros_converter.hpp"
#include "ptc_core/state/admission_model.hpp"
#include "ptc_core/state/task_state.hpp"
#include "ptc_core/vision/grasp_pose_model.hpp"
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_core/vision/bin_height_model.hpp"
#include "ptc_interfaces/msg/rgbd_frame.hpp"
#include "ptc_interfaces/action/move_to_pose.hpp"
#include "ptc_interfaces/action/check_motion.hpp"
#include "ptc_interfaces/action/evaluate_grasp_candidates.hpp"
#include "ptc_interfaces/action/follow_target.hpp"
#include "ptc_interfaces/action/capture_grasp_surface.hpp"
#include "ptc_interfaces/action/gripper_command.hpp"
#include "ptc_interfaces/action/sync_scene.hpp"
#include "ptc_interfaces/msg/task_status.hpp"
#include "ptc_interfaces/msg/capability_status.hpp"
#include "ptc_interfaces/msg/tracked_object_array.hpp"
#include "ptc_interfaces/msg/target_update.hpp"
#include "ptc_core/approach_timing.hpp"
#include "ptc_core/move_to_pose.hpp"
#include "ptc_interfaces/srv/task_command.hpp"
#include "ptc_interfaces/srv/stop_arm.hpp"
#include "ptc_interfaces/msg/grasp_neighborhood.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <filesystem>
#include <map>
#include <set>
#include <vector>
namespace ptc_task
{
using namespace ptc_core;using ptc_app::from_ros;using ptc_app::to_ros;
struct TaskCoordinatorNode::Runtime
{
  using Move=ptc_interfaces::action::MoveToPose;using Follow=ptc_interfaces::action::FollowTarget;
  using Capture=ptc_interfaces::action::CaptureGraspSurface;
  using Grip=ptc_interfaces::action::GripperCommand;
  using Sync=ptc_interfaces::action::SyncScene;
  using Check=ptc_interfaces::action::CheckMotion;
  using Evaluate=ptc_interfaces::action::EvaluateGraspCandidates;
  using Status=ptc_interfaces::msg::TaskStatus;
  using Neighborhood=ptc_interfaces::msg::GraspNeighborhood;
  rclcpp::Subscription<Neighborhood>::SharedPtr neighborhood_sub;
  Neighborhood::ConstSharedPtr latest_neighborhood;
  TaskCoordinatorNode & node;ptc_app::SettingsLoadResult settings;ptc_app::RobotDescriptionModel robot;
  TaskState task;SceneState scene,ack_scene;ArmSnapshot arm;GripperState gripper;
  rclcpp::Subscription<ptc_interfaces::msg::RgbdFrame>::SharedPtr bin_frame_sub;
  std::optional<BinHeightEstimate> bin_height_estimate;
  Nanoseconds bin_height_stamp{0};
  Nanoseconds valid_bin_height_stamp{0};
  EventKey bin_height_key;
  std::uint64_t bin_height_epoch{0};
  static constexpr double bin_height_sample_period_sec=0.5;
  double measured_bin_offset()
  {
    if(!bin_height_estimate || !config().grasp.bin_height || bin_height_epoch!=task.context.key.epoch ||
      !source_is_fresh(valid_bin_height_stamp,now(),
        std::max(config().vision.frame_max_age_sec,2.*bin_height_sample_period_sec),
        config().time.sensor_future_tolerance_sec))return 0.;
    double below_tcp=0.06;
    // Keep the captured object's lower surface clear of the pile for this grasp.
    const auto object_in_tcp=inverse_pose(task.grasp.grasp_tcp_in_object);
    if(object_in_tcp){
      Pose3d tcp;tcp.orientation=task.grasp.held_tcp_orientation_base.value_or(arm.state.tcp_pose.orientation);
      const auto object_in_base=compose_pose(tcp,*object_in_tcp);
      if(object_in_base)for(const auto & point:demo_grasp_surface.points){
        below_tcp=std::max(below_tcp,-transform_point(*object_in_base,point.position_object_m).z_m);
      }
    }
    return std::max(0.,bin_height_estimate->surface_z_m+below_tcp+
      config().grasp.bin_height->release_clearance_m-config().grasp.bin_release_pose.position.z_m);
  }
  void observe_bin_height(const ptc_interfaces::msg::RgbdFrame & wire)
  {
    const auto time=now();
    if(time.epoch!=bin_height_epoch){bin_height_estimate.reset();bin_height_stamp=Nanoseconds{0};bin_height_epoch=time.epoch;}
    const auto stamp=from_ros(wire.depth.header.stamp);
    if(!config().grasp.bin_height || (task.phase!=TaskPhase::kTransporting && task.phase!=TaskPhase::kRecovering &&
      task.phase!=TaskPhase::kFaulted) || time.paused || wire.key.epoch!=time.epoch ||
      stamp<=bin_height_stamp || (bin_height_stamp.count() && stamp-bin_height_stamp<*seconds_to_duration(bin_height_sample_period_sec)) ||
      !source_is_fresh(stamp,time,config().vision.frame_max_age_sec,config().time.sensor_future_tolerance_sec))return;
    bin_height_stamp=stamp;
    const auto began=std::chrono::steady_clock::now();
    try{
      if(wire.camera_world.header.frame_id!=config().frames.world || !wire.depth_registered)return;
      auto frame=ptc_app::image_frame(wire,config().vision);
      const auto transform=tf.lookupTransform(config().frames.robot_base,config().frames.world,
        rclcpp::Time(stamp.count(),node.get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(0.));
      const Pose3d world_in_base{{transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z},
        {transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w}};
      const auto camera=compose_pose(world_in_base,frame.metadata.camera_in_world);
      if(!camera)return;
      frame.metadata.camera_in_world=*camera;
      const auto points=region_points_world(frame,{0,0,frame.camera.width,frame.camera.height},4096);
      std::vector<std::array<Vector3d,2>> exclusions;
      const auto tcp=compose_pose(world_in_base,from_ros(wire.tcp_world.pose));
      const auto object_in_tcp=inverse_pose(task.grasp.grasp_tcp_in_object);
      if(tcp && object_in_tcp && task.grasp.possession!=Possession::kEmpty && !demo_grasp_surface.points.empty()){
        const auto object_in_base=compose_pose(*tcp,*object_in_tcp);
        if(object_in_base){
          Vector3d low{1e6,1e6,1e6},high{-1e6,-1e6,-1e6};
          for(const auto & point:demo_grasp_surface.points){
            const auto p=transform_point(*object_in_base,point.position_object_m);
            low.x_m=std::min(low.x_m,p.x_m-.015);low.y_m=std::min(low.y_m,p.y_m-.015);low.z_m=std::min(low.z_m,p.z_m-.015);
            high.x_m=std::max(high.x_m,p.x_m+.015);high.y_m=std::max(high.y_m,p.y_m+.015);high.z_m=std::max(high.z_m,p.z_m+.015);
          }
          exclusions.push_back({low,high});
        }
      }
      if(tcp){
        // The palm and fingers are above this conservative tool envelope.
        const auto & p=tcp->position;
        exclusions.push_back({Vector3d{p.x_m-.09,p.y_m-.09,p.z_m-.015},Vector3d{p.x_m+.09,p.y_m+.09,p.z_m+.25}});
      }
      const auto estimate=estimate_bin_height(points,*config().grasp.bin_height,exclusions);
      const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
      if(estimate){
        bin_height_estimate=estimate;
        valid_bin_height_stamp=stamp;
        bin_height_key=task.context.key;
      }
      RCLCPP_INFO(node.get_logger(),"bin surface: valid=%d points=%zu cells=%zu surface_z_m=%.4f offset_m=%.4f compute_ms=%.3f",
        estimate.has_value(),estimate?estimate->points:0,estimate?estimate->cells:0,
        estimate?estimate->surface_z_m:0.,measured_bin_offset(),elapsed);
    }catch(const std::exception & error){
      RCLCPP_DEBUG(node.get_logger(),"bin surface unavailable: %s",error.what());
    }
  }
  EventKey ack_grasp_surface_key;std::uint64_t ack_grasp_surface_revision{0};
  std::optional<TrackedObjectState> latest_selected_visual_track;
  bool continuous_operation{false};
  std::optional<std::chrono::steady_clock::time_point> last_semantic_preview;
  rclcpp::Subscription<ptc_interfaces::msg::TrackedObjectArray>::SharedPtr semantic_preview_sub;
  EventKey last_completed_key;
  CycleOutcome last_outcome{CycleOutcome::kNone};
  std::uint8_t manual_home_state{Status::HOME_NONE};
  Nanoseconds manual_home_deadline{0};
  Nanoseconds next_waiting_open{0};
  std::string manual_home_message;
  const EventKey & grasp_surface_cycle_key()const
  {return task.context.key;}
  std::vector<Vector3d> demo_grasp_cloud;GraspSurfaceSnapshot demo_grasp_surface;
  std::vector<GraspCandidate> grasp_candidates;std::size_t grasp_candidate_index{0};
  std::optional<std::pair<std::uint64_t,std::uint64_t>> grasp_proposal_pending,grasp_proposal_completed;
  std::optional<std::uint64_t> observation_open_command;
  struct CloseTiming {
    EventKey key;
    std::uint64_t command_id;
    Nanoseconds source_started;
    std::chrono::steady_clock::time_point wall_started;
    double initial_width_m;
    bool movement_recorded{false};
  };
  std::optional<CloseTiming> close_timing;
  std::optional<Command> recovery_retreat_command;
  Nanoseconds next_bin_recovery_attempt{0};
  std::vector<Pose3d> failed_bin_recovery_targets;
  std::optional<double> recovery_bin_height_offset;
  std::optional<Quaterniond> recovery_bin_orientation;
  bool recovery_open_failed{false};
  bool recover_pregrasp_open{false};
  std::optional<GraspApproachReference> grasp_approach_reference;
  std::optional<GraspMotionReference> grasp_motion_reference;
  GraspOffsetRamp grasp_motion_offset;
  std::optional<double> empty_descent_clearance_z;
  double grasp_motion_elapsed{0},grasp_motion_rate{1};
  std::optional<GraspEntryState> grasp_command_state;double grasp_entry_minimum_sec{0};
  Nanoseconds grasp_motion_tick{0},grasp_motion_visual_stamp{0};

  std::optional<Pose3d> grasp_follow_hold_in_object;
  std::optional<Follow::Feedback> grasp_follow_feedback;
  ptc_interfaces::msg::CapabilityStatus vision_status,inference_status;
  using CandidateKey=std::pair<std::uint64_t,std::uint64_t>;
  struct AdmissionAttempt {Nanoseconds observation_stamp{0},retry_after{0};std::size_t attempts_for_observation{0};};
  std::vector<TrackedObjectState> tracks;std::map<CandidateKey,Nanoseconds> retired;
  std::map<CandidateKey,AdmissionAttempt> admission_attempts;
  std::map<CandidateKey,std::uint64_t> deferred_tracks;
  std::uint64_t last_admission_track_id{0};
  GraspCandidate grasp_pose;
  ptc_app::TimeAdapter clock;ptc_app::CompletionQueue completions;ptc_app::BoundedWorker grasp_worker{1};
  tf2_ros::Buffer tf;tf2_ros::TransformListener listener;
  std::uint64_t sequence{1},cycle{0},runtime_command{1000000000},generation{1},epoch{0};
  std::function<void(bool)> motion_completion; Nanoseconds motion_settle_deadline{0};
  bool move_busy{false},follow_active{false},capture_busy{false},sync_busy{false},scene_initialized{false};
  std::vector<double> grasp_proposal_wall_ms_samples;
  std::uint64_t grasp_proposal_sample_count{0},grasp_proposal_over_100ms_count{0};
  std::string observation_diagnostic;
  std::string reason{"waiting for ready assets and devices"};EventKey follow_key;
  std::uint64_t follow_command{0};Nanoseconds last_tcp_stamp{0};Pose3d previous_tcp_world;double relative_speed{kUnset};
  Nanoseconds entry_follow_started_source{0};
  std::chrono::steady_clock::time_point grasp_diagnostic_at{};
  bool pending_attach{false},pending_release{false};
  std::optional<Nanoseconds> attachment_ack_joint_stamp;
  bool pending_reset{false},motion_command_ready{false};std::uint64_t arm_command_id{0};
  bool checking_target{false};
  Nanoseconds source_sample{0};std::chrono::steady_clock::time_point source_sample_wall{};
  double source_rate{kUnset},control_period_source{kUnset},observation_period_source{kUnset};Nanoseconds tracks_sample_stamp{0};
  TrackedObjectState probe_baseline;Pose3d probe_relative,probe_tcp;bool probe_valid{false};
  Nanoseconds visual_evidence_stamp{0},last_target_stamp{0};
  rclcpp_action::Client<Move>::SharedPtr move_client;rclcpp_action::Client<Follow>::SharedPtr follow_client;
  rclcpp_action::Client<Capture>::SharedPtr grasp_capture_client;
  rclcpp_action::Client<Grip>::SharedPtr grip_client;
  rclcpp_action::Client<Sync>::SharedPtr sync_client;
  rclcpp_action::Client<Check>::SharedPtr check_client;
  rclcpp_action::Client<Evaluate>::SharedPtr evaluate_client;
  rclcpp::Client<ptc_interfaces::srv::StopArm>::SharedPtr stop_client;
  std::optional<Nanoseconds> recovery_stop_ack_stamp;
  rclcpp::Service<ptc_interfaces::srv::TaskCommand>::SharedPtr command_server;
  rclcpp::Subscription<ptc_interfaces::msg::ArmStatus>::SharedPtr arm_sub;
  rclcpp::Subscription<ptc_interfaces::msg::GripperStatus>::SharedPtr grip_sub;
  rclcpp::Subscription<ptc_interfaces::msg::TrackedObjectArray>::SharedPtr tracks_sub;
  rclcpp::Subscription<ptc_interfaces::msg::CapabilityStatus>::SharedPtr vision_sub,inference_sub;
  rclcpp::Publisher<ptc_interfaces::msg::TaskStatus>::SharedPtr task_pub;
  rclcpp::Publisher<ptc_interfaces::msg::EventKey>::SharedPtr bin_release_pub;
  rclcpp::Publisher<ptc_interfaces::msg::TargetUpdate>::SharedPtr target_pub;rclcpp::TimerBase::SharedPtr timer;
  const SystemSettings & config()const{return settings.snapshot->get();}
  TimeContext now(){return clock.sample(node,config());}
  bool semantic_preview_fresh() const
  {
    return last_semantic_preview &&
      std::chrono::steady_clock::now()-*last_semantic_preview <
        std::chrono::duration<double>(config().tracking.max_prediction_horizon_sec);
  }
  EventKey event_key(std::uint64_t operation=0)
  {auto key=task.context.key;key.epoch=now().epoch;key.settings_revision=settings.snapshot->revision();key.sequence=sequence++;if(operation){key.operation_id=operation;}if(!key.operation_id){key.operation_id=1;}if(!key.cycle_id){key.cycle_id=cycle+1;}return key;}
  TrackedObjectState selected()const
  {
    const auto found=std::find_if(tracks.begin(),tracks.end(),[this](const auto & t){return same_track_identity(t.context.key,task.selected_track.context.key);});
    if(found!=tracks.end() && found->velocity_valid && (found->phase==TrackPhase::kTracking||found->phase==TrackPhase::kTemporarilyLost)){return *found;}
    auto track=latest_selected_visual_track && same_track_identity(latest_selected_visual_track->context.key,task.selected_track.context.key)?*latest_selected_visual_track:task.selected_track;
    // Absence or an unconfirmed reacquisition is not a valid new motion model.
    // Preserve the last confirmed velocity instead of overwriting it with zero.
    // Keep only the last actual visual
    // sample, and let the existing bounded TemporarilyLost prediction expire.
    if(track.phase==TrackPhase::kTracking){track.phase=TrackPhase::kTemporarilyLost;}
    return track;
  }
  StampedPose3d tcp_world()
  {
    const auto wire=tf.lookupTransform(config().frames.world,config().frames.robot_base,rclcpp::Time(arm.tcp_timestamp.count(),node.get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(config().vision.tf_lookup_timeout_sec));
    Pose3d base{{wire.transform.translation.x,wire.transform.translation.y,wire.transform.translation.z},{wire.transform.rotation.x,wire.transform.rotation.y,wire.transform.rotation.z,wire.transform.rotation.w}};
    const auto pose=compose_pose(base,arm.state.tcp_pose);if(!pose){throw std::runtime_error("invalid world TCP TF");}return {arm.tcp_timestamp,config().frames.world,*pose};
  }
  ReadinessStatus readiness()
  {
    const auto t=now();const auto valid=[&](const auto & s){return s.ready && s.key.epoch==t.epoch && s.key.settings_revision==settings.snapshot->revision() && s.binding_revision==settings.binding_revision &&
      source_is_fresh(from_ros(s.header.stamp),t,config().vision.frame_max_age_sec,config().time.sensor_future_tolerance_sec);};
    return {valid(vision_status),valid(inference_status),vision_status.key.settings_revision,inference_status.key.settings_revision,motion_command_ready?settings.snapshot->revision():""};
  }
  TaskEvent event(TaskEventKind kind,std::uint64_t operation=0)
  {
    TaskEvent e;if(kind==TaskEventKind::kGraspUpdated || kind==TaskEventKind::kMotionFinished){e.bin_height_offset_m=measured_bin_offset();}e.kind=kind;e.key=event_key(operation?operation:task.expected_operation_id);e.arm=arm;e.gripper=gripper;e.scene=ack_scene;e.readiness=readiness();e.track=selected();e.grasp_event.motion_prediction=current_grasp_motion_prediction();
    if(kind==TaskEventKind::kMotionFinished && task.phase==TaskPhase::kPlacing &&
      config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model"){
      e.grasp_event.expected_contact_span_m=grasp_pose.opening_width_m;
    }
    return e;
  }
  Disposition apply(TaskEvent e)
  {try{const auto previous=task.phase;const auto previous_reason=reason;const bool previous_grasp_faulted=task.grasp.faulted;
    const auto previous_possession=task.grasp.possession;
    const bool failed_before_close=previous==TaskPhase::kPicking &&
      task.grasp.phase==GraspPhase::kApproaching && previous_possession==Possession::kEmpty;
    const auto time=now();
    const auto decision=advance_task(task,e,*settings.snapshot,time);task=decision.state;
    if(failed_before_close && task.phase==TaskPhase::kRecovering){
      // An approach timeout had no close command. Use the existing open
      // confirmation path instead of synthesizing a held scene object.
      recover_pregrasp_open=true;
    }
    if(e.kind==TaskEventKind::kReportAcknowledged && decision.status.disposition==Disposition::kApplied){
      last_completed_key=task.context.key;last_outcome=task.outcome;
    }
    if(previous!=task.phase || (decision.status.disposition==Disposition::kApplied && e.kind!=TaskEventKind::kTick)){reason=decision.status.reason;}
    if(previous!=task.phase && previous!=TaskPhase::kFaulted && task.phase==TaskPhase::kRecovering && (task.outcome==CycleOutcome::kFailed || task.outcome==CycleOutcome::kInconclusive)){last_failed_grasp=(e.kind==TaskEventKind::kTargetLost || e.kind==TaskEventKind::kFaultRaised) && !previous_reason.empty()?previous_reason:reason;}
    // An in-flight open belongs to its completion callback even if recovery
    // enters Faulted; clearing it here submitted overlapping recovery opens.
    if(previous!=task.phase){RCLCPP_INFO(node.get_logger(),"task transition %d -> %d, event=%d, track=%llu: %s",static_cast<int>(previous),static_cast<int>(task.phase),static_cast<int>(e.kind),static_cast<unsigned long long>(task.context.key.track_id),reason.c_str());}
    if(!previous_grasp_faulted && task.grasp.faulted){
      const auto detail=task.grasp.context.history.empty()?std::string{"no grasp transition record"}:task.grasp.context.history.back().reason;
      std::string evidence;
      if(e.grasp_event.kind==GraspEventKind::kApproachCompleted){
        const auto age_ms=[&](Nanoseconds stamp){return std::to_string(std::chrono::duration<double,std::milli>(time.source_now-stamp).count());};
        evidence=" tcp_age_ms="+age_ms(e.grasp_event.tcp_world.timestamp)+" joint_age_ms="+age_ms(e.arm.joint_timestamp)+
          " gripper_age_ms="+age_ms(e.gripper.timestamp)+" track_age_ms="+age_ms(e.grasp_event.track.observation.timestamp)+
          " relative_speed_mps="+std::to_string(e.grasp_event.relative_speed_mps);
      }
      record_observation_step("grasp_fault","event="+std::to_string(static_cast<int>(e.grasp_event.kind))+
        " phase="+std::to_string(static_cast<int>(task.grasp.phase))+" width_m="+std::to_string(e.gripper.width_m)+
        " contact="+std::to_string(e.gripper.contact_confirmed)+" stopped="+std::to_string(e.gripper.stopped)+"; "+detail+evidence,task.context.key.track_id);
    }
    if(close_timing){
      if(task.phase==TaskPhase::kRecovering || task.phase==TaskPhase::kFaulted){
        record_close_timing("aborted",close_timing->command_id);
        close_timing.reset();
      }else if(previous_possession!=Possession::kHolding && task.grasp.possession==Possession::kHolding){
        record_close_timing("holding_confirmed",close_timing->command_id);
        close_timing.reset();
      }
    }
    commands(decision.commands);return decision.status.disposition;}catch(const std::exception & error){reason=error.what();stop();return Disposition::kRejected;}}
  GraspEvent grasp_event(GraspEventKind kind)
  {GraspEvent e;e.kind=kind;e.key=task.grasp.context.key;e.key.sequence=sequence++;e.arm=arm;e.gripper=gripper;e.track=selected();e.relative_speed_mps=relative_speed;e.grasp_window_valid=grasp_pose.valid;e.grasp_tcp_in_object=grasp_pose.tcp_in_object;
    e.motion_prediction=current_grasp_motion_prediction();
    try{e.tcp_world=tcp_world();}catch(...){e.grasp_window_valid=false;}return e;}
  void grasp(GraspEvent e)
  {auto outer=event(TaskEventKind::kGraspUpdated,e.key.operation_id);outer.key=e.key;outer.grasp_event=std::move(e);apply(outer);}
  void fault(const std::string & message)
  {RCLCPP_WARN(node.get_logger(),"task fault: %s",message.c_str());reason=message;auto e=event(TaskEventKind::kFaultRaised);apply(e);}
  void skip_observation_target(const std::string & message)
  {
    const bool target_operation=task.phase==TaskPhase::kPreparingPick || task.phase==TaskPhase::kPicking;
    if(!target_operation || task.grasp.possession!=Possession::kEmpty){fault(message);return;}
    reason=message;
    RCLCPP_WARN(node.get_logger(),"target recovery requested: %s",message.c_str());
    record_observation_step("skip_observation_target",message+"; stop, retire target and return to observation",task.context.key.track_id);
    apply(event(TaskEventKind::kTargetLost));
  }
  template<class Action,class Callback>
  void send(const typename rclcpp_action::Client<Action>::SharedPtr & client,const typename Action::Goal & goal,Callback callback,bool scoped=true,
    std::function<void(const typename Action::Feedback &)> feedback={},std::function<void()> accepted={})
  {
    const auto issued_generation=generation;
    if(!client->action_server_is_ready()){callback(std::shared_ptr<typename Action::Result>{});return;}
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback=[this,issued_generation,callback,scoped,accepted](typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle){
      if(scoped && generation!=issued_generation){return;}
      if(!handle){callback(std::shared_ptr<typename Action::Result>{});}
      else if(accepted){accepted();}
    };
    options.result_callback=[this,issued_generation,callback,scoped](const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult & result){if(!scoped||generation==issued_generation){callback(result.result);}};
    if(feedback){options.feedback_callback=[this,issued_generation,feedback,scoped](auto,const std::shared_ptr<const typename Action::Feedback> message){if(!scoped||generation==issued_generation){feedback(*message);}};}
    client->async_send_goal(goal,options);
  }
  explicit Runtime(TaskCoordinatorNode & n):node(n),settings(ptc_app::load_node_settings(n)),robot(ptc_app::load_robot_description(config())),completions(config().diagnostics.event_queue_capacity),tf(n.get_clock()),listener(tf)
  {
    continuous_operation=n.declare_parameter<bool>("continuous_operation",false);
    // Test-only; read per bin-approach command so it can be set after startup. 0 disables.
    cycle=static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());scene.revision=cycle;const auto & ep=settings.endpoints;
    move_client=rclcpp_action::create_client<Move>(&n,ep.at("arm_move"));follow_client=rclcpp_action::create_client<Follow>(&n,ep.at("arm_follow"));grasp_capture_client=rclcpp_action::create_client<Capture>(&n,ep.at("capture_grasp_surface"));
    grip_client=rclcpp_action::create_client<Grip>(&n,ep.at("gripper_command"));sync_client=rclcpp_action::create_client<Sync>(&n,ep.at("scene_sync"));stop_client=n.create_client<ptc_interfaces::srv::StopArm>(ep.at("arm_stop"));
    check_client=rclcpp_action::create_client<Check>(&n,ep.at("motion_check"));
    evaluate_client=rclcpp_action::create_client<Evaluate>(&n,ep.at("grasp_evaluate"));
    if(config().grasp.bin_height){
      bin_frame_sub=n.create_subscription<ptc_interfaces::msg::RgbdFrame>(ep.at("frame"),rclcpp::SensorDataQoS().keep_last(1),
        [this](ptc_interfaces::msg::RgbdFrame::ConstSharedPtr frame){observe_bin_height(*frame);});
    }
    arm_sub=n.create_subscription<ptc_interfaces::msg::ArmStatus>(ep.at("arm_status"),rclcpp::QoS(5),[this](ptc_interfaces::msg::ArmStatus::ConstSharedPtr m){try{if(m->key.settings_revision==settings.snapshot->revision()){arm=from_ros(*m,config().frames.tcp);motion_command_ready=m->command_ready;arm_command_id=m->command_id;}}catch(const std::exception & e){reason=e.what();}});
    grip_sub=n.create_subscription<ptc_interfaces::msg::GripperStatus>(ep.at("gripper_status"),rclcpp::QoS(5),[this](ptc_interfaces::msg::GripperStatus::ConstSharedPtr m){try{if(m->key.settings_revision==settings.snapshot->revision()){gripper=from_ros(*m);observe_close_movement();}}catch(const std::exception & e){reason=e.what();}});
    tracks_sub=n.create_subscription<ptc_interfaces::msg::TrackedObjectArray>(ep.at("tracks"),rclcpp::QoS(5),[this](ptc_interfaces::msg::TrackedObjectArray::ConstSharedPtr m){try{
      std::vector<TrackedObjectState> fresh;std::set<CandidateKey> present;const auto current_epoch=now().epoch;
      for(const auto & wire:m->objects){auto track=from_ros(wire);
        if(track.context.key.settings_revision!=settings.snapshot->revision() || track.context.key.epoch!=current_epoch){continue;}
        const auto candidate=candidate_key(track);present.insert(candidate);
        if(!retired.count(candidate)){fresh.push_back(std::move(track));}}
      Nanoseconds stamp{0};for(const auto & track:fresh){stamp=std::max(stamp,track.observation.timestamp);}
      if(stamp>tracks_sample_stamp && tracks_sample_stamp.count()>0){
        const double period=std::chrono::duration<double>(stamp-tracks_sample_stamp).count();
        if(period>0&&period<=config().tracking.observation_max_age_sec){observation_period_source=period;}
      }
      for(const auto & track:fresh){
        if(task.context.key.track_id!=0 && track.phase==TrackPhase::kTracking && track.velocity_valid && same_track_identity(track.context.key,task.selected_track.context.key) &&
          (!latest_selected_visual_track || !same_track_identity(latest_selected_visual_track->context.key,track.context.key) ||
          track.observation.timestamp>=latest_selected_visual_track->observation.timestamp)){
          latest_selected_visual_track=track;
        }
      }
      if(stamp.count()>0){tracks_sample_stamp=stamp;}tracks=std::move(fresh);
      for(auto it=retired.begin();it!=retired.end();){
        const auto stamp=now().source_now;
        if(present.count(it->first)){it->second=stamp;++it;continue;}
        const double absent_age=std::chrono::duration<double>(stamp-it->second).count();
        const double retention=config().tracking.temporary_loss_timeout_sec+config().tracking.max_prediction_horizon_sec;
        if(absent_age>retention){it=retired.erase(it);}else{++it;}}
      for(auto it=admission_attempts.begin();it!=admission_attempts.end();){if(!present.count(it->first)){it=admission_attempts.erase(it);}else{++it;}}
      for(auto it=deferred_tracks.begin();it!=deferred_tracks.end();){if(!present.count(it->first)){it=deferred_tracks.erase(it);}else{++it;}}
    }catch(const std::exception & e){reason=e.what();}});
    if(config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model" &&
       config().grasp.neighbor_clearance_mode=="rank"){
      neighborhood_sub=n.create_subscription<Neighborhood>(ep.at("grasp_neighborhood"),rclcpp::QoS(1),
        [this](Neighborhood::ConstSharedPtr sample){
          if(!same_cycle(from_ros(sample->key),task.context.key)){return;}
          if(!latest_neighborhood || !same_cycle(from_ros(latest_neighborhood->key),task.context.key) ||
             from_ros(sample->stamp)>from_ros(latest_neighborhood->stamp)){latest_neighborhood=sample;}
        });
    }
    vision_sub=n.create_subscription<ptc_interfaces::msg::CapabilityStatus>(ep.at("vision_status"),rclcpp::QoS(5),[this](ptc_interfaces::msg::CapabilityStatus::ConstSharedPtr m){vision_status=*m;});
    inference_sub=n.create_subscription<ptc_interfaces::msg::CapabilityStatus>(ep.at("inference_status"),rclcpp::QoS(5),[this](ptc_interfaces::msg::CapabilityStatus::ConstSharedPtr m){inference_status=*m;});
    bin_release_pub=n.create_publisher<ptc_interfaces::msg::EventKey>("task/bin_release", rclcpp::QoS(10));
    task_pub=n.create_publisher<ptc_interfaces::msg::TaskStatus>(ep.at("task_status"),rclcpp::QoS(5));target_pub=n.create_publisher<ptc_interfaces::msg::TargetUpdate>(ep.at("arm_target"),rclcpp::QoS(5));
    if(config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model"){
      semantic_preview_sub=n.create_subscription<ptc_interfaces::msg::TrackedObjectArray>(ep.at("shared_tracks"),rclcpp::QoS(5),
        [this](ptc_interfaces::msg::TrackedObjectArray::ConstSharedPtr msg){
          const auto time=now();
          if(msg->header.frame_id==config().frames.world && !time.paused &&
            source_is_fresh(from_ros(msg->header.stamp),time,config().tracking.observation_max_age_sec,config().time.sensor_future_tolerance_sec)){
            last_semantic_preview=std::chrono::steady_clock::now();
          }
        });
    }
    command_server=n.create_service<ptc_interfaces::srv::TaskCommand>(ep.at("task_command"),[this](const std::shared_ptr<ptc_interfaces::srv::TaskCommand::Request> request,std::shared_ptr<ptc_interfaces::srv::TaskCommand::Response> response){
      if(request->command==request->RETURN_HOME){
        request_manual_home(*response);return;
      }
      const bool start_continuous=request->command==request->START_CONTINUOUS;
      if(start_continuous && (!semantic_preview_sub || !semantic_preview_fresh())){
        response->message="Continuous Auto Pick requires live Observe tracks";return;
      }
      if(request->command==request->PAUSE_CONTINUOUS){
        pause_continuous();
        response->accepted=true;
        response->message="Continuous Auto Pick paused; active pick may finish";
        return;
      }
      TaskEventKind kind;if(request->command==request->START || start_continuous){kind=TaskEventKind::kStartRequested;}else if(request->command==request->CANCEL){kind=TaskEventKind::kCancelRequested;}else if(request->command==request->RESET){kind=TaskEventKind::kResetRequested;}else{response->message="unknown command";return;}
      if(kind==TaskEventKind::kResetRequested){
        if(manual_home_pending()){response->message="Return Home already owns recovery";return;}
        if(pending_reset || (task.phase!=TaskPhase::kFaulted && task.phase!=TaskPhase::kWaitingReady) || !arm_snapshot_ready(arm,*settings.snapshot,now()) || !arm.stopped || !gripper_snapshot_ready(gripper,*settings.snapshot,now()) || !gripper.stopped || !gripper.empty_confirmed || !stop_client->service_is_ready()) {response->message="reset requires both measured stopped, known Empty and idle/faulted task";return;}
        request_measured_empty_reset();
        response->accepted=true;response->message="manual reset requested; wait for task/status WaitingReady";return;
      }
      if(kind==TaskEventKind::kStartRequested){
        if(semantic_preview_sub && continuous_operation){
          response->message="Pause Continuous Auto Pick before starting another mode";return;
        }
        const auto d=begin_next_cycle();response->accepted=d.status.disposition==Disposition::kApplied;response->message=d.status.reason;
        if(response->accepted){
          continuous_operation=start_continuous || node.get_parameter("continuous_operation").as_bool();
          manual_home_state=Status::HOME_NONE;manual_home_message.clear();
        }
        return;
      }
      if(kind==TaskEventKind::kCancelRequested){
        continuous_operation=false;
        if(manual_home_pending()){
          const bool moving=manual_home_state==Status::HOME_MOVING;
          manual_home_state=Status::HOME_CANCELED;manual_home_message="Return Home canceled";
          if(moving){stop();}
        }
        if(task.phase==TaskPhase::kWaitingReady){response->accepted=true;reason=response->message="continuous operation paused; task idle";return;}
      }
      const auto d=advance_task(task,event(kind),*settings.snapshot,now());response->accepted=d.status.disposition==Disposition::kApplied;response->message=d.status.reason;task=d.state;reason=d.status.reason;commands(d.commands);
    });
    timer=n.create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
  }
  void pause_continuous()
  {
    const bool was_continuous=continuous_operation;
    continuous_operation=false;
    if(was_continuous && task.phase==TaskPhase::kObserving && !task.context.key.track_id){
      apply(event(TaskEventKind::kCancelRequested));
    }
  }
  TaskDecision begin_next_cycle()
  {
    if(manual_home_pending()){return TaskDecision{task,{Disposition::kRejected,"Return Home in progress"},{},{}};}
    if(pending_reset){return TaskDecision{task,{Disposition::kRejected,"scene reset acknowledgment pending"},{},{}};}
    auto e=event(TaskEventKind::kStartRequested);const auto next=std::max(cycle,task.context.key.cycle_id)+1;
    e.key.cycle_id=next;e.key.track_id=0;e.key.session_id=next;e.key.operation_id=1;
    const auto time=now();
    const auto d=advance_task(task,e,*settings.snapshot,time);task=d.state;reason=d.status.reason;
    if(d.status.disposition==Disposition::kRejected){
      RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),2000,
        "Cycle rejected: %s; vision=%d inference=%d motion=%d arm=%d/%d gripper=%d/%d/%d scene=%d coherent=%d attachment=%d paused=%d",
        reason.c_str(),e.readiness.vision_ready,e.readiness.inference_ready,!e.readiness.motion_revision.empty(),
        arm_snapshot_ready(e.arm,*settings.snapshot,time),e.arm.stopped,
        gripper_snapshot_ready(e.gripper,*settings.snapshot,time),e.gripper.stopped,e.gripper.empty_confirmed,
        scene_ready(e.scene,*settings.snapshot,time),e.scene.coherent,static_cast<int>(e.scene.attachment),time.paused);
    }
    if(d.status.disposition==Disposition::kApplied){
      grasp_follow_hold_in_object.reset();grasp_follow_feedback.reset();
      cycle=next;demo_grasp_cloud.clear();demo_grasp_surface={};grasp_candidates.clear();grasp_candidate_index=0;recovery_retreat_command.reset();next_bin_recovery_attempt=Nanoseconds{0};failed_bin_recovery_targets.clear();recovery_bin_height_offset.reset();recovery_bin_orientation.reset();recovery_open_failed=false;recover_pregrasp_open=false;grasp_proposal_pending.reset();grasp_proposal_completed.reset();observation_open_command.reset();grasp_approach_reference.reset();grasp_motion_reference.reset();grasp_command_state.reset();grasp_entry_minimum_sec=0;grasp_motion_offset={};empty_descent_clearance_z.reset();grasp_motion_elapsed=0;grasp_motion_tick=Nanoseconds{0};grasp_motion_visual_stamp=Nanoseconds{0};observation_diagnostic.clear();latest_selected_visual_track.reset();entry_follow_started_source=Nanoseconds{0};++generation;grasp_pose={};pending_attach=false;attachment_ack_joint_stamp.reset();pending_release=false;probe_valid=false;last_tcp_stamp=Nanoseconds{0};relative_speed=kUnset;
      RCLCPP_INFO(node.get_logger(),"begin_next_cycle: cycle=%llu continuous=%d",static_cast<unsigned long long>(cycle),continuous_operation);
    }
    commands(d.commands);return d;
  }
  void stop()
  {
    if(manual_home_state==Status::HOME_MOVING){
      manual_home_state=Status::HOME_FAILED;manual_home_message="Return Home interrupted before measured arrival";
    }
    grasp_motion_reference.reset();grasp_command_state.reset();grasp_entry_minimum_sec=0;
    grasp_follow_feedback.reset();
    ++generation;motion_completion={};move_busy=capture_busy=follow_active=checking_target=false;
    // Canceled-generation callbacks cannot clear their command bookkeeping.
    observation_open_command.reset();pending_release=false;recovery_retreat_command.reset();
    // The gateway Stop owns cancellation of its active motion action. A
    // delayed cancel-all request must not cancel the subsequent Home goal.
    grasp_capture_client->async_cancel_all_goals();grip_client->async_cancel_all_goals();
    check_client->async_cancel_all_goals();evaluate_client->async_cancel_all_goals();
    recovery_stop_ack_stamp.reset();
    if(stop_client->service_is_ready()){
      auto request=std::make_shared<ptc_interfaces::srv::StopArm::Request>();request->key=to_ros(event_key());request->command_id=runtime_command++;
      const auto issued=generation;
      stop_client->async_send_request(request,[this,issued](rclcpp::Client<ptc_interfaces::srv::StopArm>::SharedFuture result){
        if(issued==generation && result.get()->accepted){recovery_stop_ack_stamp=now().source_now;}
      });
    }
  }
  void move_to_pose(const Command & command,std::function<void(bool)> done,const TrackedObjectState * moving=nullptr,double velocity_fraction=1.)
  {
    const auto move_view=task.phase==TaskPhase::kRecovering?"recovery_home":"initial_observation";
    record_observation_step("move_to_pose","running: planning/execution requested",command.key.track_id,move_view);
    Move::Goal goal;goal.target_pose=ptc_app::stamped_pose(command.target,command.reference_frame,now().source_now);goal.max_velocity_scale=config().motion.max_velocity_scale;goal.max_acceleration_scale=config().motion.max_acceleration_scale;
    // Retain transit speed, but reduce the launch and braking load on a
    // friction-held target. Approach, interception and empty moves keep their profile.
    if(command.kind==CommandKind::kMoveBinApproach &&
      config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model"){
      goal.max_acceleration_scale=std::min(goal.max_acceleration_scale,0.65);
    }

    if(moving){
      goal.target_track_id=moving->context.key.track_id;const auto time=now();const double age=std::chrono::duration<double>(time.source_now-moving->observation.timestamp).count();
      const double remaining=config().tracking.max_prediction_horizon_sec-age;
      if(age<0 || remaining<=0 || !track_is_usable(*moving,*settings.snapshot,time,true)){done(false);return;}
      goal.target_linear_velocity.x=moving->velocity_mps.x_m*velocity_fraction;
      goal.target_linear_velocity.y=moving->velocity_mps.y_m*velocity_fraction;
      goal.target_linear_velocity.z=moving->velocity_mps.z_m*velocity_fraction;
      goal.prediction_horizon=ptc_app::to_ros_duration(remaining);
    }
    goal.timeout=ptc_app::to_ros_duration(config().motion.planning_timeout_sec+config().motion.execution_timeout_sec);move_busy=true;
    send<Move>(move_client,goal,[this,done,move_view](auto result){if(!result||!result->success){
      record_observation_step("move_to_pose",result?"failed: code="+std::to_string(result->error_code)+" "+result->message:"failed: goal rejected or no result",task.context.key.track_id,move_view);
      move_busy=false;done(false);return;}
      record_observation_step("wait_for_observation_arrival","waiting: action succeeded; measured stopped feedback required",task.context.key.track_id,move_view);
      motion_completion=done;motion_settle_deadline=deadline_after(now().monotonic_now,config().motion.stop_confirmation_timeout_sec).value_or(now().monotonic_now);});
  }
  void wait_for_observation_arrival(const TimeContext & time)
  {
    if(!motion_completion){return;}
    const bool settled=arm_snapshot_ready(arm,*settings.snapshot,time)&&arm.stopped;
    if(!settled && time.monotonic_now<motion_settle_deadline){return;}
    record_observation_step("wait_for_observation_arrival",settled?"passed: fresh measured stopped arrival":"failed: stopped feedback deadline",
      task.context.key.track_id);
    auto done=std::move(motion_completion);motion_completion={};move_busy=false;done(settled);
  }
  void start_following_target(const EventKey & key,const Pose3d & target,const Vector3d & velocity,std::uint8_t purpose=Follow::Goal::INSPECTION)
  {
    record_observation_step("start_following_target","running: requesting Servo owner",key.track_id);
    follow_key=key;follow_command=runtime_command++;Follow::Goal goal;goal.key=to_ros(key);goal.command_id=follow_command;goal.target=ptc_app::stamped_pose(target,config().frames.world,now().source_now);
    last_target_stamp=now().source_now;
    goal.purpose=task.phase==TaskPhase::kPicking?goal.GRASP:purpose;
    goal.velocity.linear.x=velocity.x_m;goal.velocity.linear.y=velocity.y_m;goal.velocity.linear.z=velocity.z_m;goal.timeout=ptc_app::to_ros_duration(config().inspection.acquisition_timeout_sec+config().grasp.approach_timeout_sec+config().grasp.verification_timeout_sec);
    grasp_command_state.reset();
    if(goal.purpose==Follow::Goal::GRASP){
      grasp_command_state=GraspEntryState{target,velocity,{},from_ros(goal.target.header.stamp)};
    }
    grasp_follow_feedback.reset();follow_active=true;send<Follow>(follow_client,goal,[this](auto result){if(follow_active && (!result||!result->success)){
      follow_active=false;grasp_command_state.reset();
      if((task.phase==TaskPhase::kPreparingPick || task.phase==TaskPhase::kPicking) && task.grasp.possession==Possession::kEmpty){
        skip_observation_target("follow failed: "+(result?result->message:std::string("goal rejected/unavailable")));
      }else{fault("dynamic follow ended before requested stop");}
    }},true,[this](const Follow::Feedback & feedback){if(follow_active && same_operation(from_ros(feedback.key),follow_key)){grasp_follow_feedback=feedback;}});
  }
  std::optional<Pose3d> closing_motion_object(const TimeContext & time)const
  {
    // Continue an already committed close/probe through brief occlusion. This
    // is a motion reference, never a fresh visual observation or Holding proof.
    if(task.phase!=TaskPhase::kPicking || config().runtime.operation_mode!="assumed_ng_demo" ||
      (task.grasp.phase!=GraspPhase::kClosing && task.grasp.phase!=GraspPhase::kVerifying) ||
      expired(task.grasp.context.deadline,time) || expired(task.context.deadline,time) ||
      !same_track_identity(probe_baseline.context.key,task.context.key) ||
      probe_baseline.context.key.epoch!=time.epoch || !probe_baseline.velocity_valid)return std::nullopt;
    const double dt=std::chrono::duration<double>(time.source_now-probe_baseline.observation.timestamp).count();
    if(!std::isfinite(dt)||dt<0||!std::isfinite(vector_norm(probe_baseline.velocity_mps)))return std::nullopt;
    auto pose=probe_baseline.observation.pose_world;
    pose.position.x_m+=probe_baseline.velocity_mps.x_m*dt;
    pose.position.y_m+=probe_baseline.velocity_mps.y_m*dt;
    pose.position.z_m+=probe_baseline.velocity_mps.z_m*dt;
    return is_valid_pose(pose)?std::optional<Pose3d>{pose}:std::nullopt;
  }
  void stop_following_target()
  {follow_active=false;grasp_command_state.reset();if(stop_client->service_is_ready()){auto request=std::make_shared<ptc_interfaces::srv::StopArm::Request>();request->key=to_ros(event_key());request->command_id=runtime_command++;stop_client->async_send_request(request);}}
  void commands(const std::vector<Command> & list)
  {
    const bool includes_arm_stop=std::any_of(list.begin(),list.end(),[](const auto & command){return command.kind==CommandKind::kStopArm;});
    for(const auto & command:list){switch(command.kind){
      case CommandKind::kMoveObservation:case CommandKind::kMoveBinApproach:case CommandKind::kMoveBinRelease:
        if(config().grasp.bin_height && command.kind!=CommandKind::kMoveObservation){
          record_observation_step("bin_target_height","target_z_m="+std::to_string(command.target.position.z_m)+
            " offset_m="+std::to_string(task.grasp.bin_height_offset_m),command.key.track_id);
        }
        move_to_pose(command,[this,command](bool success){auto e=event(TaskEventKind::kMotionFinished,command.key.operation_id);e.motion_succeeded=success;apply(e);});break;
      case CommandKind::kLift:
        if(grasp_motion_reference || task.grasp.probe_origin_world_z_m){
          const auto measured=tcp_world();
          const double remaining=std::max(0.,command.target.position.z_m-measured.pose.position.z_m);
          const bool arrived=arm.stopped && pose_near(measured.pose,command.target,
            config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad);
          record_observation_step("lift_handoff","remaining_m="+std::to_string(remaining)+
            " direct_transport="+std::to_string(arrived),task.context.key.track_id);
          if(arrived){grasp(grasp_event(GraspEventKind::kLiftCompleted));break;}
        }
        move_to_pose(command,[this](bool success){grasp(grasp_event(success?GraspEventKind::kLiftCompleted:GraspEventKind::kLiftFailed));});
        break;
      case CommandKind::kStopArm:stop();break;
      case CommandKind::kStopGripper:if(!includes_arm_stop){stop();}break;
      case CommandKind::kCancelInference:grasp_capture_client->async_cancel_all_goals();break;
      case CommandKind::kStartGrasp:break; // Core emits the follow command in the same decision.
      case CommandKind::kStartFollow:
        if(follow_active && config().runtime.operation_mode=="assumed_ng_demo" && task.phase==TaskPhase::kPicking && same_track_identity(follow_key,command.key)){
          record_observation_step("start_following_target","retained same-object grasp follow owner; no Stop or second Follow action",command.key.track_id);
        }else{
          auto target=command.target;
          if(task.phase==TaskPhase::kPicking && grasp_pose.valid){
            const auto object=predict_object_pose(selected(),*settings.snapshot,now(),0);
            const auto pregrasp=object?compose_pose(*object,grasp_pose.pregrasp_in_object):std::nullopt;
            if(!pregrasp){skip_observation_target("moving pregrasp unavailable");break;}target=*pregrasp;
          }else if(grasp_follow_hold_in_object){const auto object=predict_object_pose(selected(),*settings.snapshot,now(),0);
            const auto hold=object?compose_pose(*object,*grasp_follow_hold_in_object):std::nullopt;
            if(!hold){skip_observation_target("moving pregrasp follow pose unavailable");break;}target=*hold;}
          if(grasp_approach_reference){const auto object=predict_object_pose(selected(),*settings.snapshot,now(),0);
            const auto sample=object?sample_grasp_approach(*grasp_approach_reference,*object,now().source_now):std::nullopt;
            if(!sample){skip_observation_target("grasp descent reference unavailable at Follow start");break;}target=sample->tcp_world;}
          start_following_target(command.key,target,command.linear_velocity);
        }break;
      case CommandKind::kCloseGripper:case CommandKind::kOpenGripper:grip(command);break;
      case CommandKind::kAttachObject:pending_attach=true;break; // Task owns the single Stop before attachment.
      case CommandKind::kDetachObject:scene_change(SceneEventKind::kDetachRequested);break;
      case CommandKind::kReport:save_outcome(command);break;
      case CommandKind::kRetireTrack:if(command.key.track_id){retired.emplace(CandidateKey{command.key.epoch,command.key.track_id},now().source_now);}break;
      default:break;
    }}
  }
  // Remaining boundary methods are kept with their owning Node, not inside core.
  void save_outcome(const Command &);
  void grip(const Command &);
  bool scene_change(SceneEventKind);
  void sync_scene(const Command &);
  CandidateKey candidate_key(const TrackedObjectState & track) const
  {return {track.context.key.epoch,track.context.key.track_id};}
  bool candidate_admission_due(const TrackedObjectState & track,const TimeContext & time)
  {
    auto & state=admission_attempts[candidate_key(track)];
    if(track.observation.timestamp<state.observation_stamp){return false;}
    if(track.observation.timestamp>state.observation_stamp){state.observation_stamp=track.observation.timestamp;state.attempts_for_observation=0;}
    return state.attempts_for_observation<config().task.max_candidate_admission_attempts_per_observation &&
      time.monotonic_now>=state.retry_after;
  }
  void record_candidate_admission_attempt(const TrackedObjectState & track,const TimeContext & time)
  {
    auto & state=admission_attempts[candidate_key(track)];
    if(track.observation.timestamp>state.observation_stamp){state.observation_stamp=track.observation.timestamp;state.attempts_for_observation=0;}
    ++state.attempts_for_observation;
    const auto next=deadline_after(time.monotonic_now,config().task.candidate_retry_interval_sec);
    state.retry_after=next.value_or(Nanoseconds::max());
    deferred_tracks.erase(candidate_key(track));
  }
  void capture_grasp_surface()
  {
    constexpr auto phase=TaskPhase::kPreparingPick;
    if(capture_busy || task.phase!=phase)return;
    const auto camera=compose_pose(tcp_world().pose,robot.camera_in_tcp);if(!camera)return;
    Capture::Goal goal;
    goal.command_id=runtime_command++;
    goal.key=to_ros(event_key(goal.command_id));
    goal.view_id="top";
    goal.requested_camera_world=ptc_app::stamped_pose(*camera,config().frames.world,now().source_now);
    goal.timeout=ptc_app::to_ros_duration(.3);capture_busy=true;
    send<Capture>(grasp_capture_client,goal,[this,goal](auto result){capture_busy=false;
      if(task.phase!=TaskPhase::kPreparingPick || !same_cycle(from_ros(goal.key),grasp_surface_cycle_key()))return;
      if(!result||!result->success||result->command_id!=goal.command_id||!same_operation(from_ros(result->key),from_ros(goal.key))){const auto detail=result?result->message:std::string("goal rejected/no result");record_observation_step("capture_grasp_surface","capture failed: "+detail,task.context.key.track_id,goal.view_id);return;}
      const auto & cloud=result->object_cloud;
      const auto expected_frame="object_"+std::to_string(task.context.key.track_id);
      if(cloud.header.frame_id!=expected_frame){record_observation_step("capture_grasp_surface","capture rejected: unexpected object frame="+cloud.header.frame_id,task.context.key.track_id,goal.view_id);return;}
      const auto object_at_capture=from_ros(result->object_at_capture);
      if(!same_track_identity(object_at_capture.context.key,from_ros(goal.key))){record_observation_step("capture_grasp_surface","capture rejected: object identity mismatch",task.context.key.track_id,goal.view_id);return;}
      const auto stamp=from_ros(cloud.header.stamp);
      std::vector<Vector3d> measured_points;
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
      for(;x!=x.end();++x,++y,++z){if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z))measured_points.push_back({*x,*y,*z});}
      if(measured_points.empty()){record_observation_step("capture_grasp_surface","capture rejected: zero finite measured object points",task.context.key.track_id,goal.view_id);return;}
      const auto object_inverse=inverse_pose(object_at_capture.observation.pose_world);
      const auto camera_at_capture=from_ros(result->frame.camera_world.pose);
      const auto camera_in_object=object_inverse?compose_pose(*object_inverse,camera_at_capture):std::nullopt;
      const auto camera_position_object=camera_in_object?std::optional<Vector3d>(camera_in_object->position):std::nullopt;
      if(demo_grasp_surface.points.empty()||!same_track_identity(demo_grasp_surface.key,object_at_capture.context.key)){
        demo_grasp_surface={};demo_grasp_surface.key=from_ros(result->key);
        demo_grasp_surface.object_frame=cloud.header.frame_id;demo_grasp_surface.settings_revision=settings.snapshot->revision();}
      demo_grasp_surface.revision++;
      demo_grasp_surface.neighbor_stamp=stamp;
      demo_grasp_surface.neighbor_capture_ms=result->neighbor_capture_ms;
      demo_grasp_surface.neighbor_offsets_world.clear();
      for(const auto & point:result->neighbor_offsets_world)
        demo_grasp_surface.neighbor_offsets_world.push_back({point.x,point.y,point.z});
      for(const auto & point:measured_points){if(demo_grasp_cloud.size()>=60000)break;
        demo_grasp_cloud.push_back(point);std::optional<Vector3d> view_direction;
        if(camera_position_object){const auto ray=Vector3d{camera_position_object->x_m-point.x_m,
          camera_position_object->y_m-point.y_m,camera_position_object->z_m-point.z_m};
          const auto length=vector_norm(ray);if(std::isfinite(length)&&length>1e-6)
            view_direction=Vector3d{ray.x_m/length,ray.y_m/length,ray.z_m/length};}
        demo_grasp_surface.points.push_back({point,view_direction,stamp});}
      record_observation_step("capture_grasp_surface","view="+goal.view_id+" valid_points="+std::to_string(measured_points.size())+" accumulated_masked_points="+std::to_string(demo_grasp_cloud.size())+
        " source_stamp_ns="+std::to_string(stamp.count())+" camera_pose="+std::to_string(camera_position_object.has_value()),task.context.key.track_id,goal.view_id);
      prepare_grasp_candidates(demo_grasp_surface);
    });
  }
  void prepare_grasp_candidates(const GraspSurfaceSnapshot & surface)
  {
    if(surface.points.empty()){return;}
    const auto identity=std::make_pair(surface.key.track_id,surface.revision);
    if(grasp_proposal_completed==identity || grasp_proposal_pending==identity){return;}
    if(grasp_proposal_completed!=identity){grasp_candidates.clear();grasp_candidate_index=0;}
    const auto track=selected();const auto time=now();const auto settings_snapshot=*settings.snapshot;
    const auto gripper=configured_gripper_geometry(settings_snapshot,robot.collision_geometry_digest);const auto issued_generation=generation;
    const auto submitted_at=std::chrono::steady_clock::now();
    const auto clearance=config().grasp.pregrasp_clearance_m;
    const bool queued=grasp_worker.enqueue([this,surface,track,time,settings_snapshot,gripper,issued_generation,submitted_at,clearance,identity]() mutable {
      GraspProposalResult proposal;
      try{proposal=propose_top_view_grasp_candidates(surface,track,gripper,settings_snapshot,time,clearance);}
      catch(const std::exception & error){proposal.key=surface.key;proposal.surface_revision=surface.revision;
        proposal.object_frame=surface.object_frame;proposal.gripper_model_id=gripper.model_id;
        proposal.collision_geometry_digest=gripper.collision_geometry_digest;
        proposal.status=GraspSearchStatus::kInvalidInput;proposal.reason=std::string("candidate search exception: ")+error.what();}
      catch(...){proposal.key=surface.key;proposal.surface_revision=surface.revision;
        proposal.object_frame=surface.object_frame;proposal.gripper_model_id=gripper.model_id;
        proposal.collision_geometry_digest=gripper.collision_geometry_digest;
        proposal.status=GraspSearchStatus::kInvalidInput;proposal.reason="candidate search exception: unknown error";}
      completions.post([this,proposal=std::move(proposal),issued_generation,identity,submitted_at]() mutable {
        const auto worker_wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-submitted_at).count();
        const auto latency_summary=record_grasp_proposal_timing(worker_wall_ms);
        if(grasp_proposal_pending==identity){grasp_proposal_pending.reset();}
        if(generation!=issued_generation || task.phase==TaskPhase::kWaitingReady || task.phase==TaskPhase::kFaulted ||
          !same_track_identity(selected().context.key,proposal.key) ||
          proposal.surface_revision!=demo_grasp_surface.revision ||
          proposal.key.settings_revision!=settings.snapshot->revision()){
          record_observation_step("prepare_grasp_candidates","discarded late proposal; current track/revision/settings changed; "+latency_summary,
            proposal.key.track_id);return;}
        const auto current_object=predict_object_pose(selected(),*settings.snapshot,now(),config().grasp.intercept_lead_sec);
        if(!current_object){grasp_proposal_completed=identity;
          record_observation_step("prepare_grasp_candidates","discarded result; current visual intercept is no longer usable",
            proposal.key.track_id);
          if(task.phase==TaskPhase::kPreparingPick){auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);}return;}
        for(auto & candidate:proposal.candidates){const auto world=compose_pose(*current_object,candidate.tcp_in_object);
          if(world){candidate.tcp_world=*world;}else{candidate.valid=false;}}
        proposal.candidates.erase(std::remove_if(proposal.candidates.begin(),proposal.candidates.end(),
          [](const auto & candidate){return !candidate.valid;}),proposal.candidates.end());
        // Preserve the core's supported-contact and approach priority. A small
        // camera-to-grasp rotation does not establish clearance above the belt;
        // the backend evaluates actual joint travel and collision feasibility.
        order_grasp_candidates_for_wrist(proposal.candidates,tcp_world().pose.orientation);
        // Prefer opposed support, retain the wrist order for occluded
        // candidates, and demote an observed pad without aligned support.
        const auto pad_rank=[](const GraspCandidate & candidate){
          const auto & pads=candidate.pad_contact_coverage;
          if((pads[0] && *pads[0]<=0.) || (pads[1] && *pads[1]<=0.))return std::pair{2,0.};
          if(pads[0] && pads[1])return std::pair{0,-std::min(*pads[0],*pads[1])};
          return std::pair{1,0.};
        };
        std::stable_sort(proposal.candidates.begin(),proposal.candidates.end(),[&](const auto & a,const auto & b){
          return pad_rank(a)<pad_rank(b);
        });
        const auto bilateral=std::count_if(proposal.candidates.begin(),proposal.candidates.end(),[](const auto & candidate){
          return candidate.pad_contact_coverage[0] && candidate.pad_contact_coverage[1];
        });
        proposal.reason+="; bilateral_pad_candidates="+std::to_string(bilateral);
        const auto count=proposal.candidates.size();grasp_candidates=std::move(proposal.candidates);grasp_candidate_index=0;
        grasp_proposal_completed=identity;
        record_observation_step("prepare_grasp_candidates","surface_revision="+std::to_string(proposal.surface_revision)+
          " points="+std::to_string(proposal.input_point_count)+" candidates="+std::to_string(count)+
          " compute_ms="+std::to_string(proposal.elapsed_ms)+" worker_wall_ms="+std::to_string(worker_wall_ms)+" "+latency_summary+
          " status="+std::to_string(static_cast<int>(proposal.status))+"; "+proposal.reason,
          proposal.key.track_id);
        if(task.phase==TaskPhase::kPreparingPick && grasp_candidates.empty()){
          auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);}
      });
    });
    if(queued){grasp_proposal_pending=identity;}
    else{record_observation_step("prepare_grasp_candidates","bounded geometry worker is busy; latest revision will be retried by the task owner",
      surface.key.track_id);}
  }
  void open_for_grasp_preparation()
  {
    if(observation_open_command || task.phase!=TaskPhase::kPreparingPick ||
      task.grasp.possession!=Possession::kEmpty)return;
    // The waiting open may still be running when a target is selected. Use
    // that motion and its measured result; a second goal is rejected as busy.
    if(now().monotonic_now<next_waiting_open)return;
    Grip::Goal goal;goal.key=to_ros(event_key(runtime_command++));goal.command_id=goal.key.operation_id;
    goal.open=true;goal.width_m=config().grasp.open_width_m;goal.max_effort_n=config().grasp.max_effort_n;
    goal.timeout=ptc_app::to_ros_duration(config().grasp.release_timeout_sec);observation_open_command=goal.command_id;
    record_observation_step("open_for_top_observation","requested width_m="+std::to_string(goal.width_m)+"; measured stopped/Empty required before grasp preparation",task.context.key.track_id,"top");
    const bool started_empty=gripper_snapshot_ready(gripper,*settings.snapshot,now()) &&
      gripper.stopped && gripper.empty_confirmed && !gripper.contact_confirmed;
    send<Grip>(grip_client,goal,[this,goal,started_empty](auto result){
      if(observation_open_command!=goal.command_id)return;
      const bool success=result&&result->success&&result->command_id==goal.command_id&&same_operation(from_ros(result->key),from_ros(goal.key));
      if(!success)observation_open_command.reset();
      if(!success){
        // This opening started with measured Empty and never advanced to a close.
        recover_pregrasp_open=started_empty && task.phase==TaskPhase::kPreparingPick;
        const auto detail=result && !result->message.empty()?result->message:"action did not complete";
        skip_observation_target("grasp preparation opening failed: "+detail);
        return;
      }
      record_observation_step("open_for_top_observation","action completed; actual width_m="+std::to_string(gripper.width_m)+"; this is preparation, not object release",task.context.key.track_id,"top");
    });
  }
  void request_measured_empty_reset()
  {
    // A background scene upload can still own the current revision. Retry
    // from the existing recovery tick; never claim reset/reload before ACK.
    if(!scene_change(SceneEventKind::kResetRequested))return;
    pending_reset=true;completions.clear_fault();scene_initialized=false;
    auto reset=std::make_shared<ptc_interfaces::srv::StopArm::Request>();reset->key=to_ros(event_key());reset->command_id=runtime_command++;reset->reset_fault=true;
    stop_client->async_send_request(reset,[this,reset_epoch=reset->key.epoch](rclcpp::Client<ptc_interfaces::srv::StopArm>::SharedFuture result){
      if(reset_epoch==now().epoch && !result.get()->accepted){pending_reset=false;reason="gateway reset rejected";}
    });
  }
  bool manual_home_pending() const
  {
    return manual_home_state==Status::HOME_QUEUED || manual_home_state==Status::HOME_RESETTING ||
      manual_home_state==Status::HOME_MOVING;
  }
  void request_manual_home(ptc_interfaces::srv::TaskCommand::Response & response)
  {
    // Return Home is a reset request. It is accepted in every task state and
    // then waits for the measured conditions that motion itself requires.
    const auto time=now();
    if(manual_home_pending()){response.accepted=true;response.message="Return Home already in progress";return;}
    // The manual request waits within a shorter overall budget. Automatic
    // recovery already uses its own reduced budget and is counted once.
    const double manual_home_wait_budget=0.5*(config().task.cycle_timeout_sec+
      config().motion.planning_timeout_sec+config().motion.execution_timeout_sec);
    const auto deadline=deadline_after(time.monotonic_now,
      manual_home_wait_budget+config().task.recovery_timeout_sec);
    if(!deadline){response.message="Return Home deadline unavailable";return;}
    continuous_operation=false;
    failed_bin_recovery_targets.clear();recovery_bin_height_offset.reset();recovery_bin_orientation.reset();
    manual_home_deadline=*deadline;
    manual_home_state=Status::HOME_QUEUED;
    manual_home_message="Waiting for the current cycle and measured Empty";
    // An attempt that has not closed on anything is interrupted now. A cycle
    // that is closing, carrying or placing finishes through its own path first.
    const bool approaching=task.phase==TaskPhase::kPicking && task.grasp.phase==GraspPhase::kApproaching;
    if((task.phase==TaskPhase::kObserving || task.phase==TaskPhase::kPreparingPick || approaching) &&
      task.grasp.possession==Possession::kEmpty){
      apply(event(TaskEventKind::kCancelRequested));
    }
    response.accepted=true;response.message="Return Home accepted; waiting for measured arrival";
  }
  // An idle arm keeps its gripper at the open waiting width, whichever path
  // brought it there. Measured contact is unresolved possession and stays
  // with the existing recovery.
  void open_while_waiting(const TimeContext & time)
  {
    const bool waiting=task.phase==TaskPhase::kWaitingReady ||
      (task.phase==TaskPhase::kObserving && !task.context.key.track_id);
    if(!waiting || pending_reset || time.paused || time.monotonic_now<next_waiting_open || observation_open_command ||
      !gripper_snapshot_ready(gripper,*settings.snapshot,time) || !gripper.stopped || gripper.contact_confirmed ||
      gripper_measured_open_empty(gripper,config()) || !grip_client->action_server_is_ready())return;
    next_waiting_open=deadline_after(time.monotonic_now,config().grasp.release_timeout_sec).value_or(time.monotonic_now);
    Grip::Goal goal;goal.key=to_ros(event_key(runtime_command++));goal.command_id=goal.key.operation_id;
    goal.open=true;goal.width_m=config().grasp.open_width_m;goal.max_effort_n=config().grasp.max_effort_n;
    goal.timeout=ptc_app::to_ros_duration(config().grasp.release_timeout_sec);
    record_observation_step("open_while_waiting","measured width_m="+std::to_string(gripper.width_m)+
      " requested width_m="+std::to_string(goal.width_m)+"; no contact measured",task.context.key.track_id);
    // Unscoped: a target selected during the motion must still learn that it
    // ended. The time limit above covers a result that never arrives.
    send<Grip>(grip_client,goal,[this](auto result){
      if(result && result->success){next_waiting_open=Nanoseconds{0};return;}
      record_observation_step("open_while_waiting","opening did not complete: "+
        (result?result->message:std::string("goal rejected/no result")),task.context.key.track_id);
    },false);
  }
  void advance_manual_home(const TimeContext & time)
  {
    if(!manual_home_pending()){return;}
    if(expired(manual_home_deadline,time)){
      const bool moving=manual_home_state==Status::HOME_MOVING;
      manual_home_state=Status::HOME_FAILED;manual_home_message="Return Home timed out before measured arrival";
      if(moving){stop();}
      return;
    }
    if(manual_home_state==Status::HOME_MOVING || pending_reset || time.paused){return;}
    if(manual_home_state==Status::HOME_RESETTING){
      if(task.phase==TaskPhase::kFaulted || !motion_command_ready){
        manual_home_state=Status::HOME_FAILED;manual_home_message="Gateway reset failed before Return Home";
        return;
      }
      manual_home_state=Status::HOME_QUEUED;
    }
    if((task.phase!=TaskPhase::kWaitingReady && task.phase!=TaskPhase::kFaulted) ||
      move_busy || follow_active || sync_busy || !arm_snapshot_ready(arm,*settings.snapshot,time) || !arm.stopped ||
      !gripper_snapshot_ready(gripper,*settings.snapshot,time) || !gripper.stopped || !gripper.empty_confirmed ||
      ack_scene.attachment!=AttachmentPhase::kDetached || scene.attachment!=AttachmentPhase::kDetached){return;}
    if(task.phase==TaskPhase::kFaulted || arm.state.motion_state==ArmMotionState::kFault){
      if(!stop_client->service_is_ready()){return;}
      request_measured_empty_reset();
      if(pending_reset){
        manual_home_state=Status::HOME_RESETTING;manual_home_message="Reconciling the stopped empty arm";
      }
      return;
    }
    if(!motion_command_ready || !scene_ready(ack_scene,*settings.snapshot,time) ||
      arm.state.motion_state!=ArmMotionState::kIdle){return;}
    if(pose_near(arm.state.tcp_pose,config().motion.observation_pose,
      config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad)){
      manual_home_state=Status::HOME_ARRIVED;manual_home_message="Measured stopped at Home";return;
    }
    Command command;command.kind=CommandKind::kMoveObservation;command.key=event_key(runtime_command++);
    command.key.track_id=0;command.command_id=command.key.operation_id;
    command.target=config().motion.observation_pose;command.reference_frame=config().frames.robot_base;
    manual_home_state=Status::HOME_MOVING;manual_home_message="Moving to Home";
    move_to_pose(command,[this](bool arrived){
      if(manual_home_state!=Status::HOME_MOVING){return;}
      const auto time=now();
      const bool home=arrived && !time.paused && arm.stopped &&
        gripper_snapshot_ready(gripper,*settings.snapshot,time) && gripper.stopped && gripper.empty_confirmed &&
        pose_near(arm.state.tcp_pose,config().motion.observation_pose,
          config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad);
      manual_home_state=home?Status::HOME_ARRIVED:Status::HOME_FAILED;
      manual_home_message=home?"Measured stopped at Home":"Home motion failed or measured pose is outside tolerance";
      if(!home && (!arm_snapshot_ready(arm,*settings.snapshot,time) || !arm.stopped)){stop();}
    });
  }
  void retreat_failed_demo_grasp()
  {
    const auto time=now();
    if(!arm_snapshot_ready(arm,*settings.snapshot,time) || !arm.stopped || move_busy ||
      !gripper_snapshot_ready(gripper,*settings.snapshot,time) || !gripper.stopped)return;
    if(!gripper.empty_confirmed && !observation_open_command && config().grasp.fingertip_box_tcp){
      const auto measured=tcp_world().pose;
      const auto geometry=configured_gripper_geometry(*settings.snapshot,robot.collision_geometry_digest);
      const auto target=grasp_recovery_retreat(measured,geometry,config().vision.depth_workspace.support_plane_z_m,
        config().grasp.support_plane_clearance_m,config().grasp.pregrasp_clearance_m);
      const auto gap=fingertip_support_clearance(measured,geometry,config().vision.depth_workspace.support_plane_z_m);
      // Retreat only from insufficient clearance, not merely because an extra
      // pregrasp-sized buffer is absent. A clear stopped hand can open now.
      if(target && gap && *gap<config().grasp.support_plane_clearance_m){
        if(!motion_command_ready)return; // Wait for the existing gateway owner/scene; do not spam rejected goals.
        if(recovery_retreat_command)return; // The in-flight request is cleared on completion.
        Command retreat;retreat.key=event_key(runtime_command++);retreat.command_id=retreat.key.operation_id;
        retreat.target=*target;retreat.reference_frame=config().frames.world;recovery_retreat_command=retreat;
        record_observation_step("recovery_retreat","request upward support clearance; possession unchanged",task.context.key.track_id);
        move_to_pose(retreat,[this](bool arrived){
          recovery_retreat_command.reset();
          reason=arrived?"recovery retreat measured; opening may continue":"recovery retreat rejected/failed; possession unchanged";
          record_observation_step("recovery_retreat",reason,task.context.key.track_id);
          if(arrived)reopen_failed_demo_grasp();
          else if(!arm_snapshot_ready(arm,*settings.snapshot,now()) || !arm.stopped ||
            arm.state.motion_state!=ArmMotionState::kIdle){
            stop(); // An accepted/moving failure still requires measured stops.
          }
          // A rejected plan at an already idle hand must not issue another
          // StopArm that cancels a subsequent recovery opening.
        });
        return;
      }
    }
  }
  void reopen_failed_demo_grasp()
  {
    // A failed/inconclusive pick may still physically hold an object. Keep
    // the jaws closed, carry its measured geometry to the bin, then release.
    // The conservative collision attachment is not a Holding confirmation.
    const auto time=now();
    const bool invalidated_cycle=task.context.key.epoch!=time.epoch;
    // A pending Return Home also owns this recovery: a carry that was
    // stopped in place is taken to the bin and released before Home.
    if(pending_reset || (task.phase!=TaskPhase::kFaulted && task.phase!=TaskPhase::kRecovering) ||
      (task.outcome!=CycleOutcome::kFailed && task.outcome!=CycleOutcome::kInconclusive && !manual_home_pending()) ||
      !gripper_snapshot_ready(gripper,*settings.snapshot,time))return;
    const bool moving_to_bin = move_busy && !motion_completion && recovery_retreat_command &&
      (recovery_retreat_command->kind==CommandKind::kMoveBinApproach ||
       recovery_retreat_command->kind==CommandKind::kMoveBinRelease);
    if(moving_to_bin && !invalidated_cycle && !time.paused && recovery_stop_ack_stamp &&
      gripper.timestamp>*recovery_stop_ack_stamp && gripper.stopped && gripper.empty_confirmed &&
      !gripper.contact_confirmed && stop_client->service_is_ready()){
      record_observation_step("recover_failed_grasp","fresh measured Empty during bin recovery; stop detour and recover Home",task.context.key.track_id);
      stop();
      return;
    }
    if(!arm_snapshot_ready(arm,*settings.snapshot,time) || !arm.stopped || move_busy)return;
    // A rewound cycle cannot drive recovery motion with its old target. Only
    // fresh stopped/Empty feedback can reconcile it into the new epoch.
    if(invalidated_cycle && (time.paused || !gripper.stopped || !gripper.empty_confirmed))return;
    // Preserve the selected obstacle while possession is unresolved; ordinary
    // expiry resumes after a measured Empty release.
    if(!sync_busy && !scene.context.inhibited){scene_change(SceneEventKind::kExpire);}
    if(sync_busy)return;
    if(gripper.empty_confirmed && !observation_open_command){
      pending_attach=false;attachment_ack_joint_stamp.reset();
      if(scene.attachment!=AttachmentPhase::kDetached || ack_scene.attachment!=AttachmentPhase::kDetached){
        if(!sync_busy && scene_change(SceneEventKind::kDetachRequested)){
          record_observation_step("recover_failed_grasp","measured Empty; removing stale planning attachment before Home",task.context.key.track_id);
        }
        return;
      }
      if(stop_client->service_is_ready() && (invalidated_cycle || task.phase==TaskPhase::kFaulted ||
        (arm.state.motion_state==ArmMotionState::kIdle &&
         (!motion_command_ready || !scene_ready(ack_scene,*settings.snapshot,time))))){
        request_measured_empty_reset();
      }
      return;
    }
    // An accepted open waits for measured completion; do not restart the fingers every tick.
    if(observation_open_command || pending_release)return;
    // A failed preparation open can leave the jaws partially open and Unknown.
    // Wait for the stop to finish before issuing one open from measured Empty provenance.
    if(recover_pregrasp_open && !gripper.contact_confirmed){
      if(!recovery_open_failed && !invalidated_cycle && !time.paused && recovery_stop_ack_stamp &&
        gripper.stopped && gripper.timestamp>*recovery_stop_ack_stamp && grip_client->action_server_is_ready()){
        Command open;open.key=event_key(runtime_command++);open.command_id=open.key.operation_id;
        open.kind=CommandKind::kOpenGripper;open.width_m=config().grasp.open_width_m;open.effort_n=config().grasp.max_effort_n;
        observation_open_command=open.command_id;
        record_observation_step("recover_failed_grasp","pre-grasp opening interrupted: retry once before measured Empty and Home",task.context.key.track_id);
        grip(open);
      }
      return;
    }
    // A fresh fully closed, contact-free miss has no object to carry to the bin.
    // Try opening in place once. If blocked, use the existing bin recovery path.
    if(!recovery_open_failed && task.grasp.possession==Possession::kEmpty && gripper.stopped && !gripper.contact_confirmed &&
      gripper_measured_closed_empty(gripper,config())){
      Command open;open.key=event_key(runtime_command++);open.command_id=open.key.operation_id;
      open.kind=CommandKind::kOpenGripper;open.width_m=config().grasp.open_width_m;open.effort_n=config().grasp.max_effort_n;
      observation_open_command=open.command_id;
      record_observation_step("recover_failed_grasp","fully closed empty miss: open before Home",task.context.key.track_id);
      grip(open);return;
    }
    recover_grasp_to_bin(time);
  }

  void recover_grasp_to_bin(const TimeContext & time)
  {
    if(time.paused || time.monotonic_now<next_bin_recovery_attempt)return;
    auto recovery_grasp=task.grasp;
    // Freeze one measured pile offset for approach and release. New samples
    // must not ratchet the destination upward while recovering the same object.
    if(!recovery_bin_height_offset){
      recovery_bin_height_offset=std::max(task.grasp.bin_height_offset_m,measured_bin_offset());
      recovery_bin_orientation=arm.state.tcp_pose.orientation;
    }
    recovery_grasp.bin_height_offset_m=*recovery_bin_height_offset;
    auto release=bin_target_pose(config().grasp.bin_release_pose,recovery_grasp,*settings.snapshot);
    release.orientation=*recovery_bin_orientation;
    auto configured_release=release;configured_release.orientation=config().grasp.bin_release_pose.orientation;
    if(!bin_recovery_arrived(arm.state.tcp_pose,release,configured_release,config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad)){
      if(scene.attachment!=AttachmentPhase::kAttached || !scene.coherent || ack_scene.attachment!=AttachmentPhase::kAttached){
        if(!sync_busy && scene_change(SceneEventKind::kAttachRequested)){
          record_observation_step("recover_failed_grasp","uncertain possession: reconcile measured geometry for bin recovery, jaws remain closed",task.context.key.track_id);
        }
        return;
      }
      if(!motion_command_ready){
        if(arm.state.motion_state==ArmMotionState::kFault && stop_client->service_is_ready() && !recovery_retreat_command){
          Command reset;reset.key=event_key(runtime_command++);reset.command_id=reset.key.operation_id;recovery_retreat_command=reset;
          auto request=std::make_shared<ptc_interfaces::srv::StopArm::Request>();request->key=to_ros(reset.key);request->command_id=reset.command_id;request->reset_fault=true;
          stop_client->async_send_request(request,[this](rclcpp::Client<ptc_interfaces::srv::StopArm>::SharedFuture result){recovery_retreat_command.reset();reason=result.get()->accepted?"bin recovery: motion fault reset after measured stop":"bin recovery: motion reset rejected; jaws remain closed";});
        }
        return;
      }
      if(recovery_retreat_command)return;
      auto approach=bin_target_pose(config().grasp.bin_approach_pose,recovery_grasp,*settings.snapshot);approach.orientation=*recovery_bin_orientation;
      auto configured_approach=approach;configured_approach.orientation=config().grasp.bin_approach_pose.orientation;
      const bool above_bin=bin_recovery_arrived(arm.state.tcp_pose,approach,configured_approach,config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad);
      const auto preferred=above_bin?release:approach;
      auto configured=preferred;
      configured.orientation=above_bin?config().grasp.bin_release_pose.orientation:config().grasp.bin_approach_pose.orientation;
      const auto target=select_bin_recovery_target(preferred,configured,failed_bin_recovery_targets,
        config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad);
      if(!target){
        continuous_operation=false;
        const std::string message="bin recovery candidates failed; jaws remain closed; Return Home retries after correction";
        if(task.phase!=TaskPhase::kFaulted){fault(message);}else{reason=message;}
        return;
      }
      Command move;move.key=event_key(runtime_command++);move.command_id=move.key.operation_id;move.reference_frame=config().frames.robot_base;move.target=*target;
      move.kind=above_bin?CommandKind::kMoveBinRelease:CommandKind::kMoveBinApproach;
      recovery_retreat_command=move;
      record_observation_step("recover_failed_grasp",above_bin?"move held/uncertain object to bin release":"move held/uncertain object above bin",task.context.key.track_id);
      move_to_pose(move,[this,target=move.target](bool arrived){
        recovery_retreat_command.reset();
        if(!arrived){
          failed_bin_recovery_targets.push_back(target);
          const auto time=now();
          next_bin_recovery_attempt=deadline_after(time.monotonic_now,
            config().task.candidate_retry_interval_sec).value_or(time.monotonic_now);
        }
        reason=arrived?"bin recovery move completed":"bin recovery target failed; preserve gripper and try configured orientation";
      });
      return;
    }
    Command open;open.key=event_key(runtime_command++);open.command_id=open.key.operation_id;
    open.kind=CommandKind::kOpenGripper;open.width_m=config().grasp.open_width_m;open.effort_n=config().grasp.max_effort_n;
    observation_open_command=open.command_id;
    record_observation_step("recover_failed_grasp","at bin release pose: open, then measured Empty, detach, Home and next target",task.context.key.track_id);
    if(open.key.track_id){bin_release_pub->publish(to_ros(open.key));}
    grip(open);
  }
  void check_grasp_candidate()
  {
    const auto time=now();
    if(grasp_candidates.empty()){
      GraspSurfaceSnapshot surface=demo_grasp_surface;
      if(surface.points.empty()){auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);return;}
      const auto identity=std::make_pair(surface.key.track_id,surface.revision);
      prepare_grasp_candidates(surface);
      if(grasp_candidates.empty()){
        if(grasp_proposal_completed==identity){auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);}
        return;
      }
    }
    if(checking_target||grasp_candidate_index>=grasp_candidates.size()){if(grasp_candidate_index>=grasp_candidates.size()&&!checking_target){auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);}return;}
    if(sync_busy||!ack_scene.coherent||ack_grasp_surface_revision!=demo_grasp_surface.revision||
      !same_cycle(ack_grasp_surface_key,grasp_surface_cycle_key())){
      RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,
        "grasp scene handoff waiting: sync_busy=%d coherent=%d surface=%llu acknowledged=%llu same_cycle=%d scene_objects=%zu surface_frame=%s",
        sync_busy,ack_scene.coherent,static_cast<unsigned long long>(demo_grasp_surface.revision),
        static_cast<unsigned long long>(ack_grasp_surface_revision),same_cycle(ack_grasp_surface_key,grasp_surface_cycle_key()),
        scene.dynamic_objects.size(),demo_grasp_surface.object_frame.c_str());
      const auto current=selected();
      // The wrist capture provides new geometry even if the tracked pose has
      // not changed. Upload it without relabeling the last visual timestamp.
      if(!sync_busy && ack_scene.coherent && track_is_usable(current,*settings.snapshot,time)){
        scene_change(SceneEventKind::kRefresh);
      }
      RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,
        "grasp scene detail: initialized=%d local_coherent=%d inhibited=%d track_phase=%d track_usable=%d visual_age=%.6f scene_reason=%s",
        scene_initialized,scene.coherent,scene.context.inhibited,static_cast<int>(current.phase),track_is_usable(current,*settings.snapshot,time),
        std::chrono::duration<double>(time.source_now-current.observation.timestamp).count(),
        scene.context.history.empty()?"none":scene.context.history.back().reason.c_str());
      return;
    }
    // Feasibility uses the acknowledged scene's object pose, not a future
    // contact against an older obstacle. The actual approach below recomputes
    // the intercept from the current visual track and carries moving prediction.
    const auto acknowledged=std::find_if(ack_scene.dynamic_objects.begin(),ack_scene.dynamic_objects.end(),[&](const auto & item){return same_track_identity(item.key,task.context.key);});
    if(acknowledged==ack_scene.dynamic_objects.end())return;
    const auto object=std::optional<Pose3d>{acknowledged->observation.pose_world};
    Evaluate::Goal goal;goal.key=to_ros(event_key(runtime_command++));goal.command_id=goal.key.operation_id;
    goal.scene_revision=ack_scene.revision;goal.surface_revision=demo_grasp_surface.revision;
    goal.timeout=ptc_app::to_ros_duration(config().motion.planning_timeout_sec);
    std::map<std::uint32_t,std::size_t> candidate_indices;
    const bool semantic_candidates=config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model";
    for(std::size_t index=grasp_candidate_index;index<grasp_candidates.size()&&goal.candidates.size()<16&&
      (semantic_candidates||goal.candidates.empty());++index){
      const auto & candidate=grasp_candidates[index];if(!candidate.valid||candidate.surface_revision!=goal.surface_revision){continue;}
      const auto target=compose_pose(*object,candidate.tcp_in_object);const auto pregrasp=compose_pose(*object,candidate.pregrasp_in_object);
      if(!target||!pregrasp||!inside_workspace(*target,config().motion)||!inside_workspace(*pregrasp,config().motion)){continue;}
      ptc_interfaces::msg::GraspCandidate item;item.candidate_id=static_cast<std::uint32_t>(index+1);
      item.closure_width_m=candidate.opening_width_m;
      item.pregrasp=ptc_app::stamped_pose(*pregrasp,config().frames.world,time.source_now);
      item.contact=ptc_app::stamped_pose(*target,config().frames.world,time.source_now);
      bool patches_valid=true;
      for(const auto & patch:{candidate.left_contact,candidate.right_contact}){
        Pose3d center;center.position=patch.center_object_m;const auto world_center=compose_pose(*object,center);
        if(!world_center||!std::isfinite(patch.support_radius_m)||patch.support_radius_m<=0){patches_valid=false;break;}
        item.contact_patch_centers.push_back(ptc_app::stamped_pose(*world_center,config().frames.world,time.source_now));
        item.contact_patch_radii_m.push_back(patch.support_radius_m);
      }
      if(!patches_valid){continue;}candidate_indices[item.candidate_id]=index;goal.candidates.push_back(std::move(item));
    }
    if(goal.candidates.empty()){auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);return;}
    if(semantic_candidates){add_grasp_neighborhood(goal,object->position,time);}
    // The semantic batch gets a cheap measured-surface/tool clearance ranking.
    // The backend runs IK only until its first feasible candidate, preserving
    // the existing selection/approach owner without checking unused alternatives.
    checking_target=true;const auto batch_started=std::chrono::steady_clock::now();
    record_observation_step("evaluate_grasp_candidates","dispatch candidate="+std::to_string(goal.candidates.front().candidate_id)+
      " scene_revision="+std::to_string(goal.scene_revision)+" surface_revision="+std::to_string(goal.surface_revision)+
      " contact_world=["+std::to_string(goal.candidates.front().contact.pose.position.x)+","+std::to_string(goal.candidates.front().contact.pose.position.y)+","+std::to_string(goal.candidates.front().contact.pose.position.z)+"]"+
      " expected_contact_span_m="+std::to_string(goal.candidates.front().closure_width_m),task.context.key.track_id);
    send<Evaluate>(evaluate_client,goal,[this,goal,candidate_indices,batch_started](auto result){
      checking_target=false;if(task.phase!=TaskPhase::kPreparingPick)return;
      if(!result){
        record_observation_step("evaluate_grasp_candidates","request not accepted or no result; retain candidate and retry with current scene within the existing preparation deadline",
          task.context.key.track_id);
        return;
      }
      if(goal.surface_revision!=demo_grasp_surface.revision){
        record_observation_step("evaluate_grasp_candidates","surface changed while checking; use latest measured revision",task.context.key.track_id);return;}
      const bool complete=result&&result->complete&&result->command_id==goal.command_id&&
        result->scene_revision==goal.scene_revision&&result->surface_revision==goal.surface_revision&&
        same_operation(from_ros(result->key),from_ros(goal.key));
      const double wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-batch_started).count();
      if(!complete){record_observation_step("evaluate_grasp_candidates","batch incomplete wall_ms="+std::to_string(wall_ms)+
        (result?"; "+result->message:"; action unavailable/rejected"),task.context.key.track_id);
        auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);return;}
      std::optional<std::size_t> selected_index;std::string summary;
      for(const auto & feasibility:result->results){const auto found=candidate_indices.find(feasibility.candidate_id);
        if(found==candidate_indices.end()){continue;}
        if(summary.size()<700){summary+=" id="+std::to_string(feasibility.candidate_id)+":"+
          (feasibility.feasible?"feasible":feasibility.failure_reason)+";";}
        if(feasibility.evaluated&&feasibility.feasible&&!selected_index){selected_index=found->second;
          record_observation_step("evaluate_grasp_candidates","selected="+std::to_string(feasibility.candidate_id)+
            " min_joint_margin_rad="+std::to_string(feasibility.minimum_joint_margin_rad)+
            " pregrasp_time_lower_bound_sec="+std::to_string(feasibility.pregrasp_time_lower_bound_sec)+
            " batch_wall_ms="+std::to_string(wall_ms),task.context.key.track_id);}
      }
      if(!selected_index){record_observation_step("evaluate_grasp_candidates","candidate infeasible wall_ms="+
        std::to_string(wall_ms)+";"+summary,task.context.key.track_id);
        grasp_candidate_index=candidate_indices.rbegin()->second+1;
        if(grasp_candidate_index>=grasp_candidates.size()){auto e=event(TaskEventKind::kMotionFinished);e.motion_succeeded=false;apply(e);}
        return;}
      grasp_candidate_index=*selected_index;grasp_pose=grasp_candidates[grasp_candidate_index];
      const auto pad_text=[](const std::optional<double> & pad){
        return pad?std::to_string(*pad):std::string("unobserved");
      };
      record_observation_step("select_grasp_aperture",
        "approach_open_width_m="+std::to_string(config().grasp.open_width_m)+
        " pad_coverage="+pad_text(grasp_pose.pad_contact_coverage[0])+"/"+pad_text(grasp_pose.pad_contact_coverage[1])+
        " expected_contact_span_m="+std::to_string(grasp_pose.opening_width_m)+
        "; approach remains fully open; contact span is diagnostic, not an opening command",
        task.context.key.track_id);
      for(const auto & feasibility:result->results)if(feasibility.candidate_id==grasp_candidate_index+1){
        grasp_entry_minimum_sec=std::max(0.,feasibility.pregrasp_time_lower_bound_sec);break;}
      approach_grasp_candidate(from_ros(goal.key));
    });
  }

  void start_relative_grasp_reference()
  {
    if(grasp_motion_reference || task.phase!=TaskPhase::kPreparingPick || !grasp_pose.valid)return;
    const auto time=now();const auto moving=selected();const auto object=predict_object_pose(moving,*settings.snapshot,time,0.);
    if(!object){skip_observation_target("relative grasp object prediction unavailable");return;}
    if(!follow_active){
      const auto measured=tcp_world();const auto inverse=inverse_pose(*object);
      grasp_follow_hold_in_object=inverse?compose_pose(*inverse,measured.pose):std::nullopt;
      start_following_target(task.context.key,measured.pose,{},Follow::Goal::GRASP);
      return; // Build only when this existing owner is actually Following.
    }
    if(!arm_snapshot_ready(arm,*settings.snapshot,time) || arm.state.motion_state!=ArmMotionState::kFollowing || arm_command_id!=follow_command)return;
    if(grasp_command_state && std::chrono::duration<double>(time.source_now-grasp_command_state->timestamp).count()>config().motion.target_update_timeout_sec){
      skip_observation_target("relative grasp command boundary expired beyond existing target update timeout");return;}
    const auto entry=grasp_command_state?extrapolate_grasp_entry(*grasp_command_state,time.source_now):std::nullopt;
    if(!entry){skip_observation_target("relative grasp missing same-owner command boundary");return;}
    const double cadence=std::isfinite(control_period_source)?control_period_source:config().motion.observation_update_period_sec;
    const Vector3d velocity{moving.velocity_mps.x_m,moving.velocity_mps.y_m,0.};
    const double descent_speed=config().grasp.descent_speed_mps;
    std::string failure;
    grasp_motion_reference=make_relative_grasp_motion_reference(*entry,*object,velocity,
      grasp_pose.pregrasp_in_object,grasp_pose.tcp_in_object,config().grasp.verification_probe_m,
      config().motion.max_follow_linear_speed_mps,config().motion.max_follow_angular_speed_radps,
      std::max(config().motion.observation_preview_horizon_sec,2.*cadence),.5,
      std::min(config().motion.max_follow_linear_speed_mps,vector_norm(velocity)+descent_speed),
      std::min(config().motion.max_follow_linear_speed_mps,vector_norm(velocity)+config().grasp.probe_lift_speed_mps),
      grasp_entry_minimum_sec,config().grasp.intercept_lead_sec,&failure);
    if(!grasp_motion_reference){skip_observation_target("relative grasp recipe unavailable: "+failure);return;}
    grasp_motion_elapsed=0.;grasp_motion_rate=1.;grasp_motion_offset={};grasp_motion_tick=time.source_now;
    grasp_motion_visual_stamp=moving.observation.timestamp;
    const auto & ref=*grasp_motion_reference;const auto & bound=*ref.relative_approach;
    record_observation_step("plan_grasp_motion_reference","object-relative Hermite recipe: approach="+std::to_string(ref.approach_sec)+
      " follow="+std::to_string(ref.follow_sec)+" descent="+std::to_string(ref.descent_sec)+" lift="+std::to_string(ref.lift_sec)+
      " world_speed_bound="+std::to_string(bound.linear_bound_mps)+" angular_bound="+std::to_string(bound.angular_bound_radps)+
      " local_position_box=["+std::to_string(bound.position_bounds_object[0].x_m)+","+std::to_string(bound.position_bounds_object[0].y_m)+","+std::to_string(bound.position_bounds_object[0].z_m)+
      ";"+std::to_string(bound.position_bounds_object[1].x_m)+","+std::to_string(bound.position_bounds_object[1].y_m)+","+std::to_string(bound.position_bounds_object[1].z_m)+"]"+
      "; owner_command="+std::to_string(follow_command)+" acknowledged="+std::to_string(arm_command_id)+
      "; same-owner command pose/twist; object-only lead",task.context.key.track_id);
  }
  void start_timed_grasp_reference(const TrackedObjectState &,const std::optional<Pose3d> &)
  {
    start_relative_grasp_reference();
  }

  void approach_grasp_candidate(const EventKey & command_key)
  {
    if(task.phase!=TaskPhase::kPreparingPick||grasp_candidate_index>=grasp_candidates.size()){return;}
    const auto moving=selected();const auto object=predict_object_pose(moving,*settings.snapshot,now(),0);
    if(grasp_motion_reference)return;
    if(!follow_active){
      const auto overhead=object?compose_pose(*object,grasp_pose.pregrasp_in_object):std::nullopt;
      if(!overhead){skip_observation_target("moving pregrasp unavailable");return;}
      Command entry;entry.key=command_key;entry.reference_frame=config().frames.world;entry.target=*overhead;
      record_observation_step("move_above_grasp_target","fast joint route to moving overhead point; follow interval starts after measured entry",task.context.key.track_id);
      move_to_pose(entry,[this,moving](bool arrived){
        if(task.phase!=TaskPhase::kPreparingPick)return;
        if(!arrived){skip_observation_target("moving overhead entry failed; return Home and next target");return;}
        // Extrapolate the original visual observation to actual arrival.
        // Keep its measurement timestamp intact; this is a reference, not
        // fresh vision or evidence of object possession.
        auto predicted=moving.observation.pose_world;
        const double age=std::chrono::duration<double>(now().source_now-moving.observation.timestamp).count();
        predicted.position.x_m+=moving.velocity_mps.x_m*age;
        predicted.position.y_m+=moving.velocity_mps.y_m*age;
        predicted.position.z_m+=moving.velocity_mps.z_m*age;
        start_timed_grasp_reference(moving,predicted);
      },&moving);
    }else{start_timed_grasp_reference(moving,object);}
  }

  void add_grasp_neighborhood(Evaluate::Goal & goal,const Vector3d & target,const TimeContext & time)
  {
    const auto fresh=[&](Nanoseconds stamp){
      return source_is_fresh(stamp,time,config().vision.frame_max_age_sec,config().time.sensor_future_tolerance_sec);
    };
    const auto & capture=demo_grasp_surface.neighbor_offsets_world;
    const bool capture_fresh=fresh(demo_grasp_surface.neighbor_stamp);
    const bool rank=config().grasp.neighbor_clearance_mode=="rank";
    const auto late=latest_neighborhood;
    const bool use_late=rank && late && same_cycle(from_ros(late->key),task.context.key) &&
      from_ros(late->stamp)>demo_grasp_surface.neighbor_stamp && fresh(from_ros(late->stamp)) &&
      !late->offsets_world.empty();
    if(rank && !use_late){return;}
    if(!capture_fresh && !use_late){return;}
    goal.neighbor_capture_ms=(capture_fresh?demo_grasp_surface.neighbor_capture_ms:0.)+
      (use_late?late->extraction_ms:0.);
    // The two samples share world axes and a target-relative voxel grid.
    // Interleave them so one frame cannot consume the entire message budget.
    std::set<std::array<int,3>> visited;
    const auto append=[&](const Vector3d & offset){
      const double max_offset=config().grasp.open_width_m+config().grasp.pregrasp_clearance_m+kGraspNeighborVoxelM;
      if(!(std::max({std::abs(offset.x_m),std::abs(offset.y_m),std::abs(offset.z_m)})<=max_offset) ||
        !std::isfinite(offset.x_m+offset.y_m+offset.z_m)){return;}
      const auto cell=[](double p){return static_cast<int>(std::floor(p/kGraspNeighborVoxelM));};
      if(!visited.insert({cell(offset.x_m),cell(offset.y_m),cell(offset.z_m)}).second){return;}
      geometry_msgs::msg::Point32 point;
      point.x=static_cast<float>(target.x_m+offset.x_m);
      point.y=static_cast<float>(target.y_m+offset.y_m);
      point.z=static_cast<float>(target.z_m+offset.z_m);
      goal.neighbor_points_world.push_back(point);
    };
    const auto n=capture_fresh?capture.size():0;
    const auto m=use_late?late->offsets_world.size():0;
    for(std::size_t i=0;i<std::max(n,m) && goal.neighbor_points_world.size()<kGraspNeighborLimit;++i){
      if(i<m){const auto & p=late->offsets_world[i];append({p.x,p.y,p.z});}
      if(i<n){append(capture[i]);}
    }
    if(use_late){
      record_observation_step("grasp_neighborhood","capture="+std::to_string(n)+" fresh="+std::to_string(m)+
        " merged="+std::to_string(goal.neighbor_points_world.size())+" age_sec="+
        std::to_string(std::chrono::duration<double>(time.source_now-from_ros(late->stamp)).count()),
        task.context.key.track_id);
    }
  }
  std::optional<GraspMotionSample> sample_current_grasp_motion(Nanoseconds source)const
  {
    if(!grasp_motion_reference)return std::nullopt;
    const double absolute=std::chrono::duration<double>(source-grasp_motion_reference->source_start).count();
    return sample_grasp_motion_reference(*grasp_motion_reference,source,grasp_motion_elapsed,grasp_motion_rate,
      grasp_motion_offset.position(absolute),grasp_motion_offset.velocity(absolute));
  }
  std::optional<GraspMotionPrediction> current_grasp_motion_prediction()
  {
    if(!grasp_motion_reference)return std::nullopt;
    const auto time=now();const auto & ref=*grasp_motion_reference;
    const double elapsed=std::chrono::duration<double>(time.source_now-ref.source_start).count();
    auto object=ref.object_world;const auto offset=grasp_motion_offset.position(elapsed);
    object.position.x_m+=ref.object_velocity.x_m*elapsed+offset.x_m;
    object.position.y_m+=ref.object_velocity.y_m*elapsed+offset.y_m;
    object.position.z_m+=ref.object_velocity.z_m*elapsed+offset.z_m;
    StampedPose3d predicted;predicted.pose=object;predicted.timestamp=time.source_now;predicted.reference_frame=config().frames.world;
    const double raised=(task.grasp.phase==GraspPhase::kVerifying || task.grasp.phase==GraspPhase::kHolding) && probe_valid?
      std::max(0.,tcp_world().pose.position.z_m-probe_tcp.position.z_m):0.;
    return GraspMotionPrediction{task.context.key,predicted,ref.source_start,raised};
  }
  void update_planned_grasp_preview(Status & status, const TimeContext & time)
  {
    const bool closing = task.phase == TaskPhase::kPicking &&
      (task.grasp.phase == GraspPhase::kApproaching || task.grasp.phase == GraspPhase::kClosing);
    if (time.paused || !task.context.key.track_id || !grasp_pose.valid ||
      (task.phase != TaskPhase::kPreparingPick && !closing)) {return;}
    const auto prediction = current_grasp_motion_prediction();
    const auto object = prediction ? std::optional<Pose3d>(prediction->object_world.pose) :
      predict_object_pose(selected(), *settings.snapshot, time, 0.);
    const auto contact = object ? compose_pose(*object, grasp_pose.tcp_in_object) : std::nullopt;
    if (!contact) {return;}
    const auto to_point = [](const Vector3d & point) {
      geometry_msgs::msg::Point message;
      message.x = point.x_m; message.y = point.y_m; message.z = point.z_m;
      return message;
    };
    const auto & center = config().grasp.finger_center_tcp;
    const auto & axis = config().grasp.closing_axis_tcp;
    status.planned_grasp_center.header =
      ptc_app::stamped_pose(*contact, config().frames.world, time.source_now).header;
    status.planned_grasp_center.point = to_point(transform_point(*contact, center));
    for (std::size_t side = 0; side < status.planned_grasp_jaws.size(); ++side) {
      const auto offset = (side == 0 ? -.5 : .5) * grasp_pose.opening_width_m;
      status.planned_grasp_jaws[side] = to_point(transform_point(*contact,
        {center.x_m + offset * axis.x_m, center.y_m + offset * axis.y_m,
          center.z_m + offset * axis.z_m}));
    }
  }

  void advance_grasp_motion_reference(const TimeContext & time)
  {
    if(!grasp_motion_reference || (task.phase!=TaskPhase::kPreparingPick && task.phase!=TaskPhase::kPicking))return;
    if(task.phase==TaskPhase::kPicking && task.grasp.phase!=GraspPhase::kApproaching && task.grasp.phase!=GraspPhase::kClosing && task.grasp.phase!=GraspPhase::kVerifying)return;
    auto & ref=*grasp_motion_reference;const auto measured=tcp_world();const auto track=selected();
    if(last_tcp_stamp.count()>0 && measured.timestamp>last_tcp_stamp){
      const double dt=std::chrono::duration<double>(measured.timestamp-last_tcp_stamp).count();
      const auto p=transform_point(measured.pose,config().grasp.finger_center_tcp),q=transform_point(previous_tcp_world,config().grasp.finger_center_tcp);
      relative_speed=vector_norm({(p.x_m-q.x_m)/dt-ref.object_velocity.x_m,(p.y_m-q.y_m)/dt-ref.object_velocity.y_m,(p.z_m-q.z_m)/dt-ref.object_velocity.z_m});
    }
    previous_tcp_world=measured.pose;last_tcp_stamp=measured.timestamp;
    const double absolute=std::chrono::duration<double>(time.source_now-ref.source_start).count();
    const bool semantic_pick=config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model";
    const bool approaching=ref.relative_approach && task.phase==TaskPhase::kPicking && task.grasp.phase==GraspPhase::kApproaching;
    const bool late_alignment=semantic_pick && approaching;
    if(!empty_descent_clearance_z && (task.phase==TaskPhase::kPreparingPick || approaching) &&
      track.observation.timestamp>grasp_motion_visual_stamp &&
      same_track_identity(track.context.key,task.context.key) &&
      track_is_usable(track,*settings.snapshot,time,!late_alignment)){
      const double horizon=config().motion.observation_preview_horizon_sec;
      const double observed=std::chrono::duration<double>(track.observation.timestamp-ref.source_start).count();
      const auto & p=ref.object_world.position;const auto & seen=track.observation.pose_world.position;
      // Compare image and nominal reference at the same source time.
      Vector3d offset{seen.x_m-p.x_m-ref.object_velocity.x_m*observed,
        seen.y_m-p.y_m-ref.object_velocity.y_m*observed,0.};
      if(late_alignment){
        const auto started=std::chrono::steady_clock::now();
        const double image_age=std::max(0.,absolute-observed);
        const double lead=image_age+horizon;
        // Use the existing velocity estimate only while this sample is
        // fresh. The nominal conveyor reference continues through gaps.
        offset.x_m+=(track.velocity_mps.x_m-ref.object_velocity.x_m)*lead;
        offset.y_m+=(track.velocity_mps.y_m-ref.object_velocity.y_m)*lead;
        const auto current=grasp_motion_offset.position(absolute);
        const double dx=offset.x_m-current.x_m,dy=offset.y_m-current.y_m;
        const double innovation=std::hypot(dx,dy);
        const double headroom=std::max(0.,config().motion.max_follow_linear_speed_mps-vector_norm(ref.object_velocity));
        const double correction_limit=std::min(config().grasp.open_width_m*.5,headroom*horizon/1.5);
        if(innovation>correction_limit){
          offset.x_m=current.x_m+dx*correction_limit/innovation;
          offset.y_m=current.y_m+dy*correction_limit/innovation;
        }
        grasp_motion_offset.rebase(absolute,offset,{},horizon);
        const auto compute_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-started).count();
        record_observation_step("preclose_alignment","image_age_sec="+std::to_string(image_age)+
          " innovation_m="+std::to_string(innovation)+" applied_m="+
          std::to_string(std::hypot(offset.x_m-current.x_m,offset.y_m-current.y_m))+
          " compute_us="+std::to_string(compute_us),task.context.key.track_id);
      }else{
        grasp_motion_offset.rebase(absolute,offset,{},horizon);
      }
      grasp_motion_visual_stamp=track.observation.timestamp;
    }
    const auto before=sample_current_grasp_motion(time.source_now);if(!before)return;
    const auto actual=transform_point(measured.pose,config().grasp.finger_center_tcp),desired=transform_point(before->tcp_world,config().grasp.finger_center_tcp);
    const double error=vector_norm({actual.x_m-desired.x_m,actual.y_m-desired.y_m,actual.z_m-desired.z_m});
    // Use the existing same-owner motion clock for approach, descent and probe.
    grasp_motion_rate=1.;
    const bool tracking=follow_active && arm_snapshot_ready(arm,*settings.snapshot,time) &&
      arm.state.motion_state==ArmMotionState::kFollowing;
    // The simulated recipe is timed once the Follow owner starts. Brief
    // motion-state feedback differences and Cartesian errors are diagnostics;
    // fresh physical feedback and unique ownership remain necessary.
    const bool controlled=follow_active && arm_snapshot_ready(arm,*settings.snapshot,time) &&
      (tracking || grasp_motion_elapsed>0.);
    const double position_tolerance=config().motion.position_tolerance_m;
    const double contact_planar_tolerance=2.*position_tolerance;
    const double planar_error=std::hypot(actual.x_m-desired.x_m,actual.y_m-desired.y_m);
    const bool semantic_descent=semantic_pick && task.phase==TaskPhase::kPicking &&
      task.grasp.phase==GraspPhase::kApproaching && task.grasp.possession==Possession::kEmpty;
    if(!empty_descent_clearance_z && semantic_descent && controlled &&
      grasp_motion_elapsed>=ref.contact_start()-1e-8 && planar_error>2.*contact_planar_tolerance){
      // Keep the Follow owner and clear the belt before asking it to stop.
      const double clearance=config().grasp.pregrasp_clearance_m;
      empty_descent_clearance_z=std::max(actual.z_m,desired.z_m)+clearance;
      const double rise=*empty_descent_clearance_z-desired.z_m;
      auto offset=grasp_motion_offset.to;
      offset.z_m=grasp_motion_offset.position(absolute).z_m+rise;
      const double horizon=std::max(config().motion.observation_preview_horizon_sec,
        1.5*rise/config().grasp.probe_lift_speed_mps);
      grasp_motion_offset.rebase(absolute,offset,{},horizon);
      record_observation_step("retreat_empty_grasp","moving descent lost alignment; lift before stopping; planar_error_m="+
        std::to_string(planar_error),task.context.key.track_id);
    }
    if(empty_descent_clearance_z){
      grasp_motion_rate=0.;
      grasp_motion_tick=time.source_now;
      if(controlled && actual.z_m>=*empty_descent_clearance_z-position_tolerance){
        skip_observation_target("moving descent lost alignment; empty gripper cleared belt");
      }
      return;
    }
    if(!controlled || task.grasp.phase==GraspPhase::kClosing){
      grasp_motion_rate=0.;
    }else if(semantic_descent && grasp_motion_elapsed<ref.contact_start()-1e-8){
      // Let the planar controller catch up before descending farther. The
      // object reference keeps advancing on source time, including at rate 0.
      grasp_motion_rate=std::clamp(2.-planar_error/contact_planar_tolerance,0.,1.);
    }
    const double dt=grasp_motion_tick.count()>0?std::max(0.,std::chrono::duration<double>(time.source_now-grasp_motion_tick).count()):0.;
    grasp_motion_tick=time.source_now;
    const double end=task.phase==TaskPhase::kPreparingPick?ref.descent_start():
      task.grasp.phase==GraspPhase::kVerifying?ref.lift_end():ref.contact_start();
    if(controlled && task.grasp.phase!=GraspPhase::kClosing)grasp_motion_elapsed=std::min(end,grasp_motion_elapsed+dt*grasp_motion_rate);
    // After measured belt clearance, blend the existing offset velocity to
    // cancel conveyor feedforward. Its position integral stays continuous.
    const Vector3d braking_velocity{-ref.object_velocity.x_m,-ref.object_velocity.y_m,-ref.object_velocity.z_m};
    if(task.grasp.phase==GraspPhase::kVerifying && probe_valid &&
      measured.pose.position.z_m-probe_tcp.position.z_m>=
        std::max(0.,std::min(config().grasp.pregrasp_clearance_m,config().grasp.verification_probe_m)-config().motion.position_tolerance_m) &&
      vector_norm({grasp_motion_offset.to_velocity.x_m-braking_velocity.x_m,
        grasp_motion_offset.to_velocity.y_m-braking_velocity.y_m,grasp_motion_offset.to_velocity.z_m-braking_velocity.z_m})>1e-9){
      const double horizon=std::max(config().motion.observation_update_period_sec,ref.lift_end()-grasp_motion_elapsed);
      const auto p=grasp_motion_offset.position(absolute),v=grasp_motion_offset.velocity(absolute);
      grasp_motion_offset.rebase(absolute,{p.x_m+.5*horizon*(v.x_m+braking_velocity.x_m),
        p.y_m+.5*horizon*(v.y_m+braking_velocity.y_m),p.z_m+.5*horizon*(v.z_m+braking_velocity.z_m)},braking_velocity,horizon);
      record_observation_step("exit_conveyor_follow","measured clearance reached; smooth feedforward exit during lift",task.context.key.track_id);
    }
    const auto sample=sample_current_grasp_motion(time.source_now);if(!sample)return;
    if(task.phase==TaskPhase::kPreparingPick && grasp_motion_elapsed>=ref.descent_start()-1e-8 && controlled){
      record_observation_step("grasp_reference_pregrasp","timed approach and 500 ms follow completed; alignment recorded, same owner descends; error_m="+std::to_string(error),task.context.key.track_id);
      auto e=event(TaskEventKind::kGraspPrepared);e.grasp_event.grasp_window_valid=true;e.grasp_event.grasp_tcp_in_object=grasp_pose.tcp_in_object;apply(e);
    }else if(task.phase==TaskPhase::kPicking && task.grasp.phase==GraspPhase::kApproaching && grasp_motion_elapsed>=ref.contact_start()-1e-8 && controlled &&
      grasp_descent_height_reached(measured.pose,sample->tcp_world,config().grasp.finger_center_tcp,config().motion.position_tolerance_m)){
      const auto contact=transform_point(sample->tcp_world,config().grasp.finger_center_tcp);
      const double contact_planar_error=std::hypot(actual.x_m-contact.x_m,actual.y_m-contact.y_m);
      // Hold the contact segment on the same moving reference while the
      // Cartesian controller catches up. The existing approach deadline applies.
      const double height_error=actual.z_m-contact.z_m;
      // The committed continuous recipe already owns motion and the core
      // distinguishes its contact event from an ordinary velocity-gated grasp.
      // Keep measured position alignment here; raw finite-difference speed
      // remains diagnostic instead of imposing the same gate a second time.
      const bool contact_aligned=contact_planar_error<=contact_planar_tolerance &&
        std::abs(height_error)<=config().motion.position_tolerance_m;
      if(!semantic_pick || contact_aligned){
        if(semantic_pick && !gripper.empty_confirmed && gripper.stopped && !gripper.contact_confirmed &&
          gripper.width_m>=config().grasp.open_width_m-position_tolerance && gripper_snapshot_ready(gripper,*settings.snapshot,time)){
          RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),1000,
            "moving contact reached: waiting for measured Empty within the current approach");
          return;
        }
        if(semantic_pick && same_track_identity(track.context.key,task.context.key) &&
          track_is_usable(track,*settings.snapshot,time)){
          const auto observed_object=predict_object_pose(track,*settings.snapshot,time,0.);
          if(observed_object){
            auto object=ref.object_world;
            object.position.x_m=observed_object->position.x_m;
            object.position.y_m=observed_object->position.y_m;
            const auto vision_contact=compose_pose(object,ref.contact_in_object);
            if(vision_contact){
              const auto center=transform_point(*vision_contact,config().grasp.finger_center_tcp);
              record_observation_step("preclose_target_error","xy_error_m="+
                std::to_string(std::hypot(center.x_m-actual.x_m,center.y_m-actual.y_m))+
                " image_age_sec="+std::to_string(std::chrono::duration<double>(time.source_now-track.observation.timestamp).count()),
                task.context.key.track_id);
            }
          }
        }
        const auto & measured_orientation=measured.pose.orientation;
        const auto & reference_orientation=sample->tcp_world.orientation;
        const auto orientation_dot=measured_orientation.x*reference_orientation.x+measured_orientation.y*reference_orientation.y+
          measured_orientation.z*reference_orientation.z+measured_orientation.w*reference_orientation.w;
        const auto angle_error=2.*std::acos(std::clamp(std::abs(orientation_dot),0.,1.));
        record_observation_step("attempt_moving_grasp","moving contact reached; evaluating close while following; planar_error_m="+
          std::to_string(contact_planar_error)+" height_error_m="+std::to_string(height_error)+" angle_error_rad="+
          std::to_string(angle_error)+" relative_speed_mps="+std::to_string(relative_speed),task.context.key.track_id);
        grasp(grasp_event(GraspEventKind::kApproachCompleted));
      }else{
        RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),500,
          "waiting for moving grasp contact: planar_error_m=%.6f height_error_m=%.6f relative_speed_mps=%.6f",
          contact_planar_error,height_error,relative_speed);
      }
    }
    // Bounded-rate diagnostics only: distinguish changes in the reference
    // from measured tracking error without changing any motion decision.
    if(ref.relative_approach && task.phase==TaskPhase::kPicking &&
      (task.grasp.phase==GraspPhase::kApproaching || task.grasp.phase==GraspPhase::kClosing)){
      const auto observed_object=track_pose_at_observation(track,*settings.snapshot,time,measured.timestamp);
      const auto contact=observed_object?compose_pose(*observed_object,grasp_pose.tcp_in_object):std::nullopt;
      if(contact){
        const auto center=transform_point(*contact,config().grasp.finger_center_tcp);
        RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),500,
          "relative grasp alignment: feedback=%.9f visual=%.9f delta_xyz=(%.6f,%.6f,%.6f) relative_speed=%.6f phase=%d; feedback-time object contact, no command lead",
          std::chrono::duration<double>(measured.timestamp).count(),std::chrono::duration<double>(track.observation.timestamp).count(),
          actual.x_m-center.x_m,actual.y_m-center.y_m,actual.z_m-center.z_m,relative_speed,static_cast<int>(task.grasp.phase));
      }
    }
    const auto trace_center=transform_point(sample->tcp_world,config().grasp.finger_center_tcp);
    const auto trace_offset=grasp_motion_offset.position(absolute),trace_offset_velocity=grasp_motion_offset.velocity(absolute);
    RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),100,
      "continuous grasp: segment=%d path_sec=%.6f rate=%.3f source_sec=%.9f error_m=%.6f relative_speed=%.6f tracking=%d "
      "cycle=%llu phase=%d grasp=%d measured_sec=%.9f visual_sec=%.9f actual=(%.6f,%.6f,%.6f) desired=(%.6f,%.6f,%.6f) "
      "offset=(%.6f,%.6f,%.6f) offset_v=(%.6f,%.6f,%.6f) nominal_v=(%.6f,%.6f,%.6f)",
      static_cast<int>(sample->segment),grasp_motion_elapsed,grasp_motion_rate,std::chrono::duration<double>(time.source_now).count(),error,relative_speed,tracking,
      static_cast<unsigned long long>(task.context.key.cycle_id),static_cast<int>(task.phase),static_cast<int>(task.grasp.phase),
      std::chrono::duration<double>(measured.timestamp).count(),std::chrono::duration<double>(grasp_motion_visual_stamp).count(),
      actual.x_m,actual.y_m,actual.z_m,trace_center.x_m,trace_center.y_m,trace_center.z_m,
      trace_offset.x_m,trace_offset.y_m,trace_offset.z_m,trace_offset_velocity.x_m,trace_offset_velocity.y_m,trace_offset_velocity.z_m,
      ref.object_velocity.x_m,ref.object_velocity.y_m,ref.object_velocity.z_m);
  }
  void publish_follow_target();
  std::string record_grasp_proposal_timing(double wall_ms)
  {
    if(!std::isfinite(wall_ms)||wall_ms<0){return "proposal_roundtrip_ms=invalid";}
    ++grasp_proposal_sample_count;if(wall_ms>100.){++grasp_proposal_over_100ms_count;}
    grasp_proposal_wall_ms_samples.push_back(wall_ms);
    if(grasp_proposal_wall_ms_samples.size()>100){grasp_proposal_wall_ms_samples.erase(grasp_proposal_wall_ms_samples.begin());}
    auto ordered=grasp_proposal_wall_ms_samples;std::sort(ordered.begin(),ordered.end());
    const auto quantile=[&](double q){const auto index=static_cast<std::size_t>(std::ceil(q*ordered.size())-1.);return ordered[std::min(index,ordered.size()-1)];};
    return "proposal_roundtrip_samples="+std::to_string(grasp_proposal_sample_count)+
      " recent_n="+std::to_string(ordered.size())+" p50_ms="+std::to_string(quantile(.50))+
      " p95_ms="+std::to_string(quantile(.95))+" p99_ms="+std::to_string(quantile(.99))+
      " recent_max_ms="+std::to_string(ordered.back())+
      " over_100ms_total="+std::to_string(grasp_proposal_over_100ms_count);
  }
  void record_observation_step(const std::string & method,const std::string & detail,
    std::uint64_t track_id=0,const std::string & view_id="")
  {
    const auto text="["+method+"] track="+std::to_string(track_id)+" view="+view_id+" "+detail;
    if(text!=observation_diagnostic){
      RCLCPP_INFO(node.get_logger(),"observation checkpoint cycle=%llu source_sec=%.9f %s",
        static_cast<unsigned long long>(task.context.key.cycle_id),std::chrono::duration<double>(now().source_now).count(),text.c_str());
      observation_diagnostic=text;
    }
  }
  void record_close_timing(const char * step,std::uint64_t command_id)
  {
    if(!close_timing || close_timing->command_id!=command_id){return;}
    const auto & timing=*close_timing;
    const auto time=now();
    RCLCPP_INFO(node.get_logger(),"close timing step=%s epoch=%llu cycle=%llu track=%llu operation=%llu command=%llu sim_elapsed_ms=%.3f wall_elapsed_ms=%.3f width_m=%.6f",
      step,static_cast<unsigned long long>(timing.key.epoch),static_cast<unsigned long long>(timing.key.cycle_id),
      static_cast<unsigned long long>(timing.key.track_id),static_cast<unsigned long long>(timing.key.operation_id),
      static_cast<unsigned long long>(command_id),std::chrono::duration<double,std::milli>(time.source_now-timing.source_started).count(),
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-timing.wall_started).count(),gripper.width_m);
  }
  void observe_close_movement()
  {
    // Ignore measurement noise. This threshold is diagnostic only.
    constexpr double minimum_width_change_m=0.0003;
    if(!close_timing || close_timing->movement_recorded || gripper.epoch!=close_timing->key.epoch ||
      gripper.timestamp<=close_timing->source_started || close_timing->initial_width_m-gripper.width_m<minimum_width_change_m){return;}
    if(!gripper_snapshot_ready(gripper,*settings.snapshot,now())){return;}
    close_timing->movement_recorded=true;
    record_close_timing("first_measured_movement",close_timing->command_id);
  }
  std::string last_failed_grasp;
  std::string workflow_reason() const
  {
    return ((last_failed_grasp.empty()?std::string{}:"Previous failure: "+last_failed_grasp+"; ")+((task.phase==TaskPhase::kObserving) &&
      !observation_diagnostic.empty()?observation_diagnostic:
      reason+(observation_diagnostic.empty()?"":"; last_step="+observation_diagnostic))).substr(0,512);
  }
  void try_select_observation_target(const TrackedObjectState &);
  void tick();
};
void TaskCoordinatorNode::Runtime::save_outcome(const Command & command)
{
  // TaskStatus carries the outcome; completion never waits for a diagnostic file.
  apply(event(TaskEventKind::kReportAcknowledged,command.key.operation_id));
}
void TaskCoordinatorNode::Runtime::grip(const Command & command)
{
  Grip::Goal goal;goal.key=to_ros(command.key);goal.command_id=command.command_id;goal.open=command.kind==CommandKind::kOpenGripper;goal.width_m=command.width_m;goal.max_effort_n=command.effort_n;
  goal.timeout=ptc_app::to_ros_duration(goal.open?config().grasp.release_timeout_sec:close_wall_timeout_sec(config()));
  if(goal.open && command.key.track_id && task.grasp.phase==GraspPhase::kReleasing &&
    task.phase!=TaskPhase::kRecovering && task.phase!=TaskPhase::kFaulted){
    bin_release_pub->publish(goal.key);
  }
  const auto issued_at=std::chrono::steady_clock::now();
  if(command.kind==CommandKind::kCloseGripper){
    recover_pregrasp_open=false;probe_baseline=selected();
    close_timing=CloseTiming{command.key,command.command_id,now().source_now,issued_at,gripper.width_m};
    record_close_timing("requested",command.command_id);
  }
  record_observation_step(command.kind==CommandKind::kOpenGripper?"open_gripper":"close_gripper","requested; since_top_source_sec="+std::to_string(std::chrono::duration<double>(now().source_now-entry_follow_started_source).count()),task.context.key.track_id);
  send<Grip>(grip_client,goal,[this,command,issued_at](auto result){const bool success=result&&result->success&&result->command_id==command.command_id&&same_operation(from_ros(result->key),command.key);
    if(command.kind==CommandKind::kCloseGripper){record_close_timing(success?"action_succeeded":"action_failed",command.command_id);}
    record_observation_step(command.kind==CommandKind::kOpenGripper?"open_gripper":"close_gripper","success="+std::to_string(success)+" action_wall_ms="+std::to_string(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-issued_at).count())+"; action ACK alone is not Holding",task.context.key.track_id);
    if(command.kind==CommandKind::kOpenGripper){if(success){
      if(observation_open_command==command.command_id)observation_open_command.reset();
      pending_release=true;
    }else{
      if((task.phase==TaskPhase::kFaulted || task.phase==TaskPhase::kRecovering) && observation_open_command==command.command_id){
        observation_open_command.reset();recovery_open_failed=true;
        if(recover_pregrasp_open && !gripper.contact_confirmed){
          reason="pre-grasp reopening failed; waiting for measured Empty before Home";
          record_observation_step("recover_failed_grasp",reason,task.context.key.track_id);
          return;
        }
        reason="recovery opening failed; recover at bin after measured stop";
        record_observation_step("recover_failed_grasp",reason,task.context.key.track_id);
        if(arm_snapshot_ready(arm,*settings.snapshot,now()) && arm.stopped &&
          gripper_snapshot_ready(gripper,*settings.snapshot,now()) && gripper.stopped){
          retreat_failed_demo_grasp();
        }else{stop();} // Moving/stale feedback still requires the existing stop path.
      }else{grasp(grasp_event(GraspEventKind::kReleaseFailed));}
    }}
    else{
      if(success){try{probe_baseline=selected();probe_tcp=tcp_world().pose;const auto relative=compose_pose(*inverse_pose(probe_baseline.observation.pose_world),probe_tcp);probe_valid=relative.has_value();if(relative){probe_relative=*relative;}visual_evidence_stamp=probe_baseline.observation.timestamp;}catch(...){probe_valid=false;}}
      grasp(grasp_event(success?GraspEventKind::kCloseAccepted:GraspEventKind::kCloseFailed));
      if(success && grasp_motion_reference && task.grasp.phase==GraspPhase::kVerifying){
        grasp_motion_elapsed=grasp_motion_reference->contact_start();grasp_motion_tick=now().source_now;
      }}
  },true,{},[this,command]{
    if(command.kind==CommandKind::kCloseGripper){record_close_timing("goal_accepted",command.command_id);}
  });
}
bool TaskCoordinatorNode::Runtime::scene_change(SceneEventKind kind)
{
  if(sync_busy){return false;}SceneEvent e;e.kind=kind;e.key=event_key();e.arm=arm;e.gripper=gripper;
  if(kind==SceneEventKind::kStaticLoaded){e.static_geometry=ptc_app::load_static_geometry(config());}
  if(kind==SceneEventKind::kAttachRequested && (task.phase==TaskPhase::kRecovering || task.phase==TaskPhase::kFaulted) && !gripper.empty_confirmed){
    const auto relative=grasp_pose.valid?inverse_pose(grasp_pose.tcp_in_object):std::nullopt;
    if(!relative || !compose_pose(tcp_world().pose,*relative)){
      reason="bin recovery: measured grasp relation unavailable; keep jaws closed";return false;
    }
    e.object={task.selected_track.context.key,task.selected_track.observation,{}};
  }
  // A failed stopped demo no longer owns a moving target. Let ordinary
  // expiry remove its stale free obstacle before retreat or the next cycle.
  // WaitingReady still retains the last track for reporting, not ownership.
  // Attached/unknown attachments retain the core's existing protection.
  if(kind==SceneEventKind::kExpire &&
    (task.phase==TaskPhase::kWaitingReady ||
      ((task.phase==TaskPhase::kRecovering || task.phase==TaskPhase::kFaulted) &&
       (task.outcome==CycleOutcome::kFailed || task.outcome==CycleOutcome::kInconclusive))) &&
    config().runtime.operation_mode=="assumed_ng_demo" &&
    arm_snapshot_ready(arm,*settings.snapshot,now()) && arm.stopped &&
    gripper_snapshot_ready(gripper,*settings.snapshot,now()) && gripper.stopped && gripper.empty_confirmed){
    e.key.track_id=0;
  }
  const auto d=advance_scene(scene,e,*settings.snapshot,now());scene=d.state;
  for(const auto & command:d.commands){sync_scene(command);}
  if(d.status.disposition==Disposition::kRejected){reason=d.status.reason;}
  return d.status.disposition==Disposition::kApplied;
}
void TaskCoordinatorNode::Runtime::sync_scene(const Command & command)
{
  Sync::Goal goal;goal.key=to_ros(command.key);goal.command_id=command.command_id;goal.scene_revision=scene.revision;goal.attached_track_id=scene.attached_track_id;
  goal.operation=command.kind==CommandKind::kAttachObject?goal.ATTACH:command.kind==CommandKind::kDetachObject?goal.DETACH:goal.UPDATE;
  for(const auto & object:scene.static_geometry){ptc_interfaces::msg::CollisionGeometry item;item.id=object.id;item.reference_frame=object.reference_frame;item.kind=static_cast<std::uint8_t>(object.kind);item.pose=to_ros(object.pose);item.dimensions_m.x=object.dimensions_m.x_m;item.dimensions_m.y=object.dimensions_m.y_m;item.dimensions_m.z=object.dimensions_m.z_m;item.mesh_path=object.mesh_path;item.asset_digest=object.asset_digest;goal.static_geometry.push_back(item);}
  for(const auto & object:scene.dynamic_objects){TrackedObjectState track;track.context.key=object.key;track.observation=object.observation;track.phase=TrackPhase::kTracking;
    if(command.kind==CommandKind::kAttachObject && (task.phase==TaskPhase::kRecovering || task.phase==TaskPhase::kFaulted) &&
      !gripper.empty_confirmed && grasp_pose.valid && object.key.track_id==scene.attached_track_id){
      // Conservative recovery collision pose follows the measured TCP and the
      // original grasp relation. Preserve the original visual timestamp: this
      // is a planning estimate, not a new detection or proof of possession.
      const auto inverse=inverse_pose(grasp_pose.tcp_in_object);const auto estimate=inverse?compose_pose(tcp_world().pose,*inverse):std::nullopt;
      if(estimate)track.observation.pose_world=*estimate;
    }
    goal.objects.push_back(to_ros(track));}
  EventKey surface_key;std::uint64_t surface_revision=0;
  const auto append_surface=[&](const EventKey & key,std::uint64_t revision,const sensor_msgs::msg::PointCloud2 & points){
    if(!revision||points.data.empty()||points.header.frame_id!="object_"+std::to_string(key.track_id)||!same_cycle(key,grasp_surface_cycle_key()))return;
    if(revision==ack_grasp_surface_revision&&same_cycle(key,ack_grasp_surface_key) &&
      !(command.kind==CommandKind::kAttachObject && (task.phase==TaskPhase::kRecovering || task.phase==TaskPhase::kFaulted)))return;
    const auto object=std::find_if(scene.dynamic_objects.begin(),scene.dynamic_objects.end(),[&](const auto & item){return same_track_identity(item.key,key);});
    if(object==scene.dynamic_objects.end())return;
    ptc_interfaces::msg::ObjectSurfaceGeometry surface;surface.key=to_ros(key);surface.cloud_revision=revision;surface.points=points;
    goal.grasp_surfaces.push_back(std::move(surface));surface_key=key;surface_revision=revision;
  };
  if(!demo_grasp_surface.points.empty()){
    sensor_msgs::msg::PointCloud2 points;points.header.frame_id=demo_grasp_surface.object_frame;
    points.header.stamp=ptc_app::to_ros_time(demo_grasp_surface.points.back().capture_timestamp);
    sensor_msgs::PointCloud2Modifier modifier(points);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(demo_grasp_surface.points.size());
    sensor_msgs::PointCloud2Iterator<float> x(points,"x"),y(points,"y"),z(points,"z");
    for(const auto & point:demo_grasp_surface.points){
      *x=static_cast<float>(point.position_object_m.x_m);*y=static_cast<float>(point.position_object_m.y_m);*z=static_cast<float>(point.position_object_m.z_m);
      ++x;++y;++z;
    }
    append_surface(demo_grasp_surface.key,demo_grasp_surface.revision,points);
  }
  sync_busy=true;send<Sync>(sync_client,goal,[this,command,surface_key,surface_revision](auto result){if(command.command_id!=scene.pending_command_id || command.scene_revision!=scene.revision)return;sync_busy=false;SceneEvent e;e.key=event_key();e.key.track_id=command.key.track_id;e.command_id=command.command_id;e.scene_revision=command.scene_revision;
    const bool success=result&&result->success&&result->command_id==command.command_id&&result->scene_revision==command.scene_revision&&same_operation(from_ros(result->key),command.key);
    e.kind=!success?SceneEventKind::kUpdateFailed:command.kind==CommandKind::kAttachObject?SceneEventKind::kAttachConfirmed:command.kind==CommandKind::kDetachObject?SceneEventKind::kDetachConfirmed:SceneEventKind::kUpdateConfirmed;
    const auto d=advance_scene(scene,e,*settings.snapshot,now());scene=d.state;if(success&&scene.coherent){ack_scene=scene;if(surface_revision){ack_grasp_surface_key=surface_key;ack_grasp_surface_revision=surface_revision;}if(command.kind==CommandKind::kAttachObject && task.phase==TaskPhase::kPicking && task.grasp.phase==GraspPhase::kHolding){attachment_ack_joint_stamp=arm.joint_timestamp;}}
    else{reason="scene sync failed: "+d.status.reason+(result?"; backend="+result->message:"; goal rejected/unavailable");RCLCPP_WARN(node.get_logger(),"%s",reason.c_str());}
  },false);
}
void TaskCoordinatorNode::Runtime::publish_follow_target()
{
  if(!follow_active){return;}
  // A selected candidate awaiting Follow admission carries its last command,
  // not a new belt-speed step. A stopped boundary remains stopped until the
  // Hermite is admitted; an already moving boundary retains its own twist.
  if(task.phase==TaskPhase::kPreparingPick && grasp_pose.valid && !grasp_motion_reference && grasp_command_state){
    const auto time=now();
    if(time.source_now<=last_target_stamp)return;
    if(std::chrono::duration<double>(time.source_now-grasp_command_state->timestamp).count()>config().motion.target_update_timeout_sec){
      skip_observation_target("pending grasp command expired beyond existing target update timeout");return;}
    const auto command=extrapolate_grasp_entry(*grasp_command_state,time.source_now);
    if(!command){skip_observation_target("invalid pending same-owner grasp command");return;}
    last_target_stamp=time.source_now;ptc_interfaces::msg::TargetUpdate update;auto key=follow_key;key.sequence=sequence++;
    update.key=to_ros(key);update.command_id=follow_command;update.target=ptc_app::stamped_pose(command->tcp_world,config().frames.world,time.source_now);
    update.velocity.linear.x=command->linear_world.x_m;update.velocity.linear.y=command->linear_world.y_m;update.velocity.linear.z=command->linear_world.z_m;
    update.velocity.angular.x=command->angular_world.x_m;update.velocity.angular.y=command->angular_world.y_m;update.velocity.angular.z=command->angular_world.z_m;
    update.valid_for=ptc_app::to_ros_duration(config().motion.target_update_timeout_sec);target_pub->publish(update);grasp_command_state=*command;return;
  }
  if(grasp_motion_reference && (task.phase==TaskPhase::kPreparingPick || task.phase==TaskPhase::kPicking)){
    const auto time=now();const auto sample=sample_current_grasp_motion(time.source_now+*seconds_to_duration(config().grasp.intercept_lead_sec));
    if(!sample){skip_observation_target("continuous grasp sample invalid; cursor_sec="+std::to_string(grasp_motion_elapsed));return;}
    if(!inside_workspace(sample->tcp_world,config().motion)){
      const auto & p=sample->tcp_world.position;
      skip_observation_target("continuous grasp path outside workspace; xyz="+std::to_string(p.x_m)+","+std::to_string(p.y_m)+","+std::to_string(p.z_m));return;
    }
    if(time.source_now<=last_target_stamp)return;last_target_stamp=time.source_now;
    ptc_interfaces::msg::TargetUpdate update;auto key=follow_key;key.sequence=sequence++;update.key=to_ros(key);update.command_id=follow_command;
    update.target=ptc_app::stamped_pose(sample->tcp_world,config().frames.world,time.source_now);
    update.velocity.linear.x=sample->linear_world.x_m;update.velocity.linear.y=sample->linear_world.y_m;update.velocity.linear.z=sample->linear_world.z_m;
    update.velocity.angular.x=sample->angular_world.x_m;update.velocity.angular.y=sample->angular_world.y_m;update.velocity.angular.z=sample->angular_world.z_m;
    update.valid_for=ptc_app::to_ros_duration(config().motion.target_update_timeout_sec);target_pub->publish(update);
    if(task.phase==TaskPhase::kPreparingPick || task.phase==TaskPhase::kPicking){
      grasp_command_state=GraspEntryState{from_ros(update.target.pose),
        {update.velocity.linear.x,update.velocity.linear.y,update.velocity.linear.z},
        {update.velocity.angular.x,update.velocity.angular.y,update.velocity.angular.z},from_ros(update.target.header.stamp)};
    }
    if(grasp_motion_reference->relative_approach){
      const auto measured=tcp_world();const auto & p=sample->tcp_world.position;
      const auto nominal=sample_current_grasp_motion(time.source_now);
      RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),500,
        "relative grasp command: source=%.9f measured=%.9f lead_sec=%.3f path_sec=%.6f issued=(%.6f,%.6f,%.6f) nominal=(%.6f,%.6f,%.6f) actual=(%.6f,%.6f,%.6f)",
        std::chrono::duration<double>(time.source_now).count(),std::chrono::duration<double>(measured.timestamp).count(),
        config().grasp.intercept_lead_sec,grasp_motion_elapsed,p.x_m,p.y_m,p.z_m,
        nominal->tcp_world.position.x_m,nominal->tcp_world.position.y_m,nominal->tcp_world.position.z_m,
        measured.pose.position.x_m,measured.pose.position.y_m,measured.pose.position.z_m);
    }
    return;
  }const auto track=selected();const auto time=now();
  // Grasp setpoints include the existing short command-latency lead while
  // Closing and Verifying continue to receive velocity feed-forward. This is
  // a control setpoint, never a relabelled visual measurement.
  auto predicted=predict_object_pose(track,*settings.snapshot,time,task.phase==TaskPhase::kPicking?config().grasp.intercept_lead_sec:0.);
  if(!predicted){predicted=closing_motion_object(time);
    if(predicted){predicted->position.x_m+=probe_baseline.velocity_mps.x_m*config().grasp.intercept_lead_sec;predicted->position.y_m+=probe_baseline.velocity_mps.y_m*config().grasp.intercept_lead_sec;predicted->position.z_m+=probe_baseline.velocity_mps.z_m*config().grasp.intercept_lead_sec;}}
  if(!predicted){stop_following_target();skip_observation_target("moving grasp/observation prediction unavailable or expired");return;}
  std::optional<Pose3d> target;Vector3d approach_linear{},approach_angular{};
  if(task.phase==TaskPhase::kPreparingPick && grasp_follow_hold_in_object){target=compose_pose(*predicted,*grasp_follow_hold_in_object);}
  else if(task.phase==TaskPhase::kPicking && grasp_pose.valid){
    if((task.grasp.phase==GraspPhase::kApproaching || task.grasp.phase==GraspPhase::kClosing) && grasp_approach_reference){
      const auto sample=sample_grasp_approach(*grasp_approach_reference,*predicted,time.source_now);
      if(sample){target=sample->tcp_world;approach_linear=sample->relative_linear_world;approach_angular=sample->relative_angular_world;}
    }else if(task.grasp.phase==GraspPhase::kApproaching){target=compose_pose(*predicted,grasp_pose.pregrasp_in_object);}
    else{target=compose_pose(*predicted,grasp_pose.tcp_in_object);}
  }
  if(task.grasp.phase==GraspPhase::kVerifying && probe_valid){
    auto expected=probe_baseline.observation.pose_world;const double dt=std::chrono::duration<double>(time.source_now-probe_baseline.observation.timestamp).count()+config().grasp.intercept_lead_sec;
    expected.position.x_m+=probe_baseline.velocity_mps.x_m*dt;expected.position.y_m+=probe_baseline.velocity_mps.y_m*dt;expected.position.z_m+=probe_baseline.velocity_mps.z_m*dt;
    target=compose_pose(expected,probe_relative);if(target){target->position.z_m+=config().grasp.verification_probe_m;}
  }
  if(!target || !inside_workspace(*target,config().motion)){stop_following_target();skip_observation_target("dynamic target outside valid workspace");return;}
  // Stamp the command dispatch time. The configured grasp latency lead is
  // part of the desired setpoint; the visual timestamp remains unchanged and
  // predict_object_pose still enforces the original prediction horizon.
  if(time.source_now<=last_target_stamp){return;}last_target_stamp=time.source_now;
  ptc_interfaces::msg::TargetUpdate update;auto key=follow_key;key.sequence=sequence++;update.key=to_ros(key);update.command_id=follow_command;update.target=ptc_app::stamped_pose(*target,config().frames.world,time.source_now);
  const auto velocity=task.grasp.phase==GraspPhase::kVerifying&&probe_valid?probe_baseline.velocity_mps:closing_motion_object(time)?probe_baseline.velocity_mps:track.velocity_mps;
  update.velocity.linear.x=velocity.x_m+approach_linear.x_m;update.velocity.linear.y=velocity.y_m+approach_linear.y_m;update.velocity.linear.z=velocity.z_m+approach_linear.z_m;
  update.velocity.angular.x=approach_angular.x_m;update.velocity.angular.y=approach_angular.y_m;update.velocity.angular.z=approach_angular.z_m;
  update.valid_for=ptc_app::to_ros_duration(config().motion.target_update_timeout_sec);target_pub->publish(update);
    if(task.phase==TaskPhase::kPreparingPick || task.phase==TaskPhase::kPicking){
      grasp_command_state=GraspEntryState{from_ros(update.target.pose),
        {update.velocity.linear.x,update.velocity.linear.y,update.velocity.linear.z},
        {update.velocity.angular.x,update.velocity.angular.y,update.velocity.angular.z},from_ros(update.target.header.stamp)};
    }
  if(task.phase==TaskPhase::kPicking && task.grasp.phase!=GraspPhase::kApproaching){
    RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),500,
      "grasp follow update: phase=%d source=%.6f xyz=(%.4f,%.4f,%.4f) velocity=(%.4f,%.4f,%.4f)",
      static_cast<int>(task.grasp.phase),std::chrono::duration<double>(time.source_now).count(),
      target->position.x_m,target->position.y_m,target->position.z_m,
      update.velocity.linear.x,update.velocity.linear.y,update.velocity.linear.z);
  }
}
void TaskCoordinatorNode::Runtime::tick()
{
  try{
    completions.drain();const auto time=now();const auto wall=std::chrono::steady_clock::now();
    if(time.paused || time.source_now<source_sample){source_rate=control_period_source=kUnset;observation_period_source=kUnset;tracks_sample_stamp=Nanoseconds{0};source_sample=time.source_now;source_sample_wall=wall;}
    else if(time.source_now>source_sample){
      const double step=std::chrono::duration<double>(time.source_now-source_sample).count();
      if(source_sample.count()>0){control_period_source=std::isfinite(control_period_source)?control_period_source*.8+step*.2:step;}
      const double elapsed=source_sample_wall.time_since_epoch().count()?std::chrono::duration<double>(wall-source_sample_wall).count():0;
      if(elapsed>0){const double rate=std::chrono::duration<double>(time.source_now-source_sample).count()/elapsed;
        if(std::isfinite(rate)&&rate>0){source_rate=std::isfinite(source_rate)?source_rate*.8+rate*.2:rate;}}
      source_sample=time.source_now;source_sample_wall=wall;
    }
    if(epoch && epoch!=time.epoch){
      last_completed_key={};last_outcome=CycleOutcome::kNone;
      pending_reset=false;
      if(semantic_preview_sub){continuous_operation=false;last_semantic_preview.reset();}
      if(manual_home_pending()){manual_home_state=Status::HOME_FAILED;manual_home_message="Simulation clock reset during Return Home";}
      stop();scene_change(SceneEventKind::kClockReset);admission_attempts.clear();deferred_tracks.clear();retired.clear();last_admission_track_id=0;if(task.phase!=TaskPhase::kWaitingReady){apply(event(TaskEventKind::kClockReset));}
    }
    epoch=time.epoch;wait_for_observation_arrival(time);if(completions.overflowed()){fault("owner completion queue overflow");return;}
    if(sync_busy && expired(scene.context.deadline,time)){sync_busy=false;sync_client->async_cancel_all_goals();scene_change(SceneEventKind::kExpire);reason="scene acknowledgment expired";}
    // A failed initial static upload must not permanently latch an idle,
    // measured-empty workstation. Reuse the existing reset/reload protocol;
    // only rebuild while idle/recovering, never with an uncertain attachment.
    if(scene_initialized && !sync_busy && scene.context.inhibited &&
      (task.phase==TaskPhase::kWaitingReady || task.phase==TaskPhase::kRecovering) &&
      scene.attachment==AttachmentPhase::kDetached && !scene.attached_track_id &&
      !time.paused && arm.stopped && gripper.stopped && gripper.empty_confirmed &&
      arm_snapshot_ready(arm,*settings.snapshot,time) && gripper_snapshot_ready(gripper,*settings.snapshot,time)){
      scene_change(SceneEventKind::kResetRequested);
      if(!scene.context.inhibited){scene_initialized=false;RCLCPP_WARN(node.get_logger(),"reload failed scene after measured empty stop; fresh vision will repopulate obstacles");}
    }
    // Recovery gets the first available scene revision before fresh obstacle
    // uploads can occupy it again. The same existing pending_reset owns retry.
    reopen_failed_demo_grasp();
    if(!scene_initialized && !time.paused && arm_snapshot_ready(arm,*settings.snapshot,time) && gripper_snapshot_ready(gripper,*settings.snapshot,time) && sync_client->action_server_is_ready()){
      if(scene_change(SceneEventKind::kStaticLoaded))scene_initialized=true;
    }
    if(pending_attach && (task.phase!=TaskPhase::kPicking || task.grasp.phase!=GraspPhase::kHolding || task.grasp.scene_attached)){pending_attach=false;}
    if(scene_initialized && !sync_busy && !checking_target && !scene.context.inhibited){
      scene_change(SceneEventKind::kExpire);bool changed=sync_busy||scene.context.inhibited;
      if(!changed && pending_attach && arm.stopped){
        changed=scene_change(SceneEventKind::kAttachRequested);
        if(changed){pending_attach=false;}
        else{RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,"grasp attachment pending: %s",reason.c_str());}
      }
      if(!changed){std::optional<Command> latest_scene_update;for(const auto & track:tracks){if(track.context.key.track_id==scene.attached_track_id && scene.attachment!=AttachmentPhase::kDetached){continue;}
        const auto previous=std::find_if(scene.dynamic_objects.begin(),scene.dynamic_objects.end(),[&](const auto & x){return x.key.track_id==track.context.key.track_id;});
        if(track_is_usable(track,*settings.snapshot,time) && (previous==scene.dynamic_objects.end()||track.observation.timestamp>previous->observation.timestamp)){
          SceneEvent e;e.kind=SceneEventKind::kObservation;e.key=event_key();e.object={track.context.key,track.observation,{}};
          // Expire/sync callbacks earlier in this tick sampled a newer
          // monotonic time. Never submit this event with the tick's old sample.
          const auto d=advance_scene(scene,e,*settings.snapshot,now());scene=d.state;
          if(d.status.disposition==Disposition::kRejected){RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,"dynamic scene observation rejected: %s",d.status.reason.c_str());}
          // Refresh the whole received track set before one ApplyPlanningScene.
          // Repeatedly choosing the first track starves later obstacles and
          // makes their old timestamps cancel otherwise valid motion.
          if(!d.commands.empty()){latest_scene_update=d.commands.back();}}}
        if(latest_scene_update){sync_scene(*latest_scene_update);changed=true;}}
      if(!changed){scene_change(SceneEventKind::kExpire);}
    }
    if(pending_reset && scene_ready(ack_scene,*settings.snapshot,time) && motion_command_ready){
      const bool invalidated_cycle=task.context.key.epoch!=time.epoch;
      const bool failed_demo=!invalidated_cycle && config().runtime.operation_mode=="assumed_ng_demo" &&
        (task.outcome==CycleOutcome::kFailed || task.outcome==CycleOutcome::kInconclusive);
      auto recovery=event(failed_demo?TaskEventKind::kRecoveryConfirmed:TaskEventKind::kResetRequested);
      if(invalidated_cycle){recovery.key.track_id=0;}
      const auto disposition=apply(recovery);
      if(disposition==Disposition::kApplied && (task.phase==TaskPhase::kWaitingReady || task.phase==TaskPhase::kRecovering || task.phase==TaskPhase::kReporting)){pending_reset=false;}
    }
    if(continuous_operation && semantic_preview_sub && !semantic_preview_fresh()){
      pause_continuous();
      reason="Continuous Auto Pick paused; Observe data expired";
    }
    if(continuous_operation && task.phase==TaskPhase::kWaitingReady){
      if(time.paused){reason="continuous: waiting for simulation and readiness";}
      else{const auto d=begin_next_cycle();if(d.status.disposition!=Disposition::kApplied){reason="continuous: waiting; "+d.status.reason;}}
    }
    if(task.phase!=TaskPhase::kWaitingReady && task.phase!=TaskPhase::kFaulted){apply(event(TaskEventKind::kTick));}
    if((task.phase==TaskPhase::kPicking || task.phase==TaskPhase::kTransporting || task.phase==TaskPhase::kPlacing) &&
      task.grasp.phase==GraspPhase::kHolding && gripper.stopped &&
      (gripper.empty_confirmed || (config().grasp.finger_effort_thresholds_n && !gripper.contact_confirmed)) &&
      gripper_snapshot_ready(gripper,*settings.snapshot,time)){
      grasp(grasp_event(GraspEventKind::kEvidenceUpdated));
    }
    // The child deadline must advance even when no new image arrives. A task
    // tick alone does not tick the grasp state machine.
    if((task.phase==TaskPhase::kPicking || task.phase==TaskPhase::kPlacing) &&
      grasp_stage_expired(task.grasp,time)){
      record_observation_step("grasp_stage_timeout","existing stage deadline reached without completion; stop and recover, no Holding assumed",task.context.key.track_id);
      grasp(grasp_event(GraspEventKind::kTick));
    }
    if(task.phase!=TaskPhase::kWaitingReady && task.phase!=TaskPhase::kFaulted && task.phase!=TaskPhase::kRecovering && task.phase!=TaskPhase::kReporting){
      auto e=event(TaskEventKind::kReadinessChanged);if(!system_ready(e,*settings.snapshot,time,task.phase)){
        if(sync_busy && !scene.context.inhibited && !time.paused && e.readiness.vision_ready && e.readiness.inference_ready &&
          arm.state.motion_state!=ArmMotionState::kFault && arm_snapshot_ready(arm,*settings.snapshot,time) && gripper_snapshot_ready(gripper,*settings.snapshot,time)){
          RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,
            "await_scene_ack: background update pending; new motion still requires its acknowledged scene");
        }else if((task.phase==TaskPhase::kPreparingPick||task.phase==TaskPhase::kPicking)&&
          !time.paused&&arm.state.motion_state!=ArmMotionState::kFault&&arm_snapshot_ready(arm,*settings.snapshot,time)&&
          gripper_snapshot_ready(gripper,*settings.snapshot,time)&&scene_ready(e.scene,*settings.snapshot,time,task.selected_track.context.key.track_id)){
          RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),2000,
            "continue_observation_prediction: image/inference gap; device feedback remains ready, motion continues");
        }else if(unassigned_observation_wait(task,e,*settings.snapshot,time)){
          RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),2000,
            "observe_wait: no target assigned; vision=%d inference=%d motion=%d scene=%d deferred to target selection",
            e.readiness.vision_ready,e.readiness.inference_ready,motion_command_ready,scene_ready(e.scene,*settings.snapshot,time));
        }else{
          RCLCPP_WARN(node.get_logger(),"readiness lost: vision=%d inference=%d motion=%d arm=%d gripper=%d scene=%d",e.readiness.vision_ready,e.readiness.inference_ready,motion_command_ready,arm_snapshot_ready(arm,*settings.snapshot,time),gripper_snapshot_ready(gripper,*settings.snapshot,time),scene_ready(e.scene,*settings.snapshot,time));
          const auto age_ms=[&](Nanoseconds stamp){
            return std::chrono::duration<double,std::milli>(time.source_now-stamp).count();};
          record_observation_step("readiness_lost","phase="+std::to_string(static_cast<int>(task.phase))+
            " vision="+std::to_string(e.readiness.vision_ready)+" inference="+std::to_string(e.readiness.inference_ready)+
            " motion="+std::to_string(motion_command_ready)+" arm_fresh="+std::to_string(arm_snapshot_ready(arm,*settings.snapshot,time))+
            " gripper_fresh="+std::to_string(gripper_snapshot_ready(gripper,*settings.snapshot,time))+
            " scene="+std::to_string(scene_ready(e.scene,*settings.snapshot,time))+
            " selected_scene="+std::to_string(scene_ready(e.scene,*settings.snapshot,time,task.selected_track.context.key.track_id))+
            " paused="+std::to_string(time.paused)+" arm_state="+std::to_string(static_cast<int>(arm.state.motion_state))+
            " revisions="+std::to_string(e.readiness.vision_revision==settings.snapshot->revision())+
            std::to_string(e.readiness.inference_revision==settings.snapshot->revision())+
            std::to_string(e.readiness.motion_revision==settings.snapshot->revision())+
            " vision_age_ms="+std::to_string(age_ms(from_ros(vision_status.header.stamp)))+
            " vision_publisher_ready="+std::to_string(vision_status.ready)+
            " joint_age_ms="+std::to_string(age_ms(e.arm.joint_timestamp))+
            " tcp_age_ms="+std::to_string(age_ms(e.arm.tcp_timestamp))+
            " arm_feedback_ready="+std::to_string(e.arm.state.status.ready)+
            " arm_controller_ready="+std::to_string(e.arm.controller_ready)+
            " arm_tf_valid="+std::to_string(e.arm.tf_valid)+
            " gripper_age_ms="+std::to_string(age_ms(e.gripper.timestamp))+
            " gripper_feedback_ready="+std::to_string(e.gripper.status.ready)+
            " feedback_epochs="+std::to_string(e.arm.epoch)+"/"+std::to_string(e.gripper.epoch)+"/"+
            std::to_string(time.epoch),task.context.key.track_id);
          apply(e);
        }
      }
      if((task.phase==TaskPhase::kPreparingPick||task.phase==TaskPhase::kPicking)&&task.grasp.possession!=Possession::kHolding && !track_is_usable(selected(),*settings.snapshot,time,true)){
        const auto track=selected();const bool present=std::any_of(tracks.begin(),tracks.end(),[&](const auto & item){return same_track_identity(item.context.key,track.context.key);});
        record_observation_step("validate_observation_track","lost: present="+std::to_string(present)+" phase="+std::to_string(static_cast<int>(track.phase))+" age_sec="+std::to_string(std::chrono::duration<double>(time.source_now-track.observation.timestamp).count()),task.context.key.track_id);
        if(!grasp_motion_reference && !closing_motion_object(time)){skip_observation_target("selected visual track unavailable; age_sec="+std::to_string(std::chrono::duration<double>(time.source_now-track.observation.timestamp).count()));}
      }
    }
    if(task.phase==TaskPhase::kObserving && task.observation_arrived && !checking_target && !sync_busy && !tracks.empty()){
      std::size_t start=0;const auto previous=std::find_if(tracks.begin(),tracks.end(),[this](const auto & track){return track.context.key.track_id==last_admission_track_id;});
      if(previous!=tracks.end()){start=(static_cast<std::size_t>(std::distance(tracks.begin(),previous))+1)%tracks.size();}
      for(std::size_t offset=0;offset<tracks.size();++offset){const auto & track=tracks[(start+offset)%tracks.size()];const auto candidate=candidate_key(track);
        const auto deferred=deferred_tracks.find(candidate);const bool deferred_this_cycle=deferred!=deferred_tracks.end()&&deferred->second==task.context.key.cycle_id;
        if(!deferred_this_cycle && !retired.count(candidate) && track_is_usable(track,*settings.snapshot,time) &&
          candidate_admission_due(track,time)){
          last_admission_track_id=track.context.key.track_id;try_select_observation_target(track);break;}}
    }
    if(task.phase==TaskPhase::kPreparingPick && !grasp_motion_reference &&
      grasp_pose.valid && follow_active && !move_busy && !checking_target && arm.state.motion_state==ArmMotionState::kFollowing){start_relative_grasp_reference();}
    advance_grasp_motion_reference(time);
    if(task.phase==TaskPhase::kPreparingPick){
      if(demo_grasp_surface.points.empty())capture_grasp_surface();
      if(gripper_snapshot_ready(gripper,*settings.snapshot,time) && gripper.stopped && gripper.empty_confirmed){
        if(gripper_measured_open_empty(gripper,config()))observation_open_command.reset();
        else open_for_grasp_preparation();
      }
    }
    if(task.phase==TaskPhase::kPreparingPick && !grasp_motion_reference && !demo_grasp_surface.points.empty() &&
      !move_busy && !(grasp_pose.valid && follow_active) && arm.stopped && gripper_snapshot_ready(gripper,*settings.snapshot,time) && gripper_measured_open_empty(gripper,config()) && !checking_target){check_grasp_candidate();}
    else if(task.phase==TaskPhase::kPreparingPick){
      RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,
        "grasp preparation waiting: move_busy=%d arm_stopped=%d gripper_ready=%d gripper_stopped=%d empty=%d evaluating=%d",
        move_busy,arm.stopped,gripper_snapshot_ready(gripper,*settings.snapshot,time),gripper.stopped,gripper.empty_confirmed,checking_target);
    }
    if(task.phase==TaskPhase::kPicking){
      const auto measured=tcp_world();const auto track=selected();
      if(!grasp_motion_reference && last_tcp_stamp.count()>0 && measured.timestamp>last_tcp_stamp){const double dt=std::chrono::duration<double>(measured.timestamp-last_tcp_stamp).count();const auto p=transform_point(measured.pose,config().grasp.finger_center_tcp);const auto q=transform_point(previous_tcp_world,config().grasp.finger_center_tcp);
        relative_speed=vector_norm({(p.x_m-q.x_m)/dt-track.velocity_mps.x_m,(p.y_m-q.y_m)/dt-track.velocity_mps.y_m,(p.z_m-q.z_m)/dt-track.velocity_mps.z_m});}
      previous_tcp_world=measured.pose;last_tcp_stamp=measured.timestamp;
      if(task.grasp.phase==GraspPhase::kApproaching && !grasp_motion_reference){const auto object=track_pose_at_observation(track,*settings.snapshot,time,measured.timestamp);const auto desired=object?compose_pose(*object,grasp_pose.tcp_in_object):std::nullopt;
        const auto pregrasp=object?compose_pose(*object,grasp_pose.pregrasp_in_object):std::nullopt;
        const bool tracking=grasp_follow_feedback&&grasp_follow_feedback->tracking&&
          source_is_fresh(from_ros(grasp_follow_feedback->current_pose.header.stamp),time,config().vision.frame_max_age_sec,config().time.sensor_future_tolerance_sec);
        if(!grasp_approach_reference && pregrasp && tracking && arm_snapshot_ready(arm,*settings.snapshot,time) &&
          grasp_center_aligned(measured.pose,*pregrasp,config().grasp.finger_center_tcp,relative_speed,
            config().grasp.max_relative_linear_speed_mps,config().inspection.view_target_position_tolerance_m,config().inspection.view_target_orientation_tolerance_rad)){
          const auto inverse=inverse_pose(*object);const auto relative=inverse?compose_pose(*inverse,measured.pose):std::nullopt;
          const double minimum=std::isfinite(control_period_source)?3*control_period_source:0.;
          grasp_approach_reference=relative?make_grasp_approach_reference(*relative,grasp_pose.tcp_in_object,time.source_now,config().motion.max_follow_linear_speed_mps,config().motion.max_follow_angular_speed_radps,track.velocity_mps,minimum):std::nullopt;
          if(grasp_approach_reference){record_observation_step("start_grasp_descent","same-source-time grasp center aligned with moving pregrasp; relative_speed="+std::to_string(relative_speed)+" duration_sec="+std::to_string(grasp_approach_reference->duration_sec)+" minimum_from_control_cadence="+std::to_string(minimum),task.context.key.track_id);}
        }
        if(desired && wall-grasp_diagnostic_at>=std::chrono::milliseconds(500)){
          grasp_diagnostic_at=wall;const auto a=transform_point(measured.pose,config().grasp.finger_center_tcp);const auto b=transform_point(grasp_approach_reference?*desired:*pregrasp,config().grasp.finger_center_tcp);
          record_observation_step("follow_grasp_target","stage="+std::string(grasp_approach_reference?"descent":"align_pregrasp")+" tracking="+std::to_string(tracking)+" error_m="+std::to_string(vector_norm({a.x_m-b.x_m,a.y_m-b.y_m,a.z_m-b.z_m}))+" feedback_age_sec="+std::to_string(std::chrono::duration<double>(time.source_now-measured.timestamp).count())+" angle_rad="+std::to_string(2*std::acos(std::clamp(std::abs(measured.pose.orientation.x*desired->orientation.x+measured.pose.orientation.y*desired->orientation.y+measured.pose.orientation.z*desired->orientation.z+measured.pose.orientation.w*desired->orientation.w),0.,1.)))+" actual=("+std::to_string(a.x_m)+","+std::to_string(a.y_m)+","+std::to_string(a.z_m)+") target=("+std::to_string(b.x_m)+","+std::to_string(b.y_m)+","+std::to_string(b.z_m)+") relative_speed="+std::to_string(relative_speed),task.context.key.track_id);
        }
        const auto approach_sample=grasp_approach_reference&&object?sample_grasp_approach(*grasp_approach_reference,*object,time.source_now):std::nullopt;
        if(approach_sample && approach_sample->progress>=1. && desired && tracking &&
          grasp_center_aligned(measured.pose,*desired,config().grasp.finger_center_tcp,relative_speed,
            config().grasp.max_relative_linear_speed_mps,config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad)){
          record_observation_step("attempt_moving_grasp","measured grasp center at moving contact and speed matched; close while retaining follow, pickup still unproven",task.context.key.track_id);
          grasp(grasp_event(GraspEventKind::kApproachCompleted));}}
      else if(task.grasp.phase==GraspPhase::kVerifying){
        const bool demo_contact_verification=grasp_motion_reference.has_value();
        if(demo_contact_verification){
          // Complete the configured verification probe before contact can stop Follow.
          // Holding/SceneAttached owns the remaining full lift; its execution
          // budget must not be consumed inside the short verification stage.
          // Joint blockage alone is not evidence that the object was lifted.
          const bool probe_arrived=probe_valid && grasp_motion_elapsed>=grasp_motion_reference->lift_end()-1e-8 &&
            measured.pose.position.z_m-probe_tcp.position.z_m>=config().grasp.verification_probe_m-config().motion.position_tolerance_m;
          const double absolute=std::chrono::duration<double>(time.source_now-grasp_motion_reference->source_start).count();
          const bool follow_braked=vector_norm({grasp_motion_reference->object_velocity.x_m+grasp_motion_offset.to_velocity.x_m,
            grasp_motion_reference->object_velocity.y_m+grasp_motion_offset.to_velocity.y_m,grasp_motion_reference->object_velocity.z_m+grasp_motion_offset.to_velocity.z_m})<1e-9 &&
            absolute>=grasp_motion_offset.start_sec+grasp_motion_offset.duration_sec;
          if(probe_arrived && follow_braked && gripper_snapshot_ready(gripper,*settings.snapshot,time) &&
            gripper.evidence_sequence>task.grasp.evidence_sequence && gripper.timestamp>task.grasp.evidence_timestamp){
            // Semantic wrist registration and track smoothing lag the lift.
            // Its estimated rise is useful for diagnostics, not a grasp veto.
            if(config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model" &&
              gripper.contact_confirmed && track.phase==TrackPhase::kTracking &&
              same_track_identity(track.context.key,probe_baseline.context.key) &&
              track_is_usable(track,*settings.snapshot,time) && track.observation.timestamp>probe_baseline.observation.timestamp){
              const double rise=track.observation.pose_world.position.z_m-probe_baseline.observation.pose_world.position.z_m;
              record_observation_step("wrist_lift_estimate","diagnostic target_rise_m="+std::to_string(rise),task.context.key.track_id);
            }
            auto e=grasp_event(GraspEventKind::kEvidenceUpdated);e.gripper.visual_follow_confirmed=false;
            if(config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model"){
              e.expected_contact_span_m=grasp_pose.opening_width_m;
            }
            record_observation_step("verify_grasp_contact","moving probe reached; lift_m="+std::to_string(measured.pose.position.z_m-probe_tcp.position.z_m)+"; contact proxy, physical pickup still requires verification; contact="+
              std::to_string(e.gripper.contact_confirmed)+" width_m="+std::to_string(e.gripper.width_m)+
              " expected_contact_span_m="+std::to_string(grasp_pose.opening_width_m)+
              " aperture_minus_span_m="+std::to_string(e.gripper.width_m-grasp_pose.opening_width_m)+
              "; geometric comparison only; pad calibration/compression and partial visibility affect the difference",
              task.context.key.track_id);
            const auto g=e.gripper;
            grasp(e);
            if(task.grasp.phase==GraspPhase::kVerifying && !task.grasp.context.history.empty()){
              const auto & diagnostic=task.grasp.context.history.back();
              RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),500,
                "grasp probe evidence: stopped=%d contact=%d empty=%d width=%.6f ready=%d reason=%s",
                g.stopped,g.contact_confirmed,g.empty_confirmed,g.width_m,gripper_snapshot_ready(g,*settings.snapshot,now()),diagnostic.reason.c_str());
            }
          }
        }else if(probe_valid && track.observation.timestamp>visual_evidence_stamp){
          auto e=grasp_event(GraspEventKind::kEvidenceUpdated);const double dt=std::chrono::duration<double>(track.observation.timestamp-probe_baseline.observation.timestamp).count();
          const double expected_z=probe_baseline.observation.pose_world.position.z_m+probe_baseline.velocity_mps.z_m*dt;
          const auto relative=compose_pose(*inverse_pose(track.observation.pose_world),measured.pose);
          e.gripper.visual_follow_confirmed=track_is_usable(track,*settings.snapshot,time)&&relative&&
            track.observation.pose_world.position.z_m-expected_z>=config().grasp.verification_probe_m*0.5 &&
            measured.pose.position.z_m-probe_tcp.position.z_m>=config().grasp.verification_probe_m*0.5 &&
            pose_near(*relative,probe_relative,config().motion.position_tolerance_m+config().tracking.max_position_stddev_m,config().motion.orientation_tolerance_rad);
          visual_evidence_stamp=track.observation.timestamp;grasp(e);
        }
      }
      // Attachment ACK and arm status arrive independently. The first
      // planner motion needs a new stopped sample after the ACK.
      if(attachment_ack_joint_stamp && arm.joint_timestamp>*attachment_ack_joint_stamp &&
        motion_command_ready && arm.state.motion_state==ArmMotionState::kIdle &&
        arm_snapshot_ready(arm,*settings.snapshot,time) && arm.stopped){
        attachment_ack_joint_stamp.reset();
        auto e=grasp_event(GraspEventKind::kSceneAttached);
        e.scene_confirmed=ack_scene.attachment==AttachmentPhase::kAttached;
        grasp(e);
      }
    }
    if(pending_release && gripper_measured_open_empty(gripper,config())){
      if((task.phase==TaskPhase::kFaulted || task.phase==TaskPhase::kRecovering) && config().runtime.operation_mode=="assumed_ng_demo" &&
        (task.outcome==CycleOutcome::kFailed || task.outcome==CycleOutcome::kInconclusive || manual_home_pending())){
        pending_release=false;
        reopen_failed_demo_grasp();record_observation_step("recover_failed_grasp","opening completed and measured Empty; detach/reset/Home recovery continues",task.context.key.track_id);
      }else if(task.grasp.phase==GraspPhase::kReleasing){
        // Action ACK and joint-stop feedback arrive independently. Retain
        // this completion if the core rejects a transient measured sample;
        // retry on the next tick within the original release deadline.
        grasp(grasp_event(GraspEventKind::kReleaseCompleted));
        if(task.grasp.phase!=GraspPhase::kReleasing){
          pending_release=false;
          if(task.grasp.phase==GraspPhase::kEmpty){record_observation_step("confirm_gripper_release","fresh measured Empty and stops accepted; scene detach pending",task.context.key.track_id);}
        }
      }else{pending_release=false;}
    }
    if(task.phase==TaskPhase::kPlacing && task.grasp.phase==GraspPhase::kEmpty && ack_scene.attachment==AttachmentPhase::kDetached && ack_scene.coherent){apply(event(TaskEventKind::kPlaceConfirmed));}
    if(task.phase==TaskPhase::kRecovering && !move_busy && !sync_busy && recovery_stop_ack_stamp &&
      arm.tcp_timestamp>*recovery_stop_ack_stamp && motion_command_ready && arm.state.motion_state==ArmMotionState::kIdle &&
      arm.stopped && gripper.stopped){apply(event(TaskEventKind::kRecoveryConfirmed));}
    open_while_waiting(time);
    advance_manual_home(time);
    publish_follow_target();Status status;status.key=to_ros(task.context.key);status.phase=static_cast<std::uint8_t>(task.phase);status.motion_phase=static_cast<std::uint8_t>(arm.state.motion_state);status.grasp_phase=static_cast<std::uint8_t>(task.grasp.phase);status.grasp_possession=static_cast<std::uint8_t>(task.grasp.possession);status.home_state=manual_home_state;status.home_message=manual_home_message;
    status.continuous_operation=continuous_operation;status.last_completed_key=to_ros(last_completed_key);status.last_outcome=static_cast<std::uint8_t>(last_outcome);
    if(bin_height_estimate && valid_bin_height_stamp.count() &&
      bin_height_key.epoch==task.context.key.epoch &&
      bin_height_key.cycle_id==task.context.key.cycle_id &&
      bin_height_key.track_id==task.context.key.track_id &&
      source_is_fresh(valid_bin_height_stamp,time,3.,config().time.sensor_future_tolerance_sec)){
      status.bin_surface_stamp=ptc_app::to_ros_time(valid_bin_height_stamp);
      status.bin_surface_z_m=bin_height_estimate->surface_z_m;
    }
    update_planned_grasp_preview(status, time);
    status.binding_revision=settings.binding_revision;status.reason=workflow_reason();task_pub->publish(status);
  }catch(const std::exception & e){reason=e.what();if(task.phase!=TaskPhase::kWaitingReady && task.phase!=TaskPhase::kFaulted){fault(reason);}}
}
void TaskCoordinatorNode::Runtime::try_select_observation_target(const TrackedObjectState & track)
{
  const auto time=now();
  auto selection=event(TaskEventKind::kGraspTrialRequested);selection.track=track;
  if(!system_ready(selection,*settings.snapshot,time,task.phase))return;
  record_candidate_admission_attempt(track,time);
  {
    const auto current=predict_object_pose(track,*settings.snapshot,now(),0);
    if(!current)return;
    auto roi_point=current->position;
    if(config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model" &&
      track.observation.envelope_valid){
      // Registration places the model origin near its base. Test the body's
      // height for admission, while preserving the measured pose for pickup.
      roi_point.z_m=transform_point(*current,track.observation.envelope_center_object_m).z_m;
    }
    if(!is_in_target_selection_roi(roi_point,config().task))return;
    admission_attempts.erase(candidate_key(track));deferred_tracks.erase(candidate_key(track));
    entry_follow_started_source=now().source_now;
    apply(std::move(selection));
  }
}
TaskCoordinatorNode::TaskCoordinatorNode(const rclcpp::NodeOptions & options):Node("task_coordinator",options),runtime_(std::make_unique<Runtime>(*this)){}
TaskCoordinatorNode::~TaskCoordinatorNode()=default;
}
