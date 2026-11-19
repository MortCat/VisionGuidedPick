#include "ptc_motion/arm_gateway_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>

namespace ptc_motion
{
using namespace ptc_core;
using ptc_app::from_ros;
using ptc_app::to_ros;
namespace
{
double seconds(const builtin_interfaces::msg::Duration & d)
{
  if (d.sec<0 || d.nanosec>=1000000000) {throw std::invalid_argument("invalid request duration");}
  return d.sec+d.nanosec*1e-9;
}
bool valid_grasp_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const auto point_count=static_cast<std::uint64_t>(cloud.width)*cloud.height;
  if(!point_count||point_count>60000||cloud.point_step<12||cloud.data.empty()||cloud.is_bigendian){return false;}
  const auto minimum_row=static_cast<std::uint64_t>(cloud.width)*cloud.point_step;
  const auto expected_bytes=static_cast<std::uint64_t>(cloud.row_step)*cloud.height;
  if(cloud.row_step<minimum_row||expected_bytes!=cloud.data.size()){return false;}
  for(const auto * name:{"x","y","z"}){
    const auto field=std::find_if(cloud.fields.begin(),cloud.fields.end(),[&](const auto & item){return item.name==name;});
    if(field==cloud.fields.end()||field->datatype!=sensor_msgs::msg::PointField::FLOAT32||field->count!=1||
      static_cast<std::uint64_t>(field->offset)+sizeof(float)>cloud.point_step){return false;}
  }
  return true;
}
}
ArmGatewayNode::ArmGatewayNode(const rclcpp::NodeOptions & options)
: Node("arm_gateway",options),settings_(ptc_app::load_node_settings(*this)),
  completions_(settings_.snapshot->get().diagnostics.event_queue_capacity),worker_(4),tf_(get_clock()),listener_(tf_)
{
  const auto & ep=settings_.endpoints;
  arm_publisher_=create_publisher<ptc_interfaces::msg::ArmStatus>(ep.at("arm_status"),rclcpp::QoS(5));
  gripper_publisher_=create_publisher<ptc_interfaces::msg::GripperStatus>(ep.at("gripper_status"),rclcpp::QoS(5));
  move_server_=rclcpp_action::create_server<Move>(this,ep.at("arm_move"),
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Move::Goal> goal) {
      try {
        if (move_goal_ || follow_goal_ || check_busy_) {return rclcpp_action::GoalResponse::REJECT;}
        auto e=event(MotionEventKind::kMoveRequested); const auto & c=settings_.snapshot->get();
        e.key.operation_id=next_operation_++;e.key.track_id=goal->target_track_id;
        if(goal->target_track_id && (seconds(goal->prediction_horizon)<=0 ||
          std::none_of(scene_.dynamic_objects.begin(),scene_.dynamic_objects.end(),[&](const auto & object){return object.key.track_id==goal->target_track_id;}))){
          RCLCPP_WARN(get_logger(),"M1 moving target identity is not in the acknowledged scene: track=%lu",goal->target_track_id);return rclcpp_action::GoalResponse::REJECT;}
        e.request.target_pose=transform(goal->target_pose,c.frames.robot_base);
        e.request.max_velocity_scale=goal->max_velocity_scale; e.request.max_acceleration_scale=goal->max_acceleration_scale;
        e.request.prediction_horizon=std::chrono::duration_cast<Nanoseconds>(std::chrono::duration<double>(seconds(goal->prediction_horizon)));
        const auto & v=goal->target_linear_velocity;
        Pose3d rotation;
        if(goal->target_pose.header.frame_id!=c.frames.robot_base){
          const auto tf=tf_.lookupTransform(c.frames.robot_base,goal->target_pose.header.frame_id,
            rclcpp::Time(goal->target_pose.header.stamp),rclcpp::Duration::from_seconds(c.vision.tf_lookup_timeout_sec));
          rotation.orientation={tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w};
        }
        e.request.target_linear_velocity=transform_point(rotation,{v.x,v.y,v.z});
        const auto timeout=seconds_to_duration(seconds(goal->timeout));if(!timeout){return rclcpp_action::GoalResponse::REJECT;}e.request.timeout=*timeout;
        e.base_in_world=transform(ptc_app::stamped_pose(Pose3d{},c.frames.robot_base,e.arm.tcp_timestamp),c.frames.world);
        auto d=advance_motion(motion_,e,*settings_.snapshot,time_.sample(*this,c)); motion_=d.state;
        if (!d.request_accepted) {RCLCPP_WARN(get_logger(),"M1 rejected: %s",d.status.reason.c_str());return rclcpp_action::GoalResponse::REJECT;}
        accepted_commands_=std::move(d.commands); return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } catch (const std::exception & e) {RCLCPP_WARN(get_logger(),"M1 rejected: %s",e.what()); return rclcpp_action::GoalResponse::REJECT;}
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>) {stop(); return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> goal) {move_goal_=goal; execute_commands(accepted_commands_); accepted_commands_.clear();});
  follow_server_=rclcpp_action::create_server<Follow>(this,ep.at("arm_follow"),
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Follow::Goal> goal) {
      try {
        if (move_goal_ || follow_goal_ || check_busy_ || seconds(goal->timeout)<=0 || goal->purpose>Follow::Goal::GRASP) {return rclcpp_action::GoalResponse::REJECT;}
        auto e=event(MotionEventKind::kFollowRequested); e.key=from_ros(goal->key); e.key.sequence=next_sequence_++; e.follow_timeout=*seconds_to_duration(seconds(goal->timeout));
        const auto target=transform(goal->target,settings_.snapshot->get().frames.world); e.target_world=target.pose; e.target_timestamp=target.timestamp;
        if (goal->target.header.frame_id!=settings_.snapshot->get().frames.world) {return rclcpp_action::GoalResponse::REJECT;}
        e.linear_velocity={goal->velocity.linear.x,goal->velocity.linear.y,goal->velocity.linear.z};
        e.angular_velocity={goal->velocity.angular.x,goal->velocity.angular.y,goal->velocity.angular.z};
        auto d=advance_motion(motion_,e,*settings_.snapshot,time_.sample(*this,settings_.snapshot->get())); motion_=d.state;
        if (!d.request_accepted) {RCLCPP_WARN(get_logger(),"follow rejected by state: %s",d.status.reason.c_str());return rclcpp_action::GoalResponse::REJECT;}
        motion_.operation_deadline=std::min(motion_.operation_deadline,*deadline_after(motion_.context.last_monotonic,seconds(goal->timeout)));
        follow_command_id_=goal->command_id; last_target_sequence_=goal->key.sequence;follow_grasp_=goal->purpose==Follow::Goal::GRASP;
        accepted_commands_=std::move(d.commands); return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } catch (const std::exception & e) {RCLCPP_WARN(get_logger(),"follow rejected: %s",e.what()); return rclcpp_action::GoalResponse::REJECT;}
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>>) {stop(); return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> goal) {follow_goal_=goal; execute_commands(accepted_commands_); accepted_commands_.clear();});
  target_subscription_=create_subscription<ptc_interfaces::msg::TargetUpdate>(ep.at("arm_target"),rclcpp::QoS(1),
    [this](ptc_interfaces::msg::TargetUpdate::ConstSharedPtr message) {update(*message);});
  stop_service_=create_service<ptc_interfaces::srv::StopArm>(ep.at("arm_stop"),
    [this](const std::shared_ptr<ptc_interfaces::srv::StopArm::Request> request,std::shared_ptr<ptc_interfaces::srv::StopArm::Response> response) {
      try {
        const auto key=from_ros(request->key); const auto t=time_.sample(*this,settings_.snapshot->get());
        response->accepted=valid_event(key,t,settings_.snapshot->revision());
        if (response->accepted && request->reset_fault) {
          auto e=event(MotionEventKind::kResetRequested);e.key=key;e.key.sequence=next_sequence_++;
          if(motion_.phase==ArmMotionState::kFault){response->accepted=apply(e).status.disposition==Disposition::kApplied;}
          else{response->accepted=!motion_.active && arm_snapshot_ready(e.arm,*settings_.snapshot,t) && e.arm.stopped;}
          if(response->accepted && (move_goal_||follow_goal_)){MoveToPoseResult result;result.error_code=MoveToPoseError::kCanceled;result.message="manual reset; no command replay";complete_action(result);}
          if(response->accepted){completions_.clear_fault();}
        }else if (response->accepted) {stop(); const auto backend=backend_.lock(); if (backend) {backend->stop_gripper();}}
        response->message="request ACK only; wait for measured stopped status";
      } catch (const std::exception & e) {response->accepted=false; response->message=e.what();}
    });
  grip_server_=rclcpp_action::create_server<Grip>(this,ep.at("gripper_command"),
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Grip::Goal> goal) {
      try {
        const auto backend=backend_.lock(); const auto t=time_.sample(*this,settings_.snapshot->get());
        const auto & c=settings_.snapshot->get();
        if (!backend || gripper_busy_ || !valid_event(from_ros(goal->key),t,settings_.snapshot->revision()) ||
          !gripper_snapshot_ready(backend->gripper_snapshot(t),*settings_.snapshot,t) ||
          !std::isfinite(goal->width_m) || goal->width_m<0 || goal->width_m>c.grasp.open_width_m ||
          !std::isfinite(goal->max_effort_n) || goal->max_effort_n<0 || (goal->max_effort_n==0 && c.runtime.operation_mode!="assumed_ng_demo") || goal->max_effort_n>c.grasp.max_effort_n || seconds(goal->timeout)<=0) {
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
            "gripper goal rejected: backend=%d busy=%d valid_key=%d snapshot_ready=%d open=%d width=%.9f effort=%.9f timeout=%.3f",
            static_cast<bool>(backend),gripper_busy_,valid_event(from_ros(goal->key),t,settings_.snapshot->revision()),
            backend && gripper_snapshot_ready(backend->gripper_snapshot(t),*settings_.snapshot,t),
            goal->open,goal->width_m,goal->max_effort_n,seconds(goal->timeout));
          return rclcpp_action::GoalResponse::REJECT;
        }
        gripper_busy_=true; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } catch (...) {return rclcpp_action::GoalResponse::REJECT;}
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>>) {const auto backend=backend_.lock(); if (backend) {backend->stop_gripper();} return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Grip>> goal) {
      const auto issued=cancellation_generation_.load();const auto work=[this,goal,issued] {
        auto result=std::make_shared<Grip::Result>(); result->key=goal->get_goal()->key; result->command_id=goal->get_goal()->command_id;
        try {const auto backend=backend_.lock(); result->success=backend && issued==cancellation_generation_.load() && !goal->is_canceling() && backend->gripper(goal->get_goal()->width_m,goal->get_goal()->max_effort_n,goal->get_goal()->open,seconds(goal->get_goal()->timeout));}
        catch (const std::exception & e) {result->message=e.what();}
        completions_.post([this,goal,result] {gripper_busy_=false; if (goal->is_canceling()) {goal->canceled(result);} else if (result->success) {goal->succeed(result);} else {goal->abort(result);}});
      };
      if (!gripper_worker_.enqueue(work)) {gripper_busy_=false; auto result=std::make_shared<Grip::Result>(); result->key=goal->get_goal()->key; result->command_id=goal->get_goal()->command_id; result->message="gripper worker busy"; goal->abort(result);}
    });
  scene_server_=rclcpp_action::create_server<Sync>(this,ep.at("scene_sync"),
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Sync::Goal> goal) {
      try {
        const auto t=time_.sample(*this,settings_.snapshot->get());
        if (scene_busy_ || !backend_.lock() || !valid_event(from_ros(goal->key),t,settings_.snapshot->revision()) ||
          goal->scene_revision<=scene_.revision || goal->static_geometry.empty() || goal->operation>Sync::Goal::DETACH ||
          check_busy_ || (!goal->object_clouds.empty() && goal->object_clouds.size()!=goal->objects.size()) || goal->grasp_surfaces.size()>1) {
          RCLCPP_WARN(get_logger(),"scene goal rejected: busy=%d backend=%d valid_key=%d revision=%lu current=%lu static=%zu operation=%u",
            scene_busy_,static_cast<bool>(backend_.lock()),valid_event(from_ros(goal->key),t,settings_.snapshot->revision()),
            goal->scene_revision,scene_.revision,goal->static_geometry.size(),goal->operation);
          return rclcpp_action::GoalResponse::REJECT;}

        for (const auto & geometry : goal->static_geometry) {
          if (geometry.id.empty() || geometry.reference_frame!=settings_.snapshot->get().frames.world || geometry.kind>geometry.MESH ||
            geometry.dimensions_m.x<=0 || geometry.dimensions_m.y<=0 || geometry.dimensions_m.z<=0) {return rclcpp_action::GoalResponse::REJECT;}
          from_ros(geometry.pose);
        }
        for (const auto & object : goal->objects) {
          const auto track=from_ros(object);
          const auto & cfg=settings_.snapshot->get();
          // Attachment may conservatively place an old measured cloud at the
          // current TCP during recovery. Keep its original timestamp; it is
          // collision geometry, not a new visual observation.
          const bool attaching_estimate=goal->operation==Sync::Goal::ATTACH && goal->attached_track_id &&
            track.context.key.track_id==goal->attached_track_id && cfg.runtime.operation_mode=="assumed_ng_demo" &&
            cfg.grasp.moving_intercept && track.observation.timestamp<=t.source_now;
          if (track.observation.reference_frame!=cfg.frames.world ||
            (track.context.key.track_id!=scene_.attached_track_id && !attaching_estimate && !source_is_fresh(track.observation.timestamp,t,
              settings_.snapshot->get().runtime.operation_mode=="assumed_ng_demo" &&
                settings_.snapshot->get().grasp.moving_intercept &&
                goal->key.track_id && track.context.key.track_id==goal->key.track_id?
                std::max(settings_.snapshot->get().scene.dynamic_object_max_age_sec,settings_.snapshot->get().task.cycle_timeout_sec):settings_.snapshot->get().scene.dynamic_object_max_age_sec,
              settings_.snapshot->get().time.sensor_future_tolerance_sec))) {
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        for(const auto & surface:goal->grasp_surfaces){
          const auto key=from_ros(surface.key);const auto track=std::find_if(goal->objects.begin(),goal->objects.end(),[&](const auto & item){return item.key.track_id==key.track_id;});
          const auto point_count=static_cast<std::uint64_t>(surface.points.width)*surface.points.height;
          if(surface.cloud_revision==0||track==goal->objects.end()||!same_track_identity(key,from_ros(*track).context.key)||
            !same_cycle(key,from_ros(goal->key))||surface.points.header.frame_id!="object_"+std::to_string(key.track_id)||
            point_count==0||!valid_grasp_cloud(surface.points)){
            RCLCPP_WARN(get_logger(),"scene grasp surface rejected: track=%llu revision=%llu points=%llu frame=%s layout_valid=%d",
              static_cast<unsigned long long>(key.track_id),static_cast<unsigned long long>(surface.cloud_revision),
              static_cast<unsigned long long>(point_count),surface.points.header.frame_id.c_str(),valid_grasp_cloud(surface.points));
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        scene_busy_=true; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } catch (...) {return rclcpp_action::GoalResponse::REJECT;}
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Sync>>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Sync>> goal) {
      const auto work=[this,goal] {
        auto result=std::make_shared<Sync::Result>(); const auto request=goal->get_goal(); result->key=request->key;
        result->command_id=request->command_id; result->scene_revision=request->scene_revision;
        try {const auto backend=backend_.lock(); result->success=backend && backend->sync_scene(*request);} catch (const std::exception & e) {result->message=e.what();}
        completions_.post([this,goal,result,request] {
          scene_busy_=false;
          if (result->success && !goal->is_canceling()) {
            scene_.context.key=from_ros(request->key); scene_.revision=request->scene_revision; scene_.coherent=true;
            scene_.static_geometry.clear(); scene_.dynamic_objects.clear();
            for (const auto & g : request->static_geometry) {scene_.static_geometry.push_back({g.id,g.reference_frame,static_cast<GeometryKind>(g.kind),from_ros(g.pose),{g.dimensions_m.x,g.dimensions_m.y,g.dimensions_m.z},g.asset_digest});}
            for (const auto & object : request->objects) {auto track=from_ros(object); scene_.dynamic_objects.push_back({track.context.key,track.observation,{}});}
            if(!request->grasp_surfaces.empty()){
              scene_grasp_surface_key_=from_ros(request->grasp_surfaces.front().key);
              scene_grasp_surface_revision_=request->grasp_surfaces.front().cloud_revision;
            }
            scene_.attachment=request->operation==Sync::Goal::ATTACH ? AttachmentPhase::kAttached :
              request->operation==Sync::Goal::DETACH ? AttachmentPhase::kDetached : scene_.attachment;
            scene_.attached_track_id=scene_.attachment==AttachmentPhase::kAttached ? request->attached_track_id : 0;
            goal->succeed(result);
          } else {scene_.coherent=false; if (goal->is_canceling()) {goal->canceled(result);} else {goal->abort(result);} stop();}
        });
      };
      if (!scene_worker_.enqueue(work)) {scene_busy_=false; auto result=std::make_shared<Sync::Result>(); result->key=goal->get_goal()->key; result->command_id=goal->get_goal()->command_id; result->message="worker busy"; goal->abort(result);}
    });
  check_server_=rclcpp_action::create_server<Check>(this,ep.at("motion_check"),
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Check::Goal> goal){
      try{const auto backend=backend_.lock();const auto t=time_.sample(*this,settings_.snapshot->get());
        const auto & c=settings_.snapshot->get();
        const bool checking_follow=follow_goal_ && follow_grasp_ && goal->purpose==goal->GRASP &&
          c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept &&
          motion_.phase==ArmMotionState::kFollowing && same_track_identity(from_ros(goal->key),motion_.context.key);
        // Endpoint feasibility runs on the worker while the existing
        // Servo owner keeps receiving target updates; it does not issue motion.
        const bool matching_scene=goal->scene_revision==scene_.revision;
        const bool matching_surface=goal->purpose!=goal->GRASP || (goal->surface_revision!=0&&scene_grasp_surface_revision_==goal->surface_revision&&same_cycle(scene_grasp_surface_key_,from_ros(goal->key)));
        const bool valid_grasp_geometry=goal->purpose!=goal->GRASP ||
          (goal->targets.size()==2&&goal->contact_patch_centers.size()==2&&goal->contact_patch_radii_m.size()==2&&
          std::isfinite(goal->closure_width_m)&&goal->closure_width_m>=c.grasp.close_width_m&&goal->closure_width_m<=c.grasp.open_width_m&&
          std::all_of(goal->contact_patch_radii_m.begin(),goal->contact_patch_radii_m.end(),[](double radius){return std::isfinite(radius)&&radius>0;}));
        if(check_busy_ || scene_busy_ || move_goal_ || (follow_goal_ && !checking_follow) || !backend || !backend->check_motion || goal->purpose>goal->GRASP || !valid_event(from_ros(goal->key),t,settings_.snapshot->revision()) || goal->targets.empty() || !seconds_to_duration(seconds(goal->timeout)) || !valid_grasp_geometry ||
          !matching_scene || !matching_surface ||
          (!backend->arm_snapshot(t).stopped && !checking_follow) || !scene_ready(scene_,*settings_.snapshot,t,settings_.snapshot->get().runtime.operation_mode=="assumed_ng_demo"?goal->key.track_id:0)){return rclcpp_action::GoalResponse::REJECT;}
        for(const auto & pose:goal->targets){transform(pose,settings_.snapshot->get().frames.robot_base);}
        if(goal->purpose==goal->GRASP){for(const auto & center:goal->contact_patch_centers){transform(center,settings_.snapshot->get().frames.world);}}
        check_busy_=true;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }catch(...){return rclcpp_action::GoalResponse::REJECT;}},
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Check>>){return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Check>> handle){
      check_goal_=handle;check_deadline=*deadline_after(time_.sample(*this,settings_.snapshot->get()).monotonic_now,seconds(handle->get_goal()->timeout));
      MotionCheckRequest request;const auto goal=handle->get_goal();request.key=from_ros(goal->key);request.track_id=goal->key.track_id;request.surface_revision=goal->surface_revision;
      request.timeout_sec=seconds(goal->timeout);request.grasp=goal->purpose==Check::Goal::GRASP;request.closure_width_m=goal->closure_width_m;
      for(const auto & pose:goal->targets){request.targets.push_back(transform(pose,settings_.snapshot->get().frames.robot_base));}
      if(request.grasp){for(std::size_t i=0;i<goal->contact_patch_centers.size();++i){request.contact_patches.push_back({
          transform(goal->contact_patch_centers[i],settings_.snapshot->get().frames.world),goal->contact_patch_radii_m[i]});}}
      const auto backend=backend_.lock();const auto issued=cancellation_generation_.load();
      if(!worker_.enqueue([this,handle,request,backend,issued]{auto result=std::make_shared<Check::Result>();result->key=handle->get_goal()->key;result->command_id=handle->get_goal()->command_id;
        try{result->success=backend&&issued==cancellation_generation_.load()&&!handle->is_canceling()&&backend->check_motion(request);if(!result->success&&result->message.empty()){result->message="motion or contact check failed";}}catch(const std::exception & e){result->message=e.what();}
        completions_.post([this,handle,result]{check_busy_=false;if(check_goal_!=handle){return;}if(handle->is_canceling()){handle->canceled(result);}else if(result->success){handle->succeed(result);}else{handle->abort(result);}check_goal_.reset();});
      })){check_busy_=false;auto result=std::make_shared<Check::Result>();result->key=handle->get_goal()->key;result->command_id=handle->get_goal()->command_id;result->message="worker queue full";handle->abort(result);check_goal_.reset();}
    });
  evaluate_server_=rclcpp_action::create_server<Evaluate>(this,ep.at("grasp_evaluate"),
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Evaluate::Goal> goal){
      try{
        const auto backend=backend_.lock();const auto t=time_.sample(*this,settings_.snapshot->get());const auto & c=settings_.snapshot->get();
        const bool checking_follow=follow_goal_&&follow_grasp_&&c.runtime.operation_mode=="assumed_ng_demo"&&
          c.grasp.moving_intercept&&motion_.phase==ArmMotionState::kFollowing&&
          same_track_identity(from_ros(goal->key),motion_.context.key);
        const auto timeout=seconds(goal->timeout);
        const bool arm_stopped=backend&&backend->arm_snapshot(t).stopped;
        const bool usable_scene=scene_ready(scene_,*settings_.snapshot,t,c.runtime.operation_mode=="assumed_ng_demo"?goal->key.track_id:0);
        if(check_busy_||scene_busy_||move_goal_||(follow_goal_&&!checking_follow)||!backend||
          !backend->evaluate_grasp_candidates||goal->candidates.empty()||goal->candidates.size()>16||timeout<=0||
          !seconds_to_duration(timeout)||goal->scene_revision!=scene_.revision||goal->surface_revision==0||
          scene_grasp_surface_revision_!=goal->surface_revision||!same_cycle(scene_grasp_surface_key_,from_ros(goal->key))||
          !valid_event(from_ros(goal->key),t,settings_.snapshot->revision())||
          (!arm_stopped&&!checking_follow)||!usable_scene){
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
            "grasp batch admission: check_busy=%d scene_busy=%d move=%d follow=%d stopped=%d scene_ready=%d scene_revision=%llu/%llu surface_revision=%llu/%llu same_cycle=%d event_valid=%d",
            check_busy_,scene_busy_,static_cast<bool>(move_goal_),static_cast<bool>(follow_goal_),arm_stopped,usable_scene,
            static_cast<unsigned long long>(goal->scene_revision),static_cast<unsigned long long>(scene_.revision),
            static_cast<unsigned long long>(goal->surface_revision),static_cast<unsigned long long>(scene_grasp_surface_revision_),
            same_cycle(scene_grasp_surface_key_,from_ros(goal->key)),valid_event(from_ros(goal->key),t,settings_.snapshot->revision()));
          return rclcpp_action::GoalResponse::REJECT;
        }
        std::set<std::uint32_t> ids;
        for(const auto & candidate:goal->candidates){
          if(!candidate.candidate_id||!ids.insert(candidate.candidate_id).second||
            candidate.contact_patch_centers.size()!=2||candidate.contact_patch_radii_m.size()!=2||
            !std::isfinite(candidate.closure_width_m)||candidate.closure_width_m<c.grasp.close_width_m||
            candidate.closure_width_m>c.grasp.open_width_m||
            !std::all_of(candidate.contact_patch_radii_m.begin(),candidate.contact_patch_radii_m.end(),
              [](double radius){return std::isfinite(radius)&&radius>0;})){
            return rclcpp_action::GoalResponse::REJECT;
          }
          transform(candidate.pregrasp,c.frames.robot_base);transform(candidate.contact,c.frames.robot_base);
          for(const auto & center:candidate.contact_patch_centers){transform(center,c.frames.world);}
        }
        check_busy_=true;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }catch(const std::exception & ex){RCLCPP_WARN(get_logger(),"grasp candidate batch rejected: %s",ex.what());return rclcpp_action::GoalResponse::REJECT;}
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Evaluate>>){return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Evaluate>> handle){
      evaluate_goal_=handle;const auto goal=handle->get_goal();
      check_deadline=*deadline_after(time_.sample(*this,settings_.snapshot->get()).monotonic_now,seconds(goal->timeout));
      GraspCandidatesRequest request;request.key=from_ros(goal->key);request.scene_revision=goal->scene_revision;
      request.surface_revision=goal->surface_revision;request.timeout_sec=seconds(goal->timeout);
      request.neighbor_capture_ms=goal->neighbor_capture_ms;
      for(const auto & point:goal->neighbor_points_world)
        if(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z))
          request.neighbor_points_world.push_back({point.x,point.y,point.z});
      for(const auto & candidate:goal->candidates){
        GraspCandidateCheck item;item.candidate_id=candidate.candidate_id;item.motion.key=request.key;
        item.motion.track_id=request.key.track_id;item.motion.surface_revision=request.surface_revision;
        item.motion.timeout_sec=request.timeout_sec;item.motion.grasp=true;item.motion.closure_width_m=candidate.closure_width_m;
        item.motion.targets.push_back(transform(candidate.pregrasp,settings_.snapshot->get().frames.robot_base));
        item.motion.targets.push_back(transform(candidate.contact,settings_.snapshot->get().frames.robot_base));
        for(std::size_t i=0;i<candidate.contact_patch_centers.size();++i){item.motion.contact_patches.push_back({
          transform(candidate.contact_patch_centers[i],settings_.snapshot->get().frames.world),candidate.contact_patch_radii_m[i]});}
        request.candidates.push_back(std::move(item));
      }
      const auto backend=backend_.lock();
      if(!worker_.enqueue([this,handle,goal,request,backend]{
        auto result=std::make_shared<Evaluate::Result>();result->key=goal->key;result->command_id=goal->command_id;
        result->scene_revision=goal->scene_revision;result->surface_revision=goal->surface_revision;
        try{
          if(backend&&backend->evaluate_grasp_candidates){
            const auto checked=backend->evaluate_grasp_candidates(request);result->measured_joint_stamp=ptc_app::to_ros_time(checked.measured_joint_stamp);
            for(const auto & item:checked.candidates){ptc_interfaces::msg::GraspFeasibility feasibility;
              feasibility.candidate_id=item.candidate_id;feasibility.evaluated=item.evaluated;
              feasibility.feasible=item.feasible;feasibility.failure_reason=item.failure_reason;
              feasibility.minimum_joint_margin_rad=item.minimum_joint_margin_rad;feasibility.pregrasp_time_lower_bound_sec=item.pregrasp_duration_sec;
              result->results.push_back(std::move(feasibility));}
            std::set<std::uint32_t> returned_ids;
            result->complete=result->results.size()==request.candidates.size()&&
              std::all_of(result->results.begin(),result->results.end(),[&](const auto & item){
                return (!item.feasible||item.evaluated)&&returned_ids.insert(item.candidate_id).second&&
                  std::any_of(request.candidates.begin(),request.candidates.end(),[&](const auto & candidate){
                    return candidate.candidate_id==item.candidate_id;});
              }) && (std::any_of(result->results.begin(),result->results.end(),[](const auto & item){return item.evaluated&&item.feasible;}) ||
                std::all_of(result->results.begin(),result->results.end(),[](const auto & item){return item.evaluated;}));
            result->message=result->complete?"candidate selection completed; see per-candidate evaluated status":"batch returned partial candidate results";
          }else{result->message="grasp candidate evaluator unavailable";}
        }catch(const std::exception & ex){result->message=ex.what();}
        completions_.post([this,handle,result]{
          check_busy_=false;if(evaluate_goal_!=handle){return;}evaluate_goal_.reset();
          if(handle->is_canceling()){handle->canceled(result);}else if(result->complete){handle->succeed(result);}else{handle->abort(result);}
        });
      })){
        check_busy_=false;auto result=std::make_shared<Evaluate::Result>();result->key=goal->key;result->command_id=goal->command_id;
        result->scene_revision=goal->scene_revision;result->surface_revision=goal->surface_revision;result->message="candidate evaluation worker queue full";
        handle->abort(result);evaluate_goal_.reset();
      }
    });
  timer_=create_wall_timer(std::chrono::milliseconds(20),[this] {tick();});
}
ArmGatewayNode::~ArmGatewayNode() {stop_runtime();}
void ArmGatewayNode::stop_runtime()
{
  if (stopped_runtime_) {return;} stopped_runtime_=true;
  if (timer_) {timer_->cancel();} const auto backend=backend_.lock();
  if (backend) {backend->stop_arm(); backend->stop_gripper();} worker_.shutdown();gripper_worker_.shutdown();scene_worker_.shutdown();
}
StampedPose3d ArmGatewayNode::transform(const geometry_msgs::msg::PoseStamped & input,const std::string & frame)
{
  const auto transformed=tf_.transform(input,frame,tf2::durationFromSec(settings_.snapshot->get().vision.tf_lookup_timeout_sec));
  return {from_ros(transformed.header.stamp),transformed.header.frame_id,from_ros(transformed.pose)};
}
MotionEvent ArmGatewayNode::event(MotionEventKind kind)
{
  MotionEvent out; out.kind=kind; const auto t=time_.sample(*this,settings_.snapshot->get());
  out.key=motion_.context.key; if (!out.key.operation_id) {out.key.operation_id=next_operation_++;}
  out.key.epoch=t.epoch; out.key.settings_revision=settings_.snapshot->revision(); out.key.sequence=next_sequence_++;
  const auto backend=backend_.lock(); if (backend) {out.arm=backend->arm_snapshot(t);} out.scene=scene_; return out;
}
MotionDecision ArmGatewayNode::apply(MotionEvent e)
{
  const auto t=time_.sample(*this,settings_.snapshot->get());
  if(e.kind==MotionEventKind::kPlanSucceeded && (!scene_ready(e.scene,*settings_.snapshot,t)||!arm_snapshot_ready(e.arm,*settings_.snapshot,t)||!e.arm.stopped)){
    double velocity=0,scene_age=0;for(const auto & joint:e.arm.state.joints){if(joint.velocity){velocity=std::max(velocity,std::abs(*joint.velocity));}}
    for(const auto & object:e.scene.dynamic_objects){scene_age=std::max(scene_age,std::chrono::duration<double>(t.source_now-object.observation.timestamp).count());}
    RCLCPP_WARN(get_logger(),"validate_planned_motion_readiness: scene_ready=%d arm_ready=%d stopped=%d tf_valid=%d controller_ready=%d joint_age=%.6f tcp_age=%.6f max_joint_velocity=%.6f scene_age=%.6f coherent=%d",
      scene_ready(e.scene,*settings_.snapshot,t),arm_snapshot_ready(e.arm,*settings_.snapshot,t),e.arm.stopped,e.arm.tf_valid,e.arm.controller_ready,
      std::chrono::duration<double>(t.source_now-e.arm.joint_timestamp).count(),std::chrono::duration<double>(t.source_now-e.arm.tcp_timestamp).count(),velocity,scene_age,e.scene.coherent);
  }
  auto d=advance_motion(motion_,e,*settings_.snapshot,t);
  if(d.state.phase!=motion_.phase && (d.state.phase==ArmMotionState::kStopping||d.state.phase==ArmMotionState::kFault)){
    RCLCPP_WARN(get_logger(),"motion stopping: %s; cause=%d event=%d",d.status.reason.c_str(),static_cast<int>(d.state.pending_result),static_cast<int>(e.kind));
    if(d.state.pending_result==MoveToPoseError::kNotReady){
      double oldest_age=0.;std::uint64_t oldest_track=0;
      for(const auto & object:e.scene.dynamic_objects){const double age=std::chrono::duration<double>(t.source_now-object.observation.timestamp).count();if(age>oldest_age){oldest_age=age;oldest_track=object.key.track_id;}}
      RCLCPP_WARN(get_logger(),"motion readiness loss: arm_ready=%d tf_valid=%d controller_ready=%d scene_ready=%d coherent=%d inhibited=%d scene_revision=%lu oldest_track=%lu oldest_age_sec=%.6f joint_age_sec=%.6f",
        arm_snapshot_ready(e.arm,*settings_.snapshot,t),e.arm.tf_valid,e.arm.controller_ready,scene_ready(e.scene,*settings_.snapshot,t),e.scene.coherent,e.scene.context.inhibited,e.scene.revision,oldest_track,oldest_age,std::chrono::duration<double>(t.source_now-e.arm.joint_timestamp).count());
    }
  }
  motion_=d.state;
  execute_commands(d.commands); if (d.result) {complete_action(*d.result);} return d;
}
void ArmGatewayNode::worker_event(EventKey key,MotionEventKind kind)
{
  completions_.post([this,key,kind] {
    if (!same_operation(key,motion_.context.key)) {return;}
    const auto decision=apply(event(kind));
    if(kind==MotionEventKind::kExecutionSucceeded && decision.status.disposition==Disposition::kRejected && motion_.phase==ArmMotionState::kMoving){
      // Keep a completed worker result until fresh stopped feedback accepts it.
      // The existing operation deadline/cancel still wins; no motion is replayed.
      pending_execution_completion_=key;
      RCLCPP_INFO(get_logger(),"defer_execution_completion: %s; awaiting next measured snapshot within original deadline",decision.status.reason.c_str());
    }
  });
}
void ArmGatewayNode::execute_commands(const std::vector<Command> & commands)
{
  for (const auto & command : commands) {
    const auto backend=backend_.lock(); if (!backend) {return;}
    if (command.kind==CommandKind::kStopArm) {++cancellation_generation_;backend->stop_arm();}
    else if (command.kind==CommandKind::kStartFollow || command.kind==CommandKind::kUpdateTarget) {
      auto target=command;target.producer=follow_grasp_?"grasp":"inspection";
      if (!backend->follow(target)) {apply(event(MotionEventKind::kDeviceUnavailable));}
    } else if (command.kind==CommandKind::kPlan || command.kind==CommandKind::kExecuteTrajectory) {
      const auto request=motion_.request;
      const auto issued=cancellation_generation_.load();const auto work=[this,backend,command,request,issued] {
        if(issued!=cancellation_generation_.load()){return;}
        bool okay=false;
        try {okay=command.kind==CommandKind::kPlan?backend->plan(request):backend->execute();}
        catch (const std::exception & e) {RCLCPP_ERROR(get_logger(),"motion backend: %s",e.what());}
        if(issued!=cancellation_generation_.load()){return;}
        if(command.kind==CommandKind::kPlan && okay && request.prediction_horizon.count()>0){
          const auto endpoint=backend->planned_target ? backend->planned_target() : std::nullopt;
          completions_.post([this,key=command.key,endpoint,issued]{
            if(issued!=cancellation_generation_.load() || !same_operation(key,motion_.context.key)){return;}
            auto e=event(endpoint?MotionEventKind::kPlanSucceeded:MotionEventKind::kPlanFailed);
            if(endpoint){e.request.target_pose=*endpoint;}apply(e);
          });
        }else{worker_event(command.key,command.kind==CommandKind::kPlan ? (okay ? MotionEventKind::kPlanSucceeded : MotionEventKind::kPlanFailed) :
          (okay ? MotionEventKind::kExecutionSucceeded : MotionEventKind::kExecutionFailed));}
      };
      if (!worker_.enqueue(work)) {apply(event(MotionEventKind::kDeviceUnavailable));}
    }
  }
}
void ArmGatewayNode::stop() {pending_execution_completion_.reset();++cancellation_generation_;const auto backend=backend_.lock(); if (backend) {backend->stop_arm();} if (motion_.active) {apply(event(MotionEventKind::kCancelRequested));}}
void ArmGatewayNode::complete_action(const MoveToPoseResult & result)
{
  pending_execution_completion_.reset();
  if (move_goal_) {
    auto wire=std::make_shared<Move::Result>(); wire->success=result.success; wire->error_code=static_cast<std::int32_t>(result.error_code); wire->message=result.message;
    if (!result.final_pose.reference_frame.empty()) {wire->final_pose=ptc_app::stamped_pose(result.final_pose.pose,result.final_pose.reference_frame,result.final_pose.timestamp);}
    if (move_goal_->is_canceling()) {move_goal_->canceled(wire);} else if (result.success) {move_goal_->succeed(wire);} else {move_goal_->abort(wire);} move_goal_.reset();
  }
  if (follow_goal_) {
    auto wire=std::make_shared<Follow::Result>(); wire->key=follow_goal_->get_goal()->key; wire->command_id=follow_command_id_;
    wire->success=result.error_code==MoveToPoseError::kCanceled; wire->message=result.message;
    if (follow_goal_->is_canceling()) {follow_goal_->canceled(wire);} else if (wire->success) {follow_goal_->succeed(wire);} else {follow_goal_->abort(wire);} follow_goal_.reset();
  }
}
void ArmGatewayNode::update(const ptc_interfaces::msg::TargetUpdate & update)
{
  try {
    const auto key=from_ros(update.key);
    if (!follow_goal_ || update.command_id!=follow_command_id_ || !same_operation(key,motion_.context.key) || update.key.sequence<=last_target_sequence_ ||
      update.target.header.frame_id!=settings_.snapshot->get().frames.world || seconds(update.valid_for)<=0) {return;}
    const auto time=time_.sample(*this,settings_.snapshot->get());
    const auto target_stamp=from_ros(update.target.header.stamp);
    // DDS topics are independent: a target may arrive before its /clock tick.
    // Keep only the newest bounded command; retain its original timestamp and
    // apply the usual freshness/future guards once the local clock catches up.
    if(!time.paused && target_stamp>time.source_now &&
       target_stamp-time.source_now<=*ptc_core::seconds_to_duration(settings_.snapshot->get().motion.target_update_timeout_sec)){
      if(!pending_target_update_ || update.key.sequence>pending_target_update_->key.sequence){pending_target_update_=update;}
      return;
    }
    auto e=event(MotionEventKind::kTargetUpdated); e.target_world=from_ros(update.target.pose); e.target_timestamp=from_ros(update.target.header.stamp);
    e.linear_velocity={update.velocity.linear.x,update.velocity.linear.y,update.velocity.linear.z};
    e.angular_velocity={update.velocity.angular.x,update.velocity.angular.y,update.velocity.angular.z};
    const auto decision=apply(e);
    if(decision.status.disposition==Disposition::kApplied && motion_.pending_result==ptc_core::MoveToPoseError::kInvalidGoal){
      RCLCPP_WARN(get_logger(),"invalid target update: age=%.6f linear=%.4f angular=%.4f workspace=%d",std::chrono::duration<double>(time.source_now-target_stamp).count(),ptc_core::vector_norm(e.linear_velocity),ptc_core::vector_norm(e.angular_velocity),ptc_core::inside_workspace(e.target_world,settings_.snapshot->get().motion));}
    if(decision.status.disposition!=Disposition::kApplied){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"target update rejected by state: %s",decision.status.reason.c_str());} if (decision.status.disposition==Disposition::kApplied && motion_.phase==ArmMotionState::kFollowing) {
      last_target_sequence_=update.key.sequence;
      motion_.update_deadline=std::min(motion_.update_deadline,*deadline_after(motion_.context.last_monotonic,seconds(update.valid_for)));
    }
  } catch (const std::exception & e) {RCLCPP_WARN(get_logger(),"target update rejected: %s",e.what());}
}
void ArmGatewayNode::tick()
{
  completions_.drain(); const auto backend=backend_.lock(); if (!backend) {return;}
  const auto & c=settings_.snapshot->get(); auto t=time_.sample(*this,c); const auto arm=backend->arm_snapshot(t);const auto grip=backend->gripper_snapshot(t);t=time_.sample(*this,c);
  if(pending_target_update_){
    if(t.paused || !follow_goal_ || pending_target_update_->key.epoch!=t.epoch){pending_target_update_.reset();}
    else if(from_ros(pending_target_update_->target.header.stamp)<=t.source_now){auto pending=std::move(*pending_target_update_);pending_target_update_.reset();update(pending);}
  }
  if(check_goal_&&(check_goal_->is_canceling()||t.monotonic_now>=check_deadline||t.paused)){
    auto result=std::make_shared<Check::Result>();result->key=check_goal_->get_goal()->key;result->command_id=check_goal_->get_goal()->command_id;result->message="feasibility canceled/expired/clock paused";
    if(check_goal_->is_canceling()){check_goal_->canceled(result);}else{check_goal_->abort(result);}check_goal_.reset();}
  if(evaluate_goal_&&(evaluate_goal_->is_canceling()||t.monotonic_now>=check_deadline||t.paused)){
    auto result=std::make_shared<Evaluate::Result>();const auto goal=evaluate_goal_->get_goal();result->key=goal->key;
    result->command_id=goal->command_id;result->scene_revision=goal->scene_revision;result->surface_revision=goal->surface_revision;
    result->message="candidate feasibility canceled/expired/clock paused";
    if(evaluate_goal_->is_canceling()){evaluate_goal_->canceled(result);}else{evaluate_goal_->abort(result);}evaluate_goal_.reset();}
  if(pending_execution_completion_ && (motion_.phase!=ArmMotionState::kMoving || !same_operation(*pending_execution_completion_,motion_.context.key))){pending_execution_completion_.reset();}
  if (completions_.overflowed()) {backend->stop_arm(); backend->stop_gripper(); motion_.context.inhibited=true; motion_.phase=ArmMotionState::kFault;}
  else if(pending_execution_completion_){
    const auto decision=apply(event(MotionEventKind::kExecutionSucceeded));
    if(decision.status.disposition!=Disposition::kRejected || motion_.phase!=ArmMotionState::kMoving){pending_execution_completion_.reset();}
  }
  else if (motion_.phase==ArmMotionState::kStopping && arm.stopped) {apply(event(MotionEventKind::kStopConfirmed));}
  else if (motion_.active) {apply(event(MotionEventKind::kTick));}
  else {apply(event(MotionEventKind::kDeviceReady));}
  ptc_interfaces::msg::ArmStatus wire; auto key=motion_.context.key; key.epoch=t.epoch; key.settings_revision=settings_.snapshot->revision();
  wire.key=to_ros(key); wire.header.stamp=ptc_app::to_ros_time(t.source_now); wire.header.frame_id=c.frames.robot_base;
  wire.tcp=ptc_app::stamped_pose(arm.state.tcp_pose,c.frames.robot_base,arm.tcp_timestamp);
  wire.joints.header.stamp=ptc_app::to_ros_time(arm.joint_timestamp);
  for (const auto & joint : arm.state.joints) {wire.joints.name.push_back(joint.name); wire.joints.position.push_back(joint.position);
    wire.joints.velocity.push_back(joint.velocity.value_or(0)); wire.joints.effort.push_back(joint.effort.value_or(0));}
  wire.message=arm.state.status.message.substr(0,512);wire.ready=arm_snapshot_ready(arm,*settings_.snapshot,t);
  if(!wire.ready){
    wire.message=(wire.message+" arm feedback: tf="+std::to_string(arm.tf_valid)+" controller="+std::to_string(arm.controller_ready)+
      " device="+std::to_string(arm.state.status.ready)+" stale="+std::to_string(arm.state.status.data_stale)+
      " joint_age_sec="+std::to_string(std::chrono::duration<double>(t.source_now-arm.joint_timestamp).count())+
      " tcp_age_sec="+std::to_string(std::chrono::duration<double>(t.source_now-arm.tcp_timestamp).count())).substr(0,512);
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"%s",wire.message.c_str());
  }
  const auto continuing_track=motion_.phase==ArmMotionState::kFollowing && follow_grasp_ &&
    c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept?motion_.context.key.track_id:0;
  wire.command_ready=wire.ready && scene_ready(scene_,*settings_.snapshot,t,continuing_track) && !motion_.context.inhibited;
  wire.phase=static_cast<std::uint8_t>(motion_.phase);
  wire.stopped=arm.stopped; wire.owner=motion_.phase==ArmMotionState::kFollowing ? wire.OWNER_SERVO :
    motion_.phase==ArmMotionState::kStopping ? wire.OWNER_STOPPING : motion_.active ? wire.OWNER_PLANNER : wire.OWNER_NONE;
  wire.command_id=follow_goal_ ? follow_command_id_ : motion_.context.key.operation_id; arm_publisher_->publish(wire);
  ptc_interfaces::msg::GripperStatus g; g.key=wire.key; g.header.stamp=ptc_app::to_ros_time(grip.timestamp);
  g.width_m=std::isfinite(grip.width_m) ? grip.width_m : 0; g.ready=gripper_snapshot_ready(grip,*settings_.snapshot,t); g.stopped=grip.stopped;
  g.empty_confirmed=grip.empty_confirmed; g.contact_confirmed=grip.contact_confirmed; g.visual_follow_confirmed=grip.visual_follow_confirmed;
  g.evidence_sequence=grip.evidence_sequence;
  if(!g.ready){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"gripper feedback not ready: backend=%d stale=%d width=%.6f age=%.6f epoch=%llu/%llu",grip.status.ready,grip.status.data_stale,g.width_m,std::chrono::duration<double>(t.source_now-grip.timestamp).count(),static_cast<unsigned long long>(grip.epoch),static_cast<unsigned long long>(t.epoch));}
  gripper_publisher_->publish(g);
  if (move_goal_) {auto feedback=std::make_shared<Move::Feedback>(); feedback->phase=static_cast<std::uint8_t>(m1_action_phase(motion_.phase)); feedback->current_pose=wire.tcp; feedback->progress=-1; move_goal_->publish_feedback(feedback);}
  if (follow_goal_) {auto feedback=std::make_shared<Follow::Feedback>(); feedback->key=wire.key; feedback->current_pose=wire.tcp; feedback->tracking=motion_.phase==ArmMotionState::kFollowing && arm.state.motion_state==ArmMotionState::kFollowing; follow_goal_->publish_feedback(feedback);}
}
}  // namespace ptc_motion
