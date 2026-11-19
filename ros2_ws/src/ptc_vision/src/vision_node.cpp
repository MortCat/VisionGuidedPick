#include <string_view>
#include "ptc_vision/vision_node.hpp"
#include "ptc_vision/cloud_registration_adapter.hpp"
#include "ptc_interfaces/msg/task_status.hpp"
#include "ptc_interfaces/msg/grasp_neighborhood.hpp"
#include "ptc_core/state/task_state.hpp"
#include "ptc_app/node_runtime.hpp"
#include "ptc_app/bounded_worker.hpp"
#include "ptc_app/rgbd_converter.hpp"
#include "ptc_app/ros_converter.hpp"
#include "ptc_app/reference_cloud_loader.hpp"
#include "ptc_app/robot_description_loader.hpp"
#include "ptc_core/vision/depth_workspace_model.hpp"
#include "ptc_interfaces/action/capture_grasp_surface.hpp"
#include "ptc_interfaces/msg/detection_array.hpp"
#include "ptc_interfaces/msg/tracked_object_array.hpp"
#include "ptc_interfaces/msg/capability_status.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
namespace ptc_vision
{
using namespace ptc_core;
using ptc_app::from_ros; using ptc_app::to_ros;
struct VisionNode::Runtime
{
  using Capture=ptc_interfaces::action::CaptureGraspSurface;
  using CaptureHandle=rclcpp_action::ServerGoalHandle<Capture>;
  struct ObjectFrame {std::uint64_t track_id; Pose3d pose; ImageRegion roi; std::vector<std::uint8_t> mask;};
  using Neighborhood=ptc_interfaces::msg::GraspNeighborhood;
  rclcpp::Publisher<Neighborhood>::SharedPtr neighborhood_pub;
  std::uint64_t neighborhood_cycle{0};
  Nanoseconds neighborhood_stamp{0};
  struct StoredFrame {std::shared_ptr<const RgbdImageFrame> image; std::vector<ObjectFrame> objects; bool observer{false};};
  VisionNode & node; ptc_app::SettingsLoadResult settings; ptc_app::RobotDescriptionModel robot;
  ptc_interfaces::msg::TaskStatus::ConstSharedPtr grasp_patch_task;Nanoseconds grasp_patch_task_seen{0};
  std::optional<CloudRegistrationAdapter> registration;
  ptc_app::TimeAdapter clock; ptc_app::CompletionQueue completions;
  ptc_app::BoundedWorker worker{2},image_worker{1}; tf2_ros::Buffer tf; tf2_ros::TransformListener listener;
  VisualTrackModel tracks; std::map<std::uint64_t,StoredFrame> frames;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> colors,depths;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr color_info,depth_info,observer_color_info,observer_depth_info;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> observer_colors,observer_depths;
  std::uint64_t last_wrist_frame{0};bool next_observer{false};
  ptc_interfaces::msg::DetectionArray::ConstSharedPtr pending_detection;
  struct PreparedDetection {ptc_interfaces::msg::DetectionArray message; std::vector<ForegroundObjectObservation> foreground;};
  std::optional<PreparedDetection> prepared_wrist,prepared_observer;
  bool next_detection_observer{false};
  std::string wrist_wait_reason{"waiting for input"};
  std::uint64_t next_frame{1},sequence{1},epoch{0}; Nanoseconds last_track_stamp{0};
  bool processing{false},image_processing{false}; std::string status{"waiting for RGB-D"};
  rclcpp::Subscription<ptc_interfaces::msg::TaskStatus>::SharedPtr task_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub,depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub,depth_info_sub;
  rclcpp::Subscription<ptc_interfaces::msg::DetectionArray>::SharedPtr detections_sub;
  rclcpp::Subscription<ptc_interfaces::msg::TrackedObjectArray>::SharedPtr shared_tracks_sub;
  std::vector<TrackedObjectState> wrist_search_hints;
  Nanoseconds wrist_search_hint_stamp{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr observer_color_sub,observer_depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr observer_color_info_sub,observer_depth_info_sub;
  rclcpp::Publisher<ptc_interfaces::msg::RgbdFrame>::SharedPtr frame_pub;
  rclcpp::Publisher<ptc_interfaces::msg::TrackedObjectArray>::SharedPtr tracks_pub;
  rclcpp::Publisher<ptc_interfaces::msg::CapabilityStatus>::SharedPtr status_pub;
  rclcpp_action::Server<Capture>::SharedPtr grasp_capture_server;
  std::shared_ptr<CaptureHandle> grasp_capture;
  Nanoseconds grasp_capture_deadline{0};
  std::string grasp_capture_wait_reason;
  rclcpp::TimerBase::SharedPtr timer;
  const SystemSettings & config() const {return settings.snapshot->get();}
  TimeContext now() {return clock.sample(node,config());}
  EventKey key(const TimeContext & time) {EventKey k; k.epoch=time.epoch;k.sequence=sequence++;k.settings_revision=settings.snapshot->revision();return k;}
  Pose3d transform(const std::string & target,const std::string & source,Nanoseconds stamp)
  {
    const auto wire=tf.lookupTransform(target,source,rclcpp::Time(stamp.count(),node.get_clock()->get_clock_type()),
      rclcpp::Duration::from_seconds(config().vision.tf_lookup_timeout_sec));
    geometry_msgs::msg::Pose p; p.position.x=wire.transform.translation.x;p.position.y=wire.transform.translation.y;p.position.z=wire.transform.translation.z;
    p.orientation=wire.transform.rotation; return from_ros(p);
  }
  static double duration(const builtin_interfaces::msg::Duration & d)
  {if(d.sec<0 || d.nanosec>=1000000000) {return -1;} return d.sec+d.nanosec*1e-9;}
  bool valid_goal(const ptc_interfaces::msg::EventKey & k,const builtin_interfaces::msg::Duration & timeout)
  {try {const auto time=now();return !time.paused && valid_event(from_ros(k),time,settings.snapshot->revision()) && k.track_id && k.cycle_id && seconds_to_duration(duration(timeout)).has_value();} catch (...) {return false;}}
  explicit Runtime(VisionNode & n):node(n),settings(ptc_app::load_node_settings(n)),robot(ptc_app::load_robot_description(config())),
    completions(config().diagnostics.event_queue_capacity),tf(n.get_clock()),listener(tf)
  {
    next_frame=static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count()); tracks.next_track_id=next_frame;
    const auto & ep=settings.endpoints; const auto qos=rclcpp::SensorDataQoS();
    color_sub=n.create_subscription<sensor_msgs::msg::Image>(ep.at("rgb"),qos,[this](sensor_msgs::msg::Image::ConstSharedPtr m){colors.push_back(m);trim(colors);});
    depth_sub=n.create_subscription<sensor_msgs::msg::Image>(ep.at("depth"),qos,[this](sensor_msgs::msg::Image::ConstSharedPtr m){depths.push_back(m);trim(depths);});
    color_info_sub=n.create_subscription<sensor_msgs::msg::CameraInfo>(ep.at("rgb_intrinsics"),qos,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){color_info=m;});
    depth_info_sub=n.create_subscription<sensor_msgs::msg::CameraInfo>(ep.at("depth_intrinsics"),qos,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){depth_info=m;});
    frame_pub=n.create_publisher<ptc_interfaces::msg::RgbdFrame>(ep.at("frame"),qos);
    if(config().vision.tracking_camera.role=="external" && config().vision.foreground_source=="model" &&
       config().grasp.neighbor_clearance_mode=="rank"){
      neighborhood_pub=n.create_publisher<Neighborhood>(ep.at("grasp_neighborhood"),rclcpp::QoS(1));
    }
    tracks_pub=n.create_publisher<ptc_interfaces::msg::TrackedObjectArray>(ep.at("tracks"),rclcpp::QoS(5));
    status_pub=n.create_publisher<ptc_interfaces::msg::CapabilityStatus>(ep.at("vision_status"),rclcpp::QoS(5));
    detections_sub=n.create_subscription<ptc_interfaces::msg::DetectionArray>(ep.at("detections"),rclcpp::QoS(5),[this](ptc_interfaces::msg::DetectionArray::ConstSharedPtr m){if(config().vision.foreground_source!="model"){return;}if(processing){pending_detection=m;}else{detect(*m);}});
    task_sub=n.create_subscription<ptc_interfaces::msg::TaskStatus>(ep.at("task_status"),rclcpp::QoS(5),[this](ptc_interfaces::msg::TaskStatus::ConstSharedPtr m){
      if(!grasp_patch_task || m->key.epoch!=grasp_patch_task->key.epoch || m->key.cycle_id!=grasp_patch_task->key.cycle_id || m->key.track_id!=grasp_patch_task->key.track_id || m->phase!=grasp_patch_task->phase || m->grasp_phase!=grasp_patch_task->grasp_phase)grasp_patch_task_seen=now().source_now;
      grasp_patch_task=m;
    });
    if(config().vision.tracking_camera.role=="source") {
      observer_color_sub=n.create_subscription<sensor_msgs::msg::Image>(ep.at("observer_rgb"),qos,[this](sensor_msgs::msg::Image::ConstSharedPtr m){observer_colors.push_back(m);trim(observer_colors);});
      observer_depth_sub=n.create_subscription<sensor_msgs::msg::Image>(ep.at("observer_depth"),qos,[this](sensor_msgs::msg::Image::ConstSharedPtr m){observer_depths.push_back(m);trim(observer_depths);});
      observer_color_info_sub=n.create_subscription<sensor_msgs::msg::CameraInfo>(ep.at("observer_rgb_intrinsics"),qos,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){observer_color_info=m;});
      observer_depth_info_sub=n.create_subscription<sensor_msgs::msg::CameraInfo>(ep.at("observer_depth_intrinsics"),qos,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){observer_depth_info=m;});
    } else if(config().vision.tracking_camera.role=="external") {
      shared_tracks_sub=n.create_subscription<ptc_interfaces::msg::TrackedObjectArray>(ep.at("shared_tracks"),rclcpp::QoS(5),[this](ptc_interfaces::msg::TrackedObjectArray::ConstSharedPtr m){
        const auto time=now();if(m->header.frame_id!=config().frames.world||m->objects.size()>config().tracking.max_tracked_objects){return;}
        std::vector<TrackedObjectState> incoming;
        std::vector<TrackedObjectState> hints;
        try {for(const auto & wire:m->objects){auto track=from_ros(wire);
          if(track.phase==TrackPhase::kCandidate && !track.observation.yaw_valid){
            if(track.context.key.epoch==time.epoch && track.context.key.track_id && track.velocity_valid &&
              track.observation.reference_frame==config().frames.world &&
              track.observation.timestamp<=from_ros(m->header.stamp)){
              track.context.key.settings_revision=settings.snapshot->revision();
              hints.push_back(std::move(track));
            }
            continue;
          }
          if(track.context.key.epoch!=time.epoch||!is_valid_pose(track.observation.pose_world)||!source_is_fresh(track.observation.timestamp,time,config().tracking.observation_max_age_sec,config().time.sensor_future_tolerance_sec)){continue;}track.context.key.settings_revision=settings.snapshot->revision();incoming.push_back(std::move(track));}}catch(...){return;}
        wrist_search_hints=std::move(hints);
        wrist_search_hint_stamp=from_ros(m->header.stamp);
        if(semantic_hint_max_age() == 0.0){
          tracks.tracks=std::move(incoming);publish_tracks(time);
        }
      });
    }
    if(config().vision.tracking_camera.role!="external" || config().vision.foreground_source!="model"){
      const PointIndex reference(ptc_app::load_reference_cloud(config().inspection.geometry));
      registration.emplace(reference,config().inspection.geometry.registration);
    }
    grasp_capture_server=rclcpp_action::create_server<Capture>(&n,ep.at("capture_grasp_surface"),
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Capture::Goal> goal){
        if(grasp_capture || !valid_goal(goal->key,goal->timeout) ||
          goal->requested_camera_world.header.frame_id!=config().frames.world ||
          !grasp_capture_matches_task(goal->key)){return rclcpp_action::GoalResponse::REJECT;}
        try{from_ros(goal->requested_camera_world.pose);}catch(...){return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<CaptureHandle>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<CaptureHandle> handle){
        grasp_capture=handle;
        grasp_capture_deadline=*deadline_after(now().monotonic_now,duration(handle->get_goal()->timeout));
        grasp_capture_wait_reason="no processed wrist frame";
        for(auto frame=frames.rbegin();grasp_capture && frame!=frames.rend();++frame){
          offer_grasp_capture(frame->second);
        }
      });
    timer=n.create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
  }
  ~Runtime() {image_worker.shutdown();worker.shutdown();}
  void trim(std::deque<sensor_msgs::msg::Image::ConstSharedPtr> & queue)
  {while(queue.size()>config().vision.max_buffered_frames) {queue.pop_front();}}
  void receive(bool observer=false)
  {
    if(observer&&config().vision.tracking_camera.role!="source"){return;}
    auto & colors=observer?observer_colors:this->colors;auto & depths=observer?observer_depths:this->depths;
    const auto color_info=observer?observer_color_info:this->color_info,depth_info=observer?observer_depth_info:this->depth_info;
    const auto & camera_config=config().vision.tracking_camera;
    const auto color_frame=observer?camera_config.color_optical:config().frames.color_optical;
    const auto depth_frame=observer?camera_config.depth_optical:config().frames.depth_optical;
    if(image_processing){return;}
    if(colors.empty() || depths.empty() || !color_info || !depth_info){
      if(!observer){wrist_wait_reason="input queues RGB="+std::to_string(colors.size())+" depth="+std::to_string(depths.size());}return;}
    try {
      const auto admission_started=std::chrono::steady_clock::now();
      const auto time=now(); if(time.paused) {return;}
      // Depth may arrive one frame later. Preserve the bounded RGB queue
      // and choose the newest color that already has a valid depth partner.
      // Never reuse images or widen the capture-time synchronization gate.
      auto selected_color=colors.end(),best=depths.end();Nanoseconds delta=Nanoseconds::max();
      for(auto ci=colors.rbegin();ci!=colors.rend();++ci){
        const auto color_stamp=from_ros((*ci)->header.stamp);
        if(!source_is_fresh(color_stamp,time,config().vision.frame_max_age_sec,config().time.sensor_future_tolerance_sec)){continue;}
        auto candidate=depths.end();Nanoseconds candidate_delta=Nanoseconds::max();
        for(auto di=depths.begin();di!=depths.end();++di){
          const auto difference=from_ros((*di)->header.stamp)-color_stamp;
          const auto absolute=difference.count()<0?-difference:difference;
          if(absolute<candidate_delta){candidate_delta=absolute;candidate=di;}
        }
        delta=std::min(delta,candidate_delta);
        if(candidate!=depths.end() && std::chrono::duration<double>(candidate_delta).count()<=config().vision.rgb_depth_sync_tolerance_sec){
          selected_color=std::prev(ci.base());best=candidate;break;}
      }
      if(selected_color==colors.end()){
        if(!observer){wrist_wait_reason="RGB/depth timestamp delta="+std::to_string(std::chrono::duration<double>(delta).count());}return;}
      const auto color=*selected_color,depth=*best;const auto stamp=from_ros(color->header.stamp);
      colors.erase(colors.begin(),std::next(selected_color));depths.erase(depths.begin(),std::next(best));
      if(color->header.frame_id!=color_frame || depth->header.frame_id!=depth_frame ||
        color_info->header.frame_id!=color->header.frame_id || depth_info->header.frame_id!=depth->header.frame_id ||
        color->width!=color_info->width || color->height!=color_info->height || depth->width!=depth_info->width || depth->height!=depth_info->height) {throw std::invalid_argument("camera frames/intrinsics mismatch");}
      const auto camera_world=transform(config().frames.world,color->header.frame_id,stamp);
      const auto tcp_world=transform(config().frames.world,config().frames.tcp,stamp);
      const auto expected=observer?std::optional<Pose3d>(camera_config.camera_world):compose_pose(tcp_world,robot.camera_in_tcp);
      if(!expected || !pose_near(*expected,camera_world,config().motion.position_tolerance_m,config().motion.orientation_tolerance_rad)) {
        if(expected){const auto & a=expected->position;const auto & b=camera_world.position;
          const auto & qa=expected->orientation;const auto & qb=camera_world.orientation;
          const double angle=2*std::acos(std::clamp(std::abs(qa.x*qb.x+qa.y*qb.y+qa.z*qb.z+qa.w*qb.w),0.,1.));
          RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,"RGB-D hand-eye mismatch camera=%s position=%.4f m angle=%.4f rad",observer?"observer":"wrist",vector_norm({a.x_m-b.x_m,a.y_m-b.y_m,a.z_m-b.z_m}),angle);}
        throw std::invalid_argument("TF does not match known hand-eye extrinsics");}
      const auto depth_stamp=from_ros(depth->header.stamp);
      const auto depth_world=transform(config().frames.world,depth->header.frame_id,depth_stamp);
      const auto depth_color=compose_pose(*inverse_pose(camera_world),depth_world);
      const auto rgb_camera=ptc_app::camera_model(*color_info),depth_camera=ptc_app::camera_model(*depth_info);
      if(!depth_color) {throw std::invalid_argument("invalid depth transform");}
      if(depth->encoding!="16UC1" && depth->encoding!="32FC1") {throw std::invalid_argument("depth encoding must be 16UC1 or 32FC1");}
      ptc_interfaces::msg::RgbdFrame wire;wire.key=to_ros(key(time));wire.frame_id=next_frame++;wire.calibration_id=observer?camera_config.calibration_id:config().calibration.expected_calibration_id;
      wire.color=*color;wire.intrinsics=*color_info;wire.depth_source_frame=depth->header.frame_id;
      wire.camera_world=ptc_app::stamped_pose(camera_world,config().frames.world,stamp);wire.depth_registered=true;
      wire.tcp_world=ptc_app::stamped_pose(tcp_world,config().frames.world,stamp);
      wire.depth.header=depth->header;wire.depth.header.frame_id=color->header.frame_id;wire.depth.width=color->width;wire.depth.height=color->height;wire.depth.encoding="32FC1";wire.depth.step=color->width*4;
      const auto tf_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-admission_started).count();
      const auto saved_epoch=time.epoch;image_processing=true;
      if(!image_worker.enqueue([this,color,depth,wire,rgb_camera,depth_camera,depth_color,saved_epoch,observer,tf_ms,admission_started]()mutable{
        try {
          const auto decoded=decode_depth(depth->width,depth->height,depth->step,depth->is_bigendian!=0,depth->encoding=="16UC1",depth->data,config().vision);
          const auto aligned=register_depth(decoded,depth_camera,rgb_camera,*depth_color);
          wire.depth.data.resize(aligned.size()*4);std::memcpy(wire.depth.data.data(),aligned.data(),wire.depth.data.size());
          const auto image=std::make_shared<const RgbdImageFrame>(ptc_app::image_frame(wire,config().vision));
          const auto foreground=config().vision.foreground_source=="depth_workspace"?
            detect_workspace_objects(*image,config().vision.depth_workspace):std::vector<ForegroundObjectObservation>{};
          const auto worker_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-admission_started).count()-tf_ms;
          completions.post([this,image,depth,depth_camera,depth_color,saved_epoch,foreground,observer,tf_ms,worker_ms]{image_processing=false;
            RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),2000,"RGB-D latency camera=%s TF/preparation=%.1f ms worker=%.1f ms",observer?"observer":"wrist",tf_ms,worker_ms);if(now().epoch!=saved_epoch){return;} frames[image->metadata.frame_id]={image,{},observer};if(!observer){last_wrist_frame=image->metadata.frame_id;}
            while(frames.size()>config().vision.max_buffered_frames){frames.erase(frames.begin());}
            if(!observer){frame_pub->publish(ptc_app::to_ros(*image,key(now()),config().frames.world));}status="RGB-D and capture-time TF ready";
            if(config().vision.foreground_source=="depth_workspace") {
              ptc_interfaces::msg::DetectionArray observations;observations.key=to_ros(key(now()));observations.frame_id=image->metadata.frame_id;
              observations.capture_stamp=ptc_app::to_ros_time(image->metadata.color_timestamp);observations.model_version=config().inspection.inference.model_version;
              observations.complete=true;observations.decision_source="depth_workspace";
              for(const auto & object:foreground) {ptc_interfaces::msg::Detection candidate;candidate.class_id=config().inspection.inference.object_class_ids.front();
                candidate.confidence=1.;candidate.bbox_xyxy={static_cast<double>(object.region.x),static_cast<double>(object.region.y),
                  static_cast<double>(object.region.x+object.region.width),static_cast<double>(object.region.y+object.region.height)};
                observations.detections.push_back(std::move(candidate));}
              auto & prepared=observer?prepared_observer:prepared_wrist;
              prepared=PreparedDetection{std::move(observations),foreground};
            }
          });
        }catch(const std::exception & e){const std::string reason=e.what();completions.post([this,reason]{image_processing=false;status=reason;});}
      })) {image_processing=false;status="vision worker queue full";}
    }catch(const std::exception & e) {status=e.what();if(!observer){wrist_wait_reason=e.what();}}
  }
  double semantic_hint_max_age() const
  {
    if(!shared_tracks_sub){return 0.0;}
    const std::string_view topic=shared_tracks_sub->get_topic_name();
    if(topic=="/factory/observe/pick_preview_tracks"){return 22.0;}
    if(topic=="/factory/observe/pick_preview_tracks_arm_02"){return 50.0;}
    return 0.0;
  }
  std::optional<ObjectObservation> measure_masked_object(
    const std::vector<Vector3d> & points,Nanoseconds stamp) const
  {
    Vector3d minimum{INFINITY,INFINITY,INFINITY};
    Vector3d maximum{-INFINITY,-INFINITY,-INFINITY};
    std::size_t count=0;
    for(const auto & p:points){
      if(!std::isfinite(vector_norm(p)) || p.z_m<=config().vision.depth_workspace.support_plane_z_m+.002){continue;}
      minimum={std::min(minimum.x_m,p.x_m),std::min(minimum.y_m,p.y_m),std::min(minimum.z_m,p.z_m)};
      maximum={std::max(maximum.x_m,p.x_m),std::max(maximum.y_m,p.y_m),std::max(maximum.z_m,p.z_m)};
      ++count;
    }
    if(count<3){return std::nullopt;}
    const double resolution=config().inspection.geometry.fusion.evidence_voxel_size_m;
    const double radius=.5*std::hypot(maximum.x_m-minimum.x_m,maximum.y_m-minimum.y_m);
    if(radius<resolution){return std::nullopt;}
    Pose3d pose{{(minimum.x_m+maximum.x_m)*.5,(minimum.y_m+maximum.y_m)*.5,
      (minimum.z_m+maximum.z_m)*.5},{}};
    // The object coordinate axes stay parallel to world. Candidate scoring
    // estimates contact directions from the captured surface.
    ObjectObservation measured{pose,stamp,config().frames.world,resolution,true,0.,true};
    measured.envelope_radius_xy_m=radius+resolution;
    measured.envelope_half_height_m=(maximum.z_m-minimum.z_m)*.5+resolution;
    measured.envelope_valid=true;
    return measured;
  }

  void detect(const ptc_interfaces::msg::DetectionArray & message,const std::vector<ForegroundObjectObservation> & foreground={})
  {
    const double hint_max_age=semantic_hint_max_age();
    const bool semantic_input=hint_max_age>0.0;
    const bool valid_model=semantic_input
      ? message.decision_source=="model" && !message.model_version.empty()
      : message.model_version==config().inspection.inference.model_version;
    if(processing || !message.complete || !valid_model || (!semantic_input && !registration) ||
      message.key.settings_revision!=settings.snapshot->revision()) {return;}
    const auto found=frames.find(message.frame_id);const auto time=now();
    const bool existing_only=found!=frames.end() && !found->second.observer && config().vision.tracking_camera.role=="source";
    // The fixed source owns global track updates. A wrist frame associates
    // visible existing identities only; its empty ROI cannot mark all
    // shared tracks missing when the wrist looks away during a transition.
    const bool tracking_source_frame=found!=frames.end() &&
      (config().vision.tracking_camera.role=="source" ? found->second.observer :
       config().vision.tracking_camera.role!="external");
    const bool update_tracks=tracking_source_frame && found->second.image->metadata.color_timestamp>last_track_stamp;
    if(found==frames.end() || message.key.epoch!=time.epoch ||
      from_ros(message.capture_stamp)!=found->second.image->metadata.color_timestamp) {return;}
    const auto image=found->second.image;processing=true;
    const auto track_snapshot=tracks.tracks;
    const bool semantic_wrist=semantic_input && !found->second.observer;
    const auto search_hints=semantic_wrist && source_is_fresh(wrist_search_hint_stamp,time,
      config().tracking.observation_max_age_sec,config().time.sensor_future_tolerance_sec)
      ? wrist_search_hints : std::vector<TrackedObjectState>{};

    if(!worker.enqueue([this,image,message,time,track_snapshot,foreground,update_tracks,existing_only,semantic_wrist,search_hints,hint_max_age]{
      const auto registration_started=std::chrono::steady_clock::now();
      try {
        std::vector<ObjectObservation> observations;std::vector<ObjectFrame> objects;
        std::size_t valid_fits=0, valid_yaws=0;
        double nearest_hint_m=std::numeric_limits<double>::infinity();
        for(const auto & detection:message.detections) {
          const auto & cfg=config().inspection.inference;
          if(detection.is_defect || detection.confidence<cfg.object_confidence_threshold || !std::isfinite(detection.confidence) ||
            std::find(cfg.object_class_ids.begin(),cfg.object_class_ids.end(),detection.class_id)==cfg.object_class_ids.end()) {continue;}
          const auto roi=image_region(detection.bbox_xyxy,image->camera,config().vision.roi_margin_ratio);
          if(!frame_quality(*image,roi,config().vision).valid) {continue;}
          const auto index=static_cast<std::size_t>(&detection-&message.detections.front());
          auto mask=index<foreground.size()?foreground[index].mask:ptc_app::decode_mask(detection.mask,image->camera);
          if(semantic_wrist && mask.empty()){continue;}
          const auto points=region_points_world(*image,roi,config().inspection.geometry.fusion.max_points,mask);
          std::optional<Pose3d> initial;double best=std::numeric_limits<double>::infinity();
          double second=std::numeric_limits<double>::infinity();std::uint64_t candidate_id=0;
          if(!semantic_wrist && !points.empty()){
            Vector3d center{};for(const auto & p:points){center.x_m+=p.x_m;center.y_m+=p.y_m;center.z_m+=p.z_m;}const double count=static_cast<double>(points.size());center={center.x_m/count,center.y_m/count,center.z_m/count};
            const auto nominal=registration->model().center_object;
            auto capture_time=time;capture_time.source_now=image->metadata.color_timestamp;
            for(const auto & track:track_snapshot){if(!track.observation.yaw_valid){continue;}const auto predicted=track_pose_at_observation(track,*settings.snapshot,capture_time,image->metadata.color_timestamp);if(!predicted){continue;}
              const auto p=transform_point(*predicted,nominal);const double distance=vector_norm({p.x_m-center.x_m,p.y_m-center.y_m,p.z_m-center.z_m});
              if(distance<best){second=best;best=distance;initial=*predicted;candidate_id=track.context.key.track_id;}else{second=std::min(second,distance);}}
          }
          if(best>config().tracking.association_distance_m){initial.reset();candidate_id=0;}
          if(existing_only && !initial){continue;}
          if(initial && std::isfinite(second)&&second-best<config().tracking.max_position_stddev_m){continue;}
          if(candidate_id && std::any_of(objects.begin(),objects.end(),[&](const auto & o){return o.track_id==candidate_id;})){continue;}
          ObjectObservation observation;
          if(semantic_wrist){
            const auto measured=measure_masked_object(points,image->metadata.color_timestamp);
            if(!measured){continue;}
            observation=*measured;
          }else{
            const auto associated=std::find_if(track_snapshot.begin(),track_snapshot.end(),
              [candidate_id](const auto & track){return track.context.key.track_id==candidate_id;});
            auto fit_initial=initial;
            if(fit_initial && associated!=track_snapshot.end() && !associated->velocity_samples.empty()){
              fit_initial->orientation=associated->velocity_samples.back().pose_world.orientation;
            }
            const auto pose=registration->fit(points,config(),fit_initial);
            if(!pose.valid){continue;}
            if(update_tracks && !initial && std::abs(pose.object_world.position.z_m-
              config().vision.depth_workspace.support_plane_z_m)>config().tracking.association_distance_m){continue;}
            const auto & model=registration->model();
            observation={pose.object_world,image->metadata.color_timestamp,config().frames.world,
              pose.error_m,true,std::atan2(pose.error_m,model.radius_m),pose.yaw_valid};
            observation.envelope_center_object_m=model.center_object;
            observation.envelope_radius_xy_m=model.radius_m;
            observation.envelope_half_height_m=model.half_height_m;
            observation.envelope_valid=std::isfinite(model.radius_m)&&model.radius_m>0 &&
              std::isfinite(model.half_height_m)&&model.half_height_m>0;
          }
          ++valid_fits;
          if(semantic_wrist){
            candidate_id=0;
            if(!observation.yaw_valid){continue;}
            ++valid_yaws;
            for(const auto & hint:search_hints){
              const double age=std::chrono::duration<double>(observation.timestamp-hint.observation.timestamp).count();
              if(age<0 || age>hint_max_age){continue;}
              const auto & p=hint.observation.pose_world.position;
              const auto & v=hint.velocity_mps;
              const auto & q=observation.pose_world.position;
              const auto distance=std::hypot(q.x_m-p.x_m-v.x_m*age,q.y_m-p.y_m-v.y_m*age);
              nearest_hint_m=std::min(nearest_hint_m,distance);
              if(distance>config().tracking.association_distance_m){continue;}
              // Preserve identity across missed wrist frames. A new fit must
              // agree with both the gantry hint and the last wrist measurement.
              const auto prior=std::find_if(track_snapshot.begin(),track_snapshot.end(),[&](const auto & item){
                return item.context.key.track_id==hint.context.key.track_id;
              });
              // A lost local estimator cannot anchor a new wrist fit forever.
              // The still-live gantry hint owns identity; reacquisition below
              // requires new wrist measurements and a new velocity fit.
              if(prior!=track_snapshot.end() && prior->phase!=TrackPhase::kLost){
                const double gap=std::chrono::duration<double>(observation.timestamp-prior->observation.timestamp).count();
                const auto & previous=prior->observation.pose_world.position;
                if(gap<0 || std::hypot(q.x_m-previous.x_m-v.x_m*gap,q.y_m-previous.y_m-v.y_m*gap)>0.035 ||
                  std::abs(q.z_m-previous.z_m)>0.035){continue;}
              }
              if(candidate_id){candidate_id=0;break;}
              candidate_id=hint.context.key.track_id;
            }
            if(!candidate_id){continue;}
          }
          objects.push_back({candidate_id,observation.pose_world,roi,std::move(mask)});
          observations.push_back(std::move(observation));
        }
        if(semantic_wrist){
          RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),1000,
            "Semantic wrist detections=%zu fits=%zu yaws=%zu hints=%zu nearest=%.3f matches=%zu",
            message.detections.size(),valid_fits,valid_yaws,search_hints.size(),nearest_hint_m,objects.size());
        }
        const double registration_elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-registration_started).count();
        completions.post([this,image,time,observations,objects,update_tracks,existing_only,semantic_wrist,search_hints,registration_elapsed_ms]()mutable{
          processing=false;const auto current=now();if(current.epoch!=time.epoch || (update_tracks && image->metadata.color_timestamp<=last_track_stamp)){return;}
          RCLCPP_INFO_THROTTLE(node.get_logger(),*node.get_clock(),2000,"RGB-D registration camera=%s worker=%.3f ms source_age=%.6f s objects=%zu",update_tracks?"source":"wrist",registration_elapsed_ms,std::chrono::duration<double>(current.source_now-image->metadata.color_timestamp).count(),objects.size());
          if(update_tracks){std::vector<std::uint64_t> candidate_ids;candidate_ids.reserve(objects.size());
            for(const auto & object:objects){candidate_ids.push_back(object.track_id);}
            const auto accepted=update_visual_tracks(tracks,observations,candidate_ids,*settings.snapshot,current);
            for(std::size_t i=0;i<objects.size();++i){objects[i].track_id=accepted[i];}
            last_track_stamp=image->metadata.color_timestamp;}
          // A wrist observation can refresh an already associated identity
          // through fixed-camera occlusion. It cannot acquire new identities
          // or mark unrelated tracks missing when looking elsewhere.
          if(existing_only){
            for(std::size_t i=0;i<objects.size();++i){
              const auto target=std::find_if(tracks.tracks.begin(),tracks.tracks.end(),[&](const auto & track){return track.context.key.track_id==objects[i].track_id;});
              if(target==tracks.tracks.end() || observations[i].timestamp<=target->observation.timestamp){continue;}
              // A fresh fixed-camera measurement already owns this track's world pose.
              // Wrist frames still feed capture/cloud, but only refresh tracking through occlusion.
              if(target->observation.timestamp==last_track_stamp&&source_is_fresh(last_track_stamp,current,
                config().tracking.observation_max_age_sec,config().time.sensor_future_tolerance_sec)){continue;}
              const auto expected=track_pose_at_observation(*target,*settings.snapshot,current,observations[i].timestamp);
              const auto & observed=observations[i].pose_world.position;
              if(!expected||vector_norm({observed.x_m-expected->position.x_m,observed.y_m-expected->position.y_m,observed.z_m-expected->position.z_m})>config().tracking.association_distance_m){objects[i].track_id=0;continue;}
              auto observation_key=target->context.key;observation_key.sequence=tracks.next_sequence++;
              auto decision=advance_track(*target,{TrackEventKind::kObservation,observation_key,observations[i]},*settings.snapshot,current);
              if(decision.status.disposition==Disposition::kApplied){*target=std::move(decision.state);}else{objects[i].track_id=0;}
            }
          }
          std::set<std::uint64_t> matched;
          if(semantic_wrist){
            for(const auto & object:objects){
              const auto id=object.track_id;
              if(std::count_if(objects.begin(),objects.end(),[id](const auto & item){return item.track_id==id;})==1){
                matched.insert(id);
              }
            }
            auto & live=tracks.tracks;
            live.erase(std::remove_if(live.begin(),live.end(),[&](const auto & track){
              return std::none_of(search_hints.begin(),search_hints.end(),[&](const auto & hint){
                return hint.context.key.track_id==track.context.key.track_id;
              });
            }),live.end());
            for(std::size_t i=0;i<objects.size();++i){
              const auto id=objects[i].track_id;
              if(!matched.count(id)){
                objects[i].track_id=0;
                continue;
              }
              auto target=std::find_if(tracks.tracks.begin(),tracks.tracks.end(),[id](const auto & track){return track.context.key.track_id==id;});
              if(target==tracks.tracks.end() ||
                (target->phase==TrackPhase::kLost && observations[i].timestamp>target->observation.timestamp)){
                TrackedObjectState seed;
                seed.context.key=key(current);
                seed.context.key.track_id=id;
                seed.observation=observations[i];
                seed.confirmations=1;
                 const auto hint=std::find_if(search_hints.begin(),search_hints.end(),[id](const auto & item){
                   return item.context.key.track_id==id;
                 });
                 if(hint!=search_hints.end()){
                   const double age=std::chrono::duration<double>(seed.observation.timestamp-hint->observation.timestamp).count();
                   const auto & p=hint->observation.pose_world.position;
                   const auto & q=seed.observation.pose_world.position;
                   const auto & v=hint->velocity_mps;
                   RCLCPP_INFO(node.get_logger(),
                     "semantic wrist association: epoch=%llu track=%llu kind=%s gantry_source=%.9f wrist_source=%.9f processed_source=%.9f planar_residual_m=%.6f",
                     static_cast<unsigned long long>(current.epoch),static_cast<unsigned long long>(id),
                     target==tracks.tracks.end()?"first":"reacquired",
                     std::chrono::duration<double>(hint->observation.timestamp).count(),
                     std::chrono::duration<double>(seed.observation.timestamp).count(),
                     std::chrono::duration<double>(current.source_now).count(),
                     std::hypot(q.x_m-p.x_m-v.x_m*age,q.y_m-p.y_m-v.y_m*age));
                 }
                // Keep the shared ID but discard a terminal local fit. Do not
                // revive retired targets or carry its stale velocity/history.
                if(target==tracks.tracks.end()){tracks.tracks.push_back(std::move(seed));}
                else{*target=std::move(seed);}
              }else if(observations[i].timestamp>target->observation.timestamp){
                auto observation_key=key(current);
                observation_key.track_id=id;
                auto decision=advance_track(*target,{TrackEventKind::kObservation,observation_key,observations[i]},*settings.snapshot,current);
                if(decision.status.disposition!=Disposition::kApplied){continue;}
                 const bool had_velocity=target->velocity_valid;
                 *target=std::move(decision.state);
                const auto & limits=config().tracking;
                const auto & samples=target->velocity_samples;
                 if(!had_velocity && target->velocity_valid){
                   RCLCPP_INFO(node.get_logger(),
                     "semantic wrist velocity: epoch=%llu track=%llu method=wrist_fit wrist_source=%.9f samples=%zu residual_m=%.6f uncertainty_mps=%.6f",
                     static_cast<unsigned long long>(current.epoch),static_cast<unsigned long long>(id),
                     std::chrono::duration<double>(observations[i].timestamp).count(),samples.size(),
                     target->velocity_fit_residual_m,target->velocity_uncertainty_mps);
                 }
                if(target->velocity_valid || target->confirmations<limits.min_confirmed_observations ||
                  samples.size()<limits.velocity_min_samples){continue;}
                const auto span=std::chrono::duration<double>(samples.back().timestamp-samples.front().timestamp).count();
                if(span<=0 || span>=limits.velocity_min_span_sec){continue;}
                const auto hint=std::find_if(search_hints.begin(),search_hints.end(),[id](const auto & item){
                  return item.context.key.track_id==id;
                });
                // Only a measured gantry fit carries confirmations. A belt-speed
                // search prior alone cannot make a wrist candidate selectable.
                if(hint==search_hints.end() || hint->confirmations<limits.min_confirmed_observations){continue;}
                const auto & latest=samples.back();
                const auto & position=latest.pose_world.position;
                const auto & velocity=hint->velocity_mps;
                double squared_error=0.;
                for(const auto & sample:samples){
                  const auto dt=std::chrono::duration<double>(sample.timestamp-latest.timestamp).count();
                  const auto & point=sample.pose_world.position;
                  const Vector3d residual{point.x_m-position.x_m-velocity.x_m*dt,
                    point.y_m-position.y_m-velocity.y_m*dt,point.z_m-position.z_m-velocity.z_m*dt};
                  const auto distance=vector_norm(residual);
                  squared_error+=distance*distance;
                }
                const auto wrist_residual=std::sqrt(squared_error/static_cast<double>(samples.size()));
                const auto residual=std::max(wrist_residual,hint->velocity_fit_residual_m);
                const auto uncertainty=std::hypot(hint->velocity_uncertainty_mps,wrist_residual/span);
                if(!std::isfinite(residual) || !std::isfinite(uncertainty) ||
                  residual>limits.max_velocity_fit_residual_m || uncertainty>limits.max_velocity_uncertainty_mps){continue;}
                target->velocity_mps=velocity;
                target->velocity_valid=true;
                target->velocity_fit_residual_m=residual;
                target->velocity_uncertainty_mps=uncertainty;
                target->phase=TrackPhase::kTracking;
                 RCLCPP_INFO(node.get_logger(),
                   "semantic wrist velocity: epoch=%llu track=%llu method=measured_gantry_prior gantry_source=%.9f wrist_source=%.9f samples=%zu span_sec=%.6f residual_m=%.6f uncertainty_mps=%.6f",
                   static_cast<unsigned long long>(current.epoch),static_cast<unsigned long long>(id),
                   std::chrono::duration<double>(hint->observation.timestamp).count(),
                   std::chrono::duration<double>(observations[i].timestamp).count(),samples.size(),span,residual,uncertainty);
              }
            }
          }
          for(auto & object:objects){if(!object.track_id){continue;}
            const auto track=std::find_if(tracks.tracks.begin(),tracks.tracks.end(),[&](const auto & item){return item.context.key.track_id==object.track_id;});
            if(track==tracks.tracks.end()){object.track_id=0;continue;}
            const auto expected=track_pose_at_observation(*track,*settings.snapshot,current,image->metadata.color_timestamp);
            const auto & observed=object.pose.position;
            if(!expected||vector_norm({observed.x_m-expected->position.x_m,observed.y_m-expected->position.y_m,observed.z_m-expected->position.z_m})>config().tracking.association_distance_m){object.track_id=0;continue;}
            // Cloud/capture and published tracking share the accepted capture-time pose.
            object.pose=*expected;}
          auto entry=frames.find(image->metadata.frame_id);if(entry!=frames.end()){entry->second.objects=std::move(objects);
            if(!entry->second.observer&&grasp_patch_task){const auto & associated=entry->second.objects;
              const auto selected=std::find_if(associated.begin(),associated.end(),[this](const auto & o){return o.track_id==grasp_patch_task->key.track_id;});
              if(selected!=associated.end()){publish_grasp_neighborhood(*entry->second.image,*selected,*grasp_patch_task,current);}
            }
            offer_grasp_capture(entry->second);}
          publish_tracks(current,semantic_wrist?&matched:nullptr);
        });
      }catch(const std::exception & e){const std::string reason=e.what();completions.post([this,reason]{processing=false;status=reason;});}
    })) {processing=false;}
  }
  void publish_tracks(const TimeContext & time,const std::set<std::uint64_t> * measured=nullptr)
  {
    ptc_interfaces::msg::TrackedObjectArray message;message.header.frame_id=config().frames.world;message.header.stamp=ptc_app::to_ros_time(time.source_now);
    for(const auto & track:tracks.tracks){
      if(!measured || measured->count(track.context.key.track_id)){message.objects.push_back(to_ros(track));}
    }
    tracks_pub->publish(message);
  }
  void publish_grasp_neighborhood(const RgbdImageFrame & image,const ObjectFrame & object,
    const ptc_interfaces::msg::TaskStatus & task,const TimeContext & time)
  {
    const auto stamp=image.metadata.color_timestamp;
    if(!neighborhood_pub || task.phase!=static_cast<std::uint8_t>(TaskPhase::kPreparingPick) ||
      !task.key.cycle_id || task.key.epoch!=image.metadata.epoch || object.mask.empty() ||
      !rgbd_frame_valid(image.metadata,*settings.snapshot,time) ||
      (neighborhood_cycle==task.key.cycle_id && stamp-neighborhood_stamp<std::chrono::milliseconds(150))){return;}
    const auto start=std::chrono::steady_clock::now();
    const auto points=grasp_neighbor_offsets(image,object.roi,object.mask,object.pose.position,
      config().grasp.open_width_m+config().grasp.pregrasp_clearance_m,config().vision.depth_workspace.support_plane_z_m);
    Neighborhood sample;
    sample.key=task.key;sample.stamp=ptc_app::to_ros_time(stamp);
    sample.offsets_world.reserve(points.size());
    for(const auto & p:points){
      geometry_msgs::msg::Point32 wire;
      wire.x=static_cast<float>(p.x_m);wire.y=static_cast<float>(p.y_m);wire.z=static_cast<float>(p.z_m);
      sample.offsets_world.push_back(wire);
    }
    sample.extraction_ms=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-start).count();
    neighborhood_cycle=task.key.cycle_id;neighborhood_stamp=stamp;
    neighborhood_pub->publish(sample);
  }
  bool grasp_capture_matches_task(const ptc_interfaces::msg::EventKey & key) const
  {
    return grasp_patch_task &&
      grasp_patch_task->phase==static_cast<std::uint8_t>(TaskPhase::kPreparingPick) &&
      same_cycle(from_ros(grasp_patch_task->key),from_ros(key));
  }

  bool populate_capture_surface(const StoredFrame & frame,const ObjectFrame & object,
    const Capture::Goal & goal,Capture::Result & result)
  {
    const auto inverse=inverse_pose(object.pose);if(!inverse)return false;
    std::vector<Vector3d> surface;
    for(const auto & point:region_points_world(*frame.image,object.roi,12000,object.mask)){
      if(point.z_m>config().vision.depth_workspace.support_plane_z_m+.002)surface.push_back(transform_point(*inverse,point));}
    result.object_cloud.header.frame_id="object_"+std::to_string(goal.key.track_id);result.object_cloud.header.stamp=ptc_app::to_ros_time(frame.image->metadata.color_timestamp);
    sensor_msgs::PointCloud2Modifier modifier(result.object_cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(surface.size());
    sensor_msgs::PointCloud2Iterator<float> px(result.object_cloud,"x"),py(result.object_cloud,"y"),pz(result.object_cloud,"z");
    for(const auto & point:surface){*px=point.x_m;*py=point.y_m;*pz=point.z_m;++px;++py;++pz;}
    if(semantic_hint_max_age()>0. && !object.mask.empty()){
      const auto start=std::chrono::steady_clock::now();
      const auto neighbors=grasp_neighbor_offsets(*frame.image,object.roi,object.mask,object.pose.position,
        config().grasp.open_width_m+config().grasp.pregrasp_clearance_m,config().vision.depth_workspace.support_plane_z_m);
      result.neighbor_offsets_world.reserve(neighbors.size());
      for(const auto & point:neighbors){
        geometry_msgs::msg::Point32 wire;
        wire.x=static_cast<float>(point.x_m);wire.y=static_cast<float>(point.y_m);wire.z=static_cast<float>(point.z_m);
        result.neighbor_offsets_world.push_back(wire);
      }
      result.neighbor_capture_ms=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-start).count();
    }
    return !surface.empty();
  }

  void finish_grasp_capture(const std::string & reason)
  {
    if(!grasp_capture)return;
    auto result=std::make_shared<Capture::Result>();
    result->key=grasp_capture->get_goal()->key;
    result->command_id=grasp_capture->get_goal()->command_id;
    result->message=(reason+"; "+grasp_capture_wait_reason).substr(0,512);
    if(grasp_capture->is_canceling())grasp_capture->canceled(result);
    else grasp_capture->abort(result);
    grasp_capture.reset();
  }

  void offer_grasp_capture(const StoredFrame & frame)
  {
    if(frame.observer || !grasp_capture || grasp_capture->is_canceling())return;
    const auto goal=grasp_capture->get_goal();
    const auto time=now();
    if(time.paused || expired(grasp_capture_deadline,time) || !grasp_capture_matches_task(goal->key))return;
    const auto & metadata=frame.image->metadata;
    if(!rgbd_frame_valid(metadata,*settings.snapshot,time) ||
      !pose_near(metadata.camera_in_world,from_ros(goal->requested_camera_world.pose),.03,.35)){
      grasp_capture_wait_reason="waiting for fresh RGB-D at the requested wrist pose";
      return;
    }
    const auto object=std::find_if(frame.objects.begin(),frame.objects.end(),
      [&](const auto & candidate){return candidate.track_id==goal->key.track_id && !candidate.mask.empty();});
    const auto track=std::find_if(tracks.tracks.begin(),tracks.tracks.end(),
      [&](const auto & candidate){return same_track_identity(candidate.context.key,from_ros(goal->key));});
    if(object==frame.objects.end() || track==tracks.tracks.end()){
      grasp_capture_wait_reason="target not associated with a wrist mask";
      return;
    }
    auto result=std::make_shared<Capture::Result>();
    result->key=goal->key;
    result->command_id=goal->command_id;
    if(!populate_capture_surface(frame,*object,*goal,*result)){
      grasp_capture_wait_reason="no finite masked surface above the support plane";
      return;
    }
    auto observed=*track;
    observed.observation.pose_world=object->pose;
    observed.observation.timestamp=metadata.color_timestamp;
    result->object_at_capture=to_ros(observed);
    result->frame.frame_id=metadata.frame_id;
    result->frame.camera_world=ptc_app::stamped_pose(metadata.camera_in_world,config().frames.world,metadata.color_timestamp);
    result->success=true;
    result->message="fresh masked wrist surface accepted";
    grasp_capture->succeed(result);
    grasp_capture.reset();
  }

  void tick()
  {
    completions.drain();const auto time=now();
    if(!processing && pending_detection){auto message=std::move(pending_detection);detect(*message);}
    if(epoch && epoch!=time.epoch){frames.clear();colors.clear();depths.clear();observer_colors.clear();observer_depths.clear();last_wrist_frame=0;prepared_wrist.reset();prepared_observer.reset();tracks.tracks.clear();last_track_stamp=Nanoseconds{0};finish_grasp_capture("clock epoch changed");}
    epoch=time.epoch;
    if(grasp_capture && (grasp_capture->is_canceling() || time.paused ||
      expired(grasp_capture_deadline,time) || !grasp_capture_matches_task(grasp_capture->get_goal()->key))){
      finish_grasp_capture("grasp capture canceled, paused, timed out or target changed");
    }
    ptc_interfaces::msg::CapabilityStatus message;message.key=to_ros(key(time));message.header.frame_id=config().frames.world;message.header.stamp=node.now();
    message.ready=!completions.overflowed() && !time.paused && frames.count(last_wrist_frame) && rgbd_frame_valid(frames.at(last_wrist_frame).image->metadata,*settings.snapshot,time);
    message.binding_revision=settings.binding_revision;message.calibration_id=config().calibration.expected_calibration_id;message.message=(message.ready?status:status+"; wrist: "+wrist_wait_reason).substr(0,512);status_pub->publish(message);
    // Registration must not block fresh RGB-D admission. Retain at most the
    // newest prepared detection per camera; process them in alternating order.
    if(!processing && (prepared_wrist || prepared_observer)){
      const bool observer=prepared_observer && (next_detection_observer || !prepared_wrist);
      auto & pending=observer?prepared_observer:prepared_wrist;
      auto work=std::move(*pending);pending.reset();detect(work.message,work.foreground);
      if(processing){next_detection_observer=!observer;}
    }
    if(!image_processing){const bool observer=next_observer;receive(observer);
      if(image_processing){next_observer=!observer;}else{receive(!observer);if(image_processing){next_observer=observer;}}}
  }
};
VisionNode::VisionNode(const rclcpp::NodeOptions & options):Node("vision",options),runtime_(std::make_unique<Runtime>(*this)){}
VisionNode::~VisionNode()=default;
}
