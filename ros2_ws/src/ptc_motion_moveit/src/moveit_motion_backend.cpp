#include "ptc_motion_moveit/moveit_motion_backend.hpp"
#include "ptc_motion_moveit/grasp_contact_policy.hpp"
#include "ptc_motion_moveit/grasp_clearance_model.hpp"
#include <geometric_shapes/body_operations.h>
#include "ptc_motion_moveit/gripper_feedback_policy.hpp"
#include "ptc_app/robot_description_loader.hpp"
#include "ptc_core/approach_timing.hpp"
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_app/bounded_worker.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <geometric_shapes/shapes.h>
#include "ptc_interfaces/msg/tracked_object_array.hpp"
#include <Eigen/SVD>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometric_shapes/shape_operations.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <fstream>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <condition_variable>
#include <atomic>
#include <set>
#include <array>
#include <tuple>
#include <deque>
#include <algorithm>
#include <cmath>
#include <map>
#include <thread>
#include <future>
#include <limits>
namespace ptc_motion_moveit
{
using namespace ptc_core;
namespace
{
struct VoxelizedSurface {std::vector<Eigen::Vector3d> centers_object;double resolution_m{0};std::size_t finite_points{0};};
struct CachedSurface {EventKey key;std::uint64_t revision{0};VoxelizedSurface geometry;};
std::optional<VoxelizedSurface> voxelize_surface(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const auto point_count=static_cast<std::uint64_t>(cloud.width)*cloud.height;
  if(!point_count||point_count>60000||cloud.point_step<12||cloud.data.empty()||cloud.is_bigendian){return std::nullopt;}
  const auto minimum_row=static_cast<std::uint64_t>(cloud.width)*cloud.point_step;
  const auto expected_bytes=static_cast<std::uint64_t>(cloud.row_step)*cloud.height;
  if(cloud.row_step<minimum_row||expected_bytes!=cloud.data.size()){return std::nullopt;}
  for(const auto * name:{"x","y","z"}){
    const auto field=std::find_if(cloud.fields.begin(),cloud.fields.end(),[&](const auto & item){return item.name==name;});
    if(field==cloud.fields.end()||field->datatype!=sensor_msgs::msg::PointField::FLOAT32||field->count!=1||
      static_cast<std::uint64_t>(field->offset)+sizeof(float)>cloud.point_step){return std::nullopt;}
  }
  std::vector<Eigen::Vector3d> points;points.reserve(point_count);
  try{
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    for(;x!=x.end();++x,++y,++z){if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)){points.emplace_back(*x,*y,*z);}}
  }catch(const std::exception &){return std::nullopt;}
  if(points.empty()){return std::nullopt;}
  double resolution=.004;std::set<std::tuple<long long,long long,long long>> cells;
  for(unsigned pass=0;pass<24;++pass){
    cells.clear();for(const auto & point:points){
      const double x=std::floor(point.x()/resolution),y=std::floor(point.y()/resolution),z=std::floor(point.z()/resolution);
      const double integer_limit=static_cast<double>(std::numeric_limits<long long>::max()/2);
      if(std::abs(x)>integer_limit||std::abs(y)>integer_limit||std::abs(z)>integer_limit){return std::nullopt;}
      cells.emplace(static_cast<long long>(x),static_cast<long long>(y),static_cast<long long>(z));}
    if(cells.size()<=1024){break;}
    if(resolution>=.020){return std::nullopt;}
    resolution=std::min(.020,resolution*1.2);
  }
  if(cells.empty()||cells.size()>1024||!std::isfinite(resolution)){return std::nullopt;}
  VoxelizedSurface result;result.resolution_m=resolution;result.finite_points=points.size();result.centers_object.reserve(cells.size());
  for(const auto & item:cells){result.centers_object.emplace_back((std::get<0>(item)+.5)*resolution,
    (std::get<1>(item)+.5)*resolution,(std::get<2>(item)+.5)*resolution);}
  return result;
}
std::string read(const std::string & path)
{std::ifstream file(path);if(!file){throw std::runtime_error("cannot read robot semantic file");}std::ostringstream text;text<<file.rdbuf();return text.str();}
DeviceStatus status(bool ready)
{DeviceStatus out;out.ready=ready;out.data_stale=!ready;out.connection_state=ready?DeviceConnectionState::kConnected:DeviceConnectionState::kConnecting;out.health_state=ready?DeviceHealthState::kHealthy:DeviceHealthState::kDegraded;return out;}
struct MoveitBackendRuntime
{
  using Group=moveit::planning_interface::MoveGroupInterface;
  using Grip=control_msgs::action::GripperCommand;
  using Execute=moveit_msgs::action::ExecuteTrajectory;
  using Planning=moveit_msgs::srv::GetMotionPlan;
  ptc_app::SettingsLoadResult settings;ptc_app::RobotDescriptionModel robot;
  rclcpp::Node::SharedPtr node;rclcpp::executors::SingleThreadedExecutor executor;std::thread spin_thread;
  std::unique_ptr<Group> group;
  tf2_ros::Buffer tf;tf2_ros::TransformListener listener;
  std::mutex mutex,start_mutex,plan_mutex;std::condition_variable changed;
  ptc_app::BoundedWorker diagnostic_worker{1};
  sensor_msgs::msg::JointState joints;std::uint64_t joint_sequence{0};
  std::deque<std::pair<sensor_msgs::msg::JointState,std::uint64_t>> joint_history;
  std::atomic<std::uint64_t> cancellation{1},gripper_cancellation{1};std::uint64_t planned_generation{0};Group::Plan plan;
  Command target;bool following{false},stopping{false},servo_paused{false},pause_pending{false},twist_type{false},type_pending{false},contact_ready{false},contact_pending{false};
  MoveToPoseRequest planned_request;StampedPose3d planned_endpoint;
  // Timing samples belong to this backend, protected by plan_mutex and bounded to eight.
  std::deque<double> travel_ratios,planning_source_costs,execution_source_overheads;
  double planned_joint_duration_sec{0},planned_planning_source_sec{0};Nanoseconds timing_source_stamp{0},planned_prediction_overhead{0};
  std::uint64_t pause_generation{0},owner_generation{0},type_generation{0};
  std::chrono::steady_clock::time_point pause_deadline,owner_deadline,type_deadline;
  bool handoff_failed{false};
  bool plan_valid{false};int servo_code{0};std::string servo_message,contact_object;std::set<std::string> known_objects;
  std::map<std::uint64_t,CachedSurface> measured_surfaces;
  bool contact_from_controller{false};
  bool possession_unknown{false};
  bool measured_empty_latched{false}, effort_contact_latched{false};
  std::optional<std::uint64_t> gripper_epoch;
  std::uint64_t gripper_epoch_joint_sequence{0};
  std::optional<std::uint64_t> grip_close_joint_sequence;
  std::uint64_t grip_command_generation{0};
  std::vector<std::string> measured_finger_joints;
  std::optional<std::vector<GraspSweepBox>> grasp_sweep_tool;
  bool owner_enabled{false},owner_pending{false};
  std::uint64_t contact_generation{0};std::chrono::steady_clock::time_point contact_deadline;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr servo_sub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client,owner_client;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr type_client;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client;
  rclcpp_action::Client<Grip>::SharedPtr grip_client;
  rclcpp_action::Client<Execute>::SharedPtr execute_client;
  rclcpp::Client<Planning>::SharedPtr planning_client;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client;
  rclcpp::TimerBase::SharedPtr timer;
  const SystemSettings & config()const{return settings.snapshot->get();}
  static rclcpp::Node::SharedPtr transport(const std::shared_ptr<ptc_motion::ArmGatewayNode> & gateway,const ptc_app::RobotDescriptionModel & description)
  {
    std::vector<rclcpp::Parameter> parameters;
    for(const auto & name:gateway->list_parameters({},100).names){
      if(name=="robot_description"||name=="robot_description_semantic"){continue;}
      parameters.push_back(gateway->get_parameter(name));
    }
    parameters.emplace_back("robot_description",description.urdf_xml);parameters.emplace_back("robot_description_semantic",read(gateway->settings().snapshot->get().motion.srdf_path));
    rclcpp::NodeOptions options;options.use_global_arguments(false).automatically_declare_parameters_from_overrides(true).parameter_overrides(parameters);
    return std::make_shared<rclcpp::Node>("moveit_transport",gateway->get_namespace(),options);
  }
  explicit MoveitBackendRuntime(const std::shared_ptr<ptc_motion::ArmGatewayNode> & gateway):settings(gateway->settings()),robot(ptc_app::load_robot_description(config())),node(transport(gateway,robot)),tf(node->get_clock()),listener(tf)
  {
    if(config().runtime.motion_backend!="moveit"){throw std::runtime_error("MoveIt backend requires runtime.motion_backend=moveit");}
    const auto & ep=settings.endpoints;
    joint_sub=node->create_subscription<sensor_msgs::msg::JointState>(ep.at("joint_states"),rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr message){std::lock_guard<std::mutex> lock(mutex);if(!joint_history.empty() && ptc_app::from_ros(message->header.stamp)<ptc_app::from_ros(joint_history.back().first.header.stamp)){joint_history.clear();}
      joints=*message;++joint_sequence;joint_history.emplace_back(joints,joint_sequence);// Keep enough measured samples for the existing 250 ms HOLD window
      // at 30/60/120 Hz. Eight samples only span 233 ms at 30 Hz and make
      // the stationary-position fallback impossible after a frame-rate change.
      while(joint_history.size()>64){joint_history.pop_front();}changed.notify_all();});
    servo_sub=node->create_subscription<moveit_msgs::msg::ServoStatus>(ep.at("servo_status"),rclcpp::QoS(5),[this](moveit_msgs::msg::ServoStatus::ConstSharedPtr message){std::lock_guard<std::mutex> lock(mutex);servo_code=message->code;servo_message=message->message;});
    twist_pub=node->create_publisher<geometry_msgs::msg::TwistStamped>(ep.at("servo_twist"),rclcpp::QoS(5));pause_client=node->create_client<std_srvs::srv::SetBool>(ep.at("servo_pause"));type_client=node->create_client<moveit_msgs::srv::ServoCommandType>(ep.at("servo_command_type"));
    if(config().runtime.operation_mode=="assumed_ng_demo"){owner_client=node->create_client<std_srvs::srv::SetBool>(ep.at("controller_owner_enable"));}
    get_scene_client=node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");apply_scene_client=node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    execute_client=rclcpp_action::create_client<Execute>(node,"execute_trajectory");
    planning_client=node->create_client<Planning>("plan_kinematic_path");
    validity_client=node->create_client<moveit_msgs::srv::GetStateValidity>("check_state_validity");
    grip_client=rclcpp_action::create_client<Grip>(node,ep.at("controller_gripper"));arm_client=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node,ep.at("controller_trajectory"));
  }
  void initialize()
  {
    executor.add_node(node);spin_thread=std::thread([this]{executor.spin();});
    Group::Options options(config().motion.planning_group,"robot_description",node->get_namespace());
    group=std::make_unique<Group>(node,options,std::shared_ptr<tf2_ros::Buffer>{},rclcpp::Duration::from_seconds(config().motion.planning_timeout_sec));
    group->setPoseReferenceFrame(config().frames.robot_base);group->setEndEffectorLink(config().frames.tcp);group->setPlanningTime(config().motion.planning_timeout_sec);
    const auto * finger=group->getRobotModel()->getJointModel(config().motion.gripper_joint);
    if(!finger || finger->getVariableCount()!=1)throw std::runtime_error("configured finger joint unavailable");
    measured_finger_joints.push_back(finger->getName());
    for(const auto * joint:group->getRobotModel()->getJointModels()){
      if(joint->getMimic()==finger && joint->getVariableCount()==1)measured_finger_joints.push_back(joint->getName());
    }
    timer=node->create_wall_timer(std::chrono::milliseconds(20),[this]{servo_tick();});
  }
  ~MoveitBackendRuntime(){if(timer){timer->cancel();}diagnostic_worker.shutdown();if(group){group->stop();}executor.cancel();if(spin_thread.joinable()){spin_thread.join();}}
  bool measured_stop(const sensor_msgs::msg::JointState & sample)const
  {
    if(sample.velocity.size()!=sample.name.size()){return false;}
    bool velocity_stopped=true;
    for(const auto & name:robot.arm_joint_names){const auto found=std::find(sample.name.begin(),sample.name.end(),name);if(found==sample.name.end()){return false;}const auto i=static_cast<std::size_t>(found-sample.name.begin());if(!std::isfinite(sample.velocity[i]))return false;if(std::abs(sample.velocity[i])>config().motion.joint_stop_velocity)velocity_stopped=false;}
    if(velocity_stopped)return true;
    // PhysX may retain solver velocity despite fixed measured joint positions.
    // In demo HOLD only, verify motion from the existing measured history. Never
    // turn a Stop ACK or a relaxed velocity threshold into measured stop.
    if(config().runtime.operation_mode!="assumed_ng_demo" || following ||
      !servo_paused || owner_enabled || owner_pending || joint_history.size()<4)return false;
    const auto end=ptc_app::from_ros(sample.header.stamp);
    std::vector<const sensor_msgs::msg::JointState *> history;
    for(const auto & entry:joint_history)if(ptc_app::from_ros(entry.first.header.stamp)<=end)history.push_back(&entry.first);
    if(history.size()<4)return false;
    // Evaluate the recent stationary window, not earlier motion that happens
    // to remain in the transport history. Keep at least four real samples.
    while(history.size()>4 && std::chrono::duration<double>(end-
      ptc_app::from_ros(history[1]->header.stamp)).count()>=.25){history.erase(history.begin());}
    const double span=std::chrono::duration<double>(end-ptc_app::from_ros(history.front()->header.stamp)).count();
    if(span<.25 || span>config().motion.joint_state_max_age_sec+1.)return false;
    for(const auto & name:robot.arm_joint_names){
      double previous=0,low=0,high=0;Nanoseconds previous_stamp{0};bool first=true;
      for(const auto * item:history){
        const auto found=std::find(item->name.begin(),item->name.end(),name);if(found==item->name.end())return false;
        const auto index=static_cast<std::size_t>(found-item->name.begin());if(index>=item->position.size() || !std::isfinite(item->position[index]))return false;
        const double position=item->position[index];const auto stamp=ptc_app::from_ros(item->header.stamp);
        if(first){low=high=position;first=false;}else{
          const double dt=std::chrono::duration<double>(stamp-previous_stamp).count();
          if(dt<=0 || std::abs(position-previous)/dt>config().motion.joint_stop_velocity)return false;
          low=std::min(low,position);high=std::max(high,position);
        }
        previous=position;previous_stamp=stamp;
      }
      if(high-low>config().motion.joint_position_tolerance_rad)return false;
    }
    return true;
  }
  // Called under mutex. Independent /clock and JointState delivery can arrive
  // in either order; select measured history without relabelling timestamps.
  std::pair<sensor_msgs::msg::JointState,std::uint64_t> clock_aligned_joints(const TimeContext & time)const
  {
    if(!valid_time(time)){return {joints,joint_sequence};}
    return clock_aligned_joints(time.source_now);
  }
  std::pair<sensor_msgs::msg::JointState,std::uint64_t> clock_aligned_joints(Nanoseconds source_now)const
  {
    const auto maximum=*seconds_to_duration(config().motion.joint_state_max_age_sec);
    const auto future=*seconds_to_duration(config().time.sensor_future_tolerance_sec);
    for(auto it=joint_history.rbegin();it!=joint_history.rend();++it){
      const auto stamp=ptc_app::from_ros(it->first.header.stamp);
      if(stamp.count()>=0 && stamp<=source_now+future && source_now-stamp<maximum){return *it;}
    }
    return {joints,joint_sequence}; // Existing readiness checks reject stale/future data.
  }
  ArmSnapshot arm(const TimeContext & time)
  {
    sensor_msgs::msg::JointState sample;bool paused,follow,owner_released,control_ok,tracking,stationary;{std::lock_guard<std::mutex> lock(mutex);sample=clock_aligned_joints(time).first;stationary=measured_stop(sample);control_ok=!handoff_failed;paused=servo_paused;follow=following;owner_released=!stopping&&!owner_enabled&&!owner_pending&&!pause_pending;
      tracking=following&&!stopping&&!handoff_failed&&contact_ready&&!contact_pending&&twist_type&&!type_pending&&owner_enabled&&!owner_pending&&!servo_paused&&!pause_pending;}
    ArmSnapshot out;out.epoch=time.epoch;out.state.base_frame=config().frames.robot_base;out.state.tcp_frame=config().frames.tcp;
    try{out.joint_timestamp=ptc_app::from_ros(sample.header.stamp);out.state.timestamp=out.joint_timestamp;
      for(const auto & name:robot.arm_joint_names){const auto found=std::find(sample.name.begin(),sample.name.end(),name);if(found==sample.name.end()){return out;}const auto i=static_cast<std::size_t>(found-sample.name.begin());if(i>=sample.position.size()||!std::isfinite(sample.position[i])){return out;}
        ArmJointState joint;joint.name=name;joint.position=sample.position[i];if(i<sample.velocity.size()){joint.velocity=sample.velocity[i];}if(i<sample.effort.size()){joint.effort=sample.effort[i];}out.state.joints.push_back(joint);}
      // Isaac joint stamps use float simulation time; TF uses double. Clamp
      // only the bounded float quantization discrepancy, retaining the actual TF stamp.
      const auto latest=tf.lookupTransform(config().frames.robot_base,config().frames.tcp,tf2::TimePointZero);
      const auto latest_stamp=ptc_app::from_ros(latest.header.stamp);
      const float sdk_seconds=static_cast<float>(std::chrono::duration<double>(out.joint_timestamp).count());
      const double sdk_ulp=std::nextafter(sdk_seconds,std::numeric_limits<float>::infinity())-sdk_seconds;
      const auto rounding_ns=static_cast<std::int64_t>(std::ceil(std::min(.001,std::max(.0001,.5*sdk_ulp+1e-7))*1e9));
      auto actual_tcp_stamp=out.joint_timestamp;
      const auto transform=[&]{
        if(std::abs((out.joint_timestamp-latest_stamp).count())<=rounding_ns){actual_tcp_stamp=latest_stamp;return latest;}
        try{return tf.lookupTransform(config().frames.robot_base,config().frames.tcp,rclcpp::Time(out.joint_timestamp.count(),node->get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(config().vision.tf_lookup_timeout_sec));}
        catch(const tf2::TransformException &){
          // A new TF can arrive after the first latest lookup. Recheck only
          // float-stamp quantization; never substitute an arbitrary latest pose.
          const auto arrived=tf.lookupTransform(config().frames.robot_base,config().frames.tcp,tf2::TimePointZero);
          const auto stamp=ptc_app::from_ros(arrived.header.stamp);
          if(std::abs((out.joint_timestamp-stamp).count())>rounding_ns){throw;}
          actual_tcp_stamp=stamp;return arrived;
        }
      }();
      out.state.tcp_pose={{transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z},{transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w}};
      out.tcp_timestamp=actual_tcp_stamp;out.state.timestamp=out.tcp_timestamp;out.tf_valid=is_valid_pose(out.state.tcp_pose);
      const bool trajectory_ready=arm_client->action_server_is_ready(),pause_ready=pause_client->service_is_ready(),type_ready=type_client->service_is_ready();
      out.controller_ready=control_ok&&trajectory_ready&&pause_ready&&type_ready;
      out.stopped=stationary&&paused&&!follow&&owner_released;out.state.status=status(out.tf_valid&&out.controller_ready);out.state.motion_state=out.stopped?ArmMotionState::kIdle:tracking?ArmMotionState::kFollowing:ArmMotionState::kMoving;
      if(!out.controller_ready){out.state.status.message="controller readiness: handoff_ok="+std::to_string(control_ok)+
        " trajectory="+std::to_string(trajectory_ready)+" pause="+std::to_string(pause_ready)+" command_type="+std::to_string(type_ready);}
    }catch(const std::exception & error){out.state.status.message=error.what();RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"TCP feedback TF: %s",error.what());}return out;
  }
  GripperState grip(const TimeContext & time)
  {
    std::lock_guard<std::mutex> lock(mutex);const auto aligned=clock_aligned_joints(time);const auto & sample=aligned.first;GripperState out;out.epoch=time.epoch;out.evidence_sequence=aligned.second;
    if(gripper_epoch && *gripper_epoch!=time.epoch){
      ++grip_command_generation;
      // Recheck measured Empty after the rewind. Preserve uncertainty from a
      // close or contact, but an idle open gripper needs no recovery command.
      possession_unknown=possession_unknown || contact_from_controller || effort_contact_latched;
      contact_from_controller=false;
      effort_contact_latched=false;
      measured_empty_latched=false;
      grip_close_joint_sequence.reset();
      gripper_epoch_joint_sequence=joint_sequence;
    }
    gripper_epoch=time.epoch;
    try{out.timestamp=ptc_app::from_ros(sample.header.stamp);const auto found=std::find(sample.name.begin(),sample.name.end(),config().motion.gripper_joint);if(found==sample.name.end()){return out;}const auto i=static_cast<std::size_t>(found-sample.name.begin());
      if(i>=sample.position.size()||!std::isfinite(sample.position[i])){return out;}
      out.width_m=sample.position[i]*config().motion.gripper_width_factor;
      // A closed simulated finger may overshoot its zero joint limit slightly.
      // Clamp simulated overshoot before classifying the measured aperture.
      // Retain raw JointState values for diagnostics and allow recovery opening.
      if(config().runtime.operation_mode=="assumed_ng_demo" && out.width_m<0){
        RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"negative simulated finger aperture %.6f m; report closed aperture, retain raw joints",out.width_m);
        out.width_m=0.;
      }
      std::vector<ptc_motion_moveit::FingerPositionSample> positions;
      const auto end=out.timestamp;
      if(!source_is_fresh(end,time,config().motion.joint_state_max_age_sec,config().time.sensor_future_tolerance_sec)){
        measured_empty_latched=false;
        effort_contact_latched=false;
        return out;
      }
      const auto & thresholds=config().grasp.finger_effort_thresholds_n;
      bool low_force=thresholds && measured_finger_joints.size()==2;
      bool high_force=low_force,force_window_valid=low_force;
      bool after_close_command=true;
      bool unexpected_force=false;
      std::vector<double> latest_finger_effort;
      for(auto it=joint_history.rbegin();it!=joint_history.rend();++it){
        if(it->second<=gripper_epoch_joint_sequence)break;
        const auto stamp=ptc_app::from_ros(it->first.header.stamp);
        if(stamp>end)continue;
        ptc_motion_moveit::FingerPositionSample point{std::chrono::duration<double>(stamp-end).count(),{}};
        after_close_command=after_close_command && (!grip_close_joint_sequence || it->second>*grip_close_joint_sequence);
        for(const auto & name:measured_finger_joints){
          const auto joint=std::find(it->first.name.begin(),it->first.name.end(),name);
          const auto index=static_cast<std::size_t>(joint-it->first.name.begin());
          if(joint==it->first.name.end() || index>=it->first.position.size()){
            out.status=status(false);out.status.message="finger position history missing joint: "+name;return out;
          }
          point.positions.push_back(it->first.position[index]);
          const auto & efforts=it->first.effort;
          if(positions.empty()){
            latest_finger_effort.push_back(index<efforts.size()?efforts[index]:kUnset);
          }
          if(thresholds){
            const bool valid=index<efforts.size()&&std::isfinite(efforts[index]);
            force_window_valid=force_window_valid && valid;
            low_force=low_force && valid && std::abs(efforts[index])<=(*thresholds)[0];
            high_force=high_force && valid && grip_close_joint_sequence && it->second>*grip_close_joint_sequence &&
              -efforts[index]>=(*thresholds)[1];
            unexpected_force=unexpected_force || (valid && -efforts[index]>=(*thresholds)[1]);
          }
        }
        positions.push_back(std::move(point));
        if(std::chrono::duration<double>(end-stamp).count()>=.1)break;
      }
      std::reverse(positions.begin(),positions.end());
      const auto motion=ptc_motion_moveit::estimate_finger_motion(positions,config().motion.joint_stop_velocity);
      if(!motion){out.status=status(false);out.status.message="finger stop unknown: need fresh increasing position history covering 100 ms";return out;}
      out.stopped=motion->stopped;
      if(i<sample.velocity.size() && std::isfinite(sample.velocity[i]) &&
        std::abs(sample.velocity[i]-motion->velocity_mps.front())>config().motion.joint_stop_velocity){
        RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),2000,
          "finger velocity disagreement: joint=%s raw=%.6f position_rate=%.6f m/s; stopped from all measured fingers=%d",
          config().motion.gripper_joint.c_str(),sample.velocity[i],motion->velocity_mps.front(),out.stopped);
      }
      // A stopped closed-empty sample invalidates the earlier grasp result
      // for later samples, including a partially completed opening.
      const bool closed_empty=after_close_command &&
        gripper_measured_closed_empty(out,config());
      ptc_motion_moveit::clear_latched_possession_if_closed_empty(
        closed_empty,contact_from_controller,possession_unknown);
      constexpr double full_open_joint_tolerance_m=.001;
      const bool full_open_empty=ptc_motion_moveit::measured_full_open_empty(
        out.stopped,low_force,positions,config().grasp.open_width_m/config().motion.gripper_width_factor,
        full_open_joint_tolerance_m);
      if(full_open_empty){
        contact_from_controller=false;
        possession_unknown=false;
        effort_contact_latched=false;
      }
      const auto minimum_width=config().grasp.minimum_holding_width_m;
      const bool above_minimum=minimum_width ? out.width_m>=*minimum_width : out.width_m>config().grasp.close_width_m;
      const bool valid_aperture=!closed_empty && above_minimum && out.width_m<config().grasp.open_width_m;
      const bool prior_effort_contact=effort_contact_latched;
      if(thresholds && out.stopped){
        if(!valid_aperture || !force_window_valid || low_force)effort_contact_latched=false;
        else if(high_force)effort_contact_latched=true;
      }
      out.contact_confirmed=valid_aperture &&
        (thresholds ? effort_contact_latched && force_window_valid : contact_from_controller);
      if(unexpected_force && !closed_empty)measured_empty_latched=false;
      const bool open_empty=ptc_motion_moveit::measured_empty(out.stopped,
        contact_from_controller || possession_unknown,out.width_m,config().grasp.open_width_m,config().motion.position_tolerance_m);
      out.empty_confirmed=!out.contact_confirmed && (closed_empty ||
        (out.stopped && (!thresholds || low_force) && (open_empty || measured_empty_latched)));
      if(out.empty_confirmed)measured_empty_latched=true;
      if(thresholds && latest_finger_effort.size()==2 && prior_effort_contact!=effort_contact_latched){
        RCLCPP_INFO(node->get_logger(),"finger effort contact=%d aperture_m=%.4f left_n=%.3f right_n=%.3f window_valid=%d",
          effort_contact_latched,out.width_m,latest_finger_effort[0],latest_finger_effort[1],force_window_valid);
      }
      out.status=status(grip_client->action_server_is_ready()&&out.width_m>=0);return out;
    }catch(...){return out;}
  }
  void request_pause(bool pause)
  {
    if(servo_paused==pause||pause_pending||!pause_client->service_is_ready()){return;}pause_pending=true;const auto version=++pause_generation;
    pause_deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config().motion.stop_confirmation_timeout_sec));auto request=std::make_shared<std_srvs::srv::SetBool::Request>();request->data=pause;
    pause_client->async_send_request(request,[this,pause,version](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
      std::lock_guard<std::mutex> lock(mutex);if(version!=pause_generation||!pause_pending||std::chrono::steady_clock::now()>pause_deadline){return;}
      pause_pending=false;if(future.get()->success){servo_paused=pause;}else{handoff_failed=true;following=false;stopping=true;}
    });
  }
  void request_owner(bool enable)
  {
    if(owner_enabled==enable){return;}
    if(!owner_client){owner_enabled=enable;return;}
    if(owner_pending || !owner_client->service_is_ready()){return;}
    owner_pending=true;const auto version=++owner_generation;
    owner_deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config().motion.stop_confirmation_timeout_sec));auto request=std::make_shared<std_srvs::srv::SetBool::Request>();request->data=enable;
    owner_client->async_send_request(request,[this,enable,version](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response){
      std::lock_guard<std::mutex> lock(mutex);if(version!=owner_generation||!owner_pending||std::chrono::steady_clock::now()>owner_deadline){return;}owner_pending=false;
      if(response.get()->success){owner_enabled=enable;}else{handoff_failed=true;following=false;stopping=true;}
    });
  }
  std::vector<std::string> grasp_contact_links() const
  {
    auto links=config().motion.touch_links;
    if(config().runtime.operation_mode!="assumed_ng_demo" || !config().grasp.moving_intercept)return links;
    const auto * finger=group->getRobotModel()->getJointModel(config().motion.gripper_joint);
    if(!finger || !finger->getParentLinkModel())throw std::runtime_error("gripper palm link unavailable");
    std::vector<const moveit::core::LinkModel *> rigid{finger->getParentLinkModel()};
    for(std::size_t link=0;link<rigid.size();++link){
      const auto & name=rigid[link]->getName();
      if(std::find(links.begin(),links.end(),name)==links.end())links.push_back(name);
      for(const auto * joint:rigid[link]->getChildJointModels()){
        if(joint->getType()==moveit::core::JointModel::FIXED && joint->getChildLinkModel())rigid.push_back(joint->getChildLinkModel());
      }
    }
    return links;
  }
  void request_contact(const std::string & id,bool allow)
  {
    if(contact_pending||!get_scene_client->service_is_ready()||!apply_scene_client->service_is_ready()){return;}contact_pending=true;contact_ready=false;
    const auto version=++contact_generation;contact_deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config().motion.planning_timeout_sec));
    if(allow){contact_object=id;} // Also revoke an in-flight allowance on stop.
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();request->components.components=request->components.ALLOWED_COLLISION_MATRIX | request->components.WORLD_OBJECT_NAMES;
    get_scene_client->async_send_request(request,[this,id,allow,version](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future){
      std::lock_guard<std::mutex> lock(mutex);if(version!=contact_generation||!contact_pending){return;}
      const auto & current=future.get()->scene;auto matrix=current.allowed_collision_matrix;
      const bool demo=config().runtime.operation_mode=="assumed_ng_demo" && config().grasp.moving_intercept;
      auto contact_links=grasp_contact_links();std::vector<std::string> contact_targets;
      if(allow)contact_targets.push_back(id);
      try{
        if(demo){
          // Keep the existing commissioning scope: tool assembly against
          // tracked models only; physical contacts still resolve in Isaac.
          if(allow || demo)for(const auto & object:current.world.collision_objects){
            if(object.id.rfind("tracked_",0)==0 && std::find(contact_targets.begin(),contact_targets.end(),object.id)==contact_targets.end())contact_targets.push_back(object.id);
          }
        }
        replace_grasp_touch_permissions(matrix,contact_links,contact_targets);
      }
      catch(const std::exception & error){contact_pending=false;contact_ready=false;following=false;stopping=true;
        RCLCPP_ERROR(node->get_logger(),"grasp contact policy rejected: %s",error.what());return;}
      auto apply=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();apply->scene.is_diff=true;apply->scene.robot_state.is_diff=true;apply->scene.allowed_collision_matrix=matrix;
      apply_scene_client->async_send_request(apply,[this,id,allow,version,demo,pair_count=contact_links.size()*contact_targets.size()](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture result){std::lock_guard<std::mutex> guard(mutex);if(version!=contact_generation){return;}contact_pending=false;contact_ready=result.get()->success;
        if(contact_ready){contact_object=allow?id:std::string{};
          RCLCPP_INFO(node->get_logger(),"grasp contact policy: target=%s pairs=%zu mode=%s; static/self/other robot links remain checked; physical contact and Holding evidence unchanged",allow?id.c_str():"none",pair_count,demo?"demo rigid end effector with observed rabbits":"formal selected target/fingers");}
      });
    });
  }
  void servo_tick()
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto wall=std::chrono::steady_clock::now();
    if((pause_pending&&wall>=pause_deadline)||(owner_pending&&wall>=owner_deadline)||(type_pending&&wall>=type_deadline)){
      if(pause_pending&&wall>=pause_deadline){++pause_generation;pause_pending=false;servo_paused=false;}
      if(owner_pending&&wall>=owner_deadline){++owner_generation;owner_pending=false;owner_enabled=true;}
      if(type_pending&&wall>=type_deadline){++type_generation;type_pending=false;}
      handoff_failed=true;following=false;stopping=true;
      RCLCPP_WARN(node->get_logger(),"Servo handoff timed out; reassert pause and measured-position HOLD");
    }
    if(contact_pending&&wall>=contact_deadline){++contact_generation;contact_pending=false;following=false;stopping=true;}
    // A handoff failure must not bypass output shutdown. Disconnect Servo
    // from the bridge's measured-position HOLD before waiting for zero speed.
    if(handoff_failed){following=false;stopping=true;}
    if(!following){
      if(stopping){
        geometry_msgs::msg::TwistStamped zero;zero.header.frame_id=config().frames.world;zero.header.stamp=node->now();twist_pub->publish(zero);
        request_pause(true);request_owner(false);
        if(!contact_pending&&!contact_object.empty()){request_contact("",false);}
        const auto source_now=Nanoseconds{node->now().nanoseconds()};
        const auto sample=clock_aligned_joints(source_now).first;
        const auto stamp=ptc_app::from_ros(sample.header.stamp);
        const double age=std::chrono::duration<double>(source_now-stamp).count();
        const bool fresh=joint_sequence>0&&stamp.count()>=0&&age>=-config().time.sensor_future_tolerance_sec&&age<config().motion.joint_state_max_age_sec;
        if(servo_paused&&!pause_pending&&!owner_enabled&&!owner_pending&&!contact_pending&&contact_object.empty()&&fresh&&measured_stop(sample)){
          stopping=false;handoff_failed=false;
          RCLCPP_INFO(node->get_logger(),"stop confirmed: Servo paused, bridge HOLD acknowledged, fresh measured joints stopped");
          changed.notify_all();
        }else{
          double peak_velocity=0.;std::string peak_joint="missing_velocity";
          for(std::size_t i=0;i<sample.name.size()&&i<sample.velocity.size();++i){if(std::find(robot.arm_joint_names.begin(),robot.arm_joint_names.end(),sample.name[i])==robot.arm_joint_names.end())continue;
            if(!std::isfinite(sample.velocity[i])||std::abs(sample.velocity[i])>peak_velocity){peak_velocity=std::abs(sample.velocity[i]);peak_joint=sample.name[i];}}
          RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,
            "stop awaiting measured feedback: paused=%d pause_pending=%d owner=%d owner_pending=%d fresh=%d stopped=%d sample_age=%.6f latest_age=%.6f peak_joint=%s peak_velocity=%.6f threshold=%.6f",
            servo_paused,pause_pending,owner_enabled,owner_pending,fresh,measured_stop(sample),age,
            std::chrono::duration<double>(source_now-ptc_app::from_ros(joints.header.stamp)).count(),peak_joint.c_str(),peak_velocity,config().motion.joint_stop_velocity);
        }
      }else if(!servo_paused){request_pause(true);}else if(owner_enabled){request_owner(false);}return;
    }
    const auto contact_target=target.producer=="grasp"?"tracked_"+std::to_string(target.key.track_id):std::string{};
    if(!contact_ready||contact_object!=contact_target){request_contact(contact_target,!contact_target.empty());return;}
    if(!twist_type){if(!type_pending&&type_client->service_is_ready()){type_pending=true;const auto version=++type_generation;type_deadline=wall+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config().motion.stop_confirmation_timeout_sec));auto request=std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();request->command_type=request->TWIST;type_client->async_send_request(request,[this,version](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture response){std::lock_guard<std::mutex> lock(mutex);if(version!=type_generation||!type_pending||std::chrono::steady_clock::now()>type_deadline){return;}type_pending=false;twist_type=response.get()->success;if(!twist_type){handoff_failed=true;following=false;stopping=true;}});}return;}
    if(!owner_enabled){request_owner(true);return;}
    if(servo_paused){request_pause(false);return;}
    if(servo_code==-1||servo_code==2||servo_code==5||servo_code==6){RCLCPP_WARN(node->get_logger(),"Servo protective stop, status=%d: %s",servo_code,servo_message.c_str());
      const auto sample=clock_aligned_joints(Nanoseconds{node->now().nanoseconds()}).first;const auto command=target;const auto code=servo_code;
      if(!diagnostic_worker.enqueue([this,sample,command,code]{save_protective_stop_diagnostic(sample,command,code);})){RCLCPP_WARN(node->get_logger(),"protective-stop diagnostic queue full; stop remains active");}
      following=false;stopping=true;return;}
    try{
      const auto transform=tf.lookupTransform(config().frames.world,config().frames.tcp,tf2::TimePointZero);
      if((node->now()-rclcpp::Time(transform.header.stamp,node->get_clock()->get_clock_type())).seconds()>config().vision.frame_max_age_sec){following=false;stopping=true;return;}
      const Vector3d error{target.target.position.x_m-transform.transform.translation.x,target.target.position.y_m-transform.transform.translation.y,target.target.position.z_m-transform.transform.translation.z};
      Eigen::Quaterniond desired(target.target.orientation.w,target.target.orientation.x,target.target.orientation.y,target.target.orientation.z);
      Eigen::Quaterniond measured(transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z);
      Eigen::Quaterniond delta=desired*measured.conjugate();if(delta.w()<0){delta.coeffs()*=-1;}const Eigen::AngleAxisd angle(delta.normalized());
      geometry_msgs::msg::TwistStamped command;command.header.frame_id=config().frames.world;command.header.stamp=node->now();
      // Keep rapid interception, then continuously soften correction near
      // contact. Feedforward and ownership never stop or switch here. Reuse
      // the configured pregrasp clearance instead of adding a mode/threshold.
      const bool commissioning_grasp=target.producer=="grasp" && config().runtime.operation_mode=="assumed_ng_demo" &&
        config().grasp.moving_intercept;
      const double gain_scale=commissioning_grasp?.5+.5*std::clamp(vector_norm(error)/(2.*config().grasp.pregrasp_clearance_m),0.,1.):1.;
      const double position_gain=config().motion.servo_position_gain*gain_scale;
      const double orientation_gain=config().motion.servo_orientation_gain*gain_scale;
      Eigen::Vector3d linear(target.linear_velocity.x_m+position_gain*error.x_m,target.linear_velocity.y_m+position_gain*error.y_m,target.linear_velocity.z_m+position_gain*error.z_m);
      Eigen::Vector3d angular=Eigen::Vector3d(target.angular_velocity.x_m,target.angular_velocity.y_m,target.angular_velocity.z_m)+angle.axis()*angle.angle()*orientation_gain;
      if(linear.norm()>config().motion.max_follow_linear_speed_mps){linear*=config().motion.max_follow_linear_speed_mps/linear.norm();}
      if(angular.norm()>config().motion.max_follow_angular_speed_radps){angular*=config().motion.max_follow_angular_speed_radps/angular.norm();}
      command.twist.linear.x=linear.x();command.twist.linear.y=linear.y();command.twist.linear.z=linear.z();command.twist.angular.x=angular.x();command.twist.angular.y=angular.y();command.twist.angular.z=angular.z();twist_pub->publish(command);
    }catch(...){following=false;stopping=true;}
  }
  moveit::core::RobotStatePtr measured_robot_state(Nanoseconds * measured_stamp=nullptr)
  {
    sensor_msgs::msg::JointState sample;{std::lock_guard<std::mutex> lock(mutex);sample=joints;}
    if(measured_stamp){*measured_stamp=sample.header.stamp.sec>=0?ptc_app::from_ros(sample.header.stamp):Nanoseconds{0};}
    if(sample.name.size()!=sample.position.size()){return {};}
    auto state=std::make_shared<moveit::core::RobotState>(group->getRobotModel());state->setToDefaultValues();
    std::set<std::string> observed;
    const auto & variables=group->getRobotModel()->getVariableNames();
    for(std::size_t i=0;i<sample.name.size();++i){
      if(std::find(variables.begin(),variables.end(),sample.name[i])!=variables.end()&&std::isfinite(sample.position[i])){
        state->setVariablePosition(sample.name[i],sample.position[i]);observed.insert(sample.name[i]);}}
    const auto * planning_group=group->getRobotModel()->getJointModelGroup(config().motion.planning_group);
    if(!planning_group||std::any_of(planning_group->getVariableNames().begin(),planning_group->getVariableNames().end(),
        [&](const auto & name){return !observed.count(name);})||!observed.count(config().motion.gripper_joint)){
      RCLCPP_WARN(node->get_logger(),"measured_robot_state: joint snapshot is missing a planning or gripper variable");return {};}
    state->update();return state;
  }
  bool select_collision_valid_joint_target(const moveit::core::RobotStatePtr & start,const Pose3d & target,
    moveit::core::RobotStatePtr * selected=nullptr,
    std::chrono::steady_clock::time_point deadline_limit=std::chrono::steady_clock::time_point::max(),std::string * failure_reason=nullptr,
    const planning_scene::PlanningScenePtr & scene_snapshot={},double * minimum_joint_margin=nullptr,
    const ptc_motion::MotionCheckRequest * contact_request=nullptr)
  {
    if(failure_reason){failure_reason->clear();}
    if(!start||(!scene_snapshot&&!validity_client->service_is_ready())){if(failure_reason){*failure_reason="state_validity_unavailable";}return false;}
    const auto * model=start->getJointModelGroup(config().motion.planning_group);if(!model){if(failure_reason){*failure_reason="missing_joint_group";}return false;}
    const Eigen::Quaterniond q(target.orientation.w,target.orientation.x,target.orientation.y,target.orientation.z);
    Eigen::Isometry3d goal=Eigen::Isometry3d::Identity();goal.linear()=q.normalized().toRotationMatrix();goal.translation()=Eigen::Vector3d(target.position.x_m,target.position.y_m,target.position.z_m);
    goal=start->getGlobalLinkTransform(config().frames.robot_base)*goal;
    const auto issued=cancellation.load();const auto local_deadline=std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config().motion.planning_timeout_sec*.45));
    const auto deadline=std::min(deadline_limit,local_deadline);
    const bool enforce_joint_reserve=!(config().grasp.moving_intercept);
    RCLCPP_INFO(node->get_logger(),"select_collision_valid_joint_target: Servo joint reserve enforced=%d; bounds/singularity/collision remain checked",enforce_joint_reserve);
    moveit::core::RobotStatePtr best;double best_score=INFINITY,best_condition=INFINITY;
    unsigned ik_rejected=0,singularity_rejected=0,margin_rejected=0,collision_rejected=0;bool validity_timed_out=false;
    std::string collision_detail;
    std::vector<double> original;start->copyJointGroupPositions(model,original);
    for(unsigned attempt=0;attempt<24&&std::chrono::steady_clock::now()<deadline;++attempt){
      if(issued!=cancellation.load()){return false;}auto candidate=std::make_shared<moveit::core::RobotState>(*start);
      if(attempt){auto seed=original;const auto & names=model->getVariableNames();for(std::size_t j=0;j<seed.size();++j){const auto & bound=group->getRobotModel()->getVariableBounds(names[j]);const double fraction=std::fmod((attempt+1)*(j+1)*.6180339887498948,1.);if(bound.position_bounded_){seed[j]=bound.min_position_+fraction*(bound.max_position_-bound.min_position_);}}candidate->setJointGroupPositions(model,seed);candidate->update();}
      if(!candidate->setFromIK(model,goal,config().frames.tcp,.025)||!candidate->satisfiesBounds(model)){++ik_rejected;continue;}
      candidate->update();Eigen::MatrixXd jacobian;if(!candidate->getJacobian(model,candidate->getLinkModel(config().frames.tcp),Eigen::Vector3d::Zero(),jacobian)){continue;}
      const auto singular=Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian).singularValues();if(singular.size()<6||singular[5]<=1e-8){++singularity_rejected;continue;}
      const double condition=singular[0]/singular[5];std::vector<double> solved;candidate->copyJointGroupPositions(model,solved);double distance=0;for(std::size_t i=0;i<solved.size();++i){distance+=std::abs(solved[i]-original[i]);}
      double margin_penalty=0;bool reserved=true;const auto & variables=model->getVariableNames();
      for(std::size_t i=0;i<solved.size();++i){const auto & bound=group->getRobotModel()->getVariableBounds(variables[i]);if(!bound.position_bounded_){continue;}
        const double range=bound.max_position_-bound.min_position_;const double margin=std::min(solved[i]-bound.min_position_,bound.max_position_-solved[i]);
        // Formal observation follows with Servo; its reserve remains a hard gate.
        // Motion tests execute bounded joint trajectories without entering Servo.
        // Actual bounds were checked above; margin remains a preference in scoring.
        if(enforce_joint_reserve && margin<std::min(.45,.10*range)){
          if(!margin_rejected){RCLCPP_WARN(node->get_logger(),"IK joint margin rejected: joint=%s q=%.5f bounds=[%.5f,%.5f] margin=%.5f required=%.5f",variables[i].c_str(),solved[i],bound.min_position_,bound.max_position_,margin,std::min(.45,.10*range));}
          reserved=false;break;
        }margin_penalty+=.20/(margin/range+.01);}
      if(!reserved){++margin_rejected;continue;}const double score=distance+.5*std::max(0.,condition-12.)+margin_penalty;if(score>=best_score){continue;}
      bool valid=false;
      if(scene_snapshot){
        collision_detection::CollisionRequest collision_request;collision_request.group_name="";
        collision_request.contacts=true;collision_request.max_contacts=contact_request?128:4;collision_request.max_contacts_per_pair=contact_request?128:1;
        collision_detection::CollisionResult collision_result;scene_snapshot->checkCollision(collision_request,collision_result,*candidate);valid=!collision_result.collision;
        if(collision_result.collision&&contact_request&&collision_result.contact_count>0&&collision_result.contact_count<128){
          valid=true;
          for(const auto & pair:collision_result.contacts){for(const auto & contact:pair.second){
            if(!is_expected_grasp_contact(*contact_request,contact,scene_snapshot->getPlanningFrame())){valid=false;break;}
          }if(!valid)break;}
        }
        if(collision_result.collision&&collision_detail.empty()){
          for(const auto & pair:collision_result.contacts){if(!pair.second.empty()){
            const auto & contact=pair.second.front();collision_detail=contact.body_name_1+" <-> "+contact.body_name_2+
              " penetration_m="+std::to_string(contact.depth);break;}}
        }
      }else{
        auto check=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();check->group_name="";moveit::core::robotStateToRobotStateMsg(*candidate,check->robot_state);
        auto response=validity_client->async_send_request(check);if(response.wait_until(deadline)!=std::future_status::ready){validity_client->remove_pending_request(response);validity_timed_out=true;RCLCPP_WARN(node->get_logger(),"select_collision_valid_joint_target: GetStateValidity response deadline expired");break;}
        const auto result=response.get();valid=result->valid;
        if(!valid&&collision_rejected==0){
          RCLCPP_WARN(node->get_logger(),"IK state validity rejected: contacts=%zu",result->contacts.size());
          for(std::size_t k=0;k<std::min<std::size_t>(3,result->contacts.size());++k){const auto & contact=result->contacts[k];
            RCLCPP_WARN(node->get_logger(),"IK collision: %s <-> %s penetration=%.5f m",contact.contact_body_1.c_str(),contact.contact_body_2.c_str(),contact.depth);}
        }
      }
      if(valid){best=candidate;best_score=score;best_condition=condition;}
      else{++collision_rejected;}
    }
    if(!best){
      RCLCPP_WARN(node->get_logger(),"select_collision_valid_joint_target failed: IK/bounds=%u singularity=%u joint_margin=%u collision=%u target_base=(%.4f,%.4f,%.4f)",
        ik_rejected,singularity_rejected,margin_rejected,collision_rejected,target.position.x_m,target.position.y_m,target.position.z_m);
      if(failure_reason){*failure_reason=validity_timed_out?"state_validity_timeout":collision_rejected?
        (collision_detail.empty()?"collision":"collision: "+collision_detail):singularity_rejected?"singularity":
        margin_rejected?"joint_margin":ik_rejected?"ik_or_joint_bounds":"deadline_or_cancel";}
      return false;
    }RCLCPP_INFO(node->get_logger(),"selected collision-valid IK, Jacobian condition %.2f",best_condition);if(!group->setJointValueTarget(*best)){if(failure_reason){*failure_reason="joint_target_rejected";}return false;}if(selected){*selected=best;}
    if(minimum_joint_margin){*minimum_joint_margin=std::numeric_limits<double>::infinity();const auto * jmg=best->getJointModelGroup(config().motion.planning_group);
      if(jmg){for(const auto & name:jmg->getVariableNames()){const auto & bound=best->getRobotModel()->getVariableBounds(name);if(bound.position_bounded_){const double q=best->getVariablePosition(name);*minimum_joint_margin=std::min(*minimum_joint_margin,std::min(q-bound.min_position_,bound.max_position_-q));}}}}
    return true;
  }
  bool plan_collision_checked_trajectory(Group::Plan & candidate)
  {
    if(!planning_client->service_is_ready()){return false;}
    const auto issued=cancellation.load();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(config().motion.planning_timeout_sec);
    auto request=std::make_shared<Planning::Request>();group->constructMotionPlanRequest(request->motion_plan_request);
    // Leave bounded transport/result time around the actual planner budget.
    request->motion_plan_request.allowed_planning_time=config().motion.planning_timeout_sec*.8;
    auto response=planning_client->async_send_request(request);
    while(response.wait_for(std::chrono::milliseconds(10))!=std::future_status::ready){
      if(issued!=cancellation.load()||std::chrono::steady_clock::now()>=deadline){planning_client->remove_pending_request(response);return false;}
    }
    const auto result=response.get()->motion_plan_response;
    if(issued!=cancellation.load()||result.error_code.val!=moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
      RCLCPP_WARN(node->get_logger(),"planning failed: %d",result.error_code.val);return false;
    }
    candidate.trajectory=result.trajectory;candidate.start_state=result.trajectory_start;candidate.planning_time=result.planning_time;
    return !candidate.trajectory.joint_trajectory.points.empty();
  }
  std::optional<StampedPose3d> predict_arrival_tcp_pose(const MoveToPoseRequest & request,Nanoseconds source_time)
  {
    const auto target=predict_move_target(request,source_time);if(!target){
      RCLCPP_WARN(node->get_logger(),"predict_arrival_tcp_pose rejected: valid_request=%d elapsed_source=%.6f horizon=%.6f s",
        is_valid_move_to_pose_request(request),std::chrono::duration<double>(source_time-request.target_pose.timestamp).count(),
        std::chrono::duration<double>(request.prediction_horizon).count());return std::nullopt;
    }
    try{
      const auto base=tf.lookupTransform(config().frames.world,config().frames.robot_base,tf2::TimePointZero);
      Pose3d base_world{{base.transform.translation.x,base.transform.translation.y,base.transform.translation.z},
        {base.transform.rotation.x,base.transform.rotation.y,base.transform.rotation.z,base.transform.rotation.w}};
      const auto world=compose_pose(base_world,target->pose);
      if(!world||!inside_workspace(*world,config().motion)){
        if(world){RCLCPP_WARN(node->get_logger(),"predict_arrival_tcp_pose rejected: predicted world TCP outside workspace xyz=(%.4f,%.4f,%.4f)",
          world->position.x_m,world->position.y_m,world->position.z_m);}
        else{RCLCPP_WARN(node->get_logger(),"predict_arrival_tcp_pose rejected: invalid base/TCP composition");}
        return std::nullopt;
      }return target;
    }catch(const tf2::TransformException & e){RCLCPP_WARN(node->get_logger(),"predict_arrival_tcp_pose rejected: base TF %s",e.what());return std::nullopt;}
  }
  void save_protective_stop_diagnostic(const sensor_msgs::msg::JointState & sample,const Command & command,int code,const std::string & trigger="Servo protective stop")
  {
    try{
      if(!get_scene_client->service_is_ready())throw std::runtime_error("planning scene service unavailable");
      auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();request->components.components=1023;
      auto future=get_scene_client->async_send_request(request);
      if(future.wait_for(std::chrono::duration<double>(config().motion.planning_timeout_sec))!=std::future_status::ready){get_scene_client->remove_pending_request(future);throw std::runtime_error("planning scene diagnostic timed out");}
      planning_scene::PlanningScene scene(group->getRobotModel());scene.setPlanningSceneMsg(future.get()->scene);
      auto state=scene.getCurrentState();
      if(sample.name.size()!=sample.position.size())throw std::runtime_error("invalid measured joint sample");
      const auto & variables=state.getRobotModel()->getVariableNames();
      for(std::size_t i=0;i<sample.name.size();++i){if(std::find(variables.begin(),variables.end(),sample.name[i])!=variables.end())state.setVariablePosition(sample.name[i],sample.position[i]);}state.update();
      collision_detection::DistanceRequest distance;distance.group_name="";distance.enableGroup(state.getRobotModel());
      distance.acm=&scene.getAllowedCollisionMatrix();distance.enable_nearest_points=true;distance.enable_signed_distance=true;distance.distance_threshold=.20;
      collision_detection::DistanceResult self,world;scene.getCollisionEnv()->distanceSelf(distance,self,state);scene.getCollisionEnv()->distanceRobot(distance,world,state);
      const auto & nearest=self.minimum_distance.distance<world.minimum_distance.distance?self.minimum_distance:world.minimum_distance;
      const auto * jmg=state.getJointModelGroup(config().motion.planning_group);Eigen::MatrixXd jacobian;
      double condition=INFINITY;if(state.getJacobian(jmg,state.getLinkModel(config().frames.tcp),Eigen::Vector3d::Zero(),jacobian)){
        const auto singular=Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian).singularValues();if(singular.size()>=6&&singular[5]>1e-12)condition=singular[0]/singular[5];}
      const auto log_distance=[&](const std::string & name,const collision_detection::DistanceResult & result){
        const auto & pair=result.minimum_distance;
        RCLCPP_WARN(this->node->get_logger(),"grasp-stop %s distance: %s/%s distance_m=%.6f collision=%d",name.c_str(),pair.link_names[0].c_str(),pair.link_names[1].c_str(),pair.link_names[0].empty()?NAN:pair.distance,result.collision);
      };log_distance("self",self);log_distance("world",world);
      RCLCPP_WARN(node->get_logger(),"grasp-stop analysis: trigger=%s status=%d cycle=%lu command=%lu track=%lu pair=%s/%s distance_m=%.6f condition=%.2f (planning model, not physical contact)",trigger.c_str(),code,command.key.cycle_id,command.command_id,command.key.track_id,nearest.link_names[0].c_str(),nearest.link_names[1].c_str(),nearest.link_names[0].empty()?NAN:nearest.distance,condition);
    }catch(const std::exception & error){RCLCPP_WARN(node->get_logger(),"protective-stop analysis unavailable: %s",error.what());}
  }
  bool planning_prediction_near(const Pose3d & planned,const Pose3d & latest)
  {
    const bool moving_intercept=config().grasp.moving_intercept;
    const double position=moving_intercept?config().inspection.view_target_position_tolerance_m:config().motion.position_tolerance_m;
    const double orientation=moving_intercept?config().inspection.view_target_orientation_tolerance_rad:config().motion.orientation_tolerance_rad;
    const bool near=pose_near(planned,latest,position,orientation);
    if(!near){
      const double error=vector_norm({latest.position.x_m-planned.position.x_m,latest.position.y_m-planned.position.y_m,latest.position.z_m-planned.position.z_m});
      RCLCPP_WARN(node->get_logger(),"planning_prediction_near: position_error_m=%.6f limit_m=%.6f orientation_limit_rad=%.6f",error,position,orientation);
    }
    return near;
  }
  static double timing_median(const std::deque<double> & samples,double fallback)
  {if(samples.empty())return fallback;std::vector<double> values(samples.begin(),samples.end());std::sort(values.begin(),values.end());return values[values.size()/2];}
  static void record_timing_sample(std::deque<double> & samples,double value)
  {if(!std::isfinite(value)||value<0)return;samples.push_back(value);while(samples.size()>8)samples.pop_front();}
  std::optional<double> estimate_joint_approach_duration(const moveit::core::RobotState & start,
    const moveit::core::RobotState & goal,const MoveToPoseRequest & request)
  {
    const auto * model=start.getJointModelGroup(config().motion.planning_group);if(!model)return std::nullopt;
    double seconds=0;
    for(const auto & name:model->getVariableNames()){
      const auto & bounds=start.getRobotModel()->getVariableBounds(name);
      if(!bounds.velocity_bounded_||bounds.max_velocity_<=0)return std::nullopt;
      const double acceleration=bounds.acceleration_bounded_?bounds.max_acceleration_*request.max_acceleration_scale:0.;
      const auto duration=estimate_joint_travel_duration(std::abs(goal.getVariablePosition(name)-start.getVariablePosition(name)),
        bounds.max_velocity_*request.max_velocity_scale,acceleration);
      if(!duration){return std::nullopt;}seconds=std::max(seconds,*duration);
    }
    return seconds;
  }
  std::optional<double> estimate_approach_duration(const moveit::core::RobotStatePtr & start,
    const Pose3d & pose,const MoveToPoseRequest & request)
  {
    if(!start){return std::nullopt;}const auto * model=start->getJointModelGroup(config().motion.planning_group);if(!model)return std::nullopt;
    Eigen::Isometry3d goal=Eigen::Isometry3d::Identity();
    goal.linear()=Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z).normalized().toRotationMatrix();
    goal.translation()=Eigen::Vector3d(pose.position.x_m,pose.position.y_m,pose.position.z_m);
    goal=start->getGlobalLinkTransform(config().frames.robot_base)*goal;
    moveit::core::RobotState candidate(*start);
    // This cheap IK is an ETA seed, not acceptance of an unvalidated trajectory.
    if(!candidate.setFromIK(model,goal,config().frames.tcp,.01)||!candidate.satisfiesBounds(model))return std::nullopt;
    return estimate_joint_approach_duration(*start,candidate,request);
  }
  void record_completed_approach_timing(double nominal_travel,Nanoseconds execution_started)
  {
    const auto source=Nanoseconds{node->now().nanoseconds()};if(source<execution_started)return;
    std::lock_guard<std::mutex> lock(plan_mutex);
    if(planned_joint_duration_sec>.05&&nominal_travel>.05){record_timing_sample(travel_ratios,nominal_travel/planned_joint_duration_sec);}
    record_timing_sample(planning_source_costs,planned_planning_source_sec);
    record_timing_sample(execution_source_overheads,std::max(0.,std::chrono::duration<double>(source-execution_started).count()-nominal_travel));
    RCLCPP_INFO(node->get_logger(),"approach timing completed: nominal_source=%.6f actual_source=%.6f joint_estimate=%.6f samples=%zu",nominal_travel,
      std::chrono::duration<double>(source-execution_started).count(),planned_joint_duration_sec,travel_ratios.size());
  }
  bool plan_move_to_pose(const MoveToPoseRequest & request)
  {
    const auto issued=cancellation.load();
    {std::lock_guard<std::mutex> lock(mutex);if(!servo_paused||following||stopping||owner_enabled||owner_pending||handoff_failed){return false;}}
    std::lock_guard<std::mutex> lock(plan_mutex);plan_valid=false;
    group->setMaxVelocityScalingFactor(request.max_velocity_scale);group->setMaxAccelerationScalingFactor(request.max_acceleration_scale);
    const bool moving=request.prediction_horizon.count()>0;
    const auto initial=Nanoseconds{node->now().nanoseconds()};
    if(timing_source_stamp.count()&&initial<timing_source_stamp){travel_ratios.clear();planning_source_costs.clear();execution_source_overheads.clear();}
    timing_source_stamp=initial;
    auto target=predict_arrival_tcp_pose(request,initial);if(!target){return false;}
    if(moving){
      const auto estimate=estimate_approach_duration(measured_robot_state(),target->pose,request);
      if(estimate){
        const double ratio=timing_median(travel_ratios,1.);
        const double overhead=timing_median(planning_source_costs,0.)+timing_median(execution_source_overheads,0.);
        const auto lead=seconds_to_duration(*estimate*ratio+overhead);
        if(!lead)return false;
        const auto estimated_arrival=Nanoseconds{node->now().nanoseconds()}+*lead;
        target=predict_arrival_tcp_pose(request,estimated_arrival);if(!target)return false;
        RCLCPP_INFO(node->get_logger(),"estimate_approach_duration: joint_source=%.6f calibrated_ratio=%.6f overhead_source=%.6f lead_source=%.6f samples=%zu",*estimate,ratio,overhead,std::chrono::duration<double>(*lead).count(),travel_ratios.size());
      }else{RCLCPP_WARN(node->get_logger(),"estimate_approach_duration: IK/limits unavailable; using bounded planning correction fallback");}
    }
    planned_prediction_overhead=moving?seconds_to_duration(timing_median(execution_source_overheads,0.)).value_or(Nanoseconds{0}):Nanoseconds{0};
    const auto wall_start=std::chrono::steady_clock::now();
    for(unsigned attempt=0;attempt<2;++attempt){
      const auto state=measured_robot_state();if(!state){return false;}group->setStartState(*state);
      moveit::core::RobotStatePtr selected;
      if(!select_collision_valid_joint_target(state,target->pose,&selected)){return false;}
      Group::Plan candidate;
      if(!plan_collision_checked_trajectory(candidate)||issued!=cancellation.load()||candidate.trajectory.joint_trajectory.points.empty()){return false;}
      auto now=Nanoseconds{node->now().nanoseconds()};
      double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_start).count();
      if(moving && wall>.1 && now==initial){
        // A 10-Hz simulation tick already takes 100 ms before scheduling jitter.
        // Qualify the plan with a new source tick instead of treating one tick
        // interval as a pause. No execution is allowed without clock progress.
        const auto clock_deadline=wall_start+std::chrono::duration<double>(
          std::min(config().vision.frame_max_age_sec,config().motion.planning_timeout_sec));
        while(issued==cancellation.load() && now==initial && std::chrono::steady_clock::now()<clock_deadline){
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          now=Nanoseconds{node->now().nanoseconds()};
        }
        wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_start).count();
        RCLCPP_INFO(node->get_logger(),"qualify_planning_clock: wall=%.6f source_delta=%.6f canceled=%d",wall,
          std::chrono::duration<double>(now-initial).count(),issued!=cancellation.load());
      }
      const auto & duration=candidate.trajectory.joint_trajectory.points.back().time_from_start;
      const auto travel=Nanoseconds{static_cast<std::int64_t>(duration.sec)*1000000000+duration.nanosec};
      RCLCPP_INFO(node->get_logger(),"view planning candidate: attempt=%u wall=%.6f source=%.6f travel=%.6f arrival_elapsed=%.6f horizon=%.6f s moving=%d",
        attempt+1,wall,std::chrono::duration<double>(now-initial).count(),std::chrono::duration<double>(travel).count(),
        std::chrono::duration<double>(now+travel+planned_prediction_overhead-request.target_pose.timestamp).count(),std::chrono::duration<double>(request.prediction_horizon).count(),moving);
      if(issued!=cancellation.load() || travel.count()<0||now<initial || (moving && wall>.1 && now==initial)){
        RCLCPP_WARN(node->get_logger(),"plan_move_to_pose rejected: negative_travel=%d clock_rewound=%d unchanged_clock_over_100ms=%d",
          travel.count()<0,now<initial,moving && wall>.1 && now==initial);return false;
      }
      const auto arrival=predict_arrival_tcp_pose(request,now+travel+planned_prediction_overhead);if(!arrival){return false;}
      if(moving&&!planning_prediction_near(target->pose,arrival->pose)){
        if(attempt==1){RCLCPP_WARN(node->get_logger(),"moving-pose correction did not converge in two plans");return false;}
        target=arrival;continue;
      }
      // Store the endpoint actually planned, not a later nearby prediction.
      planned_joint_duration_sec=selected?estimate_joint_approach_duration(*state,*selected,request).value_or(0.):0.;
      planned_planning_source_sec=std::chrono::duration<double>(now-initial).count();
      planned_endpoint=*target;planned_request=request;plan=std::move(candidate);planned_generation=issued;plan_valid=true;
      RCLCPP_INFO(node->get_logger(),"view plan: calls=%u wall=%.3f source=%.3f travel=%.3f moving=%d",attempt+1,wall,
        std::chrono::duration<double>(now-initial).count(),std::chrono::duration<double>(travel).count(),moving);
      return true;
    }
    return false;
  }
  bool execute_planned_trajectory()
  {
    Group::Plan trajectory;Nanoseconds prediction_overhead{0};std::uint64_t issued;
    {std::lock_guard<std::mutex> lock(plan_mutex);if(!plan_valid||planned_generation!=cancellation.load()){return false;}trajectory=plan;prediction_overhead=planned_prediction_overhead;issued=planned_generation;plan_valid=false;}
    if(trajectory.trajectory.joint_trajectory.points.empty()){return false;}
    const auto duration=trajectory.trajectory.joint_trajectory.points.back().time_from_start;RCLCPP_INFO(node->get_logger(),"executing planned trajectory, simulation duration %.3f s",duration.sec+duration.nanosec*1e-9);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(config().motion.execution_timeout_sec);
    if(!execute_client->action_server_is_ready()){return false;}
    {std::lock_guard<std::mutex> lock(mutex);if(handoff_failed||following||!servo_paused||owner_enabled||owner_pending||pause_pending){return false;}}
    const auto source_now=Nanoseconds{node->now().nanoseconds()};
    const auto duration_source=Nanoseconds{static_cast<std::int64_t>(duration.sec)*1000000000+duration.nanosec};
    const auto latest=predict_arrival_tcp_pose(planned_request,source_now+duration_source+prediction_overhead);
    if(!latest||!planning_prediction_near(planned_endpoint.pose,latest->pose)){return false;}
    const auto measured=measured_robot_state();if(!measured){return false;}
    const auto & start=trajectory.start_state.joint_state;
    if(start.name.size()!=start.position.size()){return false;}
    for(std::size_t i=0;i<start.name.size();++i){
      if(std::find(measured->getRobotModel()->getVariableNames().begin(),measured->getRobotModel()->getVariableNames().end(),start.name[i])==measured->getRobotModel()->getVariableNames().end()||std::abs(measured->getVariablePosition(start.name[i])-start.position[i])>config().motion.joint_position_tolerance_rad){return false;}
    }
    Execute::Goal goal;goal.trajectory=trajectory.trajectory;
    const auto execution_started=Nanoseconds{node->now().nanoseconds()};
    rclcpp_action::Client<Execute>::SendGoalOptions options;
    options.goal_response_callback=[this,issued](rclcpp_action::ClientGoalHandle<Execute>::SharedPtr handle){
      if(handle&&issued!=cancellation.load()){execute_client->async_cancel_goal(handle);}
    };
    decltype(execute_client->async_send_goal(goal)) accepted;
    {
      // Serialize command submission with Stop, preserving the plan's token.
      // Never hold this lock while waiting for a ROS reply.
      std::lock_guard<std::mutex> start(start_mutex);
      if(issued!=cancellation.load()){return false;}
      {std::lock_guard<std::mutex> lock(mutex);if(stopping||following||handoff_failed||!servo_paused||owner_enabled||owner_pending||pause_pending){return false;}}
      accepted=execute_client->async_send_goal(goal,options);
    }
    while(accepted.wait_for(std::chrono::milliseconds(20))!=std::future_status::ready){
      if(issued!=cancellation.load()){RCLCPP_INFO(node->get_logger(),"trajectory canceled while awaiting acceptance; late acceptance will be canceled");return false;}
      if(std::chrono::steady_clock::now()>=deadline){stop_arm();return false;}
    }
    const auto handle=accepted.get();if(!handle){return false;}
    const auto result=execute_client->async_get_result(handle);
    while(issued==cancellation.load()&&std::chrono::steady_clock::now()<deadline&&result.wait_for(std::chrono::milliseconds(20))!=std::future_status::ready){}
    if(issued!=cancellation.load()||result.wait_for(std::chrono::seconds(0))!=std::future_status::ready){if(result.wait_for(std::chrono::seconds(0))!=std::future_status::ready){try{execute_client->async_cancel_goal(handle);}catch(const rclcpp_action::exceptions::UnknownGoalHandleError &){/* Completed concurrently with cancellation. */}}stop_arm();return false;}
    const auto completed=result.get();
    if(completed.code!=rclcpp_action::ResultCode::SUCCEEDED||!completed.result||completed.result->error_code.val!=moveit_msgs::msg::MoveItErrorCodes::SUCCESS){return false;}
    if(trajectory.trajectory.joint_trajectory.points.empty()){stop_arm();return false;}
    const auto & final=trajectory.trajectory.joint_trajectory.points.back();const auto & names=trajectory.trajectory.joint_trajectory.joint_names;
    std::unique_lock<std::mutex> lock(mutex);
    while(issued==cancellation.load()&&std::chrono::steady_clock::now()<deadline){bool arrived=measured_stop(joints);
      for(std::size_t k=0;k<names.size()&&arrived;++k){const auto found=std::find(joints.name.begin(),joints.name.end(),names[k]);const auto i=static_cast<std::size_t>(found-joints.name.begin());arrived=found!=joints.name.end()&&i<joints.position.size()&&k<final.positions.size()&&std::abs(joints.position[i]-final.positions[k])<=config().motion.joint_position_tolerance_rad;}
      if(arrived){lock.unlock();record_completed_approach_timing(std::chrono::duration<double>(duration_source).count(),execution_started);return true;}changed.wait_for(lock,std::chrono::milliseconds(20));}lock.unlock();stop_arm();return false;
  }
  void stop_arm()
  {
    {std::lock_guard<std::mutex> lock(mutex);if(following&&target.producer=="grasp"){
      const auto sample=clock_aligned_joints(Nanoseconds{node->now().nanoseconds()}).first;const auto command=target;const auto code=servo_code;
      if(!diagnostic_worker.enqueue([this,sample,command,code]{save_protective_stop_diagnostic(sample,command,code,"requested grasp stop; correlate Task outcome");})){RCLCPP_WARN(node->get_logger(),"grasp-stop diagnostic queue full");}
    }}
    std::lock_guard<std::mutex> start(start_mutex);++cancellation;group->stop();
    if(execute_client->action_server_is_ready()){execute_client->async_cancel_all_goals();}

    {std::lock_guard<std::mutex> lock(mutex);following=false;stopping=true;}changed.notify_all();
  }
  bool follow(const Command & command)
  {
    std::lock_guard<std::mutex> lock(mutex);if(stopping){return false;}
    if(!following){contact_ready=false;servo_code=0;servo_message.clear();}target=command;following=true;return true;
  }
  bool command_grip(double width,double effort,bool open,double timeout)
  {
    std::uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex);
      generation=++grip_command_generation;
      if(!open){
        grip_close_joint_sequence=joint_sequence;
        possession_unknown=true;
        effort_contact_latched=false;
        measured_empty_latched=false;
        contact_from_controller=false;
      }
    }
    const auto issued=cancellation.load(),grip_issued=gripper_cancellation.load();Grip::Goal goal;goal.command.position=width/config().motion.gripper_width_factor;goal.command.max_effort=effort;
    const auto started=std::chrono::steady_clock::now();
    const auto deadline=started+std::chrono::duration<double>(timeout);
    const auto acceptance_deadline=started+std::chrono::duration<double>(open?timeout:std::min(timeout,config().grasp.close_timeout_sec));
    auto future=grip_client->async_send_goal(goal);
    while(future.wait_for(std::chrono::milliseconds(20))!=std::future_status::ready){if(issued!=cancellation.load()||grip_issued!=gripper_cancellation.load()||std::chrono::steady_clock::now()>=acceptance_deadline){grip_client->async_cancel_all_goals();return false;}}
    const auto handle=future.get();if(!handle){return false;}auto result=grip_client->async_get_result(handle);
    while(result.wait_for(std::chrono::milliseconds(20))!=std::future_status::ready){
      if(issued!=cancellation.load()||grip_issued!=gripper_cancellation.load()||std::chrono::steady_clock::now()>=deadline){
        grip_client->async_cancel_goal(handle);
        // Do not advertise command completion while the lower finger action
        // is still canceling. Reuse the existing stop-confirmation budget.
        const auto stop_deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(config().motion.stop_confirmation_timeout_sec);
        while(result.wait_for(std::chrono::milliseconds(20))!=std::future_status::ready && std::chrono::steady_clock::now()<stop_deadline){}
        return false;
      }
    }
    const auto response=result.get();const bool success=response.code==rclcpp_action::ResultCode::SUCCEEDED;
    {std::lock_guard<std::mutex> lock(mutex);if(generation==grip_command_generation){contact_from_controller=ptc_motion_moveit::contact_after_command(contact_from_controller,success,open,response.result&&response.result->stalled);if(open&&success&&response.result&&!response.result->stalled){possession_unknown=false;grip_close_joint_sequence.reset();effort_contact_latched=false;}}}return success;
  }
  bool synchronize(const ptc_interfaces::action::SyncScene::Goal & goal);
  bool apply_scene(moveit_msgs::msg::PlanningScene scene)
  {
    // The action server may become discoverable before MoveIt's scene service.
    // Wait within the existing planning budget rather than fail immediately.
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(config().motion.planning_timeout_sec);
    if(!apply_scene_client->wait_for_service(std::chrono::duration<double>(config().motion.planning_timeout_sec))){throw std::runtime_error("ApplyPlanningScene service unavailable within planning timeout");}
    scene.is_diff=true;scene.robot_state.is_diff=true;
    auto request=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();request->scene=std::move(scene);auto result=apply_scene_client->async_send_request(request);
    if(result.wait_until(deadline)!=std::future_status::ready){apply_scene_client->remove_pending_request(result);throw std::runtime_error("ApplyPlanningScene response timed out");}
    if(!result.get()->success){throw std::runtime_error("ApplyPlanningScene returned success=false");}return true;
  }
  bool is_expected_grasp_contact(const ptc_motion::MotionCheckRequest & request,
    const collision_detection::Contact & contact,const std::string & frame)
  {
    const auto cached=measured_surfaces.find(request.track_id);
    if(cached==measured_surfaces.end()||cached->second.revision!=request.surface_revision||
      !same_track_identity(cached->second.key,request.key)||request.contact_patches.size()!=2)return false;
    geometry_msgs::msg::PointStamped point;point.header.frame_id=frame;
    point.point.x=contact.pos.x();point.point.y=contact.pos.y();point.point.z=contact.pos.z();
    if(frame!=config().frames.world){
      try{point=tf.transform(point,config().frames.world,tf2::durationFromSec(.02));}
      catch(const std::exception &){return false;}
    }
    const double voxel_radius=cached->second.geometry.resolution_m*std::sqrt(3.)*.5;
    std::vector<GraspContactPatchModel> patches;
    for(const auto & patch:request.contact_patches){
      if(patch.center.reference_frame!=config().frames.world)return false;
      const auto & center=patch.center.pose.position;
      patches.push_back({{center.x_m,center.y_m,center.z_m},patch.radius_m+voxel_radius});
    }
    return expected_grasp_contact(contact.body_name_1,contact.body_name_2,
      {point.point.x,point.point.y,point.point.z},"tracked_"+std::to_string(request.track_id),config().motion.touch_links,patches);
  }
  bool check_gripper_closure(const moveit::core::RobotStatePtr & open_state,const ptc_motion::MotionCheckRequest & request,
    std::chrono::steady_clock::time_point deadline,std::string * failure,
    const planning_scene::PlanningScenePtr & scene_snapshot={})
  {
    if(request.contact_patches.size()!=2||config().motion.touch_links.size()!=2){if(failure){*failure="two measured contact patches and two configured finger links are required";}return false;}
    const auto cached=measured_surfaces.find(request.track_id);
    if(cached==measured_surfaces.end()||cached->second.revision!=request.surface_revision||!same_track_identity(cached->second.key,request.key)){
      if(failure){*failure="grasp surface revision is not present in the acknowledged MoveIt scene";}return false;
    }
    const auto * gripper_joint=open_state?open_state->getRobotModel()->getJointModel(config().motion.gripper_joint):nullptr;
    if(!gripper_joint||gripper_joint->getVariableCount()!=1||config().motion.gripper_width_factor<=0){if(failure){*failure="configured gripper joint or width conversion is invalid";}return false;}
    const double start_width=open_state->getVariablePosition(config().motion.gripper_joint)*config().motion.gripper_width_factor;
    if(!std::isfinite(start_width)||start_width<request.closure_width_m){if(failure){*failure="measured open width is below the candidate closure width";}return false;}
    const auto steps=static_cast<std::size_t>(std::ceil((start_width-request.closure_width_m)/.002));
    if(!steps||steps>64){if(failure){*failure="gripper closure sweep exceeds the 64-sample bound";}return false;}
    const std::string object_id="tracked_"+std::to_string(request.track_id);
    const double voxel_radius=cached->second.geometry.resolution_m*std::sqrt(3.)*.5;
    std::array<double,2> patch_distance_min{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
    for(std::size_t step=1;step<=steps;++step){
      if(std::chrono::steady_clock::now()>=deadline){if(failure){*failure="gripper closure sweep exceeded its wall deadline";}return false;}
      const double width=start_width+(request.closure_width_m-start_width)*static_cast<double>(step)/steps;
      auto closed=std::make_shared<moveit::core::RobotState>(*open_state);
      closed->setVariablePosition(config().motion.gripper_joint,width/config().motion.gripper_width_factor);
      closed->update();
      struct ClosureContact
      {
        std::string first,second;geometry_msgs::msg::PointStamped position;double depth{0};
      };
      std::vector<ClosureContact> contacts;bool collision=false;
      if(scene_snapshot){
        collision_detection::CollisionRequest collision_request;collision_request.group_name="";
        collision_request.contacts=true;collision_request.max_contacts=128;collision_request.max_contacts_per_pair=32;
        collision_detection::CollisionResult collision_result;scene_snapshot->checkCollision(collision_request,collision_result,*closed);
        collision=collision_result.collision;
        if(collision){
          for(const auto & pair:collision_result.contacts){for(const auto & contact:pair.second){
            ClosureContact item;item.first=contact.body_name_1;item.second=contact.body_name_2;item.depth=contact.depth;
            item.position.header.frame_id=scene_snapshot->getPlanningFrame();
            item.position.point.x=contact.pos.x();item.position.point.y=contact.pos.y();item.position.point.z=contact.pos.z();
            contacts.push_back(std::move(item));
          }}
        }
      }else{
        auto validity_request=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        validity_request->group_name="";
        moveit::core::robotStateToRobotStateMsg(*closed,validity_request->robot_state);
        auto future=validity_client->async_send_request(validity_request);
        if(future.wait_until(deadline)!=std::future_status::ready){validity_client->remove_pending_request(future);if(failure){*failure="gripper closure state-validity response timed out";}return false;}
        const auto response=future.get();if(response->valid){continue;}collision=true;
        for(const auto & contact:response->contacts){
          ClosureContact item;item.first=contact.contact_body_1;item.second=contact.contact_body_2;
          item.position.header=contact.header;item.position.point=contact.position;item.depth=contact.depth;
          contacts.push_back(std::move(item));
        }
      }
      if(!collision){continue;}
      std::array<std::set<std::string>,2> patch_links;
      double maximum_depth=0;
      for(const auto & contact:contacts){
        const bool first_is_object=contact.first==object_id,second_is_object=contact.second==object_id;
        if(first_is_object==second_is_object){if(failure){*failure="closure encountered a non-object collision";}return false;}
        const auto & link=first_is_object?contact.second:contact.first;
        if(std::find(config().motion.touch_links.begin(),config().motion.touch_links.end(),link)==config().motion.touch_links.end()){
          if(failure){*failure="closure collision outside configured finger links: "+link;}return false;
        }
        geometry_msgs::msg::PointStamped point=contact.position;
        if(point.header.frame_id.empty()){point.header.frame_id=config().frames.world;}
        if(point.header.frame_id!=config().frames.world){
          try{point=tf.transform(point,config().frames.world,tf2::durationFromSec(.02));}
          catch(const std::exception &){if(failure){*failure="closure contact point transform unavailable";}return false;}
        }
        const Eigen::Vector3d world_point(point.point.x,point.point.y,point.point.z);
        // Measured bilateral patches decide grasp contact; avoid a second URDF mesh-axis heuristic here.
        bool in_patch=false;
        for(std::size_t patch=0;patch<request.contact_patches.size();++patch){
          const auto & center=request.contact_patches[patch].center.pose.position;
          const double allowed=request.contact_patches[patch].radius_m+voxel_radius;
          const double distance=(world_point-Eigen::Vector3d(center.x_m,center.y_m,center.z_m)).norm();
          patch_distance_min[patch]=std::min(patch_distance_min[patch],distance);
          if(distance<=allowed){patch_links[patch].insert(link);in_patch=true;}
        }
        if(!in_patch){
          if(config().runtime.operation_mode=="assumed_ng_demo" && config().grasp.moving_intercept && world_point.allFinite()){
            // The physical demo may try a supported candidate without proving
            // every closing-finger contact lies inside two partial-cloud disks.
            // Identity/finger-only checks above still reject palm, camera,
            // belt and other-object contact; real possession remains measured.
            RCLCPP_INFO_THROTTLE(node->get_logger(),*node->get_clock(),1000,
              "demo closure finger contact outside measured patches: track=%lu link=%s; permit physical attempt, not Holding evidence",
              request.track_id,link.c_str());
          }else{if(failure){*failure="closure finger contact outside both measured contact patches: "+link;}return false;}
        }
        maximum_depth=std::max(maximum_depth,contact.depth);
      }
      const bool bilateral=std::any_of(patch_links[0].begin(),patch_links[0].end(),[&](const auto & first){
        return std::any_of(patch_links[1].begin(),patch_links[1].end(),[&](const auto & second){return first!=second;});});
      if(bilateral){RCLCPP_INFO(node->get_logger(),"grasp closure geometry confirmed: track=%lu surface_revision=%lu width_m=%.4f contact_depth_max_m=%.5f",
        request.track_id,request.surface_revision,width,maximum_depth);return true;}
      if(contacts.empty()){if(failure){*failure="closure state invalid but MoveIt returned no contact evidence";}return false;}
    }
    if(config().runtime.operation_mode=="assumed_ng_demo"){
      RCLCPP_INFO(node->get_logger(),"virtual bilateral patch contact unconfirmed: track=%lu nearest_m=[%.6f,%.6f]; permit physical attempt, actual closure/lift evidence still required",request.track_id,patch_distance_min[0],patch_distance_min[1]);return true;
    }
    if(failure){*failure="closure reached candidate width without bilateral contact at measured patches: nearest_m=["+
      std::to_string(patch_distance_min[0])+","+std::to_string(patch_distance_min[1])+"]";}return false;
  }
  bool check_with_state(const ptc_motion::MotionCheckRequest & request,const moveit::core::RobotStatePtr & measured,
    std::chrono::steady_clock::time_point deadline,std::uint64_t issued,bool plan_pregrasp,
    const planning_scene::PlanningScenePtr & scene_snapshot={},double * minimum_joint_margin=nullptr,double * pregrasp_time_lower_bound=nullptr)
  {
    if(!measured){return false;}
    auto state=std::make_shared<moveit::core::RobotState>(*measured);
    try{
      const auto & targets=request.targets;
      if(request.grasp&&(targets.size()!=2||request.contact_patches.size()!=2)){
        throw std::runtime_error("grasp check requires pregrasp/contact targets and two measured contact patches");}
      std::size_t checked=0;
      for(std::size_t index=0;index<targets.size();++index){
        if(issued!=cancellation.load()||std::chrono::steady_clock::now()>=deadline){throw std::runtime_error("candidate check canceled or exceeded its wall deadline");}
        const auto & target=targets[index];group->setStartState(*state);moveit::core::RobotStatePtr selected;std::string failure;double margin=std::numeric_limits<double>::infinity();
        if(request.grasp&&index==1){
          const auto & start=targets.front().pose;const auto & finish=target.pose;
          const double distance=vector_norm({finish.position.x_m-start.position.x_m,finish.position.y_m-start.position.y_m,finish.position.z_m-start.position.z_m});
          const auto & qa=start.orientation;const auto & qb=finish.orientation;
          const double quaternion_dot=std::clamp(std::abs(qa.x*qb.x+qa.y*qb.y+qa.z*qb.z+qa.w*qb.w),0.,1.);
          const double angle=2.*std::acos(quaternion_dot);const auto steps=static_cast<std::size_t>(std::max({1.,std::ceil(distance/.005),std::ceil(angle/(5.*3.141592653589793/180.))}));
          if(steps>64){throw std::runtime_error("grasp approach sweep exceeds the 64-sample bound");}
          for(std::size_t step=1;step<steps;++step){
            if(issued!=cancellation.load()||std::chrono::steady_clock::now()>=deadline){throw std::runtime_error("grasp approach sweep canceled or exceeded its wall deadline");}
            const auto pose=interpolate_pose(start,finish,static_cast<double>(step)/steps);double sample_margin=std::numeric_limits<double>::infinity();
            if(!pose||!select_collision_valid_joint_target(state,*pose,&selected,deadline,&failure,scene_snapshot,&sample_margin)){
              throw std::runtime_error("grasp approach sweep rejected at sample "+std::to_string(step)+"/"+std::to_string(steps)+" reason="+(failure.empty()?"invalid_pose":failure));}
            margin=std::min(margin,sample_margin);state=selected;
          }
          // At the contact endpoint only, permit finger contacts within the
          // measured patches. Palm/camera, other targets and earlier approach
          // samples retain normal collision checking; closure validates both patches.
          double contact_margin=std::numeric_limits<double>::infinity();
          if(!select_collision_valid_joint_target(state,target.pose,&selected,deadline,&failure,scene_snapshot,&contact_margin,&request)){
            throw std::runtime_error("open-finger contact pose rejected before closure reason="+(failure.empty()?"unknown":failure));}
          margin=std::min(margin,contact_margin);
          if(!check_gripper_closure(selected,request,deadline,&failure,scene_snapshot)){
            throw std::runtime_error("gripper closure rejected: "+(failure.empty()?"contact evidence unavailable":failure));}
          if(minimum_joint_margin){*minimum_joint_margin=std::min(*minimum_joint_margin,margin);}
          state=selected;++checked;continue;
        }
        if(!select_collision_valid_joint_target(state,target.pose,&selected,deadline,&failure,scene_snapshot,&margin)){
          throw std::runtime_error("collision-valid IK rejected target "+std::to_string(checked)+" reason="+(failure.empty()?"unknown":failure));}
        if(request.grasp&&index==0){
          if(minimum_joint_margin){*minimum_joint_margin=std::min(*minimum_joint_margin,margin);}
          if(pregrasp_time_lower_bound){const auto * jmg=selected->getJointModelGroup(config().motion.planning_group);double lower_bound=0.;
            if(jmg){for(const auto & name:jmg->getVariableNames()){const auto & bound=selected->getRobotModel()->getVariableBounds(name);
              const double speed=std::max(std::abs(bound.max_velocity_),std::abs(bound.min_velocity_));
              if(speed>1e-9){lower_bound=std::max(lower_bound,std::abs(selected->getVariablePosition(name)-measured->getVariablePosition(name))/speed);}}}
            *pregrasp_time_lower_bound=lower_bound;}
          if(!plan_pregrasp){state=selected;++checked;continue;}
        }
        // Candidate batches stop at local feasibility; the selected candidate
        // gets its complete route only after Task has ranked the batch.
        if(request.grasp&&config().runtime.operation_mode=="assumed_ng_demo"&&config().grasp.moving_intercept){state=selected;++checked;continue;}
        Group::Plan candidate;
        if(!plan_collision_checked_trajectory(candidate)||candidate.trajectory.joint_trajectory.points.empty()){
          throw std::runtime_error(request.grasp?"collision-checked route to grasp pregrasp is unavailable":"collision-checked route to target is unavailable");}
        const auto & end=candidate.trajectory.joint_trajectory.points.back();const auto & names=candidate.trajectory.joint_trajectory.joint_names;
        if(names.size()!=end.positions.size()){throw std::runtime_error("planned joint target dimensions are inconsistent");}
        for(std::size_t i=0;i<names.size();++i){state->setVariablePosition(names[i],end.positions[i]);}state->update();++checked;
      }
      if(issued!=cancellation.load()){throw std::runtime_error("candidate check canceled before completion");}
      group->setStartStateToCurrentState();return true;
    }catch(...){group->setStartStateToCurrentState();throw;}
  }
  bool check(const ptc_motion::MotionCheckRequest & request)
  {
    std::lock_guard<std::mutex> planning_lock(plan_mutex);
    if(!validity_client->service_is_ready()){throw std::runtime_error("state-validity service unavailable");}
    const auto measured=measured_robot_state();if(!measured){return false;}
    const auto issued=cancellation.load();const auto deadline=std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(request.timeout_sec));
    return check_with_state(request,measured,deadline,issued,true);
  }
  const std::vector<GraspSweepBox> & grasp_sweep_boxes(const moveit::core::RobotState & measured)
  {
    if(grasp_sweep_tool)return *grasp_sweep_tool;
    auto & tool=grasp_sweep_tool.emplace();
    const auto & model=*measured.getRobotModel();
    const auto * finger=model.getJointModel(config().motion.gripper_joint);
    if(!finger || !finger->getParentLinkModel())return tool;
    std::vector<const moveit::core::LinkModel *> links{finger->getParentLinkModel()};
    for(const auto & name:config().motion.touch_links)
      if(const auto * link=model.getLinkModel(name))links.push_back(link);
    for(std::size_t i=0;i<links.size();++i)
      for(const auto * joint:links[i]->getChildJointModels())
        if(joint->getType()==moveit::core::JointModel::FIXED && joint->getChildLinkModel())
          links.push_back(joint->getChildLinkModel());

    auto open=measured,closed=measured;
    const auto width_change=config().grasp.close_width_m-config().grasp.open_width_m;
    const auto width_factor=config().motion.gripper_width_factor;
    open.setVariablePosition(config().motion.gripper_joint,config().grasp.open_width_m/width_factor);
    closed.setVariablePosition(config().motion.gripper_joint,config().grasp.close_width_m/width_factor);
    open.update();closed.update();
    const Eigen::Isometry3d open_tcp_inverse=open.getGlobalLinkTransform(config().frames.tcp).inverse();
    const Eigen::Isometry3d closed_tcp_inverse=closed.getGlobalLinkTransform(config().frames.tcp).inverse();
    constexpr double voxel_padding=0.5*kGraspNeighborVoxelM;
    for(const auto * link:links){
      const Eigen::Isometry3d open_link=open_tcp_inverse*open.getGlobalLinkTransform(link);
      const Eigen::Isometry3d closed_link=closed_tcp_inverse*closed.getGlobalLinkTransform(link);
      const Eigen::Vector3d shift=(closed_link.translation()-open_link.translation())/width_change;
      const auto & shapes=link->getShapes();
      const auto & origins=link->getCollisionOriginTransforms();
      for(std::size_t i=0;i<shapes.size();++i){
        const bodies::BodyPtr body(bodies::createBodyFromShape(shapes[i].get()));
        if(!body)continue;
        bodies::AABB bounds;
        body->computeBoundingBox(bounds);
        bounds.min().array()-=voxel_padding;
        bounds.max().array()+=voxel_padding;
        tool.push_back({open_link*origins[i],bounds,shift});
      }
    }
    return tool;
  }

  void rank_grasp_neighbors(const ptc_motion::GraspCandidatesRequest & request,
    const moveit::core::RobotState & measured,std::vector<std::pair<std::size_t,std::size_t>> & order)
  {
    const auto start=std::chrono::steady_clock::now();
    if(request.neighbor_points_world.empty()){
      if(request.neighbor_capture_ms>0.)
        RCLCPP_INFO(node->get_logger(),"grasp neighbor ranking: track=%lu points=0 capture_ms=%.3f score_ms=0.000",
          request.key.track_id,request.neighbor_capture_ms);
      return;
    }
    const auto & tool=grasp_sweep_boxes(measured);
    if(tool.empty())return;
    std::vector<Eigen::Vector3d> points;
    points.reserve(request.neighbor_points_world.size());
    for(const auto & p:request.neighbor_points_world)points.emplace_back(p.x_m,p.y_m,p.z_m);
    std::vector<std::size_t> overlaps(request.candidates.size());
    const auto & world_from_base=measured.getGlobalLinkTransform(config().frames.robot_base);
    for(const auto & [index,target_overlap]:order){
      (void)target_overlap;
      const auto & candidate=request.candidates[index].motion;
      const auto & start_pose=candidate.targets.front().pose.position;
      const auto & finish=candidate.targets.back().pose;
      const auto & q=finish.orientation;
      Eigen::Isometry3d contact=Eigen::Isometry3d::Identity();
      contact.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
      contact.translation()=Eigen::Vector3d(finish.position.x_m,finish.position.y_m,finish.position.z_m);
      overlaps[index]=count_neighbor_sweep_intersections(tool,points,world_from_base*contact,
        world_from_base*Eigen::Vector3d(start_pose.x_m,start_pose.y_m,start_pose.z_m),
        candidate.closure_width_m-config().grasp.open_width_m,config().grasp.verification_probe_m);
    }
    auto proposed=order;
    std::stable_sort(proposed.begin(),proposed.end(),[&](const auto & a,const auto & b){
      return std::make_tuple(a.second!=0,overlaps[a.first],a.second)<
        std::make_tuple(b.second!=0,overlaps[b.first],b.second);
    });
    const auto legacy=order.front().first,best=proposed.front().first;
    const auto & mode=config().grasp.neighbor_clearance_mode;
    const auto score_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    RCLCPP_INFO(node->get_logger(),
      "grasp neighbor ranking: track=%lu mode=%s points=%zu candidates=%zu legacy=%u proposed=%u legacy_hits=%zu proposed_hits=%zu capture_ms=%.3f score_ms=%.3f",
      request.key.track_id,mode.c_str(),points.size(),order.size(),request.candidates[legacy].candidate_id,
      request.candidates[best].candidate_id,overlaps[legacy],overlaps[best],request.neighbor_capture_ms,score_ms);
    if(mode=="rank")order=std::move(proposed);
  }

  ptc_motion::GraspCandidatesResult evaluate_grasp_candidates(const ptc_motion::GraspCandidatesRequest & request)
  {
    std::lock_guard<std::mutex> planning_lock(plan_mutex);ptc_motion::GraspCandidatesResult output;
    if(request.candidates.empty()||request.candidates.size()>16){throw std::runtime_error("candidate batch size must be in [1,16]");}
    if(!get_scene_client->service_is_ready()){throw std::runtime_error("planning scene service unavailable");}
    const auto issued=cancellation.load();const auto deadline=std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(request.timeout_sec));
    auto measured=measured_robot_state(&output.measured_joint_stamp);if(!measured){throw std::runtime_error("measured joint snapshot unavailable");}
    auto scene_request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();scene_request->components.components=1023;
    auto scene_future=get_scene_client->async_send_request(scene_request);
    if(scene_future.wait_until(deadline)!=std::future_status::ready){get_scene_client->remove_pending_request(scene_future);throw std::runtime_error("coherent planning scene snapshot timed out");}
    auto scene_snapshot=std::make_shared<planning_scene::PlanningScene>(group->getRobotModel());
    scene_snapshot->setPlanningSceneMsg(scene_future.get()->scene);scene_snapshot->setCurrentState(*measured);
    // Rank the existing candidates before any IK sweep. The scene contains
    // measured target voxels; the rigid tool comes from the loaded robot model.
    // No object-specific dimensions, truth poses or new admission threshold.
    const auto ranking_started=std::chrono::steady_clock::now();
    std::vector<std::pair<std::size_t,std::size_t>> order;
    for(std::size_t index=0;index<request.candidates.size();++index)order.emplace_back(index,0);
    const auto target=scene_snapshot->getWorld()->getObject("tracked_"+std::to_string(request.key.track_id));
    const auto cached=measured_surfaces.find(request.key.track_id);
    const auto * finger=measured->getRobotModel()->getJointModel(config().motion.gripper_joint);
    if(order.size()>1 && target && cached!=measured_surfaces.end() &&
      cached->second.revision==request.surface_revision && same_track_identity(cached->second.key,request.key) &&
      finger && finger->getParentLinkModel()){
      std::vector<RigidToolCollisionShape> tool;
      std::vector<const moveit::core::LinkModel *> rigid{finger->getParentLinkModel()};
      const Eigen::Isometry3d tcp_inverse=measured->getGlobalLinkTransform(config().frames.tcp).inverse();
      for(std::size_t link=0;link<rigid.size();++link){
        const auto * model=rigid[link];
        const auto & shapes=model->getShapes();const auto & origins=model->getCollisionOriginTransforms();
        for(std::size_t shape=0;shape<shapes.size();++shape){
          bodies::BodyPtr body(bodies::createBodyFromShape(shapes[shape].get()));
          if(!body)continue;
          body->setPadding(cached->second.geometry.resolution_m*std::sqrt(3.)*.5);
          tool.push_back({tcp_inverse*measured->getGlobalLinkTransform(model)*origins[shape],std::move(body)});
        }
        for(const auto * joint:model->getChildJointModels()){
          if(joint->getType()==moveit::core::JointModel::FIXED && joint->getChildLinkModel())rigid.push_back(joint->getChildLinkModel());
        }
      }
      std::vector<Eigen::Vector3d> points;
      for(const auto & pose:target->global_shape_poses_)points.push_back(pose.translation());
      const auto base_world=measured->getGlobalLinkTransform(config().frames.robot_base);
      for(auto & ranked:order){
        const auto & targets=request.candidates[ranked.first].motion.targets;
        if(targets.empty())continue;
        const auto & pose=targets.back().pose;
        Eigen::Isometry3d contact=Eigen::Isometry3d::Identity();
        contact.linear()=Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z).normalized().toRotationMatrix();
        contact.translation()=Eigen::Vector3d(pose.position.x_m,pose.position.y_m,pose.position.z_m);
        ranked.second=count_grasp_tool_intersections(tool,points,base_world*contact);
      }
      std::stable_sort(order.begin(),order.end(),[](const auto & a,const auto & b){return a.second<b.second;});
      const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-ranking_started).count();
      RCLCPP_INFO(node->get_logger(),"grasp clearance ranking: track=%lu candidates=%zu tool_shapes=%zu points=%zu original_overlap=%zu selected_id=%u selected_overlap=%zu wall_ms=%.3f",
        request.key.track_id,order.size(),tool.size(),points.size(),
        std::find_if(order.begin(),order.end(),[](const auto & item){return item.first==0;})->second,
        request.candidates[order.front().first].candidate_id,order.front().second,elapsed);
    }
    rank_grasp_neighbors(request,*measured,order);
    output.candidates.reserve(request.candidates.size());bool selected=false;
    for(const auto & ranked:order){
      const auto & candidate=request.candidates[ranked.first];ptc_motion::GraspCandidateFeasibility result;result.candidate_id=candidate.candidate_id;
      if(selected){
        result.failure_reason="not evaluated: higher-ranked feasible candidate selected";
        output.candidates.push_back(std::move(result));continue;
      }
      if(std::chrono::steady_clock::now()>=deadline||issued!=cancellation.load()){
        result.failure_reason="batch deadline exceeded or canceled";output.candidates.push_back(std::move(result));continue;
      }
      double margin=std::numeric_limits<double>::infinity(),lower_bound=0.;
      try{
        result.feasible=check_with_state(candidate.motion,measured,deadline,issued,false,scene_snapshot,&margin,&lower_bound);
        result.evaluated=true;
        if(result.feasible){result.minimum_joint_margin_rad=std::isfinite(margin)?margin:0.;result.pregrasp_duration_sec=lower_bound;}
        else{result.failure_reason="candidate feasibility returned false";}
      }catch(const std::exception & ex){
        result.feasible=false;result.failure_reason=ex.what();
        result.evaluated=result.failure_reason.find("deadline")==std::string::npos&&
          result.failure_reason.find("canceled")==std::string::npos&&
          result.failure_reason.find("timed out")==std::string::npos&&
          result.failure_reason.find("timeout")==std::string::npos;
      }
      // Keep useful progress metrics for rejected candidates too: a collision
      // or contact failure may happen after the pregrasp IK was fully checked.
      if(result.evaluated&&std::isfinite(margin)){result.minimum_joint_margin_rad=margin;}
      if(result.evaluated&&std::isfinite(lower_bound)&&lower_bound>=0.){result.pregrasp_duration_sec=lower_bound;}
      selected=result.evaluated&&result.feasible;
      output.candidates.push_back(std::move(result));
    }
    group->setStartStateToCurrentState();return output;
  }
};
bool MoveitBackendRuntime::synchronize(const ptc_interfaces::action::SyncScene::Goal & goal)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;std::set<std::string> present;
  // Recovery may be retrying a partially applied attachment/removal. Read the
  // backend scene rather than asking MoveIt to REMOVE a nonexistent object.
  const std::string detach_id="tracked_"+std::to_string(goal.attached_track_id);
  std::set<std::string> detach_world;bool detach_attached=false;
  if(goal.operation==goal.DETACH || goal.operation==goal.ATTACH){
    if(!get_scene_client->service_is_ready())throw std::runtime_error("planning scene unavailable for recovery attachment");
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components=moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    auto response=get_scene_client->async_send_request(request);
    if(response.wait_for(std::chrono::duration<double>(config().motion.planning_timeout_sec))!=std::future_status::ready){
      get_scene_client->remove_pending_request(response);throw std::runtime_error("recovery detach scene read timed out");
    }
    const auto measured=response.get()->scene;
    for(const auto & object:measured.world.collision_objects)detach_world.insert(object.id);
    for(const auto & object:measured.robot_state.attached_collision_objects){
      if(object.object.id==detach_id)detach_attached=true;
    }
    RCLCPP_INFO(node->get_logger(),"recovery attachment observed: id=%s attached=%d world=%d",detach_id.c_str(),detach_attached,detach_world.count(detach_id)!=0);
    if(goal.operation==goal.ATTACH && detach_attached){
      moveit_msgs::msg::AttachedCollisionObject detach;detach.link_name=config().frames.tcp;detach.object.id=detach_id;detach.object.operation=detach.object.REMOVE;
      moveit_msgs::msg::PlanningScene update;update.robot_state.attached_collision_objects.push_back(detach);if(!apply_scene(update))return false;
    }
  }
  std::optional<std::pair<std::uint64_t,CachedSurface>> pending_surface_commit;
  const ptc_interfaces::msg::ObjectSurfaceGeometry * grasp_surface=goal.grasp_surfaces.empty()?nullptr:&goal.grasp_surfaces.front();
  for(const auto & geometry:goal.static_geometry){
    moveit_msgs::msg::CollisionObject object;object.id=geometry.id;object.header.frame_id=geometry.reference_frame;object.operation=object.ADD;
    if(geometry.kind==geometry.MESH){
      std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource("file://"+geometry.mesh_path));if(!mesh){throw std::runtime_error("scene mesh could not be loaded: "+geometry.id+" path="+geometry.mesh_path);}
      double minimum[3]={INFINITY,INFINITY,INFINITY},maximum[3]={-INFINITY,-INFINITY,-INFINITY};
      for(unsigned i=0;i<mesh->vertex_count;++i){for(unsigned axis=0;axis<3;++axis){minimum[axis]=std::min(minimum[axis],mesh->vertices[3*i+axis]);maximum[axis]=std::max(maximum[axis],mesh->vertices[3*i+axis]);}}
      if(maximum[0]<=minimum[0]||maximum[1]<=minimum[1]||maximum[2]<=minimum[2]){throw std::runtime_error("scene mesh has zero extent: "+geometry.id);}
      const Eigen::Vector3d scale(geometry.dimensions_m.x/(maximum[0]-minimum[0]),geometry.dimensions_m.y/(maximum[1]-minimum[1]),geometry.dimensions_m.z/(maximum[2]-minimum[2]));
      mesh.reset(shapes::createMeshFromResource("file://"+geometry.mesh_path,scale));if(!mesh){return false;}shapes::ShapeMsg message;shapes::constructMsgFromShape(mesh.get(),message);object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(message));object.mesh_poses.push_back(geometry.pose);
    }else{shape_msgs::msg::SolidPrimitive primitive;primitive.type=geometry.kind==geometry.SPHERE?primitive.SPHERE:primitive.BOX;
      if(primitive.type==primitive.SPHERE){primitive.dimensions={geometry.dimensions_m.x};}else{primitive.dimensions={geometry.dimensions_m.x,geometry.dimensions_m.y,geometry.dimensions_m.z};}object.primitives.push_back(primitive);object.primitive_poses.push_back(geometry.pose);}
    present.insert(object.id);objects.push_back(object);
  }
  for(const auto & track:goal.objects){
    const std::string id="tracked_"+std::to_string(track.key.track_id);present.insert(id);
    if(track.key.track_id==goal.attached_track_id && goal.operation==goal.UPDATE){continue;}
    moveit_msgs::msg::CollisionObject object;object.id=id;object.header.frame_id=config().frames.world;object.operation=object.ADD;
    const auto core_track=ptc_app::from_ros(track);const auto object_pose=core_track.observation.pose_world;
    const bool has_surface=grasp_surface&&grasp_surface->key.track_id==track.key.track_id&&grasp_surface->cloud_revision>0&&
      grasp_surface->points.header.frame_id=="object_"+std::to_string(track.key.track_id);
    const auto track_key=core_track.context.key;
    bool surface_refreshed=false;std::optional<CachedSurface> pending_surface;
    if(has_surface){
      const auto surface_key=ptc_app::from_ros(grasp_surface->key);
      auto cached=measured_surfaces.find(track.key.track_id);
      const bool same_identity=cached!=measured_surfaces.end()&&same_cycle(cached->second.key,surface_key);
      if(same_identity&&grasp_surface->cloud_revision<cached->second.revision){
        RCLCPP_WARN(node->get_logger(),"stale grasp surface revision rejected: track=%lu received=%lu cached=%lu",
          track.key.track_id,grasp_surface->cloud_revision,cached->second.revision);return false;
      }
      if(!same_identity||grasp_surface->cloud_revision!=cached->second.revision){
        const auto sampled=voxelize_surface(grasp_surface->points);
        if(!sampled){RCLCPP_WARN(node->get_logger(),"scene surface rejected: track=%lu revision=%lu points=%u",
          track.key.track_id,grasp_surface->cloud_revision,grasp_surface->points.width*grasp_surface->points.height);return false;}
        pending_surface=CachedSurface{surface_key,grasp_surface->cloud_revision,*sampled};
        pending_surface_commit=std::make_pair(track.key.track_id,*pending_surface);
        surface_refreshed=true;
      }
    }
    const auto cached=measured_surfaces.find(track.key.track_id);
    const VoxelizedSurface * sampled=nullptr;
    if(pending_surface){sampled=&pending_surface->geometry;}
    else if(cached!=measured_surfaces.end()&&same_track_identity(cached->second.key,track_key)){sampled=&cached->second.geometry;}
    if(sampled){
      const double radius=sampled->resolution_m*std::sqrt(3.)*.5;
      for(const auto & center_object:sampled->centers_object){
        shape_msgs::msg::SolidPrimitive sphere;sphere.type=sphere.SPHERE;sphere.dimensions={radius};object.primitives.push_back(sphere);
        Pose3d center;center.position={center_object.x(),center_object.y(),center_object.z()};const auto world=compose_pose(object_pose,center);if(!world){return false;}
        object.primitive_poses.push_back(ptc_app::to_ros(*world));
      }
      if(surface_refreshed){RCLCPP_INFO(node->get_logger(),"scene grasp surface: track=%lu revision=%lu measured=%zu collision_voxels=%zu resolution_m=%.4f",
        track.key.track_id,pending_surface->revision,sampled->finite_points,sampled->centers_object.size(),sampled->resolution_m);}
    }else{
      const auto & envelope=core_track.observation;
      if(!envelope.envelope_valid){throw std::runtime_error("scene object has no measured envelope: track="+std::to_string(track.key.track_id));}
      const double diameter=2*envelope.envelope_radius_xy_m;
      shape_msgs::msg::SolidPrimitive box;
      box.type=box.BOX;
      box.dimensions={diameter,diameter,2*envelope.envelope_half_height_m};
      object.primitives.push_back(box);
      Pose3d center{envelope.envelope_center_object_m,{}};
      const auto pose=compose_pose(object_pose,center);
      if(!pose){return false;}
      object.primitive_poses.push_back(ptc_app::to_ros(*pose));
    }
    objects.push_back(object);
  }
  for(const auto & id:known_objects){if(!present.count(id) &&
    (goal.operation!=goal.DETACH || (id!=detach_id && detach_world.count(id)))){
    moveit_msgs::msg::CollisionObject remove;remove.id=id;remove.operation=remove.REMOVE;objects.push_back(remove);}}
  moveit_msgs::msg::PlanningScene update;update.world.collision_objects=objects;
  if(goal.operation==goal.ATTACH){
    const auto object=std::find_if(goal.objects.begin(),goal.objects.end(),[&](const auto & item){return item.key.track_id==goal.attached_track_id;});if(object==goal.objects.end()){return false;}
    moveit_msgs::msg::AttachedCollisionObject attachment;attachment.link_name=config().frames.tcp;attachment.touch_links=grasp_contact_links();attachment.object.id="tracked_"+std::to_string(goal.attached_track_id);attachment.object.operation=attachment.object.ADD;
    // Referencing an existing world object preserves its measured pose on attachment.
    update.robot_state.attached_collision_objects.push_back(attachment);
  }else if(goal.operation==goal.DETACH){
    if(detach_attached){
      moveit_msgs::msg::AttachedCollisionObject detach;detach.link_name=config().frames.tcp;detach.object.id=detach_id;detach.object.operation=detach.object.REMOVE;
      moveit_msgs::msg::PlanningScene detach_update;detach_update.robot_state.attached_collision_objects.push_back(detach);if(!apply_scene(detach_update)){return false;}
    }
    if(detach_attached || detach_world.count(detach_id)){
      moveit_msgs::msg::CollisionObject remove;remove.id=detach_id;remove.operation=remove.REMOVE;update.world.collision_objects.push_back(remove);
    }
  }
  const bool demo=config().runtime.operation_mode=="assumed_ng_demo" &&
    config().grasp.moving_intercept;
  if(demo){
    // The commissioning tool may contact tracked models through grasp,
    // transfer and Empty/Home recovery. Switching owners or perception IDs
    // must not revoke this same tool-only policy. Static/self/arm collisions
    // and the simulator's physical contacts remain unchanged.
    if(!get_scene_client->service_is_ready())throw std::runtime_error("GetPlanningScene service unavailable: "+std::string(get_scene_client->get_service_name()));
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    auto response=get_scene_client->async_send_request(request);
    if(response.wait_for(std::chrono::duration<double>(config().motion.planning_timeout_sec))!=std::future_status::ready){
      get_scene_client->remove_pending_request(response);throw std::runtime_error("GetPlanningScene response timed out during scene synchronization");
    }
    const auto current=response.get()->scene;auto matrix=current.allowed_collision_matrix;
    std::vector<std::string> targets;
    if(goal.attached_track_id && goal.operation!=goal.DETACH)targets.push_back("tracked_"+std::to_string(goal.attached_track_id));
    for(const auto & id:present)if(id.rfind("tracked_",0)==0 && std::find(targets.begin(),targets.end(),id)==targets.end())targets.push_back(id);
    for(const auto & object:current.world.collision_objects){
      if(object.id.rfind("tracked_",0)==0 && std::find(targets.begin(),targets.end(),object.id)==targets.end())targets.push_back(object.id);
    }
    replace_grasp_touch_permissions(matrix,grasp_contact_links(),targets);
    update.allowed_collision_matrix=std::move(matrix);
  }
  // Commit the cache only after ApplyPlanningScene acknowledges the geometry.
  // A failed scene write must not make a later check believe this cloud
  // revision is already represented by the MoveIt world.
  if(!apply_scene(update)){return false;}
  if(pending_surface_commit){measured_surfaces[pending_surface_commit->first]=std::move(pending_surface_commit->second);}
  for(auto it=measured_surfaces.begin();it!=measured_surfaces.end();){
    if(!present.count("tracked_"+std::to_string(it->first))){it=measured_surfaces.erase(it);}else{++it;}
  }
  if(present!=known_objects){std::lock_guard<std::mutex> guard(mutex);
    if(following && target.producer=="grasp")contact_ready=false;
  }
  known_objects=std::move(present);return true;
}
}
std::shared_ptr<ptc_motion::MotionBackend> make_moveit_motion_backend(const std::shared_ptr<ptc_motion::ArmGatewayNode> & gateway)
{
  auto state=std::make_shared<MoveitBackendRuntime>(gateway);state->initialize();auto backend=std::make_shared<ptc_motion::MotionBackend>();
  backend->arm_snapshot=[state](const TimeContext & time){return state->arm(time);};backend->gripper_snapshot=[state](const TimeContext & time){return state->grip(time);};
  backend->planned_target=[state]()->std::optional<StampedPose3d>{std::lock_guard<std::mutex> lock(state->plan_mutex);return state->plan_valid?std::optional<StampedPose3d>{state->planned_endpoint}:std::nullopt;};
  backend->plan=[state](const MoveToPoseRequest & request){return state->plan_move_to_pose(request);};backend->execute=[state]{return state->execute_planned_trajectory();};backend->follow=[state](const Command & command){return state->follow(command);};
  backend->check_motion=[state](const ptc_motion::MotionCheckRequest & request){return state->check(request);};
  backend->evaluate_grasp_candidates=[state](const ptc_motion::GraspCandidatesRequest & request){return state->evaluate_grasp_candidates(request);};
  backend->stop_arm=[state]{state->stop_arm();};backend->stop_gripper=[state]{++state->gripper_cancellation;state->grip_client->async_cancel_all_goals();state->changed.notify_all();};
  backend->gripper=[state](double width,double effort,bool open,double timeout){return state->command_grip(width,effort,open,timeout);};backend->sync_scene=[state](const auto & goal){return state->synchronize(goal);};return backend;
}
}
