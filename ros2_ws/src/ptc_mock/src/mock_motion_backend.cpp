#include "ptc_mock/mock_motion_backend.hpp"
#include "ptc_app/robot_description_loader.hpp"
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>

namespace ptc_mock
{
namespace
{
struct MockMotionState
{
  explicit MockMotionState(const std::shared_ptr<ptc_motion::ArmGatewayNode> & node)
  : node(node),settings(node->settings().snapshot->get()),tf(node->get_clock()),listener(tf)
  {const auto robot=ptc_app::load_robot_description(settings); joints=robot.arm_joint_names; pose=settings.motion.observation_pose; width=settings.grasp.open_width_m;}
  std::weak_ptr<ptc_motion::ArmGatewayNode> node;
  ptc_core::SystemSettings settings;
  tf2_ros::Buffer tf; tf2_ros::TransformListener listener;
  std::mutex mutex; ptc_core::Pose3d pose,target; std::vector<std::string> joints;
  ptc_core::StampedPose3d planned_endpoint;
  double width{0}; bool stopped{true},plan_valid{false}; std::uint64_t evidence_sequence{0};
};
ptc_core::DeviceStatus healthy()
{ptc_core::DeviceStatus out; out.ready=true; out.data_stale=false; out.connection_state=ptc_core::DeviceConnectionState::kConnected; out.health_state=ptc_core::DeviceHealthState::kHealthy; out.message="synthetic mock backend, not actual hardware"; return out;}
}
std::shared_ptr<ptc_motion::MotionBackend> make_mock_motion_backend(const std::shared_ptr<ptc_motion::ArmGatewayNode> & node)
{
  if (node->settings().snapshot->get().runtime.motion_backend!="mock") {throw std::runtime_error("mock backend requires explicit runtime.motion_backend=mock");}
  auto state=std::make_shared<MockMotionState>(node); auto backend=std::make_shared<ptc_motion::MotionBackend>();
  backend->arm_snapshot=[state](const ptc_core::TimeContext & time) {
    std::lock_guard<std::mutex> lock(state->mutex); ptc_core::ArmSnapshot out; out.epoch=time.epoch;
    out.joint_timestamp=out.tcp_timestamp=out.state.timestamp=time.source_now; out.controller_ready=true; out.tf_valid=true; out.stopped=state->stopped;
    out.state.status=healthy(); out.state.tcp_pose=state->pose; out.state.base_frame=state->settings.frames.robot_base; out.state.tcp_frame=state->settings.frames.tcp;
    out.state.motion_state=state->stopped ? ptc_core::ArmMotionState::kIdle : ptc_core::ArmMotionState::kFollowing;
    for (const auto & joint : state->joints) {out.state.joints.push_back({joint,0,0,0});} return out;
  };
  backend->gripper_snapshot=[state](const ptc_core::TimeContext & time) {
    std::lock_guard<std::mutex> lock(state->mutex); ptc_core::GripperState out; out.epoch=time.epoch; out.timestamp=time.source_now;
    out.evidence_sequence=++state->evidence_sequence; out.status=healthy(); out.width_m=state->width; out.stopped=true;
    out.empty_confirmed=state->width>=state->settings.grasp.open_width_m; return out;
  };
  backend->plan=[state](const ptc_core::MoveToPoseRequest & request) {
    const auto node=state->node.lock();if(!node){return false;}
    const auto target=ptc_core::predict_move_target(request,ptc_core::Nanoseconds{node->now().nanoseconds()});
    std::lock_guard<std::mutex> lock(state->mutex);state->plan_valid=target.has_value();
    if(target){state->planned_endpoint=*target;state->target=target->pose;}return state->plan_valid;
  };
  backend->planned_target=[state]()->std::optional<ptc_core::StampedPose3d>{std::lock_guard<std::mutex> lock(state->mutex);return state->plan_valid?std::optional<ptc_core::StampedPose3d>{state->planned_endpoint}:std::nullopt;};
  backend->execute=[state] {std::lock_guard<std::mutex> lock(state->mutex); if (!state->plan_valid) {return false;} state->pose=state->target; state->stopped=true; state->plan_valid=false; return true;};
  backend->follow=[state](const ptc_core::Command & command) {
    try {
      auto input=ptc_app::stamped_pose(command.target,command.reference_frame,command.source_timestamp);
      const auto output=state->tf.transform(input,state->settings.frames.robot_base,tf2::durationFromSec(state->settings.vision.tf_lookup_timeout_sec));
      std::lock_guard<std::mutex> lock(state->mutex); state->pose=ptc_app::from_ros(output.pose); state->stopped=false; return true;
    } catch (...) {return false;}
  };
  backend->stop_arm=[state] {std::lock_guard<std::mutex> lock(state->mutex); state->stopped=true; state->plan_valid=false;};
  backend->stop_gripper=[] {};
  backend->gripper=[state](double width,double,bool,double) {std::lock_guard<std::mutex> lock(state->mutex); state->width=width; return true;};
  backend->sync_scene=[](const ptc_interfaces::action::SyncScene::Goal &) {return true;};
  backend->check_motion=[](const ptc_motion::MotionCheckRequest & request) {
    return !request.targets.empty()&&std::all_of(request.targets.begin(),request.targets.end(),[](const auto & target){return ptc_core::is_valid_pose(target.pose);});
  };
  return backend;
}
}  // namespace ptc_mock
