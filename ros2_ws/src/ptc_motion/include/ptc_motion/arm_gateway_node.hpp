#pragma once
#include "ptc_motion/motion_backend.hpp"
#include "ptc_app/bounded_worker.hpp"
#include "ptc_app/node_runtime.hpp"
#include "ptc_app/ros_converter.hpp"
#include "ptc_interfaces/action/move_to_pose.hpp"
#include "ptc_interfaces/action/follow_target.hpp"
#include "ptc_interfaces/action/gripper_command.hpp"
#include "ptc_interfaces/action/check_motion.hpp"
#include "ptc_interfaces/action/evaluate_grasp_candidates.hpp"
#include "ptc_interfaces/msg/target_update.hpp"
#include "ptc_interfaces/srv/stop_arm.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>

namespace ptc_motion
{
class ArmGatewayNode : public rclcpp::Node
{
public:
  explicit ArmGatewayNode(const rclcpp::NodeOptions & = rclcpp::NodeOptions{});
  ~ArmGatewayNode();
  void set_backend(const std::shared_ptr<MotionBackend> & backend) {backend_=backend;}
  void stop_runtime();
  const ptc_app::SettingsLoadResult & settings() const {return settings_;}
private:
  using Move=ptc_interfaces::action::MoveToPose;
  using Follow=ptc_interfaces::action::FollowTarget;
  using Grip=ptc_interfaces::action::GripperCommand;
  using Sync=ptc_interfaces::action::SyncScene;
  using Check=ptc_interfaces::action::CheckMotion;
  using Evaluate=ptc_interfaces::action::EvaluateGraspCandidates;
  ptc_core::MotionDecision apply(ptc_core::MotionEvent event);
  ptc_core::MotionEvent event(ptc_core::MotionEventKind kind);
  void execute_commands(const std::vector<ptc_core::Command> &);
  void complete_action(const ptc_core::MoveToPoseResult &);
  void tick();
  void stop();
  ptc_core::StampedPose3d transform(const geometry_msgs::msg::PoseStamped &, const std::string &);
  void worker_event(ptc_core::EventKey key,ptc_core::MotionEventKind kind);
  void update(const ptc_interfaces::msg::TargetUpdate &);
  std::optional<ptc_interfaces::msg::TargetUpdate> pending_target_update_;
  std::optional<ptc_core::EventKey> pending_execution_completion_;
  ptc_app::SettingsLoadResult settings_;
  ptc_app::TimeAdapter time_;
  ptc_core::MotionState motion_;
  ptc_core::SceneState scene_;
  std::weak_ptr<MotionBackend> backend_;
  ptc_app::CompletionQueue completions_;
  ptc_app::BoundedWorker worker_;
  ptc_app::BoundedWorker gripper_worker_{2};
  ptc_app::BoundedWorker scene_worker_{2};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::uint64_t next_operation_{1},next_sequence_{1},follow_command_id_{0},last_target_sequence_{0};
  bool gripper_busy_{false},scene_busy_{false},stopped_runtime_{false};
  bool follow_grasp_{false};std::atomic<std::uint64_t> cancellation_generation_{1};
  bool check_busy_{false};ptc_core::Nanoseconds check_deadline{0};
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Check>> check_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Evaluate>> evaluate_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> move_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> follow_goal_;
  std::vector<ptc_core::Command> accepted_commands_;
  ptc_core::EventKey scene_grasp_surface_key_;std::uint64_t scene_grasp_surface_revision_{0};
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Follow>::SharedPtr follow_server_;
  rclcpp_action::Server<Grip>::SharedPtr grip_server_;
  rclcpp_action::Server<Sync>::SharedPtr scene_server_;
  rclcpp_action::Server<Check>::SharedPtr check_server_;
  rclcpp_action::Server<Evaluate>::SharedPtr evaluate_server_;
  rclcpp::Subscription<ptc_interfaces::msg::TargetUpdate>::SharedPtr target_subscription_;
  rclcpp::Service<ptc_interfaces::srv::StopArm>::SharedPtr stop_service_;
  rclcpp::Publisher<ptc_interfaces::msg::ArmStatus>::SharedPtr arm_publisher_;
  rclcpp::Publisher<ptc_interfaces::msg::GripperStatus>::SharedPtr gripper_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ptc_motion
