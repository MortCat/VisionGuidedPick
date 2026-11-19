#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using Trajectory = control_msgs::action::FollowJointTrajectory;
using Gripper = control_msgs::action::GripperCommand;
using Clock = std::chrono::steady_clock;

class GripperAdapterNode final : public rclcpp::Node {
public:
  explicit GripperAdapterNode(const std::string &arm)
      : Node("gripper_adapter", "/" + arm) {
    controller_ = rclcpp_action::create_client<Trajectory>(this, "arm_controller/follow_joint_trajectory");
    states_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          std::optional<double> left, right;
          for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
            if (message->name[i] == "panda_finger_joint1") left = message->position[i];
            if (message->name[i] == "panda_finger_joint2") right = message->position[i];
          }
          if (left && right && std::isfinite(*left) && std::isfinite(*right)) {
            const auto now=Clock::now();
            if (!measured_ || std::abs(*left-finger_stable_position_[0])>.0003) {
              finger_stable_since_[0]=now; finger_stable_position_[0]=*left;
            }
            if (!measured_ || std::abs(*right-finger_stable_position_[1])>.0003) {
              finger_stable_since_[1]=now; finger_stable_position_[1]=*right;
            }
            left_ = *left; right_ = *right; measured_ = now;
          }
        });
    server_ = rclcpp_action::create_server<Gripper>(
        this, "gripper_controller/gripper_cmd",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Gripper::Goal> goal) {
          // Standard position is ONE finger joint, not total jaw aperture.
          // The existing controller has no effort-limit command; reject unsupported requests.
          const auto &c = goal->command;
          if (active_ || reserved_ || !fresh() || !controller_->action_server_is_ready() ||
              !std::isfinite(c.position) || c.position < 0.0 || c.position > 0.04 ||
              !std::isfinite(c.max_effort) || c.max_effort != 0.0) {
            RCLCPP_WARN(get_logger(), "Gripper goal rejected: active=%d reserved=%d fresh=%d trajectory_ready=%d position=%.9f effort=%.9f",
                static_cast<bool>(active_), reserved_, fresh(), controller_->action_server_is_ready(), c.position, c.max_effort);
            return rclcpp_action::GoalResponse::REJECT;
          }
          reserved_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>>) {
          cancel_requested_ = true;
          if (trajectory_handle_) controller_->async_cancel_goal(trajectory_handle_);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>> handle) { execute(handle); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (!active_) return;
      if (fresh()) {
        auto feedback = std::make_shared<Gripper::Feedback>();
        feedback->position = (left_ + right_) / 2.0;
        feedback->reached_goal = std::abs(left_ - target_) < 0.001 && std::abs(right_ - target_) < 0.001;
        feedback->stalled = measured_stall();  // Joint stall is a grasp proxy, not a force sensor.
        active_->publish_feedback(feedback);
      }
      if ((!fresh() || Clock::now() - started_ > std::chrono::seconds(15)) && trajectory_handle_ && !cancel_requested_) {
        cancel_requested_ = true;
        controller_->async_cancel_goal(trajectory_handle_);
      }
    });
  }

private:
  bool fresh() const { return measured_ && Clock::now() - *measured_ < std::chrono::seconds(1); }
  bool finger_partially_closed(std::size_t index, double position) const {
    return fresh() && closing_ && finger_start_position_[index]-position>.0003 && position>target_+.002;
  }
  bool measured_stall() const {
    if (!fresh() || !closing_) return false;
    const auto now=Clock::now();
    const bool left_stalled=finger_partially_closed(0,left_) && finger_stable_since_[0] &&
      now-*finger_stable_since_[0]>=std::chrono::milliseconds(50);
    const bool right_stalled=finger_partially_closed(1,right_) && finger_stable_since_[1] &&
      now-*finger_stable_since_[1]>=std::chrono::milliseconds(50);
    return left_stalled || right_stalled;
  }
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>> handle) {
    active_ = handle; reserved_ = false; cancel_requested_ = false; started_ = Clock::now();
    target_ = handle->get_goal()->command.position;
    if (fresh() && std::abs(left_-target_)<.001 && std::abs(right_-target_)<.001) {
      // Measured arrival is already satisfied; no one-second no-op trajectory.
      finish(true,false);return;
    }
    closing_=target_<(left_+right_)/2.0-.002;
    const auto close_started=Clock::now();
    finger_start_position_={left_,right_}; finger_stable_position_=finger_start_position_;
    for (auto & stable_since : finger_stable_since_) stable_since=close_started;
    Trajectory::Goal goal;
    goal.trajectory.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint first, last;
    // Isaac solver penetration can make raw closed-joint positions negative.
    // Keep measured feedback intact, but start a valid opening command at the
    // physical closed stop instead of repeatedly submitting an invalid knot.
    first.positions = {std::clamp(left_,0.,.04), std::clamp(right_,0.,.04)};
    last.positions = {target_, target_};
    // A full 40 mm finger stroke takes 0.4 s at 0.1 m/s. Use the actual
    // remaining stroke, rather than spending one second on every command.
    // This is command timing only; measured arrival/stall still decides
    // completion. The backend combines stall with partial aperture as contact.
    const double travel = std::max(std::abs(first.positions[0]-target_),std::abs(first.positions[1]-target_));
    const double duration = std::max(.1,travel/.1);
    const auto nanoseconds = static_cast<std::int64_t>(std::ceil(duration*1e9));
    last.time_from_start.sec = static_cast<std::int32_t>(nanoseconds/1000000000);
    last.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds%1000000000);
    goal.trajectory.points = {first, last};
    auto options = rclcpp_action::Client<Trajectory>::SendGoalOptions();
    options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<Trajectory>::SharedPtr handle) {
      trajectory_handle_ = handle;
      if (!handle) {
        RCLCPP_WARN(get_logger(), "Finger trajectory goal was rejected by bridge; target=%.9f measured=[%.9f,%.9f]", target_, left_, right_);
        finish(false, false);
      }
      else if (cancel_requested_) controller_->async_cancel_goal(handle);
    };
    options.result_callback = [this](const rclcpp_action::ClientGoalHandle<Trajectory>::WrappedResult &result) {
      const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                           result.result->error_code == Trajectory::Result::SUCCESSFUL;
      // This bridge reports closing stall as GOAL_TOLERANCE_VIOLATED;
      // stale feedback/reset is PATH_TOLERANCE_VIOLATED. Do not require a
      // second differently-timed stall window after that measured result.
      const bool blocked=result.code==rclcpp_action::ResultCode::ABORTED && result.result &&
          result.result->error_code==Trajectory::Result::GOAL_TOLERANCE_VIOLATED && !cancel_requested_ &&
          (finger_partially_closed(0,left_) || finger_partially_closed(1,right_));
      if (!success) RCLCPP_WARN(get_logger(), "Finger trajectory result code=%d error=%d message=%s measured=[%.9f,%.9f] target=%.9f stalled=%d",
          static_cast<int>(result.code), result.result?result.result->error_code:0,
          result.result?result.result->error_string.c_str():"no result", left_, right_, target_, measured_stall());
      finish(success || blocked, result.code == rclcpp_action::ResultCode::CANCELED, blocked);
    };
    controller_->async_send_goal(goal, options);
  }
  void finish(bool success, bool canceled, bool controller_blocked=false) {
    if (!active_) return;
    auto result = std::make_shared<Gripper::Result>();
    result->position = (left_ + right_) / 2.0;
    result->reached_goal = fresh() && std::abs(left_ - target_) < 0.002 && std::abs(right_ - target_) < 0.002;
    result->stalled = controller_blocked || measured_stall();
    if (active_->is_canceling() && canceled) active_->canceled(result);
    else if (success && (result->reached_goal || result->stalled)) active_->succeed(result);
    else active_->abort(result);
    active_.reset(); trajectory_handle_.reset(); cancel_requested_ = false;
  }
  double left_{0.0}, right_{0.0}, target_{0.0};
  bool reserved_{false}, cancel_requested_{false}, closing_{false};
  std::optional<Clock::time_point> measured_;
  std::array<std::optional<Clock::time_point>,2> finger_stable_since_{};
  std::array<double,2> finger_start_position_{},finger_stable_position_{};
  Clock::time_point started_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_;
  rclcpp_action::Client<Trajectory>::SharedPtr controller_;
  rclcpp_action::Server<Gripper>::SharedPtr server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>> active_;
  rclcpp_action::ClientGoalHandle<Trajectory>::SharedPtr trajectory_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
};
