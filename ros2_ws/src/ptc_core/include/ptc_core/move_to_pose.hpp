#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <optional>

#include "ptc_core/pose3d.hpp"

namespace ptc_core
{

enum class MoveToPosePhase : std::uint8_t
{
  kUnknown = 0,
  kValidating = 1,
  kPlanning = 2,
  kExecuting = 3,
  kCanceling = 4,
};

enum class MoveToPoseError : std::int32_t
{
  kSuccess = 0,
  kInvalidGoal = 1,
  kNotReady = 2,
  kBusy = 3,
  kTfUnavailable = 4,
  kPlanningFailed = 5,
  kExecutionFailed = 6,
  kTimedOut = 7,
  kCanceled = 8,
  kInternalError = 9,
};

struct MoveToPoseRequest
{
  StampedPose3d target_pose;
  double max_velocity_scale{1.0};
  double max_acceleration_scale{1.0};
  std::chrono::nanoseconds timeout{0};
  Vector3d target_linear_velocity;
  std::chrono::nanoseconds prediction_horizon{0};
};

struct MoveToPoseFeedback
{
  MoveToPosePhase phase{MoveToPosePhase::kUnknown};
  StampedPose3d current_pose;
  float progress{-1.0F};
};

struct MoveToPoseResult
{
  bool success{false};
  MoveToPoseError error_code{MoveToPoseError::kInternalError};
  std::string message;
  StampedPose3d final_pose;
};

inline bool is_valid_motion_scale(double scale)
{
  return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
}

inline bool is_valid_move_to_pose_request(const MoveToPoseRequest & request)
{
  return
    !request.target_pose.reference_frame.empty() &&
    is_valid_pose(request.target_pose.pose) &&
    is_valid_motion_scale(request.max_velocity_scale) &&
    is_valid_motion_scale(request.max_acceleration_scale) &&
    request.timeout > std::chrono::nanoseconds::zero() &&
    request.prediction_horizon >= std::chrono::nanoseconds::zero() &&
    std::isfinite(request.target_linear_velocity.x_m) &&
    std::isfinite(request.target_linear_velocity.y_m) &&
    std::isfinite(request.target_linear_velocity.z_m) &&
    (request.prediction_horizon == std::chrono::nanoseconds::zero() || request.target_pose.timestamp.count() > 0);
}

// Source-clock math only: no ROS, controller or object identity dependencies.
inline std::optional<StampedPose3d> predict_move_target(
  const MoveToPoseRequest & request, std::chrono::nanoseconds target_time)
{
  if (!is_valid_move_to_pose_request(request)) {return std::nullopt;}
  if (request.prediction_horizon.count() == 0) {return request.target_pose;}
  const auto elapsed = target_time - request.target_pose.timestamp;
  if (elapsed.count() < 0 || elapsed > request.prediction_horizon) {return std::nullopt;}
  auto result = request.target_pose;
  const double seconds = std::chrono::duration<double>(elapsed).count();
  result.pose.position.x_m += request.target_linear_velocity.x_m * seconds;
  result.pose.position.y_m += request.target_linear_velocity.y_m * seconds;
  result.pose.position.z_m += request.target_linear_velocity.z_m * seconds;
  result.timestamp = target_time;
  return is_valid_pose(result.pose) ? std::optional<StampedPose3d>{result} : std::nullopt;
}
}  // namespace ptc_core
