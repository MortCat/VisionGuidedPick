#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ptc_core/device_status.hpp"
#include "ptc_core/pose3d.hpp"

namespace ptc_core
{

enum class ArmMotionState : std::uint8_t
{
  kUnknown = 0,
  kIdle = 1,
  kValidating = 2,
  kPlanning = 3,
  kMoving = 4,
  kStopping = 5,
  kFault = 6,
  kNotReady = 7,
  kFollowing = 8,
};

struct ArmJointState
{
  std::string name;
  double position{0.0};
  std::optional<double> velocity;
  std::optional<double> effort;
};

struct ArmState
{
  std::chrono::nanoseconds timestamp{0};
  DeviceStatus status;
  ArmMotionState motion_state{ArmMotionState::kUnknown};
  std::string base_frame;
  std::string tcp_frame;
  Pose3d tcp_pose;
  std::vector<ArmJointState> joints;
};

}  // namespace ptc_core
