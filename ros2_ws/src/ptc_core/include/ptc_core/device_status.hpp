#pragma once

#include <cstdint>
#include <string>

namespace ptc_core
{

enum class DeviceConnectionState : std::uint8_t
{
  kUnknown = 0,
  kDisconnected = 1,
  kConnecting = 2,
  kConnected = 3,
};

enum class DeviceHealthState : std::uint8_t
{
  kUnknown = 0,
  kHealthy = 1,
  kDegraded = 2,
  kFault = 3,
};

struct DeviceStatus
{
  DeviceConnectionState connection_state{DeviceConnectionState::kUnknown};
  DeviceHealthState health_state{DeviceHealthState::kUnknown};
  bool ready{false};
  bool data_stale{true};
  std::int32_t error_code{0};
  std::string message;
};

}  // namespace ptc_core
