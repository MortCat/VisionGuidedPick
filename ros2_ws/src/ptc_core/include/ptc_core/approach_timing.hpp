#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
namespace ptc_core {
// Rest-to-rest estimate only; the actual collision-checked trajectory remains authoritative.
inline std::optional<double> estimate_joint_travel_duration(double distance,double velocity,double acceleration)
{
  if(!std::isfinite(distance)||distance<0||!std::isfinite(velocity)||velocity<=0||!std::isfinite(acceleration)||acceleration<0)return std::nullopt;
  if(distance==0)return 0.;
  if(acceleration==0)return distance/velocity; // Explicit velocity-only lower bound when acceleration limits are unavailable.
  const double ramp_distance=velocity*velocity/acceleration;
  return distance<=ramp_distance?2*std::sqrt(distance/acceleration):distance/velocity+velocity/acceleration;
}
inline bool observation_entry_follow_elapsed(std::chrono::nanoseconds started,std::chrono::nanoseconds now,double seconds)
{return std::isfinite(seconds)&&seconds>=0&&now>=started&&std::chrono::duration<double>(now-started).count()>=seconds;}
}
