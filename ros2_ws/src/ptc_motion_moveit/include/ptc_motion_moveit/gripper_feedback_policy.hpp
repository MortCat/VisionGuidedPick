#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace ptc_motion_moveit
{
struct FingerPositionSample
{
  double seconds;
  std::vector<double> positions;
};
struct FingerMotionEstimate
{
  std::vector<double> velocity_mps;
  bool stopped;
};
// Joint-local positions exclude arm/world motion. Range detects reversals
// that disappear in an endpoint-only derivative or combined aperture.
inline std::optional<FingerMotionEstimate> estimate_finger_motion(
  const std::vector<FingerPositionSample> & samples, double stop_velocity_mps)
{
  constexpr double window_sec=.1;
  if(samples.size()<3 || !std::isfinite(stop_velocity_mps) || stop_velocity_mps<0)return std::nullopt;
  const auto count=samples.front().positions.size();
  if(!count)return std::nullopt;
  const double span=samples.back().seconds-samples.front().seconds;
  if(!std::isfinite(span) || span<window_sec || span>2*window_sec)return std::nullopt;
  for(std::size_t k=0;k<samples.size();++k){
    const auto & sample=samples[k];
    if(!std::isfinite(sample.seconds) || sample.positions.size()!=count)return std::nullopt;
    if(k && (sample.seconds<=samples[k-1].seconds || sample.seconds-samples[k-1].seconds>window_sec))return std::nullopt;
    for(double q:sample.positions)if(!std::isfinite(q))return std::nullopt;
  }
  FingerMotionEstimate out;out.stopped=true;
  for(std::size_t j=0;j<count;++j){
    double low=samples.front().positions[j],high=low;
    for(const auto & sample:samples){low=std::min(low,sample.positions[j]);high=std::max(high,sample.positions[j]);}
    out.velocity_mps.push_back((samples.back().positions[j]-samples.front().positions[j])/span);
    if((high-low)/span>stop_velocity_mps)out.stopped=false;
  }
  return out;
}
// Keep contact evidence until a subsequent gripper command actually succeeds.
inline void clear_latched_possession_if_closed_empty(bool closed_empty, bool & contact, bool & unknown)
{
  if (closed_empty) {
    contact = false;
    unknown = false;
  }
}
// A full open, unloaded on both jaws across the stop window, is direct
// evidence of release even when an earlier gripper action had no result.
inline bool measured_full_open_empty(bool stopped, bool low_force,
  const std::vector<FingerPositionSample> & samples, double open_joint, double tolerance)
{
  return stopped && low_force && !samples.empty() &&
    std::all_of(samples.begin(),samples.end(),[&](const auto & sample){
      return sample.positions.size()==2 &&
        std::all_of(sample.positions.begin(),sample.positions.end(),[&](double value){
          return std::abs(value-open_joint)<=tolerance;
        });
    });
}
inline bool contact_after_command(bool prior_contact, bool success, bool open, bool stalled)
{
  return success ? (!open && stalled) : prior_contact;
}
inline bool measured_empty(bool stopped, bool latched_contact, double width, double open_width, double tolerance)
{
  return stopped && !latched_contact && width >= open_width - tolerance;
}
}
