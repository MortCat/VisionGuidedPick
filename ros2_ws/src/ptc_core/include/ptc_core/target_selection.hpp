#pragma once
#include "ptc_core/config/system_settings.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
namespace ptc_core
{
// Selection only: active tracking and orbit updates do not use this region.
inline bool is_in_target_selection_roi(const Vector3d & p,const TaskConfig & task)
{
  if(!task.target_selection_roi){return true;}
  const auto & r=*task.target_selection_roi;
  return p.x_m>=r.bounds_min_m.x_m&&p.x_m<=r.bounds_max_m.x_m&&
    p.y_m>=r.bounds_min_m.y_m&&p.y_m<=r.bounds_max_m.y_m&&
    p.z_m>=r.bounds_min_m.z_m&&p.z_m<=r.bounds_max_m.z_m;
}
// Ray/box entry time from the current visual pose and velocity. This is a
// planning lead, not an expanded execution ROI or a refreshed observation.
inline std::optional<double> time_until_target_selection_roi(const Vector3d & p,
  const Vector3d & velocity,const TaskConfig & task,double horizon_sec)
{
  if(!std::isfinite(horizon_sec)||horizon_sec<0)return std::nullopt;
  const std::array<double,3> x{p.x_m,p.y_m,p.z_m},v{velocity.x_m,velocity.y_m,velocity.z_m};
  for(unsigned axis=0;axis<3;++axis){if(!std::isfinite(x[axis])||!std::isfinite(v[axis]))return std::nullopt;}
  if(!task.target_selection_roi)return 0.;
  const auto & roi=*task.target_selection_roi;
  const std::array<double,3> lo{roi.bounds_min_m.x_m,roi.bounds_min_m.y_m,roi.bounds_min_m.z_m};
  const std::array<double,3> hi{roi.bounds_max_m.x_m,roi.bounds_max_m.y_m,roi.bounds_max_m.z_m};
  double entry=0.,exit=horizon_sec;
  for(unsigned axis=0;axis<3;++axis){
    if(std::abs(v[axis])<1e-9){if(x[axis]<lo[axis]||x[axis]>hi[axis])return std::nullopt;continue;}
    const double a=(lo[axis]-x[axis])/v[axis],b=(hi[axis]-x[axis])/v[axis];
    entry=std::max(entry,std::min(a,b));exit=std::min(exit,std::max(a,b));
    if(entry>exit)return std::nullopt;
  }
  return entry;
}

}
