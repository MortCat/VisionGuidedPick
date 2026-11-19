#pragma once
#include "ptc_core/pose_math.hpp"
#include <algorithm>

namespace ptc_core
{
// Live measurements adjust a continuous relative offset over the existing
// preview horizon. Preserve position and velocity when rebasing on a new frame.
struct GraspOffsetRamp
{
  Vector3d from,to,from_velocity,to_velocity;double start_sec{0},duration_sec{.2};
  Vector3d position(double now)const
  {
    const double elapsed=now-start_sec;
    if(elapsed>=duration_sec){const double dt=elapsed-duration_sec;return {to.x_m+dt*to_velocity.x_m,to.y_m+dt*to_velocity.y_m,to.z_m+dt*to_velocity.z_m};}
    const double u=std::max(0.,elapsed/duration_sec),h=duration_sec;
    const auto sample=[&](double a,double b,double va,double vb){return (2*u*u*u-3*u*u+1)*a+(u*u*u-2*u*u+u)*h*va+(-2*u*u*u+3*u*u)*b+(u*u*u-u*u)*h*vb;};
    return {sample(from.x_m,to.x_m,from_velocity.x_m,to_velocity.x_m),sample(from.y_m,to.y_m,from_velocity.y_m,to_velocity.y_m),sample(from.z_m,to.z_m,from_velocity.z_m,to_velocity.z_m)};
  }
  Vector3d velocity(double now)const
  {
    const double u=std::clamp((now-start_sec)/duration_sec,0.,1.),h=duration_sec;
    const auto sample=[&](double a,double b,double va,double vb){return ((6*u*u-6*u)*a+(3*u*u-4*u+1)*h*va+(-6*u*u+6*u)*b+(3*u*u-2*u)*h*vb)/h;};
    return {sample(from.x_m,to.x_m,from_velocity.x_m,to_velocity.x_m),sample(from.y_m,to.y_m,from_velocity.y_m,to_velocity.y_m),sample(from.z_m,to.z_m,from_velocity.z_m,to_velocity.z_m)};
  }
  void rebase(double now,const Vector3d & target,const Vector3d & target_velocity,double horizon)
  {const auto p=position(now),v=velocity(now);from=p;from_velocity=v;to=target;to_velocity=target_velocity;start_sec=now;duration_sec=horizon;}
};
}  // namespace ptc_core
