#include "ptc_core/pose_math.hpp"
#include <algorithm>
namespace ptc_core
{
std::optional<Pose3d> compose_pose(const Pose3d & a, const Pose3d & b)
{
  if (!is_valid_pose(a) || !is_valid_pose(b)) {return std::nullopt;}
  auto q = a.orientation; auto r = b.orientation; const auto & v = b.position;
  const auto normalize = [](Quaterniond & x) {const double n = std::sqrt(x.x*x.x+x.y*x.y+x.z*x.z+x.w*x.w); x.x /= n; x.y /= n; x.z /= n; x.w /= n;};
  normalize(q); normalize(r);
  const Vector3d uv{q.y*v.z_m-q.z*v.y_m,q.z*v.x_m-q.x*v.z_m,q.x*v.y_m-q.y*v.x_m};
  const Vector3d uuv{q.y*uv.z_m-q.z*uv.y_m,q.z*uv.x_m-q.x*uv.z_m,q.x*uv.y_m-q.y*uv.x_m};
  Pose3d p{{a.position.x_m+v.x_m+2*(q.w*uv.x_m+uuv.x_m),a.position.y_m+v.y_m+2*(q.w*uv.y_m+uuv.y_m),
      a.position.z_m+v.z_m+2*(q.w*uv.z_m+uuv.z_m)},
    {q.w*r.x+q.x*r.w+q.y*r.z-q.z*r.y,q.w*r.y-q.x*r.z+q.y*r.w+q.z*r.x,
      q.w*r.z+q.x*r.y-q.y*r.x+q.z*r.w,q.w*r.w-q.x*r.x-q.y*r.y-q.z*r.z}};
  return is_valid_pose(p) ? std::optional<Pose3d>(p) : std::nullopt;
}
std::optional<Pose3d> interpolate_pose(const Pose3d & from,const Pose3d & to,double fraction)
{
  if(!is_valid_pose(from)||!is_valid_pose(to)||!std::isfinite(fraction)||fraction<0||fraction>1){return std::nullopt;}
  auto a=from.orientation,b=to.orientation;
  const double an=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w),bn=std::sqrt(b.x*b.x+b.y*b.y+b.z*b.z+b.w*b.w);
  a={a.x/an,a.y/an,a.z/an,a.w/an};b={b.x/bn,b.y/bn,b.z/bn,b.w/bn};
  double dot=a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
  if(dot<0){b={-b.x,-b.y,-b.z,-b.w};dot=-dot;}
  dot=std::clamp(dot,0.,1.);double wa=1-fraction,wb=fraction;
  if(dot<.9995){const double angle=std::acos(dot),denominator=std::sin(angle);wa=std::sin((1-fraction)*angle)/denominator;wb=std::sin(fraction*angle)/denominator;}
  Quaterniond q{wa*a.x+wb*b.x,wa*a.y+wb*b.y,wa*a.z+wb*b.z,wa*a.w+wb*b.w};
  const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);q={q.x/n,q.y/n,q.z/n,q.w/n};
  Pose3d out{{from.position.x_m+(to.position.x_m-from.position.x_m)*fraction,from.position.y_m+(to.position.y_m-from.position.y_m)*fraction,from.position.z_m+(to.position.z_m-from.position.z_m)*fraction},q};
  return is_valid_pose(out)?std::optional<Pose3d>{out}:std::nullopt;
}
bool pose_near(const Pose3d & a, const Pose3d & b, double position, double orientation)
{
  if (!is_valid_pose(a) || !is_valid_pose(b) || !std::isfinite(position) || !std::isfinite(orientation) || position < 0 || orientation < 0) {return false;}
  const Vector3d delta{a.position.x_m-b.position.x_m,a.position.y_m-b.position.y_m,a.position.z_m-b.position.z_m};
  const auto & x = a.orientation; const auto & y = b.orientation;
  const double norm = std::sqrt((x.x*x.x+x.y*x.y+x.z*x.z+x.w*x.w)*(y.x*y.x+y.y*y.y+y.z*y.z+y.w*y.w));
  return std::hypot(delta.x_m,delta.y_m,delta.z_m) <= position &&
    2*std::acos(std::min(1.0,std::abs(x.x*y.x+x.y*y.y+x.z*y.z+x.w*y.w)/norm)) <= orientation;
}
std::optional<Pose3d> inverse_pose(const Pose3d & p)
{
  if (!is_valid_pose(p)) {return std::nullopt;}
  const auto & q = p.orientation;
  return compose_pose({{}, {-q.x,-q.y,-q.z,q.w}},{{-p.position.x_m,-p.position.y_m,-p.position.z_m},{}});
}
std::optional<Pose3d> camera_view_to_tcp(const Pose3d & camera, const Pose3d & camera_in_tcp)
{const auto inverse = inverse_pose(camera_in_tcp); return inverse ? compose_pose(camera,*inverse) : std::nullopt;}
}  // namespace ptc_core
