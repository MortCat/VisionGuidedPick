#include "ptc_core/vision/grasp_pose_model.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <cmath>

namespace ptc_core
{
namespace
{
Vector3d subtract(const Vector3d & a,const Vector3d & b)
{return {a.x_m-b.x_m,a.y_m-b.y_m,a.z_m-b.z_m};}
}

bool grasp_center_aligned(const Pose3d & measured,const Pose3d & desired,
  const Vector3d & center,double speed,double max_speed,double position_tolerance,double angle_tolerance)
{
  if(!is_valid_pose(measured)||!is_valid_pose(desired)||!std::isfinite(speed)||speed<0||
    !std::isfinite(max_speed)||max_speed<0||speed>max_speed||
    !std::isfinite(center.x_m)||!std::isfinite(center.y_m)||!std::isfinite(center.z_m))return false;
  auto actual=measured;auto target=desired;
  actual.position=transform_point(measured,center);target.position=transform_point(desired,center);
  return pose_near(actual,target,position_tolerance,angle_tolerance);
}

bool grasp_descent_height_reached(const Pose3d & measured,const Pose3d & desired,
  const Vector3d & center,double tolerance)
{
  if(!is_valid_pose(measured)||!is_valid_pose(desired)||!std::isfinite(tolerance)||tolerance<0||
    !std::isfinite(center.x_m)||!std::isfinite(center.y_m)||!std::isfinite(center.z_m))return false;
  return transform_point(measured,center).z_m<=transform_point(desired,center).z_m+tolerance;
}

std::optional<GraspApproachReference> make_grasp_approach_reference(const Pose3d & start,
  const Pose3d & goal,Nanoseconds stamp,double linear,double angular,const Vector3d & velocity,double minimum_duration)
{
  if(!is_valid_pose(start)||!is_valid_pose(goal)||stamp.count()<0||!std::isfinite(linear)||!std::isfinite(angular)||angular<=0||!std::isfinite(minimum_duration)||minimum_duration<0)return std::nullopt;
  const double available=linear-vector_norm(velocity);if(!std::isfinite(available)||available<=0)return std::nullopt;
  const auto & a=start.orientation;const auto & b=goal.orientation;
  const double norm=std::sqrt((a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w)*(b.x*b.x+b.y*b.y+b.z*b.z+b.w*b.w));
  const double angle=2*std::acos(std::clamp(std::abs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w)/norm,0.,1.));
  // Smoothstep has peak derivative 1.5; include object feed-forward in the cap.
  const double duration=std::max({1e-6,minimum_duration,1.5*vector_norm(subtract(goal.position,start.position))/available,1.5*angle/angular});
  return GraspApproachReference{start,goal,stamp,duration};
}
std::optional<GraspApproachSample> sample_grasp_approach(const GraspApproachReference & ref,
  const Pose3d & object,Nanoseconds source)
{
  if(!std::isfinite(ref.duration_sec)||ref.duration_sec<=0||source<ref.source_start)return std::nullopt;
  const double u=std::clamp(std::chrono::duration<double>(source-ref.source_start).count()/ref.duration_sec,0.,1.);
  const auto local=interpolate_pose(ref.start_in_object,ref.goal_in_object,u*u*(3-2*u));
  const auto world=local?compose_pose(object,*local):std::nullopt;if(!world)return std::nullopt;
  const double rate=6*u*(1-u)/ref.duration_sec;Pose3d rotation;rotation.orientation=object.orientation;
  auto linear=subtract(ref.goal_in_object.position,ref.start_in_object.position);linear={linear.x_m*rate,linear.y_m*rate,linear.z_m*rate};
  const auto inverse=inverse_pose(ref.start_in_object);const auto delta=inverse?compose_pose(ref.goal_in_object,*inverse):std::nullopt;if(!delta)return std::nullopt;
  auto q=delta->orientation;if(q.w<0)q={-q.x,-q.y,-q.z,-q.w};
  const double sine=std::hypot(q.x,q.y,q.z);const double scale=sine>1e-12?2*std::atan2(sine,q.w)*rate/sine:0.;
  return GraspApproachSample{*world,transform_point(rotation,linear),transform_point(rotation,{q.x*scale,q.y*scale,q.z*scale}),u};
}
std::optional<GraspMotionReference> make_grasp_motion_reference(const Pose3d & initial,
  const Pose3d & object,const Vector3d & velocity,Nanoseconds stamp,const Pose3d & pregrasp,
  const Pose3d & contact,double lift,double linear,double angular,double minimum,double follow,
  std::optional<double> descent_linear,std::optional<double> probe_linear)
{
  if(!is_valid_pose(initial)||!is_valid_pose(object)||!is_valid_pose(pregrasp)||!is_valid_pose(contact)||
    !std::isfinite(lift)||lift<=0||!std::isfinite(follow)||follow<0||!std::isfinite(minimum)||minimum<=0)return std::nullopt;
  const auto endpoint=compose_pose(object,pregrasp);if(!endpoint)return std::nullopt;
  auto entry=make_grasp_approach_reference(initial,*endpoint,stamp,linear,angular,velocity,minimum);
  auto down=make_grasp_approach_reference(pregrasp,contact,stamp,descent_linear.value_or(linear),angular,velocity,minimum);
  const auto inverse=inverse_pose(object);if(!inverse)return std::nullopt;
  auto raised=*compose_pose(object,contact);raised.position.z_m+=lift;
  const auto local_lift=compose_pose(*inverse,raised);if(!local_lift)return std::nullopt;
  auto up=make_grasp_approach_reference(contact,*local_lift,stamp,probe_linear.value_or(linear),angular,velocity,minimum);
  if(!entry||!down||!up)return std::nullopt;
  // Quintic peak derivative is 1.875 instead of cubic's 1.5. Include the
  // estimated arrival displacement when choosing the approach duration.
  for(unsigned pass=0;pass<2;++pass){
    auto future=*endpoint;future.position.x_m+=velocity.x_m*entry->duration_sec;
    future.position.y_m+=velocity.y_m*entry->duration_sec;future.position.z_m+=velocity.z_m*entry->duration_sec;
    entry=make_grasp_approach_reference(initial,future,stamp,linear,angular,velocity,minimum);
    if(!entry)return std::nullopt;
  }
  return GraspMotionReference{initial,object,pregrasp,contact,*local_lift,velocity,stamp,
    entry->duration_sec*1.25,follow,down->duration_sec*1.25,up->duration_sec*1.25,std::nullopt};
}
namespace
{
Vector3d add_vector(const Vector3d & a,const Vector3d & b)
{return {a.x_m+b.x_m,a.y_m+b.y_m,a.z_m+b.z_m};}
Vector3d scale_vector(const Vector3d & a,double k)
{return {a.x_m*k,a.y_m*k,a.z_m*k};}
Vector3d cross_vector(const Vector3d & a,const Vector3d & b)
{return {a.y_m*b.z_m-a.z_m*b.y_m,a.z_m*b.x_m-a.x_m*b.z_m,a.x_m*b.y_m-a.y_m*b.x_m};}
bool finite_vector(const Vector3d & a)
{return std::isfinite(a.x_m)&&std::isfinite(a.y_m)&&std::isfinite(a.z_m);}
Quaterniond rotation_exp(const Vector3d & phi)
{
  const double angle=vector_norm(phi),factor=angle>1e-10?std::sin(angle*.5)/angle:.5-angle*angle/48.;
  return {phi.x_m*factor,phi.y_m*factor,phi.z_m*factor,std::cos(angle*.5)};
}
Vector3d rotation_log(Quaterniond q)
{
  const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  q={q.x/norm,q.y/norm,q.z/norm,q.w/norm};
  if(q.w<0)q={-q.x,-q.y,-q.z,-q.w};
  // A deterministic axis for an exact pi tie; no blanket pi-angle rejection.
  if(std::abs(q.w)<1e-14){
    const double axis=std::abs(q.x)>1e-14?q.x:std::abs(q.y)>1e-14?q.y:q.z;
    if(axis<0)q={-q.x,-q.y,-q.z,-q.w};
  }
  const double sine=std::hypot(q.x,q.y,q.z),factor=sine>1e-12?2.*std::atan2(sine,std::abs(q.w))/sine:2.;
  return {q.x*factor,q.y*factor,q.z*factor};
}
Vector3d right_jacobian_product(const Vector3d & phi,const Vector3d & derivative)
{
  const double angle=vector_norm(phi),square=angle*angle;
  const double a=angle>1e-4?(1.-std::cos(angle))/square:.5-square/24.;
  const double b=angle>1e-4?(angle-std::sin(angle))/(square*angle):1./6.-square/120.;
  const auto cross=cross_vector(phi,derivative);
  return add_vector(add_vector(derivative,scale_vector(cross,-a)),scale_vector(cross_vector(phi,cross),b));
}
std::array<Vector3d,6> hermite_coefficients(const Vector3d & initial,const Vector3d & end,const Vector3d & initial_velocity,double duration)
{
  const auto gap=subtract(end,initial),slope=scale_vector(initial_velocity,duration);
  return {initial,slope,Vector3d{},add_vector(scale_vector(gap,10.),scale_vector(slope,-6.)),
    add_vector(scale_vector(gap,-15.),scale_vector(slope,8.)),add_vector(scale_vector(gap,6.),scale_vector(slope,-3.))};
}
Vector3d polynomial(const std::array<Vector3d,6> & coefficients,double u)
{
  Vector3d value=coefficients.back();
  for(int i=4;i>=0;--i)value=add_vector(scale_vector(value,u),coefficients[static_cast<std::size_t>(i)]);
  return value;
}
Vector3d polynomial_derivative(const std::array<Vector3d,6> & coefficients,double u,double duration)
{
  Vector3d value=scale_vector(coefficients[5],5.);
  for(int i=4;i>=1;--i)value=add_vector(scale_vector(value,u),scale_vector(coefficients[static_cast<std::size_t>(i)],static_cast<double>(i)));
  return scale_vector(value,1./duration);
}
double choose(unsigned n,unsigned k)
{
  double value=1.;for(unsigned i=1;i<=k;++i)value*=static_cast<double>(n+1-i)/i;return value;
}
double velocity_bound(const std::array<Vector3d,6> & coefficients,double duration,const Pose3d & rotation,const Vector3d & feed)
{
  double peak=0.;
  for(unsigned k=0;k<5;++k){
    Vector3d control;
    for(unsigned j=0;j<=k;++j)control=add_vector(control,scale_vector(coefficients[j+1],
      (j+1)*choose(k,j)/choose(4,j)/duration));
    peak=std::max(peak,vector_norm(add_vector(transform_point(rotation,control),feed)));
  }
  return peak;
}
}
std::optional<GraspEntryState> extrapolate_grasp_entry(const GraspEntryState & entry,Nanoseconds source)
{
  if(source<entry.timestamp||!is_valid_pose(entry.tcp_world)||!finite_vector(entry.linear_world)||!finite_vector(entry.angular_world))return std::nullopt;
  const double dt=std::chrono::duration<double>(source-entry.timestamp).count();
  auto result=entry;result.timestamp=source;result.tcp_world.position=add_vector(entry.tcp_world.position,scale_vector(entry.linear_world,dt));
  Pose3d rotation,pose;rotation.orientation=rotation_exp(scale_vector(entry.angular_world,dt));pose.orientation=entry.tcp_world.orientation;
  const auto orientation=compose_pose(rotation,pose);if(!orientation)return std::nullopt;
  result.tcp_world.orientation=orientation->orientation;return result;
}
std::optional<GraspMotionReference> make_relative_grasp_motion_reference(
  const GraspEntryState & entry,const Pose3d & object,const Vector3d & velocity,
  const Pose3d & pregrasp,const Pose3d & contact,double lift,double linear,double angular,
  double minimum,double follow,double descent_linear,double probe_linear,double minimum_approach,double lead,
  std::string * failure_reason)
{
  const auto fail=[&](const std::string & reason)->std::optional<GraspMotionReference>{if(failure_reason)*failure_reason=reason;return std::nullopt;};
  if(!is_valid_pose(entry.tcp_world)||!is_valid_pose(object)||!is_valid_pose(pregrasp)||!is_valid_pose(contact)||
    !finite_vector(velocity)||!finite_vector(entry.linear_world)||!finite_vector(entry.angular_world)||entry.timestamp.count()<0||
    !std::isfinite(linear)||linear<=0||!std::isfinite(angular)||angular<=0||!std::isfinite(minimum)||minimum<=0||
    !std::isfinite(minimum_approach)||minimum_approach<0||!std::isfinite(lead)||lead<0||!std::isfinite(lift)||lift<=0||!std::isfinite(follow)||follow<0)
    return fail("invalid relative approach boundary or speed/time parameter");
  const double available=linear-vector_norm(velocity);
  if(available<=0||vector_norm(entry.linear_world)>linear+1e-8||vector_norm(entry.angular_world)>angular+1e-8)
    return fail("entry/object twist exceeds existing world speed cap");
  auto effective_object=object;effective_object.position=add_vector(object.position,scale_vector(velocity,lead));
  const auto inverse=inverse_pose(effective_object);const auto initial=inverse?compose_pose(*inverse,entry.tcp_world):std::nullopt;
  if(!initial)return fail("invalid effective object-to-TCP transform");
  Pose3d inverse_rotation;inverse_rotation.orientation=inverse->orientation;
  const auto initial_linear=transform_point(inverse_rotation,subtract(entry.linear_world,velocity));
  const auto inverse_tcp=inverse_pose(entry.tcp_world);
  if(!inverse_tcp)return fail("invalid TCP orientation transform");
  Pose3d inverse_tcp_rotation;inverse_tcp_rotation.orientation=inverse_tcp->orientation;
  const auto initial_angular=transform_point(inverse_tcp_rotation,entry.angular_world);
  const auto initial_inverse=inverse_pose(*initial);const auto change=initial_inverse?compose_pose(*initial_inverse,pregrasp):std::nullopt;
  if(!change)return fail("invalid local orientation endpoint");
  const auto angle=rotation_log(change->orientation);
  double duration=std::max({minimum,minimum_approach,1.875*vector_norm(subtract(pregrasp.position,initial->position))/available,1.875*vector_norm(angle)/angular});
  GraspRelativeApproach approach;approach.initial_in_object=*initial;
  Pose3d object_rotation;object_rotation.orientation=object.orientation;Pose3d identity;
  bool bounded=false;
  for(unsigned pass=0;pass<12;++pass){
    approach.position_coefficients=hermite_coefficients(initial->position,pregrasp.position,initial_linear,duration);
    approach.rotation_coefficients=hermite_coefficients({},angle,initial_angular,duration);
    approach.linear_bound_mps=velocity_bound(approach.position_coefficients,duration,object_rotation,velocity);
    approach.angular_bound_radps=velocity_bound(approach.rotation_coefficients,duration,identity,{});
    if(!std::isfinite(duration)||!std::isfinite(approach.linear_bound_mps)||!std::isfinite(approach.angular_bound_radps))return fail("non-finite relative approach coefficients");
    if(approach.linear_bound_mps<=linear+1e-8&&approach.angular_bound_radps<=angular+1e-8){bounded=true;break;}
    duration*=std::max({1.1,approach.linear_bound_mps/linear,approach.angular_bound_radps/angular});
  }
  if(!bounded)return fail("relative Hermite speed bound did not converge in 12 bounded construction passes");
  // Position convex hull is diagnostic geometry, not another admission gate.
  auto low=initial->position,high=initial->position;
  for(unsigned k=0;k<6;++k){
    Vector3d control;
    for(unsigned j=0;j<=k;++j)control=add_vector(control,scale_vector(approach.position_coefficients[j],choose(k,j)/choose(5,j)));
    low={std::min(low.x_m,control.x_m),std::min(low.y_m,control.y_m),std::min(low.z_m,control.z_m)};
    high={std::max(high.x_m,control.x_m),std::max(high.y_m,control.y_m),std::max(high.z_m,control.z_m)};
  }
  approach.position_bounds_object={low,high};
  const auto down=make_grasp_approach_reference(pregrasp,contact,entry.timestamp,descent_linear,angular,velocity,minimum);
  const auto contact_world=compose_pose(object,contact);const auto nominal_inverse=inverse_pose(object);
  if(!contact_world||!nominal_inverse)return fail("invalid probe object transform");
  auto raised=*contact_world;raised.position.z_m+=lift;
  const auto local_lift=compose_pose(*nominal_inverse,raised);
  if(!local_lift)return fail("invalid object-relative probe endpoint");
  const auto up=make_grasp_approach_reference(contact,*local_lift,entry.timestamp,probe_linear,angular,velocity,minimum);
  if(!down||!up)return fail("existing descent/probe speed construction unavailable");
  GraspMotionReference result{entry.tcp_world,object,pregrasp,contact,*local_lift,velocity,entry.timestamp,
    duration,follow,down->duration_sec*1.25,up->duration_sec*1.25,approach};
  if(failure_reason)failure_reason->clear();
  return result;
}

std::optional<GraspMotionSample> sample_grasp_motion_reference(const GraspMotionReference & ref,
  Nanoseconds source,double elapsed,double rate,const Vector3d & offset,const Vector3d & offset_velocity)
{
  if(source<ref.source_start||!std::isfinite(elapsed)||elapsed<0||!std::isfinite(rate)||rate<0||rate>1)return std::nullopt;
  const double age=std::chrono::duration<double>(source-ref.source_start).count();
  auto object=ref.object_world;
  object.position.x_m+=ref.object_velocity.x_m*age+offset.x_m;
  object.position.y_m+=ref.object_velocity.y_m*age+offset.y_m;
  object.position.z_m+=ref.object_velocity.z_m*age+offset.z_m;
  const Vector3d velocity{ref.object_velocity.x_m+offset_velocity.x_m,ref.object_velocity.y_m+offset_velocity.y_m,ref.object_velocity.z_m+offset_velocity.z_m};
  if(ref.relative_approach && elapsed<ref.approach_sec){
    const auto & approach=*ref.relative_approach;
    const double u=std::clamp(elapsed/ref.approach_sec,0.,1.);
    Pose3d local=approach.initial_in_object;
    local.position=polynomial(approach.position_coefficients,u);
    const auto phi=polynomial(approach.rotation_coefficients,u);
    Pose3d turn;turn.orientation=rotation_exp(phi);
    const auto rotated=compose_pose(local,turn);const auto world=rotated?compose_pose(object,*rotated):std::nullopt;
    if(!world)return std::nullopt;
    auto linear=polynomial_derivative(approach.position_coefficients,u,ref.approach_sec);
    linear=scale_vector(linear,rate);
    const auto phi_dot=scale_vector(polynomial_derivative(approach.rotation_coefficients,u,ref.approach_sec),rate);
    const auto body=right_jacobian_product(phi,phi_dot);
    Pose3d world_rotation;world_rotation.orientation=world->orientation;
    Pose3d object_rotation;object_rotation.orientation=object.orientation;
    linear=transform_point(object_rotation,linear);
    return GraspMotionSample{*world,add_vector(linear,velocity),transform_point(world_rotation,body),GraspMotionSegment::kApproach};
  }
  Pose3d begin,end;double u=0,duration=1;GraspMotionSegment segment;
  if(elapsed<ref.approach_sec){begin=ref.initial_tcp_world;end=*compose_pose(object,ref.pregrasp_in_object);
    duration=ref.approach_sec;u=elapsed/duration;segment=GraspMotionSegment::kApproach;
  }else if(elapsed<ref.descent_start()){begin=end=ref.pregrasp_in_object;segment=GraspMotionSegment::kFollow;
  }else if(elapsed<ref.contact_start()){begin=ref.pregrasp_in_object;end=ref.contact_in_object;
    duration=ref.descent_sec;u=(elapsed-ref.descent_start())/duration;segment=GraspMotionSegment::kDescend;
  }else if(elapsed<ref.lift_end()){begin=ref.contact_in_object;end=ref.lift_in_object;
    duration=ref.lift_sec;u=(elapsed-ref.contact_start())/duration;segment=GraspMotionSegment::kLift;
  }else{begin=end=ref.lift_in_object;segment=GraspMotionSegment::kComplete;}
  u=std::clamp(u,0.,1.);
  // Quintic roundoff near an endpoint can produce 1+1e-15, which the
  // pose interpolator correctly rejects as an out-of-range fraction.
  const double blend=std::clamp(u*u*u*(10.+u*(-15.+6.*u)),0.,1.);
  const double derivative=30.*u*u*(1.-u)*(1.-u)/duration*rate;
  const auto local=interpolate_pose(begin,end,blend);if(!local)return std::nullopt;
  const auto pose=segment==GraspMotionSegment::kApproach?local:compose_pose(object,*local);if(!pose)return std::nullopt;
  auto delta=subtract(end.position,begin.position);
  Vector3d linear{delta.x_m*derivative,delta.y_m*derivative,delta.z_m*derivative};
  const auto inverse=inverse_pose(begin);const auto rotation=inverse?compose_pose(end,*inverse):std::nullopt;if(!rotation)return std::nullopt;
  auto q=rotation->orientation;if(q.w<0)q={-q.x,-q.y,-q.z,-q.w};
  const double sine=std::hypot(q.x,q.y,q.z),factor=sine>1e-12?2.*std::atan2(sine,q.w)*derivative/sine:0.;
  Vector3d angular{q.x*factor,q.y*factor,q.z*factor};
  if(segment!=GraspMotionSegment::kApproach){Pose3d turn;turn.orientation=object.orientation;linear=transform_point(turn,linear);angular=transform_point(turn,angular);}
  const double weight=segment==GraspMotionSegment::kApproach?blend:1.;
  linear={linear.x_m+weight*velocity.x_m,linear.y_m+weight*velocity.y_m,linear.z_m+weight*velocity.z_m};
  return GraspMotionSample{*pose,linear,angular,segment};
}

}
