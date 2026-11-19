#include "ptc_core/state/admission_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "ptc_core/pose_math.hpp"
#include "ptc_core/vision/rgbd_image_frame.hpp"

namespace ptc_core
{
InspectionWindowEstimate estimate_inspection_window(const TrackedObjectState & track,
  const Pose3d & planned_object_pose,const std::vector<Pose3d> & tcp_view_poses,
  const SettingsSnapshot & snapshot,const TimeContext & time,double required_sec)
{
  InspectionWindowEstimate result;result.required_sec=required_sec;
  const auto & config=snapshot.get();const auto & region=config.vision.depth_workspace;
  if(!track_is_usable(track,snapshot,time,true)||!track.observation.envelope_valid||
    !is_valid_pose(planned_object_pose)||tcp_view_poses.empty()||!std::isfinite(required_sec)||required_sec<=0){
    result.reason="track, healthy-model envelope, view path or route budget is invalid";return result;
  }
  const auto current_pose=predict_object_pose(track,snapshot,time,0);
  const auto position_uncertainty=predicted_position_uncertainty_m(track,snapshot,time,0);
  if(!current_pose||!position_uncertainty){result.reason="current track prediction or uncertainty is unavailable";return result;}
  const auto & envelope=track.observation;
  if(!std::isfinite(envelope.envelope_radius_xy_m)||envelope.envelope_radius_xy_m<=0||
    !std::isfinite(envelope.envelope_half_height_m)||envelope.envelope_half_height_m<=0||
    !std::isfinite(envelope.envelope_center_object_m.x_m)||!std::isfinite(envelope.envelope_center_object_m.y_m)||
    !std::isfinite(envelope.envelope_center_object_m.z_m)){
    result.reason="healthy-model envelope is incomplete";return result;
  }
  Vector3d minimum{region.bounds_min_m.x_m,region.bounds_min_m.y_m,region.bounds_min_m.z_m};
  Vector3d maximum{region.bounds_max_m.x_m,region.bounds_max_m.y_m,region.bounds_max_m.z_m};
  const auto envelope_center_world=transform_point(current_pose.value(),envelope.envelope_center_object_m);
  const Vector3d center_offset{envelope_center_world.x_m-current_pose->position.x_m,
    envelope_center_world.y_m-current_pose->position.y_m,envelope_center_world.z_m-current_pose->position.z_m};
  minimum.x_m+=envelope.envelope_radius_xy_m+*position_uncertainty-center_offset.x_m;
  maximum.x_m-=envelope.envelope_radius_xy_m+*position_uncertainty+center_offset.x_m;
  minimum.y_m+=envelope.envelope_radius_xy_m+*position_uncertainty-center_offset.y_m;
  maximum.y_m-=envelope.envelope_radius_xy_m+*position_uncertainty+center_offset.y_m;
  // A visually supported object rests on the configured support plane. The
  // foreground crop is not a demand that its feet float above that plane.
  const double lower_surface=envelope_center_world.z_m-envelope.envelope_half_height_m;
  const bool supported=config.vision.foreground_source=="depth_workspace" &&
    std::abs(lower_surface-region.support_plane_z_m)<=region.min_height_m+*position_uncertainty &&
    std::abs(track.velocity_mps.z_m)<=track.velocity_uncertainty_mps;
  minimum.z_m+=envelope.envelope_half_height_m+(supported?-*position_uncertainty:*position_uncertainty)-center_offset.z_m;
  maximum.z_m-=envelope.envelope_half_height_m+*position_uncertainty+center_offset.z_m;

  const auto & workspace_min=config.motion.bounds_min_m;const auto & workspace_max=config.motion.bounds_max_m;
  for(const auto & tcp:tcp_view_poses){
    if(!is_valid_pose(tcp)){result.reason="view path contains an invalid TCP pose";return result;}
    const Vector3d offset{tcp.position.x_m-planned_object_pose.position.x_m,
      tcp.position.y_m-planned_object_pose.position.y_m,tcp.position.z_m-planned_object_pose.position.z_m};
    minimum.x_m=std::max(minimum.x_m,workspace_min.x_m-offset.x_m);
    minimum.y_m=std::max(minimum.y_m,workspace_min.y_m-offset.y_m);
    minimum.z_m=std::max(minimum.z_m,workspace_min.z_m-offset.z_m);
    maximum.x_m=std::min(maximum.x_m,workspace_max.x_m-offset.x_m);
    maximum.y_m=std::min(maximum.y_m,workspace_max.y_m-offset.y_m);
    maximum.z_m=std::min(maximum.z_m,workspace_max.z_m-offset.z_m);
  }
  if(!std::isfinite(minimum.x_m)||!std::isfinite(minimum.y_m)||!std::isfinite(minimum.z_m)||
    !std::isfinite(maximum.x_m)||!std::isfinite(maximum.y_m)||!std::isfinite(maximum.z_m)||
    minimum.x_m>=maximum.x_m||minimum.y_m>=maximum.y_m||minimum.z_m>=maximum.z_m){
    result.reason="observer zone and five-view TCP reach have no common object-center region";return result;
  }
  const auto & position=current_pose->position;
  const std::array<double,3> coordinate{{position.x_m,position.y_m,position.z_m}};
  const std::array<double,3> lower{{minimum.x_m,minimum.y_m,minimum.z_m}};
  const std::array<double,3> upper{{maximum.x_m,maximum.y_m,maximum.z_m}};
  const std::array<const char *,3> axis_name{{"x","y","z"}};
  for(std::size_t axis=0;axis<coordinate.size();++axis){
    if(coordinate[axis]<lower[axis]||coordinate[axis]>upper[axis]){
      result.reason="object envelope outside region; axis="+std::string(axis_name[axis])+" position="+
        std::to_string(coordinate[axis])+" bounds=["+std::to_string(lower[axis])+","+std::to_string(upper[axis])+"]";
      return result;
    }
  }

  const auto & velocity=track.velocity_mps;const double velocity_error=track.velocity_uncertainty_mps;
  const std::array<double,3> p{{position.x_m,position.y_m,position.z_m}};
  const std::array<double,3> lo{{minimum.x_m,minimum.y_m,minimum.z_m}};
  const std::array<double,3> hi{{maximum.x_m,maximum.y_m,maximum.z_m}};
  const std::array<double,3> v{{velocity.x_m,velocity.y_m,velocity.z_m}};
  double available=config.task.cycle_timeout_sec;
  std::string limiting_axis="cycle";double limiting_margin=0,limiting_rate=0;
  for(std::size_t axis=0;axis<p.size();++axis){
    if(axis==2&&supported){continue;} // Support constrains vertical drift; TCP bounds still apply.
    const double positive_rate=std::max(0.0,v[axis])+velocity_error;
    const double negative_rate=std::max(0.0,-v[axis])+velocity_error;
    const auto consider=[&](double margin,double rate,const char * direction){
      if(rate<=1e-6){return;}const double exit_time=margin/rate;
      if(exit_time<available){available=exit_time;limiting_axis=std::string(axis_name[axis])+direction;
        limiting_margin=margin;limiting_rate=rate;}
    };
    consider(hi[axis]-p[axis],positive_rate,"+");
    consider(p[axis]-lo[axis],negative_rate,"-");
  }
  if(!std::isfinite(available)||available<0){result.reason="bounded time-to-exit estimate is invalid";return result;}
  result.feasible=true;result.available_sec=available;result.enough_time=available>=required_sec;
  result.reason=result.enough_time?"inspection route fits the conservative work-zone window":
    "inspection route exceeds the conservative work-zone window";
  result.reason+="; limiting="+limiting_axis+" margin_m="+std::to_string(limiting_margin)+
    " rate_mps="+std::to_string(limiting_rate)+" velocity_uncertainty_mps="+std::to_string(velocity_error);
  return result;
}
}  // namespace ptc_core
