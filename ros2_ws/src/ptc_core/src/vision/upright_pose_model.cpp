#include "ptc_core/vision/upright_pose_model.hpp"
#include "ptc_core/vision/cloud_registration_model.hpp"
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <stdexcept>
namespace ptc_core
{
std::vector<Vector3d> voxel_sample(const std::vector<Vector3d> & points,double size,std::size_t limit)
{
  if (!std::isfinite(size) || size<=0 || !limit) {throw std::invalid_argument("invalid voxel sampling config");}
  std::map<std::tuple<std::int64_t,std::int64_t,std::int64_t>,Vector3d> cells;
  for (const auto & p : points) {
    if (!std::isfinite(vector_norm(p)) || std::abs(p.x_m/size)>1e15 || std::abs(p.y_m/size)>1e15 || std::abs(p.z_m/size)>1e15) {continue;}
    const auto key=std::make_tuple(static_cast<std::int64_t>(std::floor(p.x_m/size)),static_cast<std::int64_t>(std::floor(p.y_m/size)),static_cast<std::int64_t>(std::floor(p.z_m/size)));
    cells.emplace(key,p);
  }
  std::vector<Vector3d> out; out.reserve(std::min(limit,cells.size()));
  const auto stride=std::max<std::size_t>(1,(cells.size()+limit-1)/limit); std::size_t i=0;
  for (const auto & cell : cells) {if (i++%stride==0) {out.push_back(cell.second);}} return out;
}
UprightPoseResult fit_upright_object(const std::vector<Vector3d> & points,const PointIndex & reference,const GeometryConfig & g,const TrackingConfig & tracking)
{
  const CloudRegistrationModel model(reference.points(),g.registration);
  const auto fit=register_cloud(points,model,g,tracking);
  UprightPoseResult result;result.pose_world=fit.object_world;result.error_m=fit.error_m;result.position_stddev_m=fit.error_m;
  result.yaw_stddev_rad=model.radius_m>0?std::atan2(fit.error_m,model.radius_m):3.141592653589793;
  result.valid=fit.valid;result.yaw_valid=fit.yaw_valid;return result;
}

std::vector<std::uint64_t> update_visual_tracks(VisualTrackModel & model,const std::vector<ObjectObservation> & observations,
  const std::vector<std::uint64_t> & candidate_track_ids,const SettingsSnapshot & settings,const TimeContext & time)
{
  if(observations.size()!=candidate_track_ids.size()){throw std::invalid_argument("observation/association count mismatch");}
  std::set<std::uint64_t> matched;
  std::vector<std::uint64_t> accepted(observations.size(),0);
  for(std::size_t i=0;i<observations.size();++i){
    const auto & observation=observations[i];const auto candidate_id=candidate_track_ids[i];
    TrackedObjectState * target=nullptr;bool created=false;
    if(candidate_id){
      const auto found=std::find_if(model.tracks.begin(),model.tracks.end(),[&](const auto & track){
        return track.context.key.track_id==candidate_id&&track.context.key.epoch==time.epoch&&
          track.phase!=TrackPhase::kLost&&track.phase!=TrackPhase::kRetired&&!matched.count(candidate_id);});
      if(found==model.tracks.end()){continue;}target=&*found;
      const auto expected=track_pose_at_observation(*target,settings,time,observation.timestamp);
      if(!expected){continue;}
      const auto & p=observation.pose_world.position;const auto & q=expected->position;
      if(vector_norm({p.x_m-q.x_m,p.y_m-q.y_m,p.z_m-q.z_m})>settings.get().tracking.association_distance_m){continue;}
    }else{
      if(model.tracks.size()>=settings.get().tracking.max_tracked_objects){continue;}
      model.tracks.emplace_back();target=&model.tracks.back();created=true;
      target->context.key.track_id=model.next_track_id++;target->context.key.epoch=time.epoch;
      target->context.key.settings_revision=settings.revision();
    }
    EventKey key=target->context.key; key.epoch=time.epoch; key.sequence=model.next_sequence++; key.settings_revision=settings.revision();
    auto decision=advance_track(*target,{TrackEventKind::kObservation,key,observation},settings,time);
    if(decision.status.disposition!=Disposition::kApplied){if(created){model.tracks.pop_back();}continue;}
    *target=std::move(decision.state);matched.insert(target->context.key.track_id);accepted[i]=target->context.key.track_id;
  }
  for (auto & track : model.tracks) {if (!matched.count(track.context.key.track_id)) {
    auto key=track.context.key; key.sequence=model.next_sequence++; const auto decision=advance_track(track,{TrackEventKind::kMissing,key,{}},settings,time); track=decision.state;
  }}
  model.tracks.erase(std::remove_if(model.tracks.begin(),model.tracks.end(),[](const auto & track) {return track.phase==TrackPhase::kLost || track.phase==TrackPhase::kRetired;}),model.tracks.end());
  return accepted;
}
}  // namespace ptc_core
