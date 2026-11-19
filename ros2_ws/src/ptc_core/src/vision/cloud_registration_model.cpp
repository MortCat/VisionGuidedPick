#include "ptc_core/vision/cloud_registration_model.hpp"
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <cmath>
namespace ptc_core
{
namespace
{
constexpr double pi=3.14159265358979323846;
Vector3d mean(const std::vector<Vector3d> & points)
{Vector3d p{};for(const auto & v:points){p.x_m+=v.x_m;p.y_m+=v.y_m;p.z_m+=v.z_m;}const double n=static_cast<double>(points.size());return {p.x_m/n,p.y_m/n,p.z_m/n};}
double yaw(const Pose3d & p){return 2*std::atan2(p.orientation.z,p.orientation.w);}
struct Pair {Vector3d source,target;double squared;};
CloudRegistrationResult align(const std::vector<Vector3d> & points,const PointIndex & target,
  const CloudRegistrationConfig & cfg,Pose3d pose)
{
  CloudRegistrationResult out;std::vector<Pair> pairs;
  const auto collect=[&]{
    pairs.clear();const auto inverse=inverse_pose(pose);if(!inverse){return;}
    for(const auto & p:points){const auto local=transform_point(*inverse,p);const auto nearest=target.nearest(local);
      if(nearest.squared_distance<=cfg.max_correspondence_distance_m*cfg.max_correspondence_distance_m){
        pairs.push_back({p,target.points()[nearest.index],nearest.squared_distance});}}
  };
  for(std::size_t iteration=0;iteration<cfg.max_iterations;++iteration){
    collect();if(pairs.size()<3 || static_cast<double>(pairs.size())/points.size()<cfg.min_inlier_fraction){break;}
    std::sort(pairs.begin(),pairs.end(),[](const Pair & a,const Pair & b){return a.squared<b.squared;});
    const auto count=std::max<std::size_t>(3,pairs.size()*4/5);Vector3d a{},b{};
    for(std::size_t i=0;i<count;++i){a.x_m+=pairs[i].source.x_m;a.y_m+=pairs[i].source.y_m;a.z_m+=pairs[i].source.z_m;b.x_m+=pairs[i].target.x_m;b.y_m+=pairs[i].target.y_m;b.z_m+=pairs[i].target.z_m;}
    const double n=static_cast<double>(count);a={a.x_m/n,a.y_m/n,a.z_m/n};b={b.x_m/n,b.y_m/n,b.z_m/n};
    double dot=0,cross=0;for(std::size_t i=0;i<count;++i){const auto & p=pairs[i];const double ax=p.source.x_m-a.x_m,ay=p.source.y_m-a.y_m,bx=p.target.x_m-b.x_m,by=p.target.y_m-b.y_m;dot+=bx*ax+by*ay;cross+=bx*ay-by*ax;}
    const double angle=std::atan2(cross,dot);Pose3d next;next.orientation={0,0,std::sin(angle/2),std::cos(angle/2)};
    const auto center=transform_point(next,b);next.position={a.x_m-center.x_m,a.y_m-center.y_m,a.z_m-center.z_m};
    const bool done=pose_near(next,pose,cfg.convergence_translation_m,cfg.convergence_yaw_rad);pose=next;if(done){break;}
  }
  collect();out.object_world=pose;out.inlier_fraction=points.empty()?0:static_cast<double>(pairs.size())/points.size();
  if(pairs.size()>=3){std::sort(pairs.begin(),pairs.end(),[](const Pair & a,const Pair & b){return a.squared<b.squared;});const auto n=std::max<std::size_t>(3,pairs.size()*4/5);double sum=0;for(std::size_t i=0;i<n;++i){sum+=pairs[i].squared;}out.error_m=std::sqrt(sum/n);}
  out.valid=std::isfinite(out.error_m)&&out.inlier_fraction>=cfg.min_inlier_fraction;return out;
}
}
CloudRegistrationModel::CloudRegistrationModel(const std::vector<Vector3d> & points,const CloudRegistrationConfig & cfg)
 :coarse(voxel_sample(points,cfg.coarse_voxel_size_m,cfg.max_points)),fine(voxel_sample(points,cfg.fine_voxel_size_m,cfg.max_points))
{
  // Envelope geometry must use the full reference bounds, not the density-weighted
  // sample centroid: a low centroid plus symmetric half-height invents a floor
  // below the physical support surface. ICP still computes its own point means.
  Vector3d minimum=points.front(),maximum=points.front();
  for(const auto & p:points){minimum.x_m=std::min(minimum.x_m,p.x_m);maximum.x_m=std::max(maximum.x_m,p.x_m);
    minimum.y_m=std::min(minimum.y_m,p.y_m);maximum.y_m=std::max(maximum.y_m,p.y_m);
    minimum.z_m=std::min(minimum.z_m,p.z_m);maximum.z_m=std::max(maximum.z_m,p.z_m);}
  center_object={(minimum.x_m+maximum.x_m)/2,(minimum.y_m+maximum.y_m)/2,(minimum.z_m+maximum.z_m)/2};
  support_z_m=minimum.z_m;half_height_m=(maximum.z_m-minimum.z_m)/2;
  for(const auto & p:points){radius_m=std::max(radius_m,std::hypot(p.x_m-center_object.x_m,p.y_m-center_object.y_m));}
}
CloudRegistrationResult register_cloud(const std::vector<Vector3d> & points,const CloudRegistrationModel & reference,
  const GeometryConfig & g,const TrackingConfig & tracking,const std::optional<Pose3d> & initial)
{
  CloudRegistrationResult out;const auto & cfg=g.registration;
  const auto coarse=voxel_sample(points,cfg.coarse_voxel_size_m,cfg.max_points),fine=voxel_sample(points,cfg.fine_voxel_size_m,cfg.max_points);
  if(fine.size()<g.min_registration_points || coarse.size()<3){out.reason="insufficient registration samples";return out;}
  const auto fit=[&](Pose3d pose){auto a=align(coarse,reference.coarse,cfg,pose);return a.valid?align(fine,reference.fine,cfg,a.object_world):a;};
  if(initial){
    if(!is_valid_pose(*initial)){out.reason="invalid capture-time prediction";return out;}
    out=fit(*initial);out.yaw_valid=out.valid&&pose_near(out.object_world,*initial,cfg.local_translation_limit_m,cfg.local_yaw_limit_rad);
  }else{
    std::vector<CloudRegistrationResult> fits;const auto a=mean(coarse),b=mean(reference.coarse.points());
    for(double angle=-pi;angle<pi;angle+=g.yaw_search_step_rad){Pose3d pose;pose.orientation={0,0,std::sin(angle/2),std::cos(angle/2)};const auto rotated=transform_point(pose,b);pose.position={a.x_m-rotated.x_m,a.y_m-rotated.y_m,a.z_m-rotated.z_m};auto candidate=fit(pose);if(candidate.valid){fits.push_back(candidate);}}
    if(fits.empty()){out.reason="initial registration failed";return out;}
    std::sort(fits.begin(),fits.end(),[](const auto & a,const auto & b){return a.error_m<b.error_m;});out=fits.front();double alternative=std::numeric_limits<double>::infinity();
    for(const auto & other:fits){if(std::abs(std::remainder(yaw(other.object_world)-yaw(out.object_world),2*pi))>tracking.max_yaw_stddev_rad*2){alternative=other.error_m;break;}}
    out.yaw_valid=alternative-out.error_m>=tracking.yaw_ambiguity_margin_m;
  }
  out.valid=out.valid&&out.yaw_valid&&out.error_m<=g.registration_max_error_m&&out.error_m<=tracking.max_position_stddev_m&&
    reference.radius_m>0&&std::atan2(out.error_m,reference.radius_m)<=tracking.max_yaw_stddev_rad;
  out.reason=out.valid?"rigid upright registration accepted":"registration support/error/yaw correction rejected";return out;
}
}
