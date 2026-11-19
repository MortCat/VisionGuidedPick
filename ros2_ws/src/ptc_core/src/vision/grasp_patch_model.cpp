#include "ptc_core/vision/grasp_patch_model.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
namespace ptc_core {
namespace {
bool inside(const Vector3d & p,const Vector3d & lo,const Vector3d & hi){return
 p.x_m>=lo.x_m&&p.x_m<=hi.x_m&&p.y_m>=lo.y_m&&p.y_m<=hi.y_m&&p.z_m>=lo.z_m&&p.z_m<=hi.z_m;}
std::vector<Pose3d> inverse_boxes(const std::vector<GraspMaskBox> & boxes){std::vector<Pose3d> inv;for(const auto & b:boxes){
  const auto p=inverse_pose(b.box_in_tcp);if(!p||!std::isfinite(vector_norm(b.half_extent))||
   b.half_extent.x_m<=0||b.half_extent.y_m<=0||b.half_extent.z_m<=0){throw std::invalid_argument("invalid self mask box");}
  inv.push_back(*p);}return inv;}
bool self_masked(const Vector3d & p,const std::vector<GraspMaskBox> & boxes,const std::vector<Pose3d> & inv){
 for(std::size_t k=0;k<boxes.size();++k){const auto q=transform_point(inv[k],p),h=boxes[k].half_extent;if(inside(q,{-h.x_m,-h.y_m,-h.z_m},h))return true;}return false;}
bool rectified(const CameraModel & c){for(double d:c.distortion)if(d!=0.)return false;return true;}
double median(std::vector<double> & v){const auto mid=v.begin()+v.size()/2;std::nth_element(v.begin(),mid,v.end());return *mid;}
std::string metres(double v){char b[32];std::snprintf(b,sizeof b,"%.4f m",v);return b;}
}
GraspPatchObservation extract_grasp_patch_tcp(const RgbdImageFrame & f,
 const std::array<Vector3d,2> & region,const std::vector<GraspMaskBox> & boxes,std::size_t limit,std::optional<double> support_z){
 GraspPatchObservation out;const auto & c=f.camera;
 if(!rectified(c)){out.reason="diagnostic requires rectified camera intrinsics";return out;}
 if(!limit||!c.width||!c.height||!std::isfinite(c.fx)||!std::isfinite(c.fy)||c.fx<=0||c.fy<=0||
  f.depth_m.size()!=static_cast<std::size_t>(c.width)*c.height||!inside(region[0],region[0],region[1])||
  !is_valid_pose(f.metadata.tcp_in_world)||!is_valid_pose(f.metadata.camera_in_world))throw std::invalid_argument("invalid grasp patch frame/region");
 const auto tcp_world=inverse_pose(f.metadata.tcp_in_world),camera_world=inverse_pose(f.metadata.camera_in_world);
 const auto camera_tcp=compose_pose(*camera_world,f.metadata.tcp_in_world);
 const auto tcp_camera=compose_pose(*tcp_world,f.metadata.camera_in_world);
 if(!tcp_camera||!camera_tcp){out.reason="invalid camera/TCP composition";return out;}
 const auto inv=inverse_boxes(boxes);
 double minx=std::numeric_limits<double>::infinity(),miny=minx,maxx=-minx,maxy=-minx;
 for(unsigned i=0;i<8;++i){Vector3d p{(i&1)?region[1].x_m:region[0].x_m,(i&2)?region[1].y_m:region[0].y_m,(i&4)?region[1].z_m:region[0].z_m};
  p=transform_point(*camera_tcp,p);if(p.z_m<=0){out.reason="observation region crosses camera plane";return out;}
  const double u=c.fx*p.x_m/p.z_m+c.cx,v=c.fy*p.y_m/p.z_m+c.cy;
  if(!std::isfinite(u)||!std::isfinite(v)){out.reason="observation region cannot be projected";return out;}
  minx=std::min(minx,u);maxx=std::max(maxx,u);miny=std::min(miny,v);maxy=std::max(maxy,v);}
 out.roi_clipped=minx<0||miny<0||maxx>=c.width||maxy>=c.height;
 if(maxx<0||maxy<0||minx>=c.width||miny>=c.height){out.reason="observation region outside image";return out;}
 const auto x0=static_cast<unsigned>(std::max(0.,std::floor(minx))),y0=static_cast<unsigned>(std::max(0.,std::floor(miny)));
 const auto x1=static_cast<unsigned>(std::min(double(c.width),std::ceil(maxx)+1)),y1=static_cast<unsigned>(std::min(double(c.height),std::ceil(maxy)+1));
 out.roi={x0,y0,x1-x0,y1-y0};out.pixel_labels.resize(f.depth_m.size());std::vector<Vector3d> points;
 for(unsigned y=y0;y<y1;++y)for(unsigned x=x0;x<x1;++x){const auto i=static_cast<std::size_t>(y)*c.width+x;const auto z=f.depth_m[i];
  out.pixel_labels[i]=1;if(!std::isfinite(z)||z<=0)continue;++out.valid_depth_pixels;
  const auto p=transform_point(*tcp_camera,{(x-c.cx)*z/c.fx,(y-c.cy)*z/c.fy,z});
  out.pixel_labels[i]=2;if(!inside(p,region[0],region[1]))continue;
  if(self_masked(p,boxes,inv)){out.pixel_labels[i]=3;++out.self_pixels;continue;}
  if(support_z&&transform_point(f.metadata.tcp_in_world,p).z_m<=*support_z){out.pixel_labels[i]=5;++out.support_pixels;continue;}
  out.pixel_labels[i]=4;++out.candidate_pixels;points.push_back(p);}
 const auto count=std::min(limit,points.size());out.points_tcp.reserve(count);
 for(std::size_t n=0;n<count;++n)out.points_tcp.push_back(points[n*points.size()/count]);
 out.reason=points.empty()?"no unmasked local depth; possession unknown":"unclassified local surface; possession not evaluated";return out;
}
std::optional<GraspSupportPlaneEstimate> measure_support_plane(const RgbdImageFrame & f,const DepthWorkspaceConfig & cfg){
 const auto & c=f.camera;
 if(!rectified(c)||!c.width||!c.height||c.fx<=0||c.fy<=0||f.depth_m.size()!=static_cast<std::size_t>(c.width)*c.height||
  !is_valid_pose(f.metadata.camera_in_world)||!(cfg.min_height_m>0)||!cfg.min_points)throw std::invalid_argument("invalid support plane frame/workspace");
 // Same band Vision uses to separate foreground from support; heights inside it are support.
 std::vector<double> heights;
 for(std::uint32_t y=0;y<c.height;++y)for(std::uint32_t x=0;x<c.width;++x){const double z=f.depth_m[static_cast<std::size_t>(y)*c.width+x];
  if(!std::isfinite(z)||z<=0)continue;const auto w=transform_point(f.metadata.camera_in_world,{(x-c.cx)*z/c.fx,(y-c.cy)*z/c.fy,z});
  if(w.x_m<cfg.bounds_min_m.x_m||w.x_m>cfg.bounds_max_m.x_m||w.y_m<cfg.bounds_min_m.y_m||w.y_m>cfg.bounds_max_m.y_m||
   !(std::abs(w.z_m-cfg.support_plane_z_m)<cfg.min_height_m))continue;heights.push_back(w.z_m);}
 if(heights.size()<cfg.min_points)return std::nullopt;
 return GraspSupportPlaneEstimate{median(heights),heights.size()};
}
std::optional<GraspTargetSeed> grasp_target_seed_tcp(const RgbdImageFrame & f,const std::vector<std::uint8_t> & mask,
 const std::vector<GraspMaskBox> & boxes,std::size_t min_points,std::string & reason){
 const auto & c=f.camera;
 if(!rectified(c)||!min_points||mask.size()!=f.depth_m.size()||f.depth_m.size()!=static_cast<std::size_t>(c.width)*c.height||c.fx<=0||c.fy<=0||
  !is_valid_pose(f.metadata.tcp_in_world)||!is_valid_pose(f.metadata.camera_in_world))throw std::invalid_argument("invalid grasp seed frame/mask");
 const auto inv=inverse_boxes(boxes);const auto tcp_world=inverse_pose(f.metadata.tcp_in_world);
 std::optional<Pose3d> tcp_camera;if(tcp_world)tcp_camera=compose_pose(*tcp_world,f.metadata.camera_in_world);
 if(!tcp_camera){reason="invalid camera/TCP composition";return std::nullopt;}
 const auto inf=std::numeric_limits<double>::infinity();GraspTargetSeed seed{{Vector3d{inf,inf,inf},Vector3d{-inf,-inf,-inf}},0,f.metadata.frame_id,f.metadata.color_timestamp,f.metadata.tcp_in_world,0};
 std::vector<double> heights;
 for(std::size_t i=0;i<mask.size();++i){const double z=f.depth_m[i];if(!mask[i]||!std::isfinite(z)||z<=0)continue;
  const auto x=static_cast<double>(i%c.width),y=static_cast<double>(i/c.width);
  const auto p=transform_point(*tcp_camera,{(x-c.cx)*z/c.fx,(y-c.cy)*z/c.fy,z});
  // Foreground segmentation can merge closing fingers into the target component.
  if(self_masked(p,boxes,inv))continue;auto & lo=seed.bounds_tcp[0];auto & hi=seed.bounds_tcp[1];
  lo={std::min(lo.x_m,p.x_m),std::min(lo.y_m,p.y_m),std::min(lo.z_m,p.z_m)};hi={std::max(hi.x_m,p.x_m),std::max(hi.y_m,p.y_m),std::max(hi.z_m,p.z_m)};++seed.points;
  heights.push_back(transform_point(f.metadata.tcp_in_world,p).z_m);}
 if(seed.points<min_points){reason="selected target surface has "+std::to_string(seed.points)+" unmasked depth points; seed unavailable";return std::nullopt;}
 seed.median_world_z_m=median(heights);return seed;
}
std::array<Vector3d,2> grasp_observation_region(const std::array<Vector3d,2> & pad,const GraspTargetSeed & seed,double margin){
 if(!std::isfinite(margin)||margin<0)throw std::invalid_argument("invalid grasp seed margin");
 const auto & lo=seed.bounds_tcp[0];const auto & hi=seed.bounds_tcp[1];
 return {Vector3d{std::min(pad[0].x_m,lo.x_m-margin),std::min(pad[0].y_m,lo.y_m-margin),std::min(pad[0].z_m,lo.z_m-margin)},
  Vector3d{std::max(pad[1].x_m,hi.x_m+margin),std::max(pad[1].y_m,hi.y_m+margin),std::max(pad[1].z_m,hi.z_m+margin)}};
}
GraspLiftPossession evaluate_lift_possession(const RgbdImageFrame & f,const GraspTargetSeed & seed,const std::vector<GraspMaskBox> & boxes,
 std::optional<double> support_z,const std::optional<GripperState> & gripper,double resolution,std::size_t min_points){
 if(!std::isfinite(resolution)||resolution<=0||!min_points||!is_valid_pose(seed.tcp_in_world)||!std::isfinite(seed.median_world_z_m))
  throw std::invalid_argument("invalid lift possession seed/resolution");
 GraspLiftPossession out;const auto & now=f.metadata.tcp_in_world;out.tcp_rise_m=now.position.z_m-seed.tcp_in_world.position.z_m;
 // The two hypotheses differ by the TCP rise; it must exceed the seed resolution on both sides.
 if(!(out.tcp_rise_m>2*resolution)){out.reason="TCP rose "+metres(out.tcp_rise_m)+" since Closing seed; follow/stay not separable";return out;}
 const auto now_inv=inverse_pose(now);std::optional<Pose3d> seed_to_now;if(now_inv)seed_to_now=compose_pose(*now_inv,seed.tcp_in_world);
 if(!seed_to_now){out.reason="invalid TCP composition";return out;}
 const auto & lo=seed.bounds_tcp[0];const auto & hi=seed.bounds_tcp[1];
 const std::array<Vector3d,2> moved{{{lo.x_m-resolution,lo.y_m-resolution,lo.z_m-resolution},{hi.x_m+resolution,hi.y_m+resolution,hi.z_m+resolution}}};
 const auto inf=std::numeric_limits<double>::infinity();std::array<Vector3d,2> stayed{{{inf,inf,inf},{-inf,-inf,-inf}}};
 for(unsigned i=0;i<8;++i){const auto p=transform_point(*seed_to_now,{(i&1)?moved[1].x_m:moved[0].x_m,(i&2)?moved[1].y_m:moved[0].y_m,(i&4)?moved[1].z_m:moved[0].z_m});
  stayed={Vector3d{std::min(stayed[0].x_m,p.x_m),std::min(stayed[0].y_m,p.y_m),std::min(stayed[0].z_m,p.z_m)},Vector3d{std::max(stayed[1].x_m,p.x_m),std::max(stayed[1].y_m,p.y_m),std::max(stayed[1].z_m,p.z_m)}};}
 const std::array<Vector3d,2> region{{{std::min(moved[0].x_m,stayed[0].x_m),std::min(moved[0].y_m,stayed[0].y_m),std::min(moved[0].z_m,stayed[0].z_m)},
  {std::max(moved[1].x_m,stayed[1].x_m),std::max(moved[1].y_m,stayed[1].y_m),std::max(moved[1].z_m,stayed[1].z_m)}}};
 const auto patch=extract_grasp_patch_tcp(f,region,boxes,4096,support_z);
 std::vector<double> heights;for(const auto & p:patch.points_tcp)if(inside(p,moved[0],moved[1])||inside(p,stayed[0],stayed[1]))heights.push_back(transform_point(now,p).z_m);
 out.surface_points=heights.size();
 // Hidden or out-of-view surface is not Empty.
 if(heights.size()<min_points){out.reason="target surface not observed after lift ("+std::to_string(heights.size())+" points; "+
  std::to_string(patch.self_pixels)+" self, "+std::to_string(patch.support_pixels)+" support); possession unknown";return out;}
 out.surface_rise_m=median(heights)-seed.median_world_z_m;const double follow=out.surface_rise_m/out.tcp_rise_m;
 const std::string motion="surface rose "+metres(out.surface_rise_m)+" while TCP rose "+metres(out.tcp_rise_m);
 if(follow>1./3&&follow<2./3){out.reason=motion+"; between follow and stay";return out;}
 if(!gripper||gripper->timestamp<=seed.stamp){out.reason=motion+"; no post-close gripper feedback";return out;}
 const bool contact=gripper->contact_confirmed&&!gripper->empty_confirmed;
 if(follow>=2./3&&contact){out.state=Possession::kHolding;out.reason=motion+"; gripper reports contact";}
 else if(follow<=1./3&&!contact){out.state=Possession::kEmpty;out.reason=motion+"; gripper reports no contact";}
 else out.reason=motion+(contact?"; gripper reports contact; conflict":"; gripper reports no contact; conflict");
 return out;
}
}
