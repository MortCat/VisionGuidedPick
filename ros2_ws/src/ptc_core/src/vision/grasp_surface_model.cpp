#include "ptc_core/vision/grasp_pose_model.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace ptc_core
{
namespace
{
constexpr double pi=3.14159265358979323846;
constexpr std::size_t max_surface_points=4096,max_surface_seeds=128;
constexpr std::size_t max_pairs_per_closing_bin=8,max_candidates=16;
using Cell=std::array<long long,3>;

double dot(const Vector3d & a,const Vector3d & b)
{return a.x_m*b.x_m+a.y_m*b.y_m+a.z_m*b.z_m;}
Vector3d add(const Vector3d & a,const Vector3d & b)
{return {a.x_m+b.x_m,a.y_m+b.y_m,a.z_m+b.z_m};}
Vector3d subtract(const Vector3d & a,const Vector3d & b)
{return {a.x_m-b.x_m,a.y_m-b.y_m,a.z_m-b.z_m};}
Vector3d scale(const Vector3d & a,double s)
{return {a.x_m*s,a.y_m*s,a.z_m*s};}
Vector3d cross(const Vector3d & a,const Vector3d & b)
{return {a.y_m*b.z_m-a.z_m*b.y_m,a.z_m*b.x_m-a.x_m*b.z_m,a.x_m*b.y_m-a.y_m*b.x_m};}
std::optional<Vector3d> unit(const Vector3d & a)
{
  const double n=vector_norm(a);if(!std::isfinite(n)||n<1e-9)return std::nullopt;
  return scale(a,1./n);
}
bool finite(const Vector3d & p)
{return std::isfinite(p.x_m)&&std::isfinite(p.y_m)&&std::isfinite(p.z_m);}
Cell cell_for(const Vector3d & p,double size)
{return {static_cast<long long>(std::floor(p.x_m/size)),static_cast<long long>(std::floor(p.y_m/size)),static_cast<long long>(std::floor(p.z_m/size))};}
Quaterniond multiply(const Quaterniond & a,const Quaterniond & b)
{
  return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
    a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
    a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
    a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};
}
Quaterniond conjugate(const Quaterniond & q){return {-q.x,-q.y,-q.z,q.w};}
Quaterniond normalized(Quaterniond q)
{
  const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if(!std::isfinite(n)||n<1e-12)return {};
  q.x/=n;q.y/=n;q.z/=n;q.w/=n;return q;
}
Quaterniond quaternion_from_axes(const Vector3d & x,const Vector3d & y,const Vector3d & z)
{
  const double m00=x.x_m,m01=y.x_m,m02=z.x_m;
  const double m10=x.y_m,m11=y.y_m,m12=z.y_m;
  const double m20=x.z_m,m21=y.z_m,m22=z.z_m;
  Quaterniond q;const double trace=m00+m11+m22;
  if(trace>0){const double s=2*std::sqrt(trace+1.);q.w=.25*s;q.x=(m21-m12)/s;q.y=(m02-m20)/s;q.z=(m10-m01)/s;}
  else if(m00>m11&&m00>m22){const double s=2*std::sqrt(1.+m00-m11-m22);q.w=(m21-m12)/s;q.x=.25*s;q.y=(m01+m10)/s;q.z=(m02+m20)/s;}
  else if(m11>m22){const double s=2*std::sqrt(1.+m11-m00-m22);q.w=(m02-m20)/s;q.x=(m01+m10)/s;q.y=.25*s;q.z=(m12+m21)/s;}
  else{const double s=2*std::sqrt(1.+m22-m00-m11);q.w=(m10-m01)/s;q.x=(m02+m20)/s;q.y=(m12+m21)/s;q.z=.25*s;}
  return normalized(q);
}
std::optional<Quaterniond> tcp_orientation(const Vector3d & closing_tcp,const Vector3d & approach_tcp,
  const Vector3d & closing_object,const Vector3d & approach_object)
{
  const auto cx=unit(closing_tcp),cz0=unit(approach_tcp),gx=unit(closing_object),gz0=unit(approach_object);
  if(!cx||!cz0||!gx||!gz0)return std::nullopt;
  const auto cy=unit(cross(*cz0,*cx)),gy=unit(cross(*gz0,*gx));if(!cy||!gy)return std::nullopt;
  const auto cz=unit(cross(*cx,*cy)),gz=unit(cross(*gx,*gy));if(!cz||!gz)return std::nullopt;
  const auto tcp_from_grasp=quaternion_from_axes(*cx,*cy,*cz);
  const auto object_from_grasp=quaternion_from_axes(*gx,*gy,*gz);
  return normalized(multiply(object_from_grasp,conjugate(tcp_from_grasp)));
}

struct Aggregate
{
  Vector3d sum,view_sum;std::size_t count{0},view_count{0};Nanoseconds latest_stamp{0};
};
struct SurfaceSample
{
  GraspSurfacePoint point;Vector3d normal,center;
  std::size_t support{0};double radius{0},quality{0};bool polarity_known{false},valid{false};
};
struct PairCandidate
{
  std::size_t left{0},right{0};Vector3d closing_axis;double span{0},score{0};
  std::size_t closing_bin{0};double priority{0};
};

std::optional<Pose3d> surface_pregrasp(const Pose3d & contact,const Vector3d & approach,
  const GraspSurfaceSnapshot & surface,const GripperGeometryModel & gripper,double clearance)
{
  const auto direction=unit(approach);if(!direction)return std::nullopt;
  const auto center=transform_point(contact,gripper.finger_center_tcp);
  double retreat=clearance;
  // Transit above/outside the measured object envelope before descending.
  // Clearance from a low contact slice alone can put an open finger through
  // a taller part of the same object during the planned entry.
  for(const auto & sample:surface.points)if(finite(sample.position_object_m)){
    retreat=std::max(retreat,dot(subtract(center,sample.position_object_m),*direction)+clearance);
  }
  return make_grasp_preapproach_pose(contact,*direction,retreat);
}

// Jacobi diagonalization of a symmetric 3x3 covariance matrix.
bool local_normal(const std::vector<GraspSurfacePoint> & points,const std::vector<std::size_t> & neighbors,
  double radius,SurfaceSample & out)
{
  if(neighbors.size()<6)return false;
  Vector3d mean{},view{};std::size_t view_count=0;
  for(const auto i:neighbors){mean=add(mean,points[i].position_object_m);
    if(points[i].view_direction_object){view=add(view,*points[i].view_direction_object);++view_count;}}
  mean=scale(mean,1./neighbors.size());
  double a[3][3]{};
  for(const auto i:neighbors){const auto d=subtract(points[i].position_object_m,mean);const double v[3]{d.x_m,d.y_m,d.z_m};
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)a[r][c]+=v[r]*v[c];}
  double eigenvectors[3][3]{{1,0,0},{0,1,0},{0,0,1}};
  for(unsigned iteration=0;iteration<16;++iteration){
    int p=0,q=1;if(std::abs(a[0][2])>std::abs(a[p][q])){p=0;q=2;}
    if(std::abs(a[1][2])>std::abs(a[p][q])){p=1;q=2;}
    if(std::abs(a[p][q])<1e-14)break;
    const double tau=(a[q][q]-a[p][p])/(2*a[p][q]);
    const double t=(tau>=0?1.:-1.)/(std::abs(tau)+std::sqrt(1.+tau*tau));
    const double c=1./std::sqrt(1.+t*t),s=t*c;
    const double app=a[p][p],aqq=a[q][q],apq=a[p][q];
    a[p][p]=app-t*apq;a[q][q]=aqq+t*apq;a[p][q]=a[q][p]=0;
    for(int k=0;k<3;++k){if(k==p||k==q)continue;const double akp=a[k][p],akq=a[k][q];
      a[k][p]=a[p][k]=c*akp-s*akq;a[k][q]=a[q][k]=s*akp+c*akq;}
    for(int k=0;k<3;++k){const double vkp=eigenvectors[k][p],vkq=eigenvectors[k][q];
      eigenvectors[k][p]=c*vkp-s*vkq;eigenvectors[k][q]=s*vkp+c*vkq;}
  }
  std::array<std::pair<double,int>,3> eigen{{{a[0][0],0},{a[1][1],1},{a[2][2],2}}};
  std::sort(eigen.begin(),eigen.end());
  const double smallest=std::max(0.,eigen[0].first),middle=std::max(0.,eigen[1].first),largest=std::max(0.,eigen[2].first);
  if(largest<1e-12||middle/(smallest+middle+largest)<.02||smallest/(smallest+middle)>0.35)return false;
  const int column=eigen[0].second;
  auto normal=unit({eigenvectors[0][column],eigenvectors[1][column],eigenvectors[2][column]});if(!normal)return false;
  bool known=false;
  if(view_count){const auto view_unit=unit(view);if(view_unit&&std::abs(dot(*normal,*view_unit))>.15){
      if(dot(*normal,*view_unit)<0)*normal=scale(*normal,-1);known=true;}}
  else{
    // Stable sign for an unoriented normal. This never claims an outward normal.
    const double components[3]{normal->x_m,normal->y_m,normal->z_m};int dominant=0;
    if(std::abs(components[1])>std::abs(components[dominant]))dominant=1;
    if(std::abs(components[2])>std::abs(components[dominant]))dominant=2;
    if(components[dominant]<0)*normal=scale(*normal,-1);
  }
  out.normal=*normal;out.center=mean;out.support=neighbors.size();out.radius=radius;
  out.quality=std::clamp(1.-smallest/std::max(1e-15,smallest+middle),0.,1.);
  out.polarity_known=known;out.valid=true;return true;
}

// Estimate observed contact over each pad at the candidate's predicted aperture.
void measure_pad_coverage(GraspCandidate & candidate,const GraspSurfaceSnapshot & snapshot,
  const GripperGeometryModel & gripper,const SettingsSnapshot & settings)
{
  const auto & size=settings.get().grasp.pad_contact_size_m;
  if(!size)return;
  auto rotation=candidate.tcp_in_object;rotation.position={};
  const auto closing=transform_point(rotation,gripper.closing_axis_tcp);
  const auto height=transform_point(rotation,gripper.approach_axis_tcp);
  const auto width=cross(closing,height);
  const auto center=transform_point(candidate.tcp_in_object,gripper.finger_center_tcp);
  const double half_span=candidate.opening_width_m*.5;
  const double band=std::min(half_span*.45,std::max(.002,settings.get().scene.voxel_size_m));
  constexpr int cells=4;
  struct Tile {std::vector<std::size_t> indices;std::array<bool,4> close_cells{};};
  std::array<std::array<Tile,4>,2> pad_tiles;
  for(std::size_t i=0;i<snapshot.points.size();++i){
    const auto & sample=snapshot.points[i];
    if(!sample.view_direction_object)continue;
    const double view=dot(*sample.view_direction_object,closing);
    if(std::abs(view)<.2)continue;
    const auto side=view<0.?0:1;
    const auto delta=subtract(sample.position_object_m,center);
    const double lateral=dot(delta,closing);
    const double gap=std::abs((side==0?-lateral:lateral)-half_span);
    if(gap>half_span*.9)continue;
    const double x=dot(delta,width)/(*size)[0]+.5,y=dot(delta,height)/(*size)[1]+.5;
    if(!(x>=0&&x<1&&y>=0&&y<1))continue;
    const int col=static_cast<int>(x*cells),row=static_cast<int>(y*cells);
    auto & tile=pad_tiles[side][(row/2)*2+col/2];
    tile.indices.push_back(i);
    if(gap<=band)tile.close_cells[(row%2)*2+col%2]=true;
  }
  for(std::size_t side=0;side<pad_tiles.size();++side){
    const auto outward=scale(closing,side==0?-1.:1.);
    const auto & tiles=pad_tiles[side];
    bool observed=false;
    double support=0.;
    for(std::size_t index=0;index<tiles.size();++index){
      const auto & tile=tiles[index];
      SurfaceSample surface;
      if(!local_normal(snapshot.points,tile.indices,(*size)[0],surface) || !surface.polarity_known)continue;
      observed=true;
      const double alignment=dot(surface.normal,outward);
      if(alignment<.90)continue;
      for(std::size_t cell=0;cell<tile.close_cells.size();++cell){
        if(!tile.close_cells[cell])continue;
        const auto row=(index/2)*2+cell/2, col=(index%2)*2+cell%2;
        const bool interior=row>0 && row<cells-1 && col>0 && col<cells-1;
        support+=(interior?2.:1.)*alignment;
      }
    }
    // Four interior cells get double weight, making a centered patch
    // preferable to an equally large patch at the edge of the pad.
    if(observed)candidate.pad_contact_coverage[side]=support/(cells*cells+4);
  }
}

std::size_t direction_bin(const Vector3d & direction)
{
  const auto n=unit(direction);if(!n)return 0;
  double azimuth=std::atan2(n->y_m,n->x_m);if(azimuth<0)azimuth+=2*pi;
  const double elevation=std::asin(std::clamp(n->z_m,-1.,1.));
  const auto az=std::min<std::size_t>(7,static_cast<std::size_t>(azimuth/(2*pi)*8));
  const auto el=std::min<std::size_t>(3,static_cast<std::size_t>((elevation+pi/2)/pi*4));
  return el*8+az;
}
Vector3d canonical_axis(Vector3d axis)
{
  if(axis.z_m<0 || (std::abs(axis.z_m)<1e-9 &&
      (axis.y_m<0 || (std::abs(axis.y_m)<1e-9&&axis.x_m<0))))axis=scale(axis,-1);
  return axis;
}
std::size_t closing_bin(const Vector3d & axis)
{
  const auto c=canonical_axis(axis);double az=std::atan2(c.y_m,c.x_m);if(az<0)az+=2*pi;
  const double polar=std::acos(std::clamp(c.z_m,0.,1.));
  const auto az_bin=std::min<std::size_t>(7,static_cast<std::size_t>(az/(2*pi)*8));
  const auto polar_bin=std::min<std::size_t>(3,static_cast<std::size_t>(polar/(pi/2)*4));
  return polar_bin*8+az_bin;
}
bool insert_pair(std::vector<PairCandidate> & bin,const PairCandidate & pair)
{
  if(bin.size()<max_pairs_per_closing_bin){bin.push_back(pair);return true;}
  const auto worst=std::min_element(bin.begin(),bin.end(),[](const auto & a,const auto & b){return a.priority<b.priority;});
  if(worst!=bin.end()&&worst->priority<pair.priority){*worst=pair;return true;}
  return false;
}
}

std::optional<Pose3d> make_grasp_preapproach_pose(const Pose3d & contact,const Vector3d & direction,double clearance)
{
  const auto approach=unit(direction);
  if(!is_valid_pose(contact)||!approach||!std::isfinite(clearance)||clearance<=0)return std::nullopt;
  auto pose=contact;pose.position=subtract(pose.position,scale(*approach,clearance));
  return is_valid_pose(pose)?std::optional<Pose3d>(pose):std::nullopt;
}

std::optional<double> fingertip_support_clearance(const Pose3d & tcp,const GripperGeometryModel & model,double plane)
{
  if(!model.fingertip_box_tcp || !is_valid_pose(tcp) || !std::isfinite(plane))return std::nullopt;
  const auto & lo=(*model.fingertip_box_tcp)[0];const auto & hi=(*model.fingertip_box_tcp)[1];
  if(!finite(lo)||!finite(hi)||lo.x_m>=hi.x_m||lo.y_m>=hi.y_m||lo.z_m>=hi.z_m)return std::nullopt;
  double low=std::numeric_limits<double>::infinity();
  for(unsigned i=0;i<8;++i){const auto p=transform_point(tcp,{i&1?hi.x_m:lo.x_m,i&2?hi.y_m:lo.y_m,i&4?hi.z_m:lo.z_m});low=std::min(low,p.z_m);}
  return low-plane;
}
std::optional<Pose3d> grasp_recovery_retreat(const Pose3d & tcp,const GripperGeometryModel & model,double plane,double margin,double distance)
{
  const auto gap=fingertip_support_clearance(tcp,model,plane);
  if(!gap||!std::isfinite(margin)||margin<0||!std::isfinite(distance)||distance<=0)return std::nullopt;
  auto target=tcp;target.position.z_m+=std::max(0.,margin+distance-*gap);return target;
}
bool grasp_support_clear(const Pose3d & tcp,const GripperGeometryModel & model,const SettingsSnapshot & settings)
{
  if(!model.fingertip_box_tcp)return true; // Legacy profiles have no calibrated envelope.
  const auto gap=fingertip_support_clearance(tcp,model,settings.get().vision.depth_workspace.support_plane_z_m);
  return gap && *gap+1e-9>=settings.get().grasp.support_plane_clearance_m;
}
GripperGeometryModel configured_gripper_geometry(const SettingsSnapshot & settings,const std::string & collision_digest)
{
  const auto & value=settings.get();const auto & grasp=value.grasp;
  GripperGeometryModel model;model.model_id=value.motion.gripper_joint;
  model.description_id=value.calibration.expected_calibration_id;
  model.collision_geometry_digest=collision_digest;model.calibration_id=value.calibration.expected_calibration_id;model.tcp_frame=value.frames.tcp;
  model.closing_axis_tcp=grasp.closing_axis_tcp;model.approach_axis_tcp={0,0,1};
  model.finger_center_tcp=grasp.finger_center_tcp;model.fingertip_box_tcp=grasp.fingertip_box_tcp;
  model.minimum_opening_m=grasp.close_width_m;model.maximum_opening_m=grasp.open_width_m;
  return model;
}

GraspProposalResult propose_grasp_candidates(const GraspSurfaceSnapshot & snapshot,const TrackedObjectState & track,
  const GripperGeometryModel & gripper,const SettingsSnapshot & settings,const TimeContext & time,double clearance)
{
  const auto started=std::chrono::steady_clock::now();
  GraspProposalResult result;result.key=snapshot.key;result.surface_revision=snapshot.revision;
  result.object_frame=snapshot.object_frame;result.gripper_model_id=gripper.model_id;result.collision_geometry_digest=gripper.collision_geometry_digest;
  result.input_point_count=snapshot.points.size();result.approach_direction_coverage.assign(32,0);
  std::size_t plane_rejected=0;double rejected_gap=0.;
  const auto finish=[&](){result.elapsed_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
    if(plane_rejected)result.reason+="; support_plane_rejected="+std::to_string(plane_rejected)+" last_gap_m="+std::to_string(rejected_gap)+" required_m="+std::to_string(settings.get().grasp.support_plane_clearance_m);
    return result;};
  const auto fail=[&](const std::string & reason){result.reason=reason;result.status=GraspSearchStatus::kInvalidInput;return finish();};
  // Candidate geometry uses an already measured object-local surface. A short
  // image gap may use the same bounded motion prediction as the intercept;
  // this is neither a new captured frame nor proof of contact/possession.
  if(!track_is_usable(track,settings,time,true)){return fail("visual track is outside the usable prediction horizon");}
  if(!same_track_identity(snapshot.key,track.context.key)){return fail("surface snapshot identity does not match the tracked object");}
  if(!snapshot.settings_revision.empty()&&snapshot.settings_revision!=settings.revision()){return fail("surface settings revision is stale");}
  if(snapshot.points.empty()||snapshot.object_frame.empty()||gripper.model_id.empty()||
    !std::isfinite(clearance)||clearance<=0)return fail("surface, object frame, gripper identity or clearance is missing");
  if(gripper.collision_geometry_digest.rfind("sha256:",0)!=0||gripper.collision_geometry_digest.size()!=71)
    return fail("verified collision geometry digest is missing or malformed");
  if(!std::isfinite(gripper.minimum_opening_m)||!std::isfinite(gripper.maximum_opening_m)||
    gripper.minimum_opening_m<0||gripper.maximum_opening_m<=gripper.minimum_opening_m||
    settings.get().grasp.min_contact_points<3)return fail("gripper opening or contact support is invalid");
  const auto closing=unit(gripper.closing_axis_tcp),approach=unit(gripper.approach_axis_tcp);
  if(!closing||!approach||std::abs(dot(*closing,*approach))>.05)return fail("gripper closing and approach axes are invalid");
  const auto predicted=predict_object_pose(track,settings,time,settings.get().grasp.intercept_lead_sec);
  if(!predicted)return fail("visual intercept prediction is unavailable");

  std::vector<GraspSurfacePoint> finite_points;finite_points.reserve(snapshot.points.size());
  Vector3d minimum{std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max()};
  Vector3d maximum{-minimum.x_m,-minimum.y_m,-minimum.z_m};
  for(const auto & point:snapshot.points){if(!finite(point.position_object_m))continue;finite_points.push_back(point);
    minimum.x_m=std::min(minimum.x_m,point.position_object_m.x_m);minimum.y_m=std::min(minimum.y_m,point.position_object_m.y_m);minimum.z_m=std::min(minimum.z_m,point.position_object_m.z_m);
    maximum.x_m=std::max(maximum.x_m,point.position_object_m.x_m);maximum.y_m=std::max(maximum.y_m,point.position_object_m.y_m);maximum.z_m=std::max(maximum.z_m,point.position_object_m.z_m);}
  result.finite_point_count=finite_points.size();
  if(finite_points.size()<settings.get().grasp.min_contact_points*2)return fail("too few finite measured surface points");
  const double extent=std::max({maximum.x_m-minimum.x_m,maximum.y_m-minimum.y_m,maximum.z_m-minimum.z_m});
  if(!std::isfinite(extent)||extent<=1e-6)return fail("measured surface has no spatial extent");
  Pose3d object_rotation;object_rotation.orientation=predicted->orientation;
  const auto contact_priority=[&](double score,const Vector3d & center,double span){
    const double height=transform_point(object_rotation,subtract(center,minimum)).z_m/extent;
    return score+.2*height+.25*span/gripper.maximum_opening_m;
  };

  const double configured_voxel=settings.get().inspection.geometry.fusion.evidence_voxel_size_m;
  double voxel=std::isfinite(configured_voxel)&&configured_voxel>1e-5?configured_voxel:extent/256.;
  std::vector<GraspSurfacePoint> points;
  bool truncated=finite_points.size()>max_surface_points;
  for(unsigned pass=0;pass<12;++pass){
    std::map<Cell,Aggregate> cells;
    for(const auto & point:finite_points){auto & aggregate=cells[cell_for(point.position_object_m,voxel)];
      aggregate.sum=add(aggregate.sum,point.position_object_m);++aggregate.count;
      if(point.view_direction_object){const auto view=unit(*point.view_direction_object);if(view){aggregate.view_sum=add(aggregate.view_sum,*view);++aggregate.view_count;}}
      aggregate.latest_stamp=std::max(aggregate.latest_stamp,point.capture_timestamp);}
    if(cells.size()<=max_surface_points){points.clear();points.reserve(cells.size());
      for(const auto & item:cells){const auto & a=item.second;GraspSurfacePoint point;
        point.position_object_m=scale(a.sum,1./a.count);point.capture_timestamp=a.latest_stamp;
        if(a.view_count){const auto direction=unit(a.view_sum);if(direction)point.view_direction_object=*direction;}
        points.push_back(point);}
      break;
    }
    voxel*=1.5;truncated=true;
  }
  result.sampled_point_count=points.size();
  if(points.empty())return fail("bounded surface sampling produced no points");
  const double radius=std::min(std::max(voxel*4.,extent*.05),extent*.15);
  std::map<Cell,std::vector<std::size_t>> spatial;
  for(std::size_t i=0;i<points.size();++i)spatial[cell_for(points[i].position_object_m,radius)].push_back(i);
  std::vector<SurfaceSample> surfaces(points.size());
  for(std::size_t i=0;i<points.size();++i){const auto base=cell_for(points[i].position_object_m,radius);std::vector<std::pair<double,std::size_t>> nearest;
    for(long long x=-1;x<=1;++x)for(long long y=-1;y<=1;++y)for(long long z=-1;z<=1;++z){
      const Cell key{base[0]+x,base[1]+y,base[2]+z};const auto cell=spatial.find(key);if(cell==spatial.end())continue;
      for(const auto j:cell->second){const auto d=subtract(points[j].position_object_m,points[i].position_object_m);const double d2=dot(d,d);if(d2<=radius*radius)nearest.emplace_back(d2,j);}}
    if(nearest.size()>64){std::partial_sort(nearest.begin(),nearest.begin()+64,nearest.end(),[](const auto & a,const auto & b){return a.first<b.first;});nearest.resize(64);}
    std::vector<std::size_t> indexes;indexes.reserve(nearest.size());for(const auto & item:nearest)indexes.push_back(item.second);
    surfaces[i].point=points[i];local_normal(points,indexes,radius,surfaces[i]);}

  std::array<std::vector<PairCandidate>,32> pair_bins;std::set<std::pair<std::size_t,std::size_t>> seen_pairs;
  std::size_t seed_count=std::min(max_surface_seeds,points.size());result.seed_count=seed_count;
  std::size_t normal_rejected=0,aperture_rejected=0,support_rejected=0,closing_pair_count=0;
  const auto required_support=settings.get().grasp.min_contact_points;
  for(std::size_t seed=0;seed<seed_count;++seed){
    const std::size_t i=seed_count==points.size()?seed:(seed*(points.size()-1))/(seed_count-1);
    if(!surfaces[i].valid)continue;
    for(std::size_t j=0;j<points.size();++j){if(i==j||!surfaces[j].valid)continue;
      const std::pair<std::size_t,std::size_t> key{std::min(i,j),std::max(i,j)};
      if(seen_pairs.find(key)!=seen_pairs.end())continue;
      const auto delta=subtract(points[j].position_object_m,points[i].position_object_m);const double span=vector_norm(delta);
      const double margin=std::max(voxel*2.,extent*.005);
      if(!std::isfinite(span)||span<=gripper.minimum_opening_m+margin||span>=gripper.maximum_opening_m-margin){++aperture_rejected;continue;}
      auto direction=unit(delta);if(!direction)continue;
      const double signed_i=dot(surfaces[i].normal,*direction),signed_j=dot(surfaces[j].normal,*direction);
      const double align_i=std::abs(signed_i),align_j=std::abs(signed_j);
      if(std::min(align_i,align_j)<.72){++normal_rejected;continue;}
      if(surfaces[i].polarity_known&&surfaces[j].polarity_known&&(signed_i>-.55||signed_j<.55)){
        ++normal_rejected;continue;}
      if(surfaces[i].support<required_support||surfaces[j].support<required_support){++support_rejected;continue;}
      seen_pairs.insert(key);++closing_pair_count;
      Vector3d canonical=canonical_axis(*direction);std::size_t left=i,right=j;
      if(dot(subtract(points[right].position_object_m,points[left].position_object_m),canonical)<0)std::swap(left,right);
      const auto bin=closing_bin(canonical);
      const double support=std::min(1.,static_cast<double>(std::min(surfaces[i].support,surfaces[j].support))/required_support);
      const double quality=std::min(surfaces[i].quality,surfaces[j].quality);
      const double score=.58*std::min(align_i,align_j)+.27*quality+.15*support-.12*span/gripper.maximum_opening_m;
      PairCandidate pair{left,right,canonical,span,score,bin,
        contact_priority(score,scale(add(surfaces[i].center,surfaces[j].center),.5),span)};
      if(!insert_pair(pair_bins[bin],pair))truncated=true;
    }
  }
  if(seed_count<points.size())truncated=true;
  std::vector<PairCandidate> pairs;for(const auto & bin:pair_bins)pairs.insert(pairs.end(),bin.begin(),bin.end());
  std::stable_sort(pairs.begin(),pairs.end(),[](const auto & a,const auto & b){return a.priority>b.priority;});
  std::vector<GraspCandidate> all;all.reserve(pairs.size()*8);std::uint64_t next_id=1;
  for(const auto & pair:pairs){
    const auto & left=surfaces[pair.left];const auto & right=surfaces[pair.right];
    const auto closing_axis=unit(subtract(right.center,left.center));if(!closing_axis)continue;
    Vector3d reference=std::abs(closing_axis->z_m)<.9?Vector3d{0,0,1}:Vector3d{0,1,0};
    const auto u=unit(cross(*closing_axis,reference));if(!u)continue;const auto v=unit(cross(*closing_axis,*u));if(!v)continue;
    const auto midpoint=scale(add(left.center,right.center),.5);
    const double span=vector_norm(subtract(right.center,left.center));
    if(span<=gripper.minimum_opening_m||span>=gripper.maximum_opening_m)continue;
    for(unsigned turn=0;turn<8;++turn){const double angle=turn*pi/4.;
      const auto approach_axis=unit(add(scale(*u,std::cos(angle)),scale(*v,std::sin(angle))));if(!approach_axis)continue;
      const auto orientation=tcp_orientation(gripper.closing_axis_tcp,gripper.approach_axis_tcp,*closing_axis,*approach_axis);if(!orientation)continue;
      Pose3d tcp;tcp.orientation=*orientation;
      const auto finger_center=transform_point(tcp,gripper.finger_center_tcp);tcp.position=subtract(midpoint,finger_center);
      const auto pregrasp=surface_pregrasp(tcp,*approach_axis,snapshot,gripper,clearance);if(!pregrasp)continue;
      const auto world=compose_pose(*predicted,tcp);const auto pregrasp_world=compose_pose(*predicted,*pregrasp);
      if(!world||!pregrasp_world||!inside_workspace(*world,settings.get().motion)||!inside_workspace(*pregrasp_world,settings.get().motion))continue;
      if(!grasp_support_clear(*world,gripper,settings)||!grasp_support_clear(*pregrasp_world,gripper,settings)){
        ++plane_rejected;rejected_gap=fingertip_support_clearance(*world,gripper,settings.get().vision.depth_workspace.support_plane_z_m).value_or(0.);continue;
      }
      GraspCandidate candidate;candidate.candidate_id=next_id++;candidate.surface_revision=snapshot.revision;
      candidate.tcp_in_object=tcp;candidate.pregrasp_in_object=*pregrasp;candidate.tcp_world=*world;
      candidate.approach_direction_object=*approach_axis;
      candidate.left_contact={left.center,left.normal,left.support,left.radius,left.quality,left.polarity_known};
      candidate.right_contact={right.center,right.normal,right.support,right.radius,right.quality,right.polarity_known};
      candidate.opening_width_m=span;candidate.geometry_score=pair.score;
      candidate.source="measured_surface_geometry";candidate.gripper_model_id=gripper.model_id;
      candidate.collision_geometry_digest=gripper.collision_geometry_digest;
      candidate.reason="opposed measured surface support; force closure and physical collision unverified";candidate.valid=true;
      ++result.approach_direction_coverage[direction_bin(*approach_axis)];all.push_back(std::move(candidate));
    }
  }
  // Prefer approaches from above and higher supported contacts before using
  // low/side/underside alternatives. This is a soft ordering, not a rejection:
  // all candidates still require the same measured support and collision check.
  const auto pickup_priority=[&](const GraspCandidate & candidate){
    const auto axis=transform_point(object_rotation,candidate.approach_direction_object);
    const auto center=scale(add(candidate.left_contact.center_object_m,candidate.right_contact.center_object_m),.5);
    return contact_priority(candidate.geometry_score,center,candidate.opening_width_m)-.4*axis.z_m;
  };
  // Apply pickup priority before both bounded reductions. Ranking only the
  // final 16 cannot recover upper contacts discarded by the pair/direction caps.
  std::stable_sort(all.begin(),all.end(),[&](const auto & a,const auto & b){return pickup_priority(a)>pickup_priority(b);});
  std::array<bool,32> available_directions{};std::vector<const GraspCandidate *> directions;
  for(const auto & candidate:all){const auto approach=direction_bin(candidate.approach_direction_object);
    if(available_directions[approach])continue;
    available_directions[approach]=true;directions.push_back(&candidate);
  }
  // Keep the best supported contact in each direction, then spread the bounded
  // set by angular distance. Taking only the first 16 ranked bins can discard
  // every horizontal alternative when downward approaches occupy those bins.
  while(!directions.empty()&&result.candidates.size()<max_candidates){
    std::size_t best=0;double widest=-1.;
    for(std::size_t i=0;i<directions.size();++i){double separation=2.;
      for(const auto & selected:result.candidates){separation=std::min(separation,
        1.-dot(directions[i]->approach_direction_object,selected.approach_direction_object));}
      if(separation>widest+1e-12){widest=separation;best=i;}
    }
    result.candidates.push_back(*directions[best]);directions.erase(directions.begin()+best);
  }
  if(result.candidates.size()<max_candidates){
    for(const auto & candidate:all){if(result.candidates.size()>=max_candidates)break;
      if(std::any_of(result.candidates.begin(),result.candidates.end(),[&](const auto & selected){return selected.candidate_id==candidate.candidate_id;}))continue;
      const auto duplicate=std::any_of(result.candidates.begin(),result.candidates.end(),[&](const auto & selected){
        const auto position=subtract(candidate.tcp_in_object.position,selected.tcp_in_object.position);
        const auto & a=candidate.tcp_in_object.orientation;const auto & b=selected.tcp_in_object.orientation;
        const double angle=2*std::acos(std::clamp(std::abs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w),0.,1.));
        return vector_norm(position)<.005&&angle<pi/12;});
      if(!duplicate)result.candidates.push_back(candidate);
    }}
  std::stable_sort(result.candidates.begin(),result.candidates.end(),[&](const auto & a,const auto & b){return pickup_priority(a)>pickup_priority(b);});
  if(result.candidates.empty()){
    result.status=GraspSearchStatus::kNoCandidates;result.reason="no opposed measured surface patches fit the gripper opening and workspace";
  }else{
    result.status=truncated?GraspSearchStatus::kBudgetLimited:GraspSearchStatus::kComplete;
    result.reason="finite="+std::to_string(result.finite_point_count)+" sampled="+std::to_string(result.sampled_point_count)+
      " seeds="+std::to_string(result.seed_count)+" supported_pairs="+std::to_string(closing_pair_count)+
      " candidates="+std::to_string(result.candidates.size())+" normal_rejected="+std::to_string(normal_rejected)+
      " aperture_rejected="+std::to_string(aperture_rejected)+" support_rejected="+std::to_string(support_rejected)+
      " direction_bins="+std::to_string(std::count_if(result.approach_direction_coverage.begin(),
        result.approach_direction_coverage.end(),[](std::uint8_t count){return count>0;}))+
      " truncated="+(truncated?"true":"false")+"; geometry support does not confirm grasp force or collision-free motion";
  }
  for(auto & candidate:result.candidates)measure_pad_coverage(candidate,snapshot,gripper,settings);
  return finish();
}
void order_grasp_candidates_for_wrist(std::vector<GraspCandidate> & candidates,const Quaterniond & measured)
{
  const auto close_to=[](const Vector3d & a,const Vector3d & b){return vector_norm(subtract(a,b))<1e-6;};
  const auto same_grasp=[&](const GraspCandidate & a,const GraspCandidate & b){
    const auto & al=a.left_contact.center_object_m;const auto & ar=a.right_contact.center_object_m;
    const auto & bl=b.left_contact.center_object_m;const auto & br=b.right_contact.center_object_m;
    return dot(a.approach_direction_object,b.approach_direction_object)>.999 &&
      ((close_to(al,bl)&&close_to(ar,br))||(close_to(al,br)&&close_to(ar,bl)));
  };
  struct Entry {std::size_t group{0};double alignment{0},proximity{0};GraspCandidate candidate;};
  std::vector<Entry> entries;entries.reserve(candidates.size());
  for(auto & candidate:candidates){
    Entry entry;entry.group=entries.size();entry.alignment=candidate.slice_normal_alignment;
    for(const auto & earlier:entries)if(same_grasp(earlier.candidate,candidate)){
      entry.group=earlier.group;entry.alignment=earlier.alignment;break;}
    const auto & q=candidate.tcp_world.orientation;
    entry.proximity=std::abs(q.x*measured.x+q.y*measured.y+q.z*measured.z+q.w*measured.w);
    entry.candidate=std::move(candidate);entries.push_back(std::move(entry));
  }
  std::stable_sort(entries.begin(),entries.end(),[](const Entry & a,const Entry & b){
    if(a.group==b.group)return a.proximity>b.proximity;
    if(a.alignment!=b.alignment)return a.alignment>b.alignment;
    return a.group<b.group;
  });
  for(std::size_t i=0;i<entries.size();++i)candidates[i]=std::move(entries[i].candidate);
}
GraspProposalResult propose_top_view_grasp_candidates(const GraspSurfaceSnapshot & snapshot,const TrackedObjectState & track,
  const GripperGeometryModel & gripper,const SettingsSnapshot & settings,const TimeContext & time,double clearance)
{
  auto result=propose_grasp_candidates(snapshot,track,gripper,settings,time,clearance);
  if(result.status!=GraspSearchStatus::kNoCandidates)return result;
  const auto began=std::chrono::steady_clock::now();
  const auto object=predict_object_pose(track,settings,time,settings.get().grasp.intercept_lead_sec);
  if(!object)return result;
  std::vector<Vector3d> points;std::vector<double> heights;
  for(const auto & sample:snapshot.points)if(finite(sample.position_object_m)){
    points.push_back(sample.position_object_m);heights.push_back(sample.position_object_m.z_m);}
  if(points.size()<settings.get().grasp.min_contact_points*2)return result;
  const auto quantile=[](std::vector<double> v,double f){std::sort(v.begin(),v.end());return v[static_cast<std::size_t>(f*(v.size()-1))];};
  const double body_top=quantile(heights,.65);Vector3d center{};std::size_t count=0;
  for(const auto & p:points)if(p.z_m<=body_top){center=add(center,p);++count;}
  if(!count)return result;center=scale(center,1./count);
  double xx=0,xy=0,yy=0;
  for(const auto & p:points)if(p.z_m<=body_top){const auto d=subtract(p,center);xx+=d.x_m*d.x_m;xy+=d.x_m*d.y_m;yy+=d.y_m*d.y_m;}
  const double major=.5*std::atan2(2*xy,xx-yy),radius=gripper.maximum_opening_m*.15;
  Pose3d rotation;rotation.orientation=object->orientation;const auto inverse_rotation=inverse_pose(rotation);
  if(!inverse_rotation)return result;const auto approach=transform_point(*inverse_rotation,{0,0,-1});
  std::size_t plane_rejected=0,scored_pairs=0;double rejected_gap=0.,normal_ms=0.;
  for(const double height_fraction:{.75,.60,.40})for(const double angle:{major+pi/2,major}){
    const double z=quantile(heights,height_fraction);const Vector3d closing{std::cos(angle),std::sin(angle),0};
    const auto lateral=cross(approach,closing);std::vector<Vector3d> band;
    for(const auto & p:points)if(std::abs(p.z_m-z)<=radius && std::abs(dot(subtract(p,center),lateral))<=radius)band.push_back(p);
    if(band.size()<settings.get().grasp.min_contact_points*2)continue;
    std::sort(band.begin(),band.end(),[&](const auto & a,const auto & b){return dot(a,closing)<dot(b,closing);});
    const auto left=band[static_cast<std::size_t>(.03*(band.size()-1))],right=band[static_cast<std::size_t>(.97*(band.size()-1))];
    const double span=vector_norm(subtract(right,left));
    if(span<=gripper.minimum_opening_m || span>=gripper.maximum_opening_m)continue;
    const auto normal_started=std::chrono::steady_clock::now();
    const auto patch=[&](const Vector3d & center_point){
      constexpr std::size_t fit_limit=32;
      GraspContactPatch out;
      out.center_object_m=center_point;
      out.support_radius_m=radius;
      std::array<std::pair<double,std::size_t>,fit_limit> nearest;
      std::size_t size=0;
      const double radius_squared=radius*radius;
      for(std::size_t i=0;i<snapshot.points.size();++i){
        const auto delta=subtract(snapshot.points[i].position_object_m,center_point);
        const double distance_squared=dot(delta,delta);
        if(!(distance_squared<=radius_squared))continue;
        ++out.support_point_count;
        if(size<fit_limit){
          nearest[size++]={distance_squared,i};
          std::push_heap(nearest.begin(),nearest.begin()+size);
        }else if(distance_squared<nearest.front().first){
          std::pop_heap(nearest.begin(),nearest.end());
          nearest.back()={distance_squared,i};
          std::push_heap(nearest.begin(),nearest.end());
        }
      }
      SurfaceSample fit;
      std::vector<std::size_t> indices;
      indices.reserve(size);
      for(std::size_t i=0;i<size;++i)indices.push_back(nearest[i].second);
      if(local_normal(snapshot.points,indices,radius,fit) && fit.polarity_known){
        out.normal_object=fit.normal;
        out.normal_quality=fit.quality;
        out.normal_polarity_known=true;
      }
      return out;
    };
    auto lp=patch(left),rp=patch(right);
    normal_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-normal_started).count();
    if(lp.support_point_count<settings.get().grasp.min_contact_points || rp.support_point_count<settings.get().grasp.min_contact_points)continue;
    // Top-only views can resolve a plane without observing either contact side.
    // Give a bonus only when both oriented normals support the closing axis.
    const double alignment=std::min(-dot(lp.normal_object,closing),dot(rp.normal_object,closing));
    const double normal_score=lp.normal_polarity_known && rp.normal_polarity_known && alignment>=.55
      ? std::clamp(alignment,0.,1.)*std::min(lp.normal_quality,rp.normal_quality) : 0.;
    if(normal_score>0.)++scored_pairs;
    for(const double sign:{1.,-1.}){
      const auto orientation=tcp_orientation(gripper.closing_axis_tcp,gripper.approach_axis_tcp,scale(closing,sign),approach);
      if(!orientation)continue;Pose3d tcp;tcp.orientation=*orientation;
      tcp.position=subtract(scale(add(left,right),.5),transform_point(tcp,gripper.finger_center_tcp));
      const auto pregrasp=surface_pregrasp(tcp,approach,snapshot,gripper,clearance);
      const auto world=compose_pose(*object,tcp);const auto pre_world=pregrasp?compose_pose(*object,*pregrasp):std::nullopt;
      if(!world || !pre_world || !inside_workspace(*world,settings.get().motion) || !inside_workspace(*pre_world,settings.get().motion))continue;
      if(!grasp_support_clear(*world,gripper,settings)||!grasp_support_clear(*pre_world,gripper,settings)){
        ++plane_rejected;rejected_gap=fingertip_support_clearance(*world,gripper,settings.get().vision.depth_workspace.support_plane_z_m).value_or(0.);continue;
      }
      GraspCandidate candidate;candidate.candidate_id=10000+result.candidates.size();candidate.surface_revision=snapshot.revision;
      candidate.tcp_in_object=tcp;candidate.pregrasp_in_object=*pregrasp;candidate.tcp_world=*world;candidate.approach_direction_object=approach;
      candidate.left_contact=sign>0?lp:rp;candidate.right_contact=sign>0?rp:lp;candidate.opening_width_m=span;
      candidate.geometry_score=0.;candidate.source="measured_top_view_slice_commissioning";
      candidate.gripper_model_id=gripper.model_id;candidate.collision_geometry_digest=gripper.collision_geometry_digest;
      candidate.slice_normal_alignment=normal_score;
      candidate.reason=normal_score>0.?"visible slice with aligned measured normals; force closure unverified"
        :"visible slice without aligned bilateral normal evidence";
      candidate.valid=true;result.candidates.push_back(candidate);
    }
  }
  for(auto & candidate:result.candidates)measure_pad_coverage(candidate,snapshot,gripper,settings);
  result.elapsed_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  if(!result.candidates.empty()){result.status=GraspSearchStatus::kComplete;result.reason="top-view PCA/slice candidates";}
  result.reason+="; slice_normal_scored_pairs="+std::to_string(scored_pairs)+
    " slice_normal_patch_ms="+std::to_string(normal_ms);
  if(plane_rejected)result.reason+="; top_support_plane_rejected="+std::to_string(plane_rejected)+" last_gap_m="+std::to_string(rejected_gap)+" required_m="+std::to_string(settings.get().grasp.support_plane_clearance_m);
  return result;
}

}
