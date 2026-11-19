#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace ptc_core
{
namespace
{
void valid_camera(const CameraModel & c)
{
  if (!c.width || !c.height || !std::isfinite(c.fx) || !std::isfinite(c.fy) || c.fx<=0 || c.fy<=0 ||
    !std::isfinite(c.cx) || !std::isfinite(c.cy) ||
    !std::all_of(c.distortion.begin(),c.distortion.end(),[](double value) {return std::isfinite(value);})) {throw std::invalid_argument("invalid camera model");}
}
std::array<double,2> distort(double x,double y,const CameraModel & c)
{
  const auto & k=c.distortion; const double r2=x*x+y*y,r4=r2*r2,r6=r4*r2;
  const double radial=(1+k[0]*r2+k[1]*r4+k[4]*r6)/(1+k[5]*r2+k[6]*r4+k[7]*r6);
  return {x*radial+2*k[2]*x*y+k[3]*(r2+2*x*x),y*radial+k[2]*(r2+2*y*y)+2*k[3]*x*y};
}
Vector3d unproject(double u,double v,double z,const CameraModel & c)
{
  const double target_x=(u-c.cx)/c.fx,target_y=(v-c.cy)/c.fy; double x=target_x,y=target_y;
  for (unsigned i=0;i<8;++i) {const auto distorted=distort(x,y,c); x+=target_x-distorted[0]; y+=target_y-distorted[1];}
  return {x*z,y*z,z};
}
double gray(const RgbdImageFrame & frame,std::uint32_t x,std::uint32_t y)
{
  const auto i=(static_cast<std::size_t>(y)*frame.camera.width+x)*3;
  return 0.299*frame.rgb[i]+0.587*frame.rgb[i+1]+0.114*frame.rgb[i+2];
}
void valid_region(const RgbdImageFrame & frame,const ImageRegion & roi)
{
  valid_camera(frame.camera); const auto pixels=static_cast<std::size_t>(frame.camera.width)*frame.camera.height;
  if (frame.rgb.size()!=pixels*3 || frame.depth_m.size()!=pixels || roi.x>frame.camera.width || roi.y>frame.camera.height ||
    roi.width>frame.camera.width-roi.x || roi.height>frame.camera.height-roi.y) {throw std::invalid_argument("invalid RGB-D buffer/ROI");}
}
}
Vector3d transform_point(const Pose3d & pose,const Vector3d & v)
{
  const auto & q=pose.orientation;
  const Vector3d cross{q.y*v.z_m-q.z*v.y_m,q.z*v.x_m-q.x*v.z_m,q.x*v.y_m-q.y*v.x_m};
  const Vector3d cross2{q.y*cross.z_m-q.z*cross.y_m,q.z*cross.x_m-q.x*cross.z_m,q.x*cross.y_m-q.y*cross.x_m};
  return {pose.position.x_m+v.x_m+2*(q.w*cross.x_m+cross2.x_m),
    pose.position.y_m+v.y_m+2*(q.w*cross.y_m+cross2.y_m),pose.position.z_m+v.z_m+2*(q.w*cross.z_m+cross2.z_m)};
}
std::optional<std::array<double,2>> project_point(const Vector3d & point,const CameraModel & camera)
{
  valid_camera(camera); if (!std::isfinite(vector_norm(point)) || point.z_m<=0) {return std::nullopt;}
  const auto uv=distort(point.x_m/point.z_m,point.y_m/point.z_m,camera);
  const std::array<double,2> out{camera.fx*uv[0]+camera.cx,camera.fy*uv[1]+camera.cy};
  if (!std::isfinite(out[0]) || !std::isfinite(out[1]) || out[0]<0 || out[1]<0 || out[0]>=camera.width || out[1]>=camera.height) {return std::nullopt;}
  return out;
}
ImageRegion image_region(const std::array<double,4> & b,const CameraModel & camera,double margin)
{
  valid_camera(camera);
  if (!std::all_of(b.begin(),b.end(),[](double value) {return std::isfinite(value);}) || b[0]>=b[2] || b[1]>=b[3] ||
    !std::isfinite(margin) || margin<0 || margin>1) {throw std::invalid_argument("invalid bbox/margin");}
  const double dx=(b[2]-b[0])*margin,dy=(b[3]-b[1])*margin;
  const auto x0=static_cast<std::uint32_t>(std::floor(std::clamp(b[0]-dx,0.0,static_cast<double>(camera.width))));
  const auto y0=static_cast<std::uint32_t>(std::floor(std::clamp(b[1]-dy,0.0,static_cast<double>(camera.height))));
  const auto x1=static_cast<std::uint32_t>(std::ceil(std::clamp(b[2]+dx,0.0,static_cast<double>(camera.width))));
  const auto y1=static_cast<std::uint32_t>(std::ceil(std::clamp(b[3]+dy,0.0,static_cast<double>(camera.height))));
  return {x0,y0,x1-x0,y1-y0};
}
std::vector<float> decode_depth(std::uint32_t width,std::uint32_t height,std::uint32_t step,bool big,bool mm,
  const std::vector<std::uint8_t> & bytes,const VisionConfig & config)
{
  const std::size_t pixel_bytes=mm ? 2 : 4;
  if (!width || !height || step<static_cast<std::uint64_t>(width)*pixel_bytes || bytes.size()!=static_cast<std::uint64_t>(step)*height ||
    static_cast<double>(width)*height*7>config.max_frame_megabytes*1024*1024) {throw std::invalid_argument("depth dimensions/stride/bytes capacity invalid");}
  std::vector<float> out(static_cast<std::size_t>(width)*height,std::numeric_limits<float>::quiet_NaN());
  for (std::uint32_t y=0;y<height;++y) {for (std::uint32_t x=0;x<width;++x) {
    const auto i=static_cast<std::size_t>(y)*step+x*pixel_bytes; std::uint32_t raw=0;
    for (std::size_t k=0;k<pixel_bytes;++k) {raw|=static_cast<std::uint32_t>(bytes[i+k])<<(8*(big ? pixel_bytes-1-k : k));}
    float value=0;
    if (mm) {value=static_cast<float>(raw)*0.001F;} else {std::memcpy(&value,&raw,4);}
    if (std::isfinite(value) && value>=config.depth_min_m && value<=config.depth_max_m) {out[static_cast<std::size_t>(y)*width+x]=value;}
  }}
  return out;
}
std::vector<float> register_depth(const std::vector<float> & input,const CameraModel & depth,const CameraModel & color,const Pose3d & d_in_c)
{
  valid_camera(depth); valid_camera(color);
  if (input.size()!=static_cast<std::size_t>(depth.width)*depth.height || !is_valid_pose(d_in_c)) {throw std::invalid_argument("invalid registration payload/transform");}
  std::vector<float> out(static_cast<std::size_t>(color.width)*color.height,std::numeric_limits<float>::quiet_NaN());
  for (std::uint32_t y=0;y<depth.height;++y) {for (std::uint32_t x=0;x<depth.width;++x) {
    const float z=input[static_cast<std::size_t>(y)*depth.width+x]; if (!std::isfinite(z) || z<=0) {continue;}
    const auto point=transform_point(d_in_c,unproject(x,y,z,depth)); if (!std::isfinite(vector_norm(point)) || point.z_m<=0) {continue;}
    const auto uv=distort(point.x_m/point.z_m,point.y_m/point.z_m,color);
    const double u=std::round(color.fx*uv[0]+color.cx),v=std::round(color.fy*uv[1]+color.cy);
    if (!std::isfinite(u) || !std::isfinite(v) || u<0 || v<0 || u>=color.width || v>=color.height) {continue;}
    auto & destination=out[static_cast<std::size_t>(v)*color.width+static_cast<std::size_t>(u)];
    if (!std::isfinite(destination) || point.z_m<destination) {destination=static_cast<float>(point.z_m);}
  }}
  return out;
}
FrameQuality frame_quality(const RgbdImageFrame & frame,const ImageRegion & roi,const VisionConfig & config)
{
  valid_region(frame,roi); FrameQuality quality; if (roi.width<3 || roi.height<3) {return quality;}
  double sum=0,squares=0; std::size_t samples=0;
  for (std::uint32_t y=roi.y;y<roi.y+roi.height;++y) {for (std::uint32_t x=roi.x;x<roi.x+roi.width;++x) {
    const float depth=frame.depth_m[static_cast<std::size_t>(y)*frame.camera.width+x];
    if (std::isfinite(depth) && depth>=config.depth_min_m && depth<=config.depth_max_m) {++quality.valid_depth_points;}
    quality.brightness+=gray(frame,x,y);
    if (x>roi.x && x+1<roi.x+roi.width && y>roi.y && y+1<roi.y+roi.height) {
      const double lap=gray(frame,x-1,y)+gray(frame,x+1,y)+gray(frame,x,y-1)+gray(frame,x,y+1)-4*gray(frame,x,y);
      sum+=lap; squares+=lap*lap; ++samples;
    }
  }}
  quality.brightness/=static_cast<double>(roi.width)*roi.height;
  quality.laplacian_variance=samples ? std::max(0.0,squares/samples-(sum/samples)*(sum/samples)) : 0;
  quality.valid=quality.valid_depth_points>=config.min_valid_depth_points && quality.laplacian_variance>=config.min_laplacian_variance &&
    quality.brightness>=config.min_brightness && quality.brightness<=config.max_brightness; return quality;
}
std::vector<Vector3d> region_points_world(const RgbdImageFrame & frame,const ImageRegion & roi,std::size_t limit,const std::vector<std::uint8_t> & mask)
{
  valid_region(frame,roi);
  if (!limit || (!mask.empty() && mask.size()!=frame.depth_m.size()) || !is_valid_pose(frame.metadata.camera_in_world)) {
    throw std::invalid_argument("invalid cloud limit/mask/capture transform");
  }
  // Registered depth can be sparse on the RGB raster. Choose from valid
  // masked pixels, not from a rectangle-based stride that can alias them away.
  std::vector<std::size_t> pixels;pixels.reserve(std::min(limit,static_cast<std::size_t>(roi.width)*roi.height));
  for(std::uint32_t y=roi.y;y<roi.y+roi.height;++y){for(std::uint32_t x=roi.x;x<roi.x+roi.width;++x){
    const auto i=static_cast<std::size_t>(y)*frame.camera.width+x;const double z=frame.depth_m[i];
    if(std::isfinite(z)&&z>0&&(mask.empty()||mask[i]))pixels.push_back(i);
  }}
  const auto count=std::min(limit,pixels.size());std::vector<Vector3d> out;out.reserve(count);
  for(std::size_t n=0;n<count;++n){
    const auto i=pixels[n*pixels.size()/count];const double z=frame.depth_m[i];
    const auto point=transform_point(frame.metadata.camera_in_world,unproject(i%frame.camera.width,i/frame.camera.width,z,frame.camera));
    if(std::isfinite(vector_norm(point)))out.push_back(point);
  }
  return out;
}
std::vector<Vector3d> grasp_neighbor_offsets(const RgbdImageFrame & frame,const ImageRegion & roi,
  const std::vector<std::uint8_t> & target_mask,const Vector3d & target_world,double tool_margin_m,double belt_z_m)
{
  valid_region(frame,roi);
  if(target_mask.size()!=frame.depth_m.size() || !roi.width || !roi.height ||
    !std::isfinite(tool_margin_m) || tool_margin_m<=0 || !std::isfinite(belt_z_m))return {};
  const auto world_in_camera=inverse_pose(frame.metadata.camera_in_world);
  if(!world_in_camera)return {};
  const double target_depth=transform_point(*world_in_camera,target_world).z_m;
  if(!std::isfinite(target_depth) || target_depth<=0)return {};
  const auto & camera=frame.camera;
  const auto margin_x=static_cast<std::uint32_t>(std::min<double>(camera.width,std::ceil(camera.fx*tool_margin_m/target_depth)));
  const auto margin_y=static_cast<std::uint32_t>(std::min<double>(camera.height,std::ceil(camera.fy*tool_margin_m/target_depth)));
  const auto x0=roi.x-std::min(roi.x,margin_x),y0=roi.y-std::min(roi.y,margin_y);
  const auto x_end=roi.x+roi.width,y_end=roi.y+roi.height;
  const auto x1=x_end+std::min(camera.width-x_end,margin_x);
  const auto y1=y_end+std::min(camera.height-y_end,margin_y);
  const double radius=std::min(0.30,tool_margin_m+
    0.5*target_depth*std::max(roi.width/camera.fx,roi.height/camera.fy));
  const double radius_squared=radius*radius;
  const bool rectified=std::all_of(camera.distortion.begin(),camera.distortion.end(),[](double value){return value==0.;});
  const auto eligible=[&](std::size_t pixel){
    const float depth=frame.depth_m[pixel];
    return !target_mask[pixel] && std::isfinite(depth) && depth>0;
  };
  // Stride over valid pixels. A raster stride can miss the sparse registered depth.
  std::size_t valid=0;
  for(auto y=y0;y<y1;++y)for(auto x=x0;x<x1;++x)
    valid+=eligible(static_cast<std::size_t>(y)*camera.width+x);
  constexpr std::size_t max_unprojected=4096;
  const auto stride=std::max<std::size_t>(1,(valid+max_unprojected-1)/max_unprojected);
  constexpr int voxel_axis_cells=122; // 0.60 m diameter at 5 mm resolution, including boundary.
  std::unordered_set<int> occupied;
  occupied.reserve(std::min(valid,max_unprojected));
  std::vector<Vector3d> result;
  result.reserve(std::min(valid,max_unprojected));
  std::size_t index=0;
  for(auto y=y0;y<y1;++y)for(auto x=x0;x<x1;++x){
    const auto pixel=static_cast<std::size_t>(y)*camera.width+x;
    if(!eligible(pixel) || index++%stride)continue;
    const double depth=frame.depth_m[pixel];
    const Vector3d in_camera=rectified?Vector3d{(x-camera.cx)*depth/camera.fx,(y-camera.cy)*depth/camera.fy,depth}:
      unproject(x,y,depth,camera);
    const auto point=transform_point(frame.metadata.camera_in_world,in_camera);
    const double dx=point.x_m-target_world.x_m,dy=point.y_m-target_world.y_m,dz=point.z_m-target_world.z_m;
    if(!std::isfinite(dx+dy+dz) || dx*dx+dy*dy>radius_squared || std::abs(dz)>radius ||
      point.z_m<=belt_z_m+kGraspNeighborVoxelM)continue;
    const auto cell=[](double offset){return static_cast<int>(std::floor(offset/kGraspNeighborVoxelM));};
    const int ix=cell(dx),iy=cell(dy),iz=cell(dz);
    const int key=(ix*voxel_axis_cells+iy)*voxel_axis_cells+iz;
    if(occupied.insert(key).second)result.push_back({(ix+.5)*kGraspNeighborVoxelM,
      (iy+.5)*kGraspNeighborVoxelM,(iz+.5)*kGraspNeighborVoxelM});
  }
  if(result.size()>kGraspNeighborLimit){
    const auto planar_distance=[](const Vector3d & p){return p.x_m*p.x_m+p.y_m*p.y_m;};
    std::nth_element(result.begin(),result.begin()+kGraspNeighborLimit,result.end(),[&](const auto & a,const auto & b){
      return planar_distance(a)<planar_distance(b);
    });
    result.resize(kGraspNeighborLimit);
  }
  return result;
}
}  // namespace ptc_core
