#include "ptc_app/rgbd_converter.hpp"
#include "ptc_app/ros_converter.hpp"
#include <png.h>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <stdexcept>
namespace ptc_app
{
ptc_core::CameraModel camera_model(const sensor_msgs::msg::CameraInfo & info)
{
  if (!info.width || !info.height || info.k[0]<=0 || info.k[4]<=0 || info.d.size()>8 ||
    (info.distortion_model!="plumb_bob" && info.distortion_model!="rational_polynomial" && !info.distortion_model.empty())) {
    throw std::invalid_argument("unsupported/invalid RGB-D camera model");
  }
  ptc_core::CameraModel out{info.width,info.height,info.k[0],info.k[4],info.k[2],info.k[5],{}};
  std::copy(info.d.begin(),info.d.end(),out.distortion.begin()); return out;
}
ptc_core::RgbdImageFrame image_frame(const ptc_interfaces::msg::RgbdFrame & message,const ptc_core::VisionConfig & config)
{
  ptc_core::RgbdImageFrame frame; frame.metadata=from_ros(message); frame.camera=camera_model(message.intrinsics);
  if (message.color.data.size()+message.depth.data.size()>config.max_frame_megabytes*1024*1024) {throw std::invalid_argument("RGB-D bytes capacity exceeded");}
  frame.rgb.resize(static_cast<std::size_t>(frame.camera.width)*frame.camera.height*3);
  for (std::uint32_t y=0;y<frame.camera.height;++y) {for (std::uint32_t x=0;x<frame.camera.width;++x) {
    const auto source=static_cast<std::size_t>(y)*message.color.step+x*3,destination=(static_cast<std::size_t>(y)*frame.camera.width+x)*3;
    frame.rgb[destination]=message.color.data[source+(message.color.encoding=="bgr8" ? 2 : 0)];
    frame.rgb[destination+1]=message.color.data[source+1]; frame.rgb[destination+2]=message.color.data[source+(message.color.encoding=="bgr8" ? 0 : 2)];
  }}
  frame.depth_m=ptc_core::decode_depth(message.depth.width,message.depth.height,message.depth.step,message.depth.is_bigendian,false,message.depth.data,config);
  return frame;
}
ptc_interfaces::msg::RgbdFrame to_ros(const ptc_core::RgbdImageFrame & frame,const ptc_core::EventKey & key,const std::string & world)
{
  ptc_interfaces::msg::RgbdFrame out; out.key=to_ros(key); out.frame_id=frame.metadata.frame_id;
  out.calibration_id=frame.metadata.calibration_id; out.depth_source_frame=frame.metadata.depth_source_frame;
  out.depth_registered=frame.metadata.depth_registered;
  out.color.header.frame_id=frame.metadata.color_frame; out.color.header.stamp=to_ros_time(frame.metadata.color_timestamp);
  out.color.width=frame.camera.width; out.color.height=frame.camera.height; out.color.encoding="rgb8"; out.color.step=out.color.width*3;
  out.color.data=frame.rgb;
  out.depth.header.frame_id=frame.metadata.depth_frame; out.depth.header.stamp=to_ros_time(frame.metadata.depth_timestamp);
  out.depth.width=frame.camera.width; out.depth.height=frame.camera.height; out.depth.encoding="32FC1"; out.depth.step=out.depth.width*4;
  out.depth.data.resize(frame.depth_m.size()*sizeof(float)); std::memcpy(out.depth.data.data(),frame.depth_m.data(),out.depth.data.size());
  const std::uint16_t one=1; out.depth.is_bigendian=*reinterpret_cast<const std::uint8_t *>(&one)==0;
  out.intrinsics.header=out.color.header; out.intrinsics.width=frame.camera.width; out.intrinsics.height=frame.camera.height;
  out.intrinsics.k={frame.camera.fx,0,frame.camera.cx,0,frame.camera.fy,frame.camera.cy,0,0,1};
  out.intrinsics.distortion_model="rational_polynomial"; out.intrinsics.d.assign(frame.camera.distortion.begin(),frame.camera.distortion.end());
  out.camera_world=stamped_pose(frame.metadata.camera_in_world,world,frame.metadata.color_timestamp);
  out.tcp_world=stamped_pose(frame.metadata.tcp_in_world,world,frame.metadata.color_timestamp);return out;
}
std::vector<std::uint8_t> decode_mask(const sensor_msgs::msg::CompressedImage & mask,const ptc_core::CameraModel & camera)
{
  if (mask.data.empty()) {return {};}
  if (mask.format!="png") {throw std::invalid_argument("object mask must be PNG");}
  struct Image {png_image value{}; Image() {value.version=PNG_IMAGE_VERSION;} ~Image() {png_image_free(&value);}} image;
  if (!png_image_begin_read_from_memory(&image.value,mask.data.data(),mask.data.size()) || image.value.width!=camera.width || image.value.height!=camera.height) {
    throw std::invalid_argument("invalid mask/dimensions");
  }
  image.value.format=PNG_FORMAT_GRAY; std::vector<std::uint8_t> decoded(static_cast<std::size_t>(camera.width)*camera.height);
  if (!png_image_finish_read(&image.value,nullptr,decoded.data(),0,nullptr)) {throw std::invalid_argument("cannot decode mask");} return decoded;
}
}  // namespace ptc_app
