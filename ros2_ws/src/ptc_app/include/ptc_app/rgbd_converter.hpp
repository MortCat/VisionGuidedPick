#pragma once
#include "ptc_core/vision/rgbd_image_frame.hpp"
#include "ptc_interfaces/msg/rgbd_frame.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
namespace ptc_app
{
ptc_core::CameraModel camera_model(const sensor_msgs::msg::CameraInfo &);
ptc_core::RgbdImageFrame image_frame(const ptc_interfaces::msg::RgbdFrame &,const ptc_core::VisionConfig &);
ptc_interfaces::msg::RgbdFrame to_ros(const ptc_core::RgbdImageFrame &,const ptc_core::EventKey &,const std::string & world_frame);
std::vector<std::uint8_t> decode_mask(const sensor_msgs::msg::CompressedImage &,const ptc_core::CameraModel &);
}
