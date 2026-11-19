#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
namespace ptc_vision
{
class VisionNode : public rclcpp::Node
{
public:
  explicit VisionNode(const rclcpp::NodeOptions & = rclcpp::NodeOptions{});
  ~VisionNode();
private:
  struct Runtime; std::unique_ptr<Runtime> runtime_;
};
}
