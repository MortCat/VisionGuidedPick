#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
namespace ptc_task
{
class TaskCoordinatorNode : public rclcpp::Node
{
public:
  explicit TaskCoordinatorNode(const rclcpp::NodeOptions & = rclcpp::NodeOptions{});
  ~TaskCoordinatorNode();
private:
  struct Runtime;std::unique_ptr<Runtime> runtime_;
};
}
