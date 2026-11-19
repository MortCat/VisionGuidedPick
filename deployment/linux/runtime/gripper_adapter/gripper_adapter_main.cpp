#include "gripper_adapter_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto first = std::make_shared<GripperAdapterNode>("arm_01");
  auto second = std::make_shared<GripperAdapterNode>("arm_02");
  executor.add_node(first); executor.add_node(second); executor.spin();
  rclcpp::shutdown();
  return 0;
}
