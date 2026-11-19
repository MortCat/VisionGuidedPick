#include "ptc_motion_moveit/moveit_motion_backend.hpp"
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<ptc_motion::ArmGatewayNode>(rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true));
  auto backend=ptc_motion_moveit::make_moveit_motion_backend(node);node->set_backend(backend);
  rclcpp::spin(node);node->stop_runtime();backend.reset();node.reset();rclcpp::shutdown();return 0;
}
