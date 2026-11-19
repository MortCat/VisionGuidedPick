#include "ptc_mock/mock_motion_backend.hpp"
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<ptc_motion::ArmGatewayNode>();
  auto backend=ptc_mock::make_mock_motion_backend(node); node->set_backend(backend);
  rclcpp::spin(node); node->stop_runtime(); rclcpp::shutdown(); return 0;
}
