#include "ptc_vision/vision_node.hpp"
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<ptc_vision::VisionNode>();
  rclcpp::spin(node); node.reset(); rclcpp::shutdown(); return 0;
}
