#include "ptc_task/task_coordinator_node.hpp"
int main(int argc,char ** argv)
{rclcpp::init(argc,argv);auto node=std::make_shared<ptc_task::TaskCoordinatorNode>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}
