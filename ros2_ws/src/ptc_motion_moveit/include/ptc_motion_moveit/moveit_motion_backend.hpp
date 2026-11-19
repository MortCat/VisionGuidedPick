#pragma once
#include "ptc_motion/arm_gateway_node.hpp"
namespace ptc_motion_moveit
{
std::shared_ptr<ptc_motion::MotionBackend> make_moveit_motion_backend(const std::shared_ptr<ptc_motion::ArmGatewayNode> &);
}
