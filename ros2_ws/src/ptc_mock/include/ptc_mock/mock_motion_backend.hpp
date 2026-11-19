#pragma once
#include "ptc_motion/arm_gateway_node.hpp"
namespace ptc_mock
{
std::shared_ptr<ptc_motion::MotionBackend> make_mock_motion_backend(const std::shared_ptr<ptc_motion::ArmGatewayNode> &);
}
