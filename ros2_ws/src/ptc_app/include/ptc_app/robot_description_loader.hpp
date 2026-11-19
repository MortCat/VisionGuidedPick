#pragma once
#include "ptc_core/config/system_settings.hpp"

namespace ptc_app
{
struct RobotDescriptionModel
{
  std::string urdf_xml;
  std::string collision_geometry_digest;
  ptc_core::Pose3d camera_in_tcp;
  std::vector<std::string> arm_joint_names;
};
// Reads verified assets at startup; compares known extrinsics against fixed URDF chain.
RobotDescriptionModel load_robot_description(const ptc_core::SystemSettings &);
}  // namespace ptc_app
