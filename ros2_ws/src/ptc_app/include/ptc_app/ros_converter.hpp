#pragma once
#include "ptc_interfaces/msg/rgbd_frame.hpp"
#include "ptc_core/state/grasp_state.hpp"
#include "ptc_core/state/track_state.hpp"
#include <builtin_interfaces/msg/duration.hpp>
#include "ptc_interfaces/msg/arm_status.hpp"
#include "ptc_interfaces/msg/gripper_status.hpp"
#include "ptc_interfaces/msg/tracked_object.hpp"

namespace ptc_app
{
// Invalid wire data throws std::invalid_argument; Node boundaries report/reject it.
ptc_core::Nanoseconds from_ros(const builtin_interfaces::msg::Time &);
builtin_interfaces::msg::Time to_ros_time(ptc_core::Nanoseconds);
builtin_interfaces::msg::Duration to_ros_duration(double seconds);
ptc_core::EventKey from_ros(const ptc_interfaces::msg::EventKey &);
ptc_interfaces::msg::EventKey to_ros(const ptc_core::EventKey &);
ptc_core::Pose3d from_ros(const geometry_msgs::msg::Pose &);
geometry_msgs::msg::Pose to_ros(const ptc_core::Pose3d &);
geometry_msgs::msg::PoseStamped stamped_pose(const ptc_core::Pose3d &, const std::string &, ptc_core::Nanoseconds);
ptc_core::TrackedObjectState from_ros(const ptc_interfaces::msg::TrackedObject &);
ptc_interfaces::msg::TrackedObject to_ros(const ptc_core::TrackedObjectState &);
ptc_core::RgbdFrame from_ros(const ptc_interfaces::msg::RgbdFrame &);
ptc_core::ArmSnapshot from_ros(const ptc_interfaces::msg::ArmStatus &, const std::string & tcp_frame);
ptc_core::GripperState from_ros(const ptc_interfaces::msg::GripperStatus &);
}  // namespace ptc_app
