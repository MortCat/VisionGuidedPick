#include "ptc_app/ros_converter.hpp"
#include "ptc_core/move_to_pose.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace ptc_app
{
namespace
{
void require(bool valid, const char * reason) {if (!valid) {throw std::invalid_argument(reason);}}
bool finite(double v) {return std::isfinite(v);}
ptc_core::DeviceStatus status(bool ready)
{
  ptc_core::DeviceStatus s;
  s.connection_state = ptc_core::DeviceConnectionState::kConnected;
  s.health_state = ready ? ptc_core::DeviceHealthState::kHealthy : ptc_core::DeviceHealthState::kDegraded;
  s.ready = ready; s.data_stale = !ready; return s;
}
}
ptc_core::Nanoseconds from_ros(const builtin_interfaces::msg::Time & t)
{
  require(t.sec >= 0 && t.nanosec < 1000000000,"invalid source timestamp");
  return ptc_core::Nanoseconds{static_cast<std::int64_t>(t.sec)*1000000000+t.nanosec};
}
builtin_interfaces::msg::Time to_ros_time(ptc_core::Nanoseconds t)
{
  require(t.count() >= 0 && t.count()/1000000000 <= std::numeric_limits<std::int32_t>::max(),"timestamp out of wire range");
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(t.count()/1000000000);
  out.nanosec = static_cast<std::uint32_t>(t.count()%1000000000); return out;
}
builtin_interfaces::msg::Duration to_ros_duration(double seconds)
{
  const auto t = ptc_core::seconds_to_duration(seconds); require(t.has_value(),"invalid duration");
  const auto time = to_ros_time(*t); builtin_interfaces::msg::Duration out;
  out.sec = time.sec; out.nanosec = time.nanosec; return out;
}
ptc_core::EventKey from_ros(const ptc_interfaces::msg::EventKey & k)
{
  require(!k.settings_revision.empty(),"missing settings revision");
  return {k.epoch,k.cycle_id,k.track_id,k.session_id,k.operation_id,k.sequence,k.settings_revision};
}
ptc_interfaces::msg::EventKey to_ros(const ptc_core::EventKey & k)
{
  ptc_interfaces::msg::EventKey out;
  out.epoch=k.epoch; out.cycle_id=k.cycle_id; out.track_id=k.track_id; out.session_id=k.session_id;
  out.operation_id=k.operation_id; out.sequence=k.sequence; out.settings_revision=k.settings_revision; return out;
}
ptc_core::Pose3d from_ros(const geometry_msgs::msg::Pose & p)
{
  ptc_core::Pose3d out{{p.position.x,p.position.y,p.position.z},{p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w}};
  require(ptc_core::is_valid_pose(out),"invalid pose/quaternion"); return out;
}
geometry_msgs::msg::Pose to_ros(const ptc_core::Pose3d & p)
{
  require(ptc_core::is_valid_pose(p),"invalid pose/quaternion");
  geometry_msgs::msg::Pose out; out.position.x=p.position.x_m; out.position.y=p.position.y_m; out.position.z=p.position.z_m;
  out.orientation.x=p.orientation.x; out.orientation.y=p.orientation.y; out.orientation.z=p.orientation.z; out.orientation.w=p.orientation.w; return out;
}
geometry_msgs::msg::PoseStamped stamped_pose(const ptc_core::Pose3d & p, const std::string & frame, ptc_core::Nanoseconds t)
{
  require(!frame.empty(),"missing pose reference frame"); geometry_msgs::msg::PoseStamped out;
  out.header.frame_id=frame; out.header.stamp=to_ros_time(t); out.pose=to_ros(p); return out;
}
ptc_core::TrackedObjectState from_ros(const ptc_interfaces::msg::TrackedObject & m)
{
  require(m.phase <= m.RETIRED && !m.pose.header.frame_id.empty(),"invalid track phase/frame");
  require(finite(m.position_stddev_m) && m.position_stddev_m >= 0,"invalid position uncertainty");
  require(!m.yaw_valid || (finite(m.yaw_stddev_rad) && m.yaw_stddev_rad >= 0),"invalid yaw uncertainty");
  require(!m.velocity_valid || (finite(m.velocity_mps.x) && finite(m.velocity_mps.y) && finite(m.velocity_mps.z) &&
    finite(m.velocity_fit_residual_m) && m.velocity_fit_residual_m>=0 &&
    finite(m.velocity_uncertainty_mps) && m.velocity_uncertainty_mps>=0),"invalid velocity/fit quality");
  require(!m.envelope_valid || (finite(m.envelope_center_object_m.x) && finite(m.envelope_center_object_m.y) &&
    finite(m.envelope_center_object_m.z) && finite(m.envelope_radius_xy_m) && m.envelope_radius_xy_m>0 &&
    finite(m.envelope_half_height_m) && m.envelope_half_height_m>0),"invalid object envelope");
  ptc_core::TrackedObjectState out; out.phase=static_cast<ptc_core::TrackPhase>(m.phase); out.context.key=from_ros(m.key);
  out.observation={from_ros(m.pose.pose),from_ros(m.pose.header.stamp),m.pose.header.frame_id,m.position_stddev_m,true,m.yaw_stddev_rad,m.yaw_valid};
  out.observation.envelope_center_object_m={m.envelope_center_object_m.x,m.envelope_center_object_m.y,m.envelope_center_object_m.z};
  out.observation.envelope_radius_xy_m=m.envelope_radius_xy_m;out.observation.envelope_half_height_m=m.envelope_half_height_m;
  out.observation.envelope_valid=m.envelope_valid;
  out.velocity_mps={m.velocity_mps.x,m.velocity_mps.y,m.velocity_mps.z}; out.velocity_valid=m.velocity_valid;
  out.velocity_fit_residual_m=m.velocity_fit_residual_m;out.velocity_uncertainty_mps=m.velocity_uncertainty_mps;
  out.confirmations=m.confirmations; return out;
}
ptc_interfaces::msg::TrackedObject to_ros(const ptc_core::TrackedObjectState & s)
{
  ptc_interfaces::msg::TrackedObject out; out.key=to_ros(s.context.key); out.phase=static_cast<std::uint8_t>(s.phase);
  out.pose=stamped_pose(s.observation.pose_world,s.observation.reference_frame,s.observation.timestamp);
  out.position_stddev_m=s.observation.position_stddev_m; out.yaw_stddev_rad=s.observation.yaw_stddev_rad; out.yaw_valid=s.observation.yaw_valid;
  out.envelope_center_object_m.x=s.observation.envelope_center_object_m.x_m;
  out.envelope_center_object_m.y=s.observation.envelope_center_object_m.y_m;
  out.envelope_center_object_m.z=s.observation.envelope_center_object_m.z_m;
  out.envelope_radius_xy_m=s.observation.envelope_radius_xy_m;out.envelope_half_height_m=s.observation.envelope_half_height_m;
  out.envelope_valid=s.observation.envelope_valid;
  out.velocity_mps.x=s.velocity_mps.x_m; out.velocity_mps.y=s.velocity_mps.y_m; out.velocity_mps.z=s.velocity_mps.z_m;
  out.velocity_valid=s.velocity_valid;out.velocity_fit_residual_m=s.velocity_fit_residual_m;
  out.velocity_uncertainty_mps=s.velocity_uncertainty_mps;
  out.confirmations=static_cast<std::uint32_t>(s.confirmations); return out;
}
ptc_core::RgbdFrame from_ros(const ptc_interfaces::msg::RgbdFrame & m)
{
  require(m.frame_id != 0 && !m.calibration_id.empty(),"invalid RGB-D identity");
  require(m.color.width && m.color.height && m.color.width == m.depth.width && m.color.height == m.depth.height,
    "depth must match RGB dimensions");
  require(!m.color.header.frame_id.empty() && !m.depth.header.frame_id.empty(),"missing RGB-D frame");
  require(m.depth.encoding == "32FC1" && m.depth.step >= static_cast<std::uint64_t>(m.depth.width)*4 &&
    m.depth.data.size() == static_cast<std::uint64_t>(m.depth.step)*m.depth.height,"invalid registered depth payload");
  require((m.color.encoding == "rgb8" || m.color.encoding == "bgr8") &&
    m.color.step >= static_cast<std::uint64_t>(m.color.width)*3 &&
    m.color.data.size() == static_cast<std::uint64_t>(m.color.step)*m.color.height,"invalid RGB payload");
  require(m.intrinsics.width == m.color.width && m.intrinsics.height == m.color.height &&
    finite(m.intrinsics.k[0]) && m.intrinsics.k[0] > 0 && finite(m.intrinsics.k[4]) && m.intrinsics.k[4] > 0,
    "invalid intrinsics");
  require(from_ros(m.camera_world.header.stamp) == from_ros(m.color.header.stamp),"camera transform must be acquisition-time");
  ptc_core::RgbdFrame out;
  out.frame_id=m.frame_id; out.epoch=m.key.epoch; out.color_timestamp=from_ros(m.color.header.stamp); out.depth_timestamp=from_ros(m.depth.header.stamp);
  out.color_frame=m.color.header.frame_id; out.depth_frame=m.depth.header.frame_id; out.calibration_id=m.calibration_id;
  out.depth_source_frame=m.depth_source_frame;
  out.depth_registered=m.depth_registered; out.intrinsics_valid=true; out.transform_valid=!m.camera_world.header.frame_id.empty();
  require(from_ros(m.tcp_world.header.stamp)==out.color_timestamp && m.tcp_world.header.frame_id==m.camera_world.header.frame_id,"TCP transform must use the capture time/world frame");
  out.tcp_in_world=from_ros(m.tcp_world.pose);out.camera_in_world=from_ros(m.camera_world.pose); return out;
}
ptc_core::ArmSnapshot from_ros(const ptc_interfaces::msg::ArmStatus & m, const std::string & tcp_frame)
{
  require(m.owner <= m.OWNER_STOPPING && m.phase<=static_cast<std::uint8_t>(ptc_core::ArmMotionState::kFollowing) && !m.tcp.header.frame_id.empty(),"invalid arm owner/frame/phase");
  require(m.joints.position.size() == m.joints.name.size() &&
    (m.joints.velocity.empty() || m.joints.velocity.size() == m.joints.name.size()) &&
    (m.joints.effort.empty() || m.joints.effort.size() == m.joints.name.size()),"joint array mismatch");
  ptc_core::ArmSnapshot out; out.epoch=m.key.epoch; out.joint_timestamp=from_ros(m.joints.header.stamp);
  out.tcp_timestamp=from_ros(m.tcp.header.stamp); out.controller_ready=m.ready; out.stopped=m.stopped; out.tf_valid=true;
  out.state.timestamp=out.tcp_timestamp; out.state.status=status(m.ready); out.state.tcp_pose=from_ros(m.tcp.pose);
  out.state.base_frame=m.tcp.header.frame_id; out.state.tcp_frame=tcp_frame;
  out.state.motion_state=static_cast<ptc_core::ArmMotionState>(m.phase);
  std::set<std::string> seen;
  for (std::size_t i=0;i<m.joints.name.size();++i) {
    require(!m.joints.name[i].empty() && seen.insert(m.joints.name[i]).second && finite(m.joints.position[i]),"invalid joint");
    ptc_core::ArmJointState joint{m.joints.name[i],m.joints.position[i],{}, {}};
    if (!m.joints.velocity.empty()) {require(finite(m.joints.velocity[i]),"invalid joint velocity"); joint.velocity=m.joints.velocity[i];}
    if (!m.joints.effort.empty()) {require(finite(m.joints.effort[i]),"invalid joint effort"); joint.effort=m.joints.effort[i];}
    out.state.joints.push_back(std::move(joint));
  }
  return out;
}
ptc_core::GripperState from_ros(const ptc_interfaces::msg::GripperStatus & m)
{
  require(finite(m.width_m) && m.width_m >= 0,"invalid gripper width"); ptc_core::GripperState out;
  out.timestamp=from_ros(m.header.stamp); out.epoch=m.key.epoch; out.evidence_sequence=m.evidence_sequence;
  out.status=status(m.ready); out.width_m=m.width_m; out.stopped=m.stopped; out.empty_confirmed=m.empty_confirmed;
  out.contact_confirmed=m.contact_confirmed; out.visual_follow_confirmed=m.visual_follow_confirmed; return out;
}
}  // namespace ptc_app
