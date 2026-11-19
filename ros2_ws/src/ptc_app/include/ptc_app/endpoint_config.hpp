#pragma once
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace ptc_app
{
// Logical keys are stable contracts; their relative ROS names are deployment settings.
struct EndpointConfig
{
  std::map<std::string,std::string> names;
  const std::string & at(const std::string & key) const {return names.at(key);}
};
inline const std::map<std::string,std::string> & default_endpoints()
{
  static const std::map<std::string,std::string> names{
    {"rgb","camera/color/image_raw"}, {"depth","camera/depth/image_raw"},
    {"rgb_intrinsics","camera/color/camera_info"}, {"depth_intrinsics","camera/depth/camera_info"},
    {"joint_states","joint_states"}, {"tracks","vision/tracks"}, {"frame","vision/frame"},
    {"detections","inference/detections"},
    {"object_cloud_preview","vision/object_cloud_preview"}, {"grasp_neighborhood","vision/grasp_neighborhood"}, {"vision_status","vision/status"}, {"inference_status","inference/status"},
    {"capture_grasp_surface","vision/capture_grasp_surface"},
    {"arm_move","arm/move_to_pose"}, {"arm_follow","arm/follow_target"},
    {"motion_check","arm/check_motion"},
    {"grasp_evaluate","arm/evaluate_grasp_candidates"},
    {"planning_scene","monitored_planning_scene"},
    {"arm_target","arm/target_update"}, {"arm_stop","arm/stop"}, {"arm_status","arm/status"},
    {"scene_sync","arm/sync_scene"},
    {"gripper_command","gripper/command"}, {"gripper_status","gripper/status"},
    {"controller_trajectory","arm_controller/follow_joint_trajectory"},
    {"controller_gripper","gripper_controller/gripper_cmd"},
    {"controller_owner_enable","control/servo_enable"},
    {"observer_rgb","observer/color/image_raw"},{"observer_depth","observer/depth/image_raw"},
    {"observer_rgb_intrinsics","observer/color/camera_info"},{"observer_depth_intrinsics","observer/depth/camera_info"},
    {"shared_tracks","shared_tracks"},
    {"servo_twist","servo_node/delta_twist_cmds"}, {"servo_pause","servo_node/pause_servo"},
    {"servo_command_type","servo_node/switch_command_type"}, {"servo_status","servo_node/status"},
    {"task_command","task/command"}, {"task_status","task/status"},
    {"inspection_view_status","inspection/view_status"}
  };
  return names;
}
// Five-view inspection endpoints with no runtime caller; settings that still name them are rejected.
inline const std::set<std::string> & retired_endpoints()
{
  static const std::set<std::string> names{"infer","geometry","capture","inspection_progress","inspection_report"};
  return names;
}
inline bool valid_relative_endpoint(const std::string & name)
{
  bool start = true;
  for (char c : name) {
    if (c == '/') {if (start) {return false;} start = true; continue;}
    const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    if (!letter && (start || c < '0' || c > '9')) {return false;} start = false;
  }
  return !start;
}
}  // namespace ptc_app
