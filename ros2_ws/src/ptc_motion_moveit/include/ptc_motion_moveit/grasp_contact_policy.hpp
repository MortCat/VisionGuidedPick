#pragma once
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace ptc_motion_moveit
{
struct GraspContactPatchModel
{std::array<double,3> center;double radius_m;};

inline bool expected_grasp_contact(const std::string & first,const std::string & second,
  const std::array<double,3> & point,const std::string & target,
  const std::vector<std::string> & fingers,const std::vector<GraspContactPatchModel> & patches)
{
  if(target.empty() || (first==target)==(second==target))return false;
  const auto & link=first==target?second:first;
  if(std::find(fingers.begin(),fingers.end(),link)==fingers.end())return false;
  for(const auto & patch:patches){
    if(!std::isfinite(patch.radius_m)||patch.radius_m<=0)continue;
    const double distance=std::hypot(point[0]-patch.center[0],point[1]-patch.center[1],point[2]-patch.center[2]);
    if(std::isfinite(distance)&&distance<=patch.radius_m)return true;
  }
  return false;
}
// Replace only the explicitly supplied robot-link / tracked-object pairs.
// Static obstacles, robot self-collision and other links are untouched.
inline void replace_grasp_touch_permissions(moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::vector<std::string> & fingers,const std::vector<std::string> & targets)
{
  const auto count=matrix.entry_names.size();
  if(matrix.entry_values.size()!=count || std::any_of(matrix.entry_values.begin(),matrix.entry_values.end(),
    [count](const auto & row){return row.enabled.size()!=count;})){
    throw std::runtime_error("grasp contact policy requires a square collision matrix");
  }
  for(const auto & target:targets){if(target.empty() || target.rfind("tracked_",0)!=0 || target=="tracked_0"){
    throw std::runtime_error("grasp contact policy requires tracked objects");
  }}
  const auto ensure=[&](const std::string & name){
    const auto found=std::find(matrix.entry_names.begin(),matrix.entry_names.end(),name);
    if(found!=matrix.entry_names.end())return static_cast<std::size_t>(found-matrix.entry_names.begin());
    matrix.entry_names.push_back(name);
    for(auto & row:matrix.entry_values)row.enabled.push_back(false);
    moveit_msgs::msg::AllowedCollisionEntry row;row.enabled.resize(matrix.entry_names.size(),false);
    matrix.entry_values.push_back(std::move(row));return matrix.entry_names.size()-1;
  };
  for(const auto & finger:fingers){
    const auto index=ensure(finger);
    for(std::size_t object=0;object<matrix.entry_names.size();++object){
      if(matrix.entry_names[object].rfind("tracked_",0)==0){
        matrix.entry_values[index].enabled[object]=false;matrix.entry_values[object].enabled[index]=false;
      }
    }
  }
  for(const auto & target:targets){
    const auto object=ensure(target);
    for(const auto & finger:fingers){const auto index=ensure(finger);
      matrix.entry_values[index].enabled[object]=true;matrix.entry_values[object].enabled[index]=true;
    }
  }
}
// Formal behavior remains selected-target / configured-finger only.
inline void replace_grasp_touch_permissions(moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::vector<std::string> & fingers,const std::string & target)
{replace_grasp_touch_permissions(matrix,fingers,target.empty()?std::vector<std::string>{}:std::vector<std::string>{target});}

}
