#pragma once
#include <geometric_shapes/bodies.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace ptc_motion_moveit
{
struct RigidToolCollisionShape
{
  Eigen::Isometry3d pose_tcp;
  bodies::BodyPtr body;
};

// A ranking cost, not a collision gate. Expected finger contact is excluded
// by the caller; only the rigid palm/camera assembly is represented here.
inline std::size_t count_grasp_tool_intersections(
  std::vector<RigidToolCollisionShape> & tool,
  const std::vector<Eigen::Vector3d> & measured_world,
  const Eigen::Isometry3d & tcp_world)
{
  for(auto & shape:tool){shape.body->setPose(tcp_world*shape.pose_tcp);}
  return std::count_if(measured_world.begin(),measured_world.end(),[&](const auto & point){
    return std::any_of(tool.begin(),tool.end(),[&](const auto & shape){return shape.body->containsPoint(point);});
  });
}
struct GraspSweepBox
{
  Eigen::Isometry3d open_pose_tcp;
  Eigen::AlignedBox3d bounds_shape;
  Eigen::Vector3d shift_tcp_per_meter;
};

struct GraspSweepSegment
{
  Eigen::Isometry3d world_in_shape;
  Eigen::AlignedBox3d bounds_shape;
  Eigen::Vector3d displacement_shape;

  bool intersects(const Eigen::Vector3d & world_point) const
  {
    const Eigen::Vector3d point=world_in_shape*world_point;
    double entry=0.,exit=1.;
    for(int axis=0;axis<3;++axis){
      const double speed=displacement_shape[axis];
      const double lower=point[axis]-bounds_shape.max()[axis];
      const double upper=point[axis]-bounds_shape.min()[axis];
      if(std::abs(speed)<1e-12){
        if(lower>0. || upper<0.)return false;
        continue;
      }
      const auto times=std::minmax({lower/speed,upper/speed});
      entry=std::max(entry,times.first);
      exit=std::min(exit,times.second);
      if(entry>exit)return false;
    }
    return true;
  }
};

// Contact orientation is constant during the local linear descent, closure
// and lift. Use translated boxes in each shape's axes, including finger travel.
inline std::size_t count_neighbor_sweep_intersections(const std::vector<GraspSweepBox> & tool,
  const std::vector<Eigen::Vector3d> & neighbors_world,const Eigen::Isometry3d & contact_world,
  const Eigen::Vector3d & pregrasp_world,double width_delta_m,double lift_m)
{
  std::vector<GraspSweepSegment> sweeps;
  sweeps.reserve(tool.size()*3);
  for(const auto & shape:tool){
    const Eigen::Isometry3d open=contact_world*shape.open_pose_tcp;
    const Eigen::Vector3d descent=contact_world.translation()-pregrasp_world;
    const Eigen::Vector3d closing=contact_world.linear()*shape.shift_tcp_per_meter*width_delta_m;
    Eigen::Isometry3d start=open;
    start.translation()-=descent;
    sweeps.push_back({start.inverse(),shape.bounds_shape,open.linear().transpose()*descent});
    if(closing.squaredNorm()>1e-18)
      sweeps.push_back({open.inverse(),shape.bounds_shape,open.linear().transpose()*closing});
    Eigen::Isometry3d closed=open;
    closed.translation()+=closing;
    sweeps.push_back({closed.inverse(),shape.bounds_shape,open.linear().transpose()*Eigen::Vector3d(0.,0.,lift_m)});
  }
  return std::count_if(neighbors_world.begin(),neighbors_world.end(),[&](const auto & point){
    return std::any_of(sweeps.begin(),sweeps.end(),[&](const auto & sweep){return sweep.intersects(point);});
  });
}
}  // namespace ptc_motion_moveit
