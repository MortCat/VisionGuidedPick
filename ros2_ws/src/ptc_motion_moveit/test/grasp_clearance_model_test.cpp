#include "ptc_motion_moveit/grasp_clearance_model.hpp"
#include <geometric_shapes/body_operations.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
  using namespace ptc_motion_moveit;
  try{
    const auto check=[](bool value){if(!value)throw std::runtime_error("rigid tool clearance regression");};
    shapes::Box box(.02,.04,.02);
    Eigen::Isometry3d camera=Eigen::Isometry3d::Identity();camera.translation().x()=.05;
    std::vector<RigidToolCollisionShape> tool{{camera,bodies::BodyPtr(bodies::createBodyFromShape(&box))}};
    Eigen::Isometry3d tcp=Eigen::Isometry3d::Identity();tcp.translation()=Eigen::Vector3d(1.,2.,3.);
    std::vector<Eigen::Vector3d> surface{{1.05,2.,3.},{1.,2.,3.}};
    check(count_grasp_tool_intersections(tool,surface,tcp)==1);
    // Same grasp centre, opposite yaw: the offset camera clears the target.
    tcp.linear()=Eigen::AngleAxisd(std::acos(-1.),Eigen::Vector3d::UnitZ()).toRotationMatrix();
    check(count_grasp_tool_intersections(tool,surface,tcp)==0);
    tcp.translation().x()+=.10;
    check(count_grasp_tool_intersections(tool,surface,tcp)==1);
    // Overlapping rigid shapes must count each measured point once.
    tool.push_back({camera,bodies::BodyPtr(bodies::createBodyFromShape(&box))});
    check(count_grasp_tool_intersections(tool,surface,tcp)==1);
    check(count_grasp_tool_intersections(tool,{},tcp)==0);
    const Eigen::AlignedBox3d finger_box(Eigen::Vector3d(-.01,-.006,-.008),Eigen::Vector3d(.01,.006,.008));
    Eigen::Isometry3d left=Eigen::Isometry3d::Identity();left.translation().y()=.04;
    const std::vector<GraspSweepBox> sweep{{left,finger_box,Eigen::Vector3d(0.,.5,0.)}};
    tcp=Eigen::Isometry3d::Identity();
    const Eigen::Vector3d pregrasp(0.,0.,.04);
    // One obstacle midway down, another in the finger's closing travel and
    // one during lift. The same voxels clear the gripper after yawing 90 degrees.
    const std::vector<Eigen::Vector3d> neighbors{{0.,.04,.02},{0.,.025,0.},{0.,.02,.027},{.05,0.,0.}};
    check(count_neighbor_sweep_intersections(sweep,neighbors,tcp,pregrasp,-.04,.04)==3);
    tcp.linear()=Eigen::AngleAxisd(std::acos(-1.)/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    check(count_neighbor_sweep_intersections(sweep,neighbors,tcp,pregrasp,-.04,.04)==0);
    std::cout<<"grasp clearance model passed\n";
  }catch(const std::exception & error){std::cerr<<error.what()<<"\n";return 1;}
}
