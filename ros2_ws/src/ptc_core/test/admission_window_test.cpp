#include "ptc_core/state/admission_model.hpp"
#include "state_test_support.hpp"
#include "ptc_core/vision/cloud_registration_model.hpp"
#include <cmath>

using namespace ptc_core;
using namespace ptc_test;

int main()
{
  return run([] {
    auto data=settings_data();
    data.vision.depth_workspace.bounds_min_m={-1,-1,-1};data.vision.depth_workspace.bounds_max_m={1,1,1};
    data.motion.bounds_min_m={-0.5,-0.5,-0.5};data.motion.bounds_max_m={0.5,0.5,0.5};
    data.task.cycle_timeout_sec=10;
    const auto config=settings(data);auto object=track(1);
    Pose3d planned;Pose3d right,left;right.position.x_m=0.2;left.position.x_m=-0.2;
    const std::vector<Pose3d> views{right,left};

    const auto accepted=estimate_inspection_window(object,planned,views,config,time(1),2.0);
    CHECK(accepted.feasible);CHECK(accepted.enough_time);CHECK(accepted.available_sec>2.0);
    const auto insufficient=estimate_inspection_window(object,planned,views,config,time(1),3.0);
    CHECK(insufficient.feasible);CHECK(!insufficient.enough_time);CHECK(insufficient.available_sec==accepted.available_sec);

    auto reverse=object;reverse.velocity_mps.x_m=-0.2;
    const auto reverse_window=estimate_inspection_window(reverse,planned,views,config,time(1),1.5);
    CHECK(reverse_window.feasible);CHECK(!reverse_window.enough_time);CHECK(reverse_window.available_sec<1.5);
    auto transverse=object;transverse.velocity_mps={0,0.2,0};
    const auto transverse_window=estimate_inspection_window(transverse,planned,views,config,time(1),3.0);
    CHECK(transverse_window.feasible);CHECK(!transverse_window.enough_time);
    CHECK(std::abs(transverse_window.available_sec-0.5/0.21)<1e-9);

    auto near_zero=object;near_zero.velocity_mps={};
    const auto bounded=estimate_inspection_window(near_zero,planned,views,config,time(1),9.0);
    CHECK(bounded.feasible);CHECK(bounded.enough_time);CHECK(std::isfinite(bounded.available_sec));
    CHECK(bounded.available_sec<=config.get().task.cycle_timeout_sec);

    auto outside=object;outside.observation.pose_world.position.x_m=0.4;
    CHECK(!estimate_inspection_window(outside,planned,views,config,time(1),1.0).feasible);
    auto broad_data=data;broad_data.motion.bounds_min_m={-2,-2,-2};broad_data.motion.bounds_max_m={2,2,2};
    const auto broad_config=settings(broad_data);
    auto envelope_edge=object;envelope_edge.observation.pose_world.position.x_m=0.95;
    CHECK(!estimate_inspection_window(envelope_edge,planned,views,broad_config,time(1),1.0).feasible);
    auto shifted_center=object;shifted_center.observation.pose_world.position.x_m=0.91;
    shifted_center.observation.envelope_center_object_m.x_m=0.04;
    CHECK(!estimate_inspection_window(shifted_center,planned,views,broad_config,time(1),1.0).feasible);
    auto support_data=broad_data;support_data.runtime.operation_mode="assumed_ng_demo";support_data.inspection.inference.backend="assumed_ng";support_data.vision.foreground_source="depth_workspace";
    support_data.vision.depth_workspace.bounds_min_m.z_m=.75;support_data.vision.depth_workspace.bounds_max_m.z_m=1.1;
    support_data.vision.depth_workspace.support_plane_z_m=.75;
    const auto support_config=settings(support_data);auto supported=object;
    supported.observation.pose_world.position.z_m=.755;
    supported.observation.envelope_half_height_m=.0375;supported.observation.envelope_center_object_m.z_m=.0375;
    Pose3d support_pose=supported.observation.pose_world;
    const auto support_window=estimate_inspection_window(supported,support_pose,{support_pose},support_config,time(1),1);
    CHECK(support_window.feasible);CHECK(support_window.enough_time);
    supported.observation.pose_world.position.z_m=.70;
    CHECK(!estimate_inspection_window(supported,support_pose,{support_pose},support_config,time(1),1).feasible);
    // Dense low body samples and a sparse high ear must not invent geometry
    // below a model whose actual base is zero, nor reject it on the support plane.
    std::vector<Vector3d> asymmetric{{-.01,-.02,0},{.01,.02,.075}};
    for(unsigned i=0;i<40;++i){asymmetric.push_back({-.009+.0004*i,-.015,.001+.0001*i});}
    CloudRegistrationConfig registration;registration.coarse_voxel_size_m=.003;registration.fine_voxel_size_m=.0015;registration.max_points=128;
    CloudRegistrationModel reference(asymmetric,registration);
    CHECK(std::abs(reference.center_object.z_m-reference.half_height_m)<1e-12);
    auto grounded=object;grounded.observation.pose_world.position.z_m=.75;
    grounded.observation.envelope_center_object_m=reference.center_object;
    grounded.observation.envelope_radius_xy_m=reference.radius_m;grounded.observation.envelope_half_height_m=reference.half_height_m;
    const auto grounded_window=estimate_inspection_window(grounded,grounded.observation.pose_world,
      {grounded.observation.pose_world},support_config,time(1),1);
    CHECK(grounded_window.feasible);CHECK(grounded_window.enough_time);
    auto missing_envelope=object;missing_envelope.observation.envelope_valid=false;
    CHECK(!estimate_inspection_window(missing_envelope,planned,views,config,time(1),1.0).feasible);
    Pose3d unreachable_right,unreachable_left;unreachable_right.position.x_m=0.8;unreachable_left.position.x_m=-0.8;
    CHECK(!estimate_inspection_window(object,planned,{unreachable_right,unreachable_left},config,time(1),1.0).feasible);
  });
}
