#include "ptc_core/vision/grasp_patch_model.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace ptc_core;
void require(bool b){if(!b)throw std::runtime_error("grasp patch regression");}
int main(){
 RgbdImageFrame f;f.camera={5,5,10,10,2,2,{}};f.depth_m.assign(25,1.f);
 // Camera is at world(1,2,3), TCP at world(1,2,4): centre depth is TCP origin.
 f.metadata.camera_in_world.position={1,2,3};f.metadata.tcp_in_world.position={1,2,4};
 std::array<Vector3d,2> region{{{-.3,-.3,-.1},{.3,.3,.1}}};
 auto p=extract_grasp_patch_tcp(f,region,{},3);require(p.candidate_pixels==25&&p.points_tcp.size()==3&&p.roi_clipped);
 GraspMaskBox box;box.half_extent={.051,.051,.02};
 p=extract_grasp_patch_tcp(f,region,{box},50);require(p.self_pixels==1&&p.candidate_pixels==24);
 // Rotate TCP 180deg about Z. Camera-right maps to TCP-left.
 f.metadata.tcp_in_world.orientation={0,0,1,0};f.depth_m.assign(25,NAN);f.depth_m[13]=1.;
 p=extract_grasp_patch_tcp(f,region,{},3);require(p.points_tcp.size()==1&&std::abs(p.points_tcp[0].x_m+.1)<1e-6);
 f.depth_m[13]=NAN;p=extract_grasp_patch_tcp(f,region,{},3);require(p.points_tcp.empty()&&p.valid_depth_pixels==0);
 f.metadata.camera_in_world.position={1,2,5};p=extract_grasp_patch_tcp(f,region,{},3);require(p.points_tcp.empty()&&p.roi.width==0);
 // Held object below the model pad gap (archived cycle ...172947): only the measured seed admits it.
 f.metadata.camera_in_world.position={1,2,3};f.metadata.tcp_in_world.orientation={0,0,0,1};f.depth_m.assign(25,NAN);
 std::vector<std::uint8_t> target(25,0);for(unsigned y=1;y<4;++y)for(unsigned x=1;x<4;++x){f.depth_m[y*5+x]=1.06f;target[y*5+x]=1;}
 const std::array<Vector3d,2> pad{{{-.05,-.05,-.02},{.05,.05,.02}}};
 p=extract_grasp_patch_tcp(f,pad,{},50);require(p.candidate_pixels==0);
 std::string why;const auto seed=grasp_target_seed_tcp(f,target,{},5,why);
 require(seed&&seed->points==9&&std::abs(seed->bounds_tcp[1].z_m-.06)<1e-6&&std::abs(seed->bounds_tcp[1].x_m-.106)<1e-6);
 p=extract_grasp_patch_tcp(f,grasp_observation_region(pad,*seed,.008),{},50);require(p.candidate_pixels==9);
 // Fingers merged into the foreground component must not stretch the seed toward the hand.
 GraspMaskBox finger;finger.box_in_tcp.position={0,0,.06};finger.half_extent={.01,.01,.01};
 const auto masked=grasp_target_seed_tcp(f,target,{finger},5,why);require(masked&&masked->points==8);
 require(!grasp_target_seed_tcp(f,target,{},10,why)&&!why.empty());
 // Missed grasp (archived cycle ...570850): belt between closed fingers is support, not candidate.
 // Camera and TCP look down (180deg about X); world height = 5 - depth, belt at 3.95, object 3 cm above.
 f.metadata.camera_in_world={{1,2,5},{1,0,0,0}};f.metadata.tcp_in_world={{1,2,4},{1,0,0,0}};
 f.depth_m.assign(25,1.05f);f.depth_m[12]=1.02f;
 DepthWorkspaceConfig belt;belt.support_plane_z_m=3.95;belt.min_points=9;belt.bounds_min_m={0,1,3};belt.bounds_max_m={2,3,5};
 const std::array<Vector3d,2> gap{{{-.3,-.3,-.02},{.3,.3,.06}}};
 p=extract_grasp_patch_tcp(f,gap,{},50);require(p.candidate_pixels==25);
 const auto plane=measure_support_plane(f,belt);require(plane&&plane->pixels==24&&std::abs(plane->z_m-3.95)<1e-6);
 p=extract_grasp_patch_tcp(f,gap,{},50,plane->z_m+belt.min_height_m);require(p.support_pixels==24&&p.candidate_pixels==1);
 belt.support_plane_z_m=4.5;require(!measure_support_plane(f,belt));
 // Lift possession. Seed at Closing: object top at world 3.98 under the TCP, belt 3.95, both looking down.
 belt.support_plane_z_m=3.95;belt.min_height_m=.008;
 f.depth_m.assign(25,1.05f);for(unsigned y=1;y<4;++y)for(unsigned x=1;x<4;++x)f.depth_m[y*5+x]=1.02f;f.metadata.color_timestamp=Nanoseconds{100};
 const auto closing=grasp_target_seed_tcp(f,target,{},5,why);require(closing&&std::abs(closing->median_world_z_m-3.98)<1e-6);
 GripperState contact;contact.timestamp=Nanoseconds{150};contact.contact_confirmed=true;contact.width_m=.058;
 GripperState no_contact=contact;no_contact.contact_confirmed=false;no_contact.width_m=0;
 auto lifted=[&](float target_depth){auto g=f;g.metadata.camera_in_world.position.z_m=5.04;g.metadata.tcp_in_world.position.z_m=4.04;
  g.depth_m.assign(25,1.09f);for(unsigned y=1;y<4;++y)for(unsigned x=1;x<4;++x)g.depth_m[y*5+x]=target_depth;return g;};
 auto possession=[&](const RgbdImageFrame & g,const std::optional<GripperState> & grip){const auto plane=measure_support_plane(g,belt);
  return evaluate_lift_possession(g,*closing,{},plane->z_m+belt.min_height_m,grip,.008,5);};
 // Held: the surface rose with the TCP (same depth from the wrist camera).
 auto r=possession(lifted(1.02f),contact);require(r.state==Possession::kHolding&&r.surface_points==9&&std::abs(r.surface_rise_m-.04)<1e-6);
 // Empty: the surface stayed at its Closing height while the TCP rose 4 cm.
 r=possession(lifted(1.06f),no_contact);require(r.state==Possession::kEmpty&&std::abs(r.surface_rise_m)<1e-6);
 // Occluded (near object in front of the wrist camera) or no depth: Unknown, never Empty, whatever the gripper says.
 r=possession(lifted(.5f),no_contact);require(r.state==Possession::kUnknown&&!r.surface_points&&std::isnan(r.surface_rise_m));
 r=possession(lifted(NAN),contact);require(r.state==Possession::kUnknown&&!r.surface_points);
 // Many points but no lift yet: not separable. Gripper contact alone is not Holding.
 auto still=lifted(1.02f);still.metadata.camera_in_world.position.z_m=5;still.metadata.tcp_in_world.position.z_m=4;
 r=possession(still,contact);require(r.state==Possession::kUnknown&&!r.surface_points);
 // Gripper and depth disagree, or the only gripper sample predates the seed: Unknown.
 r=possession(lifted(1.06f),contact);require(r.state==Possession::kUnknown&&r.surface_points==9);
 r=possession(lifted(1.02f),no_contact);require(r.state==Possession::kUnknown);
 contact.timestamp=Nanoseconds{100};r=possession(lifted(1.02f),contact);require(r.state==Possession::kUnknown);
 r=possession(lifted(1.02f),std::nullopt);require(r.state==Possession::kUnknown&&r.surface_points==9);
 std::cout<<"grasp patch transforms/mask/bounds/invalid-depth/seed/support/lift-possession pass\n";
}
