#include "state_test_support.hpp"
#include "ptc_core/vision/bin_height_model.hpp"
using namespace ptc_core;
using namespace ptc_test;

void tests()
{
  const BinHeightConfig bin{{0,0,0},{.32,.32,.65},.04};
  std::vector<Vector3d> floor;
  for(unsigned y=0;y<16;++y)for(unsigned x=0;x<16;++x){
    for(unsigned n=0;n<4;++n)floor.push_back({.01+.02*x,.01+.02*y,.12});
  }
  auto measured=estimate_bin_height(floor,bin,{});
  CHECK(measured && std::abs(measured->surface_z_m-.12)<1e-8);
  auto pile=floor;
  for(auto & p:pile)if(p.x_m>.16)p.z_m=.38;
  measured=estimate_bin_height(pile,bin,{});
  CHECK(measured && std::abs(measured->surface_z_m-.38)<1e-8);
  // Isolated high pixels and points on the excluded held object cannot raise the pile.
  pile.push_back({.01,.01,.6});
  for(unsigned n=0;n<12;++n)pile.push_back({.031,.031,.6});
  measured=estimate_bin_height(pile,bin,{{Vector3d{.02,.02,.5},Vector3d{.05,.05,.65}}});
  CHECK(measured && std::abs(measured->surface_z_m-.38)<1e-8);
  CHECK(!estimate_bin_height({{.1,.1,.2},{.1,.1,.2},{.1,.1,.2}},bin,{}));
  CHECK(!estimate_bin_height({},bin,{}));
  auto narrow_pile=floor;
  for(auto & p:narrow_pile)if(p.y_m<.02 && p.x_m<.04)p.z_m=.38;
  measured=estimate_bin_height(narrow_pile,bin,{});
  CHECK(measured && std::abs(measured->surface_z_m-.38)<1e-8);
  for(unsigned n=0;n<20;++n)narrow_pile.push_back({.15,.15,.6});
  measured=estimate_bin_height(narrow_pile,bin,{});
  CHECK(measured && std::abs(measured->surface_z_m-.38)<1e-8);
  auto data=settings_data();data.grasp.bin_height=bin;
  const auto cfg=settings(data);
  GraspState held;held.bin_height_offset_m=.15;
  const auto target=bin_target_pose(data.grasp.bin_release_pose,held,cfg);
  CHECK(std::abs(target.position.z_m-data.grasp.bin_release_pose.position.z_m-.15)<1e-8);
  CHECK(pose_near(bin_target_pose(data.grasp.bin_release_pose,held,settings()),
    data.grasp.bin_release_pose,1e-8,1e-8));
}
int main(){return run(tests);}
