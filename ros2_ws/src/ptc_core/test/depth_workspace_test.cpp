#include "test_support.hpp"
#include "ptc_core/vision/depth_workspace_model.hpp"
using namespace ptc_core;
void tests()
{
  RgbdImageFrame frame;frame.camera={24,24,100,100,12,12,{}};
  frame.depth_m.assign(24*24,.9f);
  for(const auto first:{4,16}) {for(int v=10;v<14;++v) {for(int u=first;u<first+4;++u) {
    frame.depth_m[v*24+u]=1.f;
  }}}
  DepthWorkspaceConfig config;config.bounds_min_m={-.15,-.15,.9};config.bounds_max_m={.15,.15,1.04};
  config.support_plane_z_m=.9;config.min_height_m=.01;config.max_height_m=.13;
  config.cell_size_m=.015;config.min_span_m=.02;config.max_span_m=.1;config.min_points=8;config.max_objects=8;
  const auto objects=detect_workspace_objects(frame,config);
  CHECK(objects.size()==2);
  for(const auto & object:objects) {CHECK(object.measured_points==16);CHECK(object.region.width==4);CHECK(object.region.height==4);}
  frame.metadata.camera_in_world.position.x_m=1;
  CHECK(detect_workspace_objects(frame,config).empty());
  frame.metadata.camera_in_world={};frame.camera.distortion[0]=.1;
  bool rejected=false;try {detect_workspace_objects(frame,config);}catch(const std::invalid_argument &){rejected=true;}
  CHECK(rejected);
}
int main(){return ptc_test::run(tests);}
