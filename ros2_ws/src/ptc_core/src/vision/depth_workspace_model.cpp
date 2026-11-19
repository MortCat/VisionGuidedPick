#include "ptc_core/vision/depth_workspace_model.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

namespace ptc_core
{
std::vector<ForegroundObjectObservation> detect_workspace_objects(
  const RgbdImageFrame & frame, const DepthWorkspaceConfig & cfg)
{
  const auto & camera=frame.camera;
  const auto pixels=static_cast<std::size_t>(camera.width)*camera.height;
  if (!camera.width || !camera.height || camera.fx<=0 || camera.fy<=0 ||
    frame.depth_m.size()!=pixels || !is_valid_pose(frame.metadata.camera_in_world) ||
    !std::isfinite(cfg.cell_size_m) || cfg.cell_size_m<=0 || !cfg.min_points ||
    cfg.min_height_m<=0 || cfg.max_height_m<=cfg.min_height_m ||
    cfg.min_span_m<=0 || cfg.max_span_m<=cfg.min_span_m || !cfg.max_objects ||
    cfg.bounds_min_m.x_m>=cfg.bounds_max_m.x_m || cfg.bounds_min_m.y_m>=cfg.bounds_max_m.y_m) {
    throw std::invalid_argument("invalid depth foreground frame/workspace");
  }
  // Input is the calibrated rectified RGB image after depth registration.
  for (const auto value:camera.distortion) {
    if (!std::isfinite(value) || std::abs(value)>1e-12) {
      throw std::invalid_argument("depth foreground requires rectified intrinsics");
    }
  }
  struct PixelObservation {std::size_t pixel; Vector3d world;};
  using Cell=std::pair<int,int>;
  std::map<Cell,std::vector<PixelObservation>> cells;
  for (std::uint32_t v=0;v<camera.height;++v) {
    for (std::uint32_t u=0;u<camera.width;++u) {
      const auto pixel=static_cast<std::size_t>(v)*camera.width+u;
      const double z=frame.depth_m[pixel]; if (!std::isfinite(z) || z<=0) {continue;}
      const auto world=transform_point(frame.metadata.camera_in_world,
        {(u-camera.cx)*z/camera.fx,(v-camera.cy)*z/camera.fy,z});
      const double height=world.z_m-cfg.support_plane_z_m;
      if (!std::isfinite(world.x_m) || !std::isfinite(world.y_m) || !std::isfinite(height) ||
        world.x_m<cfg.bounds_min_m.x_m || world.x_m>cfg.bounds_max_m.x_m ||
        world.y_m<cfg.bounds_min_m.y_m || world.y_m>cfg.bounds_max_m.y_m ||
        world.z_m<cfg.bounds_min_m.z_m || world.z_m>cfg.bounds_max_m.z_m ||
        height<cfg.min_height_m || height>cfg.max_height_m) {continue;}
      const Cell cell{static_cast<int>(std::floor((world.x_m-cfg.bounds_min_m.x_m)/cfg.cell_size_m)),
        static_cast<int>(std::floor((world.y_m-cfg.bounds_min_m.y_m)/cfg.cell_size_m))};
      cells[cell].push_back({pixel,world});
    }
  }
  std::set<Cell> visited;
  std::vector<ForegroundObjectObservation> result;
  for (const auto & entry:cells) {
    if (visited.count(entry.first)) {continue;}
    std::queue<Cell> pending; pending.push(entry.first); visited.insert(entry.first);
    std::vector<PixelObservation> component;
    while (!pending.empty()) {
      const auto cell=pending.front();pending.pop();
      const auto & observations=cells.at(cell);component.insert(component.end(),observations.begin(),observations.end());
      for (int dx=-1;dx<=1;++dx) {for (int dy=-1;dy<=1;++dy) {
        const Cell neighbor{cell.first+dx,cell.second+dy};
        if (cells.count(neighbor) && visited.insert(neighbor).second) {pending.push(neighbor);}
      }}
    }
    if (component.size()<cfg.min_points) {continue;}
    ForegroundObjectObservation object;object.mask.resize(pixels);
    object.bounds_min_world=object.bounds_max_world=component.front().world;
    std::uint32_t min_u=camera.width,min_v=camera.height,max_u=0,max_v=0;
    for (const auto & p:component) {
      object.mask[p.pixel]=1;const auto u=static_cast<std::uint32_t>(p.pixel%camera.width),v=static_cast<std::uint32_t>(p.pixel/camera.width);
      min_u=std::min(min_u,u);min_v=std::min(min_v,v);max_u=std::max(max_u,u);max_v=std::max(max_v,v);
      object.center_world.x_m+=p.world.x_m;object.center_world.y_m+=p.world.y_m;object.center_world.z_m+=p.world.z_m;
      object.bounds_min_world={std::min(object.bounds_min_world.x_m,p.world.x_m),std::min(object.bounds_min_world.y_m,p.world.y_m),std::min(object.bounds_min_world.z_m,p.world.z_m)};
      object.bounds_max_world={std::max(object.bounds_max_world.x_m,p.world.x_m),std::max(object.bounds_max_world.y_m,p.world.y_m),std::max(object.bounds_max_world.z_m,p.world.z_m)};
    }
    const double span=std::hypot(object.bounds_max_world.x_m-object.bounds_min_world.x_m,object.bounds_max_world.y_m-object.bounds_min_world.y_m);
    if (span<cfg.min_span_m || span>cfg.max_span_m) {continue;}
    const double count=static_cast<double>(component.size());object.center_world={object.center_world.x_m/count,object.center_world.y_m/count,object.center_world.z_m/count};
    object.measured_points=component.size();object.region={min_u,min_v,max_u-min_u+1,max_v-min_v+1};result.push_back(std::move(object));
  }
  std::sort(result.begin(),result.end(),[](const auto & a,const auto & b){return a.measured_points>b.measured_points;});
  if (result.size()>cfg.max_objects) {result.resize(cfg.max_objects);}
  return result;
}
}  // namespace ptc_core
