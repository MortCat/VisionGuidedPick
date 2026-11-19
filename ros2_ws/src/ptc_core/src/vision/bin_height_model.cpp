#include "ptc_core/vision/bin_height_model.hpp"
#include "ptc_core/pose_math.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ptc_core
{
std::optional<BinHeightEstimate> estimate_bin_height(const std::vector<Vector3d> & points,
  const BinHeightConfig & config,const std::vector<std::array<Vector3d,2>> & exclusions)
{
  const auto & low=config.bounds_min_m;const auto & high=config.bounds_max_m;
  if(!std::isfinite(vector_norm(low)) || !std::isfinite(vector_norm(high)) ||
    low.x_m>=high.x_m || low.y_m>=high.y_m || low.z_m>=high.z_m)return std::nullopt;
  const auto inside=[](const Vector3d & p,const Vector3d & a,const Vector3d & b){
    return p.x_m>=a.x_m && p.x_m<=b.x_m && p.y_m>=a.y_m && p.y_m<=b.y_m &&
      p.z_m>=a.z_m && p.z_m<=b.z_m;
  };
  std::array<std::vector<double>,256> grid;
  BinHeightEstimate result;
  for(const auto & p:points){
    if(!std::isfinite(vector_norm(p)) || !inside(p,low,high) ||
      std::any_of(exclusions.begin(),exclusions.end(),[&](const auto & b){return inside(p,b[0],b[1]);}))continue;
    const auto x=std::min<std::size_t>(15,16*(p.x_m-low.x_m)/(high.x_m-low.x_m));
    const auto y=std::min<std::size_t>(15,16*(p.y_m-low.y_m)/(high.y_m-low.y_m));
    grid[y*16+x].push_back(p.z_m);++result.points;
  }
  std::array<double,256> heights;
  heights.fill(std::numeric_limits<double>::quiet_NaN());
  for(std::size_t index=0;index<grid.size();++index){
    auto & cell=grid[index];
    if(cell.size()<3)continue;
    const auto middle=cell.begin()+cell.size()/2;
    std::nth_element(cell.begin(),middle,cell.end());
    heights[index]=*middle;
    ++result.cells;
  }
  if(result.cells<6)return std::nullopt;
  std::optional<double> supported_top;
  for(int y=0;y<16;++y)for(int x=0;x<16;++x){
    const double height=heights[y*16+x];
    if(!std::isfinite(height))continue;
    bool supported=false;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
      const int nx=x+dx,ny=y+dy;
      if((dx==0 && dy==0) || nx<0 || nx>=16 || ny<0 || ny>=16)continue;
      const double neighbor=heights[ny*16+nx];
      supported=supported || (std::isfinite(neighbor) && std::abs(height-neighbor)<=.04);
    }
    if(supported)supported_top=std::max(supported_top.value_or(height),height);
  }
  if(!supported_top)return std::nullopt;
  result.surface_z_m=*supported_top;
  return result;
}
}
