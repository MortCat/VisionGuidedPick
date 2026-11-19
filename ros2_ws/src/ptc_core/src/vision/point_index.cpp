#include "ptc_core/vision/point_index.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
namespace ptc_core
{
namespace {double axis(const Vector3d & p,int a) {return a==0 ? p.x_m : a==1 ? p.y_m : p.z_m;}}
PointIndex::PointIndex(std::vector<Vector3d> points) : points_(std::move(points))
{
  if (points_.empty() || points_.size()>1000000) {throw std::invalid_argument("point index requires 1..1000000 points");}
  for (const auto & p : points_) {if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) || !std::isfinite(p.z_m)) {throw std::invalid_argument("nonfinite reference point");}}
  std::vector<std::size_t> indices(points_.size()); std::iota(indices.begin(),indices.end(),0); branches_.reserve(points_.size());
  root_=build(indices,0,indices.size(),0);
}
int PointIndex::build(std::vector<std::size_t> & indices,std::size_t begin,std::size_t end,int a)
{
  if (begin==end) {return -1;} const auto middle=begin+(end-begin)/2;
  std::nth_element(indices.begin()+begin,indices.begin()+middle,indices.begin()+end,[&](std::size_t x,std::size_t y) {return axis(points_[x],a)<axis(points_[y],a);});
  const auto index=static_cast<int>(branches_.size()); branches_.push_back({indices[middle],-1,-1,a});
  const int left=build(indices,begin,middle,(a+1)%3),right=build(indices,middle+1,end,(a+1)%3);
  branches_[index].left=left; branches_[index].right=right; return index;
}
void PointIndex::search(int branch,const Vector3d & query,NearestPointResult & best) const
{
  if (branch<0) {return;} const auto & b=branches_[branch]; const auto & p=points_[b.point];
  const double x=query.x_m-p.x_m,y=query.y_m-p.y_m,z=query.z_m-p.z_m,d=x*x+y*y+z*z;
  if (d<best.squared_distance) {best={b.point,d};}
  const double delta=axis(query,b.axis)-axis(p,b.axis);
  search(delta<0 ? b.left : b.right,query,best);
  if (delta*delta<best.squared_distance) {search(delta<0 ? b.right : b.left,query,best);}
}
NearestPointResult PointIndex::nearest(const Vector3d & query) const
{NearestPointResult best; if (std::isfinite(query.x_m) && std::isfinite(query.y_m) && std::isfinite(query.z_m)) {search(root_,query,best);} return best;}
}  // namespace ptc_core
