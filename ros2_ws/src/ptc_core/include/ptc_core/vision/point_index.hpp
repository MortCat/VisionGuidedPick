#pragma once
#include "ptc_core/pose3d.hpp"
#include <vector>
#include <limits>
namespace ptc_core
{
struct NearestPointResult {std::size_t index{0}; double squared_distance{std::numeric_limits<double>::infinity()};};
class PointIndex
{
public:
  explicit PointIndex(std::vector<Vector3d> points);
  NearestPointResult nearest(const Vector3d &) const;
  const std::vector<Vector3d> & points() const {return points_;}
private:
  struct Branch {std::size_t point{0}; int left{-1},right{-1},axis{0};};
  int build(std::vector<std::size_t> &,std::size_t begin,std::size_t end,int axis);
  void search(int branch,const Vector3d &,NearestPointResult &) const;
  std::vector<Vector3d> points_; std::vector<Branch> branches_; int root_{-1};
};
}  // namespace ptc_core
