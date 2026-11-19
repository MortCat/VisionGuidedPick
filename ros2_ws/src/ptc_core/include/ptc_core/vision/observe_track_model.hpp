#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace ptc_core {
using ObservePoint = std::array<double, 3>;
using ObserveBox = std::array<double, 4>;
struct ObserveDetection {
  std::string label;
  ObserveBox box{};
  std::vector<ObservePoint> points_world;
};
struct ObserveGeometry {
  ObservePoint center{}, minimum{}, maximum{};
};
inline std::optional<ObserveGeometry> estimate_observed_geometry(const std::vector<ObservePoint> & points) {
  if (points.empty()) return std::nullopt;
  ObserveGeometry result;
  result.minimum.fill(std::numeric_limits<double>::infinity());
  result.maximum.fill(-std::numeric_limits<double>::infinity());
  std::size_t count = 0;
  for (const auto & point : points) {
    if (!std::all_of(point.begin(), point.end(), [](double v) {return std::isfinite(v);})) continue;
    for (std::size_t k = 0; k < 3; ++k) {
      result.center[k] += point[k];
      result.minimum[k] = std::min(result.minimum[k], point[k]);
      result.maximum[k] = std::max(result.maximum[k], point[k]);
    }
    ++count;
  }
  if (!count) return std::nullopt;
  for (auto & v : result.center) v /= static_cast<double>(count);
  return result;
}
struct ObserveBeltPrior {
  double speed_y_mps;
  double rejoin_radius_m;
};
struct ObserveDepthSample {
  ObservePoint center;
  double stamp;
};
struct ObserveTrackRecord {
  std::uint64_t id{0};
  ObserveDetection detection;
  std::optional<ObserveGeometry> geometry;
  std::optional<ObservePoint> velocity;
  std::optional<ObserveDepthSample> last_depth;
  std::array<double, 2> pixel_velocity{};
  double stamp{0};
  std::size_t confirmations{0};
};
class ObserveTrackModel {
 public:
  explicit ObserveTrackModel(double retention_sec, std::optional<ObserveBeltPrior> belt = std::nullopt)
  : retention_sec_(retention_sec), belt_(belt) {}
  void reset() {records_.clear();}
  std::vector<ObserveTrackRecord> update(const std::vector<ObserveDetection> & observations, double stamp) {
    records_.erase(std::remove_if(records_.begin(), records_.end(), [&](const auto & r) {
      return stamp < r.stamp || stamp - r.stamp > retention_sec_;
    }), records_.end());
    std::vector<std::optional<ObserveGeometry>> geometry;
    geometry.reserve(observations.size());
    for (const auto & next : observations) geometry.push_back(estimate_observed_geometry(next.points_world));
    enum class Match {kRecentImage, kBeltRejoin, kOlderImage};
    struct Pair {Match match; double cost; std::size_t record, observation;};
    std::vector<Pair> pairs;
    std::vector<std::size_t> belt_matches_per_track(records_.size());
    std::vector<std::size_t> belt_matches_per_detection(observations.size());
    for (std::size_t r = 0; r < records_.size(); ++r) {
      const auto & previous = records_[r];
      const auto dt = stamp - previous.stamp;
      for (std::size_t o = 0; o < observations.size(); ++o) {
        const auto & next = observations[o];
        if (previous.detection.label != next.label) continue;
        const bool after_gap = belt_ && dt > .25;
        if (after_gap && previous.last_depth && geometry[o]) {
          const auto & depth = *previous.last_depth;
          const auto depth_gap = stamp - depth.stamp;
          if (depth_gap <= retention_sec_) {
            const auto & p = depth.center;
            const auto & q = geometry[o]->center;
            const double distance = std::hypot(q[0]-p[0], q[1]-p[1]-belt_->speed_y_mps*depth_gap);
            if (distance <= belt_->rejoin_radius_m) {
              pairs.push_back({Match::kBeltRejoin, distance, r, o});
              ++belt_matches_per_track[r];
              ++belt_matches_per_detection[o];
            }
            continue;
          }
        }
        auto box = previous.detection.box;
        for (std::size_t k = 0; k < 4; ++k) box[k] += previous.pixel_velocity[k % 2] * dt;
        const auto width = std::max(1.0, box[2] - box[0]);
        const auto height = std::max(1.0, box[3] - box[1]);
        const auto dx = ((next.box[0]+next.box[2])-(box[0]+box[2])) / (2*width);
        const auto dy = ((next.box[1]+next.box[3])-(box[1]+box[3])) / (2*height);
        const auto distance = std::hypot(dx, dy);
        if (distance <= 2.0)
          pairs.push_back({after_gap ? Match::kOlderImage : Match::kRecentImage, distance, r, o});
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](auto a, auto b) {
      return std::tie(a.match, a.cost) < std::tie(b.match, b.cost);
    });
    std::vector<int> assignment(observations.size(), -1);
    std::vector<bool> used(records_.size(), false);
    for (const auto & pair : pairs) {
      if (pair.match == Match::kBeltRejoin &&
          (belt_matches_per_track[pair.record] != 1 || belt_matches_per_detection[pair.observation] != 1))
        continue;
      if (!used[pair.record] && assignment[pair.observation] < 0) {
        used[pair.record] = true;
        assignment[pair.observation] = static_cast<int>(pair.record);
      }
    }
    std::vector<ObserveTrackRecord> result;
    for (std::size_t i = 0; i < observations.size(); ++i) {
      ObserveTrackRecord next;
      next.detection = observations[i];
      next.geometry = geometry[i];
      if (next.geometry) next.last_depth = ObserveDepthSample{next.geometry->center, stamp};
      next.stamp = stamp;
      next.confirmations = 1;
      if (assignment[i] >= 0) {
        const auto & previous = records_[static_cast<std::size_t>(assignment[i])];
        next.id = previous.id;
        next.confirmations = previous.confirmations + 1;
        if (!next.last_depth) next.last_depth = previous.last_depth;
        const auto dt = stamp - previous.stamp;
        if (dt > 0) {
          for (std::size_t k = 0; k < 2; ++k)
            next.pixel_velocity[k] = ((next.detection.box[k]+next.detection.box[k+2]) -
              (previous.detection.box[k]+previous.detection.box[k+2])) / (2*dt);
          if (next.geometry && previous.geometry) {
            ObservePoint velocity;
            for (std::size_t k = 0; k < 3; ++k)
              velocity[k] = (next.geometry->center[k] - previous.geometry->center[k]) / dt;
            next.velocity = velocity;
          }
        }
        records_[static_cast<std::size_t>(assignment[i])] = next;
      } else {
        next.id = next_id_++;
      }
      result.push_back(next);
    }
    for (std::size_t i = 0; i < assignment.size(); ++i)
      if (assignment[i] < 0) records_.push_back(result[i]);
    // Finite history even under repeated spurious detections.
    if (records_.size() > 256) records_.erase(records_.begin(), records_.end() - 256);
    return result;
  }
 private:
  double retention_sec_;
  std::optional<ObserveBeltPrior> belt_;
  std::uint64_t next_id_{1};
  std::vector<ObserveTrackRecord> records_;
};
}  // namespace ptc_core
