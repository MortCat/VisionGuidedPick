// Thin JSON boundary for the inference worker; geometry/identity stays in pure C++.
#include <iostream>
#include <sstream>
#include <json/json.h>
#include "ptc_core/vision/observe_track_model.hpp"

int main(int argc, char ** argv) {
  std::optional<ptc_core::ObserveBeltPrior> belt;
  if (argc > 3) belt = ptc_core::ObserveBeltPrior{std::stod(argv[2]), std::stod(argv[3])};
  ptc_core::ObserveTrackModel tracker(argc > 1 ? std::stod(argv[1]) : 3.5, belt);
  std::string line, epoch;
  while (std::getline(std::cin, line)) {
    try {
      Json::Value input;
      Json::CharReaderBuilder reader;
      std::string errors;
      std::istringstream stream(line);
      if (!Json::parseFromStream(reader, stream, &input, &errors)) throw std::runtime_error(errors);
      if (input["epoch"].asString() != epoch) {tracker.reset(); epoch = input["epoch"].asString();}
      std::vector<ptc_core::ObserveDetection> observations;
      for (const auto & row : input["objects"]) {
        ptc_core::ObserveDetection item;
        item.label = row["label"].asString();
        for (int k = 0; k < 4; ++k) item.box[k] = row["bbox_xyxy"][k].asDouble();
        for (const auto & point : row["points_world"]) item.points_world.push_back({point[0].asDouble(), point[1].asDouble(), point[2].asDouble()});
        observations.push_back(std::move(item));
      }
      auto tracks = tracker.update(observations, input["stamp_sec"].asDouble());
      Json::Value output(Json::arrayValue);
      for (const auto & track : tracks) {
        Json::Value row;
        row["track_id"] = std::to_string(track.id);
        row["phase"] = track.confirmations > 1 ? "tracking" : "candidate";
        row["position_world_m"] = Json::nullValue;
        row["velocity_world_mps"] = Json::nullValue;
        row["observed_bounds_world"] = Json::nullValue;
        if (track.geometry) {
          for (int k = 0; k < 3; ++k) {
            row["position_world_m"].append(track.geometry->center[k]);
            row["observed_bounds_world"]["minimum_m"].append(track.geometry->minimum[k]);
            row["observed_bounds_world"]["maximum_m"].append(track.geometry->maximum[k]);
          }
          row["observed_bounds_world"]["orientation"] = "world_axis_aligned_visible_surface";
        }
        if (track.velocity) for (const auto v : *track.velocity) row["velocity_world_mps"].append(v);
        output.append(row);
      }
      Json::StreamWriterBuilder writer; writer["indentation"] = "";
      std::cout << Json::writeString(writer, output) << std::endl;
    } catch (const std::exception & error) {
      std::cerr << error.what() << std::endl;
      return 1;
    }
  }
}
