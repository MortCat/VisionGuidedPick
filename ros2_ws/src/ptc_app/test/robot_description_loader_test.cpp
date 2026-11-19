#include "ptc_app/robot_description_loader.hpp"
#include "ptc_app/settings_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

int main(int argc,char ** argv)
{
  if (argc!=2) {return 2;}
  const std::filesystem::path config_path=argv[1];
  auto config=YAML::LoadFile(config_path.string());
  // This test covers URDF/mesh identity, not the user's runtime journal path.
  // Keep the temporary settings beside the source config so its relative robot
  // assets resolve exactly as they do in production.
  const auto suffix=std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_config=config_path.parent_path()/(config_path.stem().string()+".robot-description-test-"+
    std::to_string(suffix)+config_path.extension().string());
  struct RemoveTemporaryConfig
  {std::filesystem::path path;~RemoveTemporaryConfig(){std::error_code error;std::filesystem::remove(path,error);}} cleanup{test_config};
  {std::ofstream output(test_config);if(!output){std::cerr<<"cannot write temporary settings for loader test\n";return 1;}
    output<<config;}
  const auto loaded=ptc_app::SettingsLoader{}.load(test_config);
  if (!loaded.snapshot) {
    for (const auto & error : loaded.errors) {std::cerr << error.path << ": " << error.message << '\n';}
    return 1;
  }
  const auto description=ptc_app::load_robot_description(loaded.snapshot->get());
  if (description.collision_geometry_digest.rfind("sha256:",0)!=0 || description.collision_geometry_digest.size()!=71) {
    std::cerr << "verified robot collision geometry digest was not produced\n";
    return 1;
  }
  std::cout << description.collision_geometry_digest << '\n';
  return 0;
}
