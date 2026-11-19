#include "ptc_app/scene_loader.hpp"
#include "ptc_app/settings_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>
#include <set>
#include <cmath>
namespace ptc_app
{
std::vector<ptc_core::StaticGeometry> load_static_geometry(const ptc_core::SystemSettings & settings)
{
  const auto root=YAML::LoadFile(settings.scene.static_geometry_path); const auto objects=root["objects"];
  if (!objects.IsSequence() || objects.size()==0 || objects.size()>256) {throw std::runtime_error("static scene requires 1..256 fixed collision objects");}
  std::vector<ptc_core::StaticGeometry> out; std::set<std::string> ids;
  for (const auto & item : objects) {
    ptc_core::StaticGeometry geometry; geometry.id=item["id"].as<std::string>(); geometry.reference_frame=settings.frames.world;
    if (geometry.id.empty() || !ids.insert(geometry.id).second) {throw std::runtime_error("static object id must be unique");}
    const auto kind=item["kind"].as<std::string>();
    if (kind!="box" && kind!="sphere" && kind!="mesh") {throw std::runtime_error("invalid collision geometry kind");}
    geometry.kind=kind=="box" ? ptc_core::GeometryKind::kBox : kind=="sphere" ? ptc_core::GeometryKind::kSphere : ptc_core::GeometryKind::kMesh;
    const auto p=item["position_m"].as<std::vector<double>>(); const auto q=item["orientation_xyzw"].as<std::vector<double>>();
    const auto dims=item["dimensions_m"].as<std::vector<double>>();
    if (p.size()!=3 || q.size()!=4 || dims.size()!=3 || dims[0]<=0 || dims[1]<=0 || dims[2]<=0) {throw std::runtime_error("invalid collision dimensions/pose");}
    geometry.pose={{p[0],p[1],p[2]},{q[0],q[1],q[2],q[3]}}; geometry.dimensions_m={dims[0],dims[1],dims[2]};
    if (!ptc_core::is_valid_pose(geometry.pose) || !std::isfinite(ptc_core::vector_norm(geometry.dimensions_m))) {throw std::runtime_error("non-finite scene geometry");}
    geometry.asset_digest=sha256_file(settings.scene.static_geometry_path);
    if (geometry.kind==ptc_core::GeometryKind::kMesh) {
      const auto path=std::filesystem::absolute(std::filesystem::path(settings.scene.static_geometry_path).parent_path()/item["mesh_path"].as<std::string>()).lexically_normal();
      geometry.mesh_path=path.u8string(); geometry.asset_digest=sha256_file(path);
      bool declared=false; const auto identity=YAML::LoadFile(settings.scene.static_geometry_path+".identity.yaml");
      if (identity["dependencies"]) {for (const auto & dependency : identity["dependencies"]) {
        const auto file=std::filesystem::absolute(std::filesystem::path(settings.scene.static_geometry_path).parent_path()/dependency["path"].as<std::string>()).lexically_normal();
        if (file==path && dependency["sha256"].as<std::string>()==geometry.asset_digest) {declared=true; break;}
      }}
      if (!declared) {throw std::runtime_error("scene mesh is not an identified asset dependency");}
    }
    out.push_back(std::move(geometry));
  }
  return out;
}
}  // namespace ptc_app
