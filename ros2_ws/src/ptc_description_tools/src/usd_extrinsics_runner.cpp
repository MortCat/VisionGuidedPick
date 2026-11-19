#include "ptc_description_tools/usd_extrinsics_runner.hpp"
#include "ptc_app/settings_loader.hpp"
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/rotation.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace ptc_description_tools
{
namespace
{
using namespace pxr;
void rigid(const GfMatrix4d & m)
{
  for (int i=0;i<4;++i) {for (int j=0;j<4;++j) {
    if (!std::isfinite(m[i][j])) {throw std::runtime_error("non-finite USD transform");}
  }}
  if (std::abs(m.GetDeterminant()-1)>1e-6 || std::abs(m[3][3]-1)>1e-6) {
    throw std::runtime_error("scale/reflection/nonrigid transform: normalize robot asset units before export");
  }
  for (int i=0;i<3;++i) {for (int j=0;j<3;++j) {
    double dot=0; for (int k=0;k<3;++k) {dot+=m[i][k]*m[j][k];}
    if (std::abs(dot-(i==j ? 1.0 : 0.0))>1e-6 || std::abs(m[i][3])>1e-6) {throw std::runtime_error("USD transform is not rigid");}
  }}
}
void fixed_branch(UsdPrim prim, const UsdPrim & mount, UsdGeomXformCache & cache)
{
  if (!prim.GetPath().HasPrefix(mount.GetPath())) {throw std::runtime_error("TCP/camera must be under declared fixed mount");}
  while (prim && prim!=mount) {
    if (cache.TransformMightBeTimeVarying(prim) || cache.GetResetXformStack(prim)) {
      throw std::runtime_error("mount branch has animated/reset transform; not a known fixed extrinsic");
    }
    const auto type=prim.GetTypeName().GetString();
    if (type.find("Joint")!=std::string::npos && type!="PhysicsFixedJoint") {throw std::runtime_error("movable joint in camera/TCP branch");}
    prim=prim.GetParent();
  }
}
void save(const std::filesystem::path & path, const YAML::Node & document)
{
  if (std::filesystem::exists(path)) {throw std::runtime_error("output already exists; choose a new versioned asset path");}
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter<<document;
  std::ofstream output(path,std::ios::binary); output<<emitter.c_str()<<'\n'; output.close();
  if (!output) {throw std::runtime_error("cannot save exported asset");}
}
}
int run_usd_extrinsics(int argc, char ** argv)
{
  if (argc!=9) {
    std::cerr<<"usage: usd_extrinsics stage.usd tcpPrim cameraPrim fixedMountPrim tcpFrame opticalFrame calibrationId output.yaml\n"; return 2;
  }
  try {
    const auto stage=UsdStage::Open(argv[1]);
    if (!stage || !stage->GetCompositionErrors().empty()) {throw std::runtime_error("cannot compose complete USD stage/references");}
    const auto tcp=stage->GetPrimAtPath(SdfPath(argv[2])); const auto camera=stage->GetPrimAtPath(SdfPath(argv[3]));
    const auto mount=stage->GetPrimAtPath(SdfPath(argv[4]));
    if (!tcp || !camera || !mount || !UsdGeomCamera(camera)) {throw std::runtime_error("TCP/camera/mount prim missing or camera type invalid");}
    const double metres=UsdGeomGetStageMetersPerUnit(stage);
    if (!std::isfinite(metres) || metres<=0) {throw std::runtime_error("invalid stage metres per unit");}
    UsdGeomXformCache cache(UsdTimeCode::Default()); fixed_branch(tcp,mount,cache); fixed_branch(camera,mount,cache);
    const auto tw=cache.GetLocalToWorldTransform(tcp); const auto cw=cache.GetLocalToWorldTransform(camera); rigid(tw); rigid(cw);
    GfMatrix4d optical(1); optical.SetRotate(GfRotation(GfVec3d(1,0,0),180));
    // USD matrices use row vectors. ROS optical +Z/-Y = USD camera -Z/+Y.
    const auto relative=optical*cw*tw.GetInverse(); rigid(relative);
    const GfTransform transform(relative); const auto p=transform.GetTranslation()*metres;
    const auto q=transform.GetRotation().GetQuat(); const auto v=q.GetImaginary();
    YAML::Node document; document["schema_version"]=1; document["calibration_id"]=argv[7];
    document["tcp_frame"]=argv[5]; document["optical_frame"]=argv[6];
    document["tcp_prim"]=argv[2]; document["camera_prim"]=argv[3]; document["fixed_mount_prim"]=argv[4];
    document["meters_per_unit"]=metres;
    for (int i=0;i<3;++i) {document["camera_in_tcp"]["position_m"].push_back(p[i]); document["camera_in_tcp"]["orientation_xyzw"].push_back(v[i]);}
    document["camera_in_tcp"]["orientation_xyzw"].push_back(q.GetReal());
    std::vector<std::string> digests;
    for (const auto & layer : stage->GetUsedLayers()) {
      if (layer==stage->GetSessionLayer() && layer->IsEmpty()) {continue;}
      const auto path=layer->GetRealPath(); if (path.empty()) {throw std::runtime_error("anonymous/unresolved source layer cannot be identified");}
      YAML::Node source; source["path"]=path; source["sha256"]=ptc_app::sha256_file(path);
      document["source_layers"].push_back(source); digests.push_back(source["sha256"].as<std::string>());
    }
    std::sort(digests.begin(),digests.end()); std::string closure;
    for (const auto & digest : digests) {closure+=digest+'\n';} document["source_digest"]=ptc_app::sha256_text(closure);
    const auto output=std::filesystem::u8path(argv[8]); const auto identity_path=std::filesystem::u8path(output.u8string()+".identity.yaml");
    if (std::filesystem::exists(identity_path)) {throw std::runtime_error("output identity already exists");}
    save(output,document); YAML::Node identity; identity["kind"]="hand_eye"; identity["id"]=argv[7]; identity["sha256"]=ptc_app::sha256_file(output);
    save(identity_path,identity); std::cout<<output.u8string()<<'\n'; return 0;
  } catch (const std::exception & e) {std::cerr<<e.what()<<'\n'; return 1;}
}
}  // namespace ptc_description_tools
