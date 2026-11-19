#include "ptc_app/robot_description_loader.hpp"
#include "ptc_app/settings_loader.hpp"
#include "ptc_core/pose_math.hpp"
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <map>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ptc_app
{
namespace
{
ptc_core::Pose3d pose(const urdf::Pose & p)
{return {{p.position.x,p.position.y,p.position.z},{p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w}};}
std::map<std::string,ptc_core::Pose3d> fixed_ancestors(urdf::LinkConstSharedPtr link)
{
  std::map<std::string,ptc_core::Pose3d> result; ptc_core::Pose3d accumulated;
  while (link) {
    result.emplace(link->name,accumulated);
    if (!link->parent_joint || link->parent_joint->type != urdf::Joint::FIXED) {break;}
    const auto next=ptc_core::compose_pose(pose(link->parent_joint->parent_to_joint_origin_transform),accumulated);
    if (!next) {throw std::runtime_error("invalid fixed URDF transform");} accumulated=*next; link=link->getParent();
  }
  return result;
}
std::string collision_geometry_digest(
  const urdf::Model & robot, const std::filesystem::path & description_path,
  const std::map<std::filesystem::path,std::string> & dependencies)
{
  std::vector<urdf::LinkSharedPtr> links; robot.getLinks(links);
  std::sort(links.begin(),links.end(),[](const auto & a,const auto & b){return a->name<b->name;});
  std::ostringstream manifest;
  manifest << "robot-collision-geometry-v2\n" << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto & link : links) {
    manifest << "link=" << link->name << '\n';
    std::vector<std::string> collisions;
    for (const auto & collision : link->collision_array) {
      if (!collision || !collision->geometry) {throw std::runtime_error("URDF collision has no geometry");}
      std::ostringstream entry;
      const auto & origin=collision->origin;
      entry << "name=" << collision->name << ";origin=" << origin.position.x << ',' << origin.position.y << ',' << origin.position.z
        << ',' << origin.rotation.x << ',' << origin.rotation.y << ',' << origin.rotation.z << ',' << origin.rotation.w << ';';
      switch (collision->geometry->type) {
        case urdf::Geometry::BOX: {
          const auto box=std::static_pointer_cast<urdf::Box>(collision->geometry);
          entry << "box=" << box->dim.x << ',' << box->dim.y << ',' << box->dim.z;
          break;
        }
        case urdf::Geometry::CYLINDER: {
          const auto cylinder=std::static_pointer_cast<urdf::Cylinder>(collision->geometry);
          entry << "cylinder=" << cylinder->radius << ',' << cylinder->length;
          break;
        }
        case urdf::Geometry::SPHERE: {
          const auto sphere=std::static_pointer_cast<urdf::Sphere>(collision->geometry);
          entry << "sphere=" << sphere->radius;
          break;
        }
        case urdf::Geometry::MESH: {
          const auto mesh=std::static_pointer_cast<urdf::Mesh>(collision->geometry);
          const auto file=std::filesystem::absolute(description_path.parent_path()/mesh->filename).lexically_normal();
          const auto dependency=dependencies.find(file);
          if (dependency==dependencies.end()) {throw std::runtime_error("URDF collision mesh missing from verified dependency manifest");}
          entry << "mesh_sha256=" << dependency->second << ";scale=" << mesh->scale.x << ',' << mesh->scale.y << ',' << mesh->scale.z;
          break;
        }
        default:
          throw std::runtime_error("unsupported URDF collision geometry type");
      }
      collisions.push_back(entry.str());
    }
    std::sort(collisions.begin(),collisions.end());
    for (const auto & collision : collisions) {manifest << "collision=" << collision << '\n';}
  }
  for (const auto & item : robot.joints_) {
    const auto & joint=item.second;
    const auto & origin=joint->parent_to_joint_origin_transform;
    manifest << "joint=" << joint->name << ";type=" << joint->type << ";parent=" << joint->parent_link_name
      << ";child=" << joint->child_link_name << ";origin=" << origin.position.x << ',' << origin.position.y << ',' << origin.position.z
      << ',' << origin.rotation.x << ',' << origin.rotation.y << ',' << origin.rotation.z << ',' << origin.rotation.w
      << ";axis=" << joint->axis.x << ',' << joint->axis.y << ',' << joint->axis.z;
    if (joint->limits) {
      manifest << ";limits=" << joint->limits->lower << ',' << joint->limits->upper << ',' << joint->limits->effort << ',' << joint->limits->velocity;
    }
    if (joint->mimic) {
      manifest << ";mimic=" << joint->mimic->joint_name << ',' << joint->mimic->multiplier << ',' << joint->mimic->offset;
    }
    manifest << '\n';
  }
  return "sha256:"+sha256_text(manifest.str());
}
}
RobotDescriptionModel load_robot_description(const ptc_core::SystemSettings & s)
{
  std::ifstream input(s.calibration.robot_description_path,std::ios::binary);
  if (!input) {throw std::runtime_error("cannot read robot URDF");}
  RobotDescriptionModel out; out.urdf_xml=std::string(std::istreambuf_iterator<char>(input),{});
  urdf::Model robot;
  if (!robot.initString(out.urdf_xml) || !robot.getLink(s.frames.robot_base) ||
    !robot.getLink(s.frames.tcp) || !robot.getLink(s.frames.color_optical) || !robot.getLink(s.frames.depth_optical)) {
    throw std::runtime_error("robot description must be expanded URDF with configured base/TCP/optical links");
  }
  const auto known=YAML::LoadFile(s.calibration.hand_eye_path);
  if (known["calibration_id"].as<std::string>() != s.calibration.expected_calibration_id ||
    known["tcp_frame"].as<std::string>() != s.frames.tcp || known["optical_frame"].as<std::string>() != s.frames.color_optical) {
    throw std::runtime_error("hand-eye identity/frame mismatch");
  }
  const auto p=known["camera_in_tcp"]["position_m"].as<std::vector<double>>();
  const auto q=known["camera_in_tcp"]["orientation_xyzw"].as<std::vector<double>>();
  if (p.size()!=3 || q.size()!=4) {throw std::runtime_error("invalid hand-eye vector dimensions");}
  out.camera_in_tcp={{p[0],p[1],p[2]},{q[0],q[1],q[2],q[3]}};
  const auto tcp=fixed_ancestors(robot.getLink(s.frames.tcp));
  const auto camera=fixed_ancestors(robot.getLink(s.frames.color_optical));
  bool matched=false;
  for (const auto & ancestor : tcp) {
    const auto found=camera.find(ancestor.first); if (found==camera.end()) {continue;}
    const auto inverse=ptc_core::inverse_pose(ancestor.second);
    const auto relative=inverse ? ptc_core::compose_pose(*inverse,found->second) : std::nullopt;
    if (!relative || !ptc_core::pose_near(*relative,out.camera_in_tcp,s.inspection.view_target_position_tolerance_m,
      s.inspection.view_target_orientation_tolerance_rad)) {throw std::runtime_error("known USD/hand-eye transform differs from fixed URDF mount");}
    matched=true; break;
  }
  if (!matched) {throw std::runtime_error("TCP and camera must share a fixed URDF mount; no calibration fitting at runtime");}
  const auto depth=fixed_ancestors(robot.getLink(s.frames.depth_optical)); bool depth_fixed=false;
  for (const auto & ancestor : camera) {if (depth.count(ancestor.first)) {depth_fixed=true; break;}}
  if (!depth_fixed) {throw std::runtime_error("RGB/depth optical links must share a fixed mount");}
  // Expanded URDF uses local paths and declares every mesh in its identity manifest.
  const auto identity=YAML::LoadFile(s.calibration.robot_description_path+".identity.yaml");
  std::map<std::filesystem::path,std::string> dependencies;
  if (identity["dependencies"]) {for (const auto & item : identity["dependencies"]) {
    const auto file=std::filesystem::absolute(std::filesystem::path(s.calibration.robot_description_path).parent_path()/item["path"].as<std::string>()).lexically_normal();
    dependencies.emplace(file,item["sha256"].as<std::string>());
  }}
  const auto mesh=[&](const urdf::GeometrySharedPtr & geometry) {
    if (!geometry || geometry->type!=urdf::Geometry::MESH) {return;}
    const auto data=std::static_pointer_cast<urdf::Mesh>(geometry);
    if (data->filename.find("://")!=std::string::npos) {throw std::runtime_error("resolve mesh URI to local asset before deployment");}
    const auto file=std::filesystem::absolute(std::filesystem::path(s.calibration.robot_description_path).parent_path()/data->filename).lexically_normal();
    const auto found=dependencies.find(file);
    if (found==dependencies.end() || sha256_file(file)!=found->second) {throw std::runtime_error("URDF mesh missing from verified dependency manifest");}
    // Verify portable relative assets first; resource_retriever needs an absolute
    // file URI only in the in-memory model passed to MoveIt.
    for(const char quote : {'\"','\''}) {
      const std::string attribute=std::string("filename=")+quote+data->filename+quote;
      const std::string resolved=std::string("filename=")+quote+"file://"+file.generic_u8string()+quote;
      std::size_t at=0;while((at=out.urdf_xml.find(attribute,at))!=std::string::npos) {
        out.urdf_xml.replace(at,attribute.size(),resolved);at+=resolved.size();
      }
    }

  };
  std::vector<urdf::LinkSharedPtr> links; robot.getLinks(links);
  for (const auto & item : links) {
    for (const auto & visual : item->visual_array) {mesh(visual->geometry);}
    for (const auto & collision : item->collision_array) {mesh(collision->geometry);}
  }
  out.collision_geometry_digest=collision_geometry_digest(robot,s.calibration.robot_description_path,dependencies);
  auto link=robot.getLink(s.frames.tcp);
  while (link && link->name!=s.frames.robot_base) {
    if (!link->parent_joint) {throw std::runtime_error("TCP is not beneath configured robot base");}
    if (link->parent_joint->type != urdf::Joint::FIXED) {out.arm_joint_names.push_back(link->parent_joint->name);}
    link=link->getParent();
  }
  if (!link || out.arm_joint_names.empty()) {throw std::runtime_error("missing arm joint chain");}
  std::reverse(out.arm_joint_names.begin(),out.arm_joint_names.end()); return out;
}
}  // namespace ptc_app
