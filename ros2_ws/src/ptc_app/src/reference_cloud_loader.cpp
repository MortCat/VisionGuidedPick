#include "ptc_app/reference_cloud_loader.hpp"
#include "ptc_core/vision/upright_pose_model.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace ptc_app
{
namespace
{
struct Property {std::string type,name;};
double binary_value(std::istream & input,const std::string & type)
{
  static const std::map<std::string,std::size_t> sizes{{"char",1},{"int8",1},{"uchar",1},{"uint8",1},{"short",2},{"int16",2},
    {"ushort",2},{"uint16",2},{"int",4},{"int32",4},{"uint",4},{"uint32",4},{"float",4},{"float32",4},{"double",8},{"float64",8}};
  const auto found=sizes.find(type); if (found==sizes.end()) {throw std::runtime_error("unsupported PLY property type");}
  std::uint64_t bits=0; for (std::size_t i=0;i<found->second;++i) {const int byte=input.get(); if (byte<0) {throw std::runtime_error("truncated PLY");} bits|=static_cast<std::uint64_t>(byte)<<(8*i);}
  if (type=="float" || type=="float32") {const auto raw=static_cast<std::uint32_t>(bits); float value; std::memcpy(&value,&raw,4); return value;}
  if (type=="double" || type=="float64") {double value; std::memcpy(&value,&bits,8); return value;}
  const bool sign=type=="char" || type=="int8" || type=="short" || type=="int16" || type=="int" || type=="int32";
  if (sign && found->second==1) {return static_cast<std::int8_t>(bits);}
  if (sign && found->second==2) {return static_cast<std::int16_t>(bits);}
  if (sign && found->second==4) {return static_cast<std::int32_t>(bits);} return static_cast<double>(bits);
}
}
std::vector<ptc_core::Vector3d> load_reference_cloud(const ptc_core::GeometryConfig & config)
{
  if (!config.fusion.max_points || !std::isfinite(config.fusion.evidence_voxel_size_m) || config.fusion.evidence_voxel_size_m<=0) {throw std::invalid_argument("reference loader requires validated geometry config");}
  std::ifstream input(config.healthy_reference_path,std::ios::binary); if (!input) {throw std::runtime_error("cannot load healthy reference");}
  std::string line; std::getline(input,line); if (!line.empty() && line.back()=='\r') {line.pop_back();}
  std::vector<ptc_core::Vector3d> points;
  if (line=="ply") {
    std::string format,element; std::size_t count=0; std::vector<Property> properties; bool end=false,vertex_first=true;
    while (std::getline(input,line)) {
      std::istringstream row(line); std::string key; row>>key;
      if (key=="format") {row>>format;}
      else if (key=="element") {row>>element; if (element=="vertex") {row>>count;} else if (!count) {vertex_first=false;}}
      else if (key=="property" && element=="vertex") {Property property; row>>property.type>>property.name;
        if (property.type=="list") {throw std::runtime_error("PLY vertex list properties unsupported");} properties.push_back(property);}
      else if (key=="end_header") {end=true; break;}
    }
    if (!end || !vertex_first || !count || properties.empty() || (format!="ascii" && format!="binary_little_endian")) {
      throw std::runtime_error("healthy reference must be XYZ, ASCII PLY or little-endian vertex-first PLY");
    }
    const auto stride=std::max<std::size_t>(1,(count+config.fusion.max_points-1)/config.fusion.max_points);
    for (std::size_t i=0;i<count;++i) {
      ptc_core::Vector3d point; bool x=false,y=false,z=false;
      for (const auto & property : properties) {
        double value=0; if (format=="ascii") {if (!(input>>value)) {throw std::runtime_error("invalid/truncated ASCII PLY");}} else {value=binary_value(input,property.type);}
        if (property.name=="x") {point.x_m=value; x=true;} else if (property.name=="y") {point.y_m=value; y=true;} else if (property.name=="z") {point.z_m=value; z=true;}
      }
      if (!x || !y || !z || !std::isfinite(ptc_core::vector_norm(point))) {throw std::runtime_error("healthy reference lacks finite XYZ");}
      if (i%stride==0) {points.push_back(point);}
    }
  } else {
    input.clear(); input.seekg(0);
    while (std::getline(input,line)) {
      if (line.empty() || line[0]=='#') {continue;} std::istringstream row(line); ptc_core::Vector3d point;
      if (!(row>>point.x_m>>point.y_m>>point.z_m) || !std::isfinite(ptc_core::vector_norm(point))) {throw std::runtime_error("invalid healthy XYZ reference");}
      points.push_back(point);
      if (points.size()>config.fusion.max_points*2) {points=ptc_core::voxel_sample(points,config.fusion.evidence_voxel_size_m,config.fusion.max_points);}
    }
  }
  points=ptc_core::voxel_sample(points,config.fusion.evidence_voxel_size_m,config.fusion.max_points);
  if (points.size()<3) {throw std::runtime_error("healthy reference needs >=3 geometry points");} return points;
}
}  // namespace ptc_app
