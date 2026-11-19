#pragma once
#include "ptc_core/config/system_settings.hpp"
namespace ptc_app
{
// XYZ or PLY vertices in canonical object coordinates, metres, +Z upright.
std::vector<ptc_core::Vector3d> load_reference_cloud(const ptc_core::GeometryConfig &);
}
