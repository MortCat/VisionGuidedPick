#include "ptc_vision/cloud_registration_adapter.hpp"
namespace ptc_vision
{
ptc_core::CloudRegistrationResult CloudRegistrationAdapter::fit(const std::vector<ptc_core::Vector3d> & points,
  const ptc_core::SystemSettings & config,const std::optional<ptc_core::Pose3d> & initial) const
{return ptc_core::register_cloud(points,model_,config.inspection.geometry,config.tracking,initial);}
}
