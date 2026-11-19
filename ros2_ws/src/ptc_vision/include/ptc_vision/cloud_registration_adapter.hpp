#pragma once
#include "ptc_core/vision/cloud_registration_model.hpp"
namespace ptc_vision
{
class CloudRegistrationAdapter
{
public:
  CloudRegistrationAdapter(const ptc_core::PointIndex & reference,const ptc_core::CloudRegistrationConfig & config)
    : model_(reference.points(),config) {}
  ptc_core::CloudRegistrationResult fit(const std::vector<ptc_core::Vector3d> & points,
    const ptc_core::SystemSettings & config,const std::optional<ptc_core::Pose3d> & initial={}) const;
  const ptc_core::CloudRegistrationModel & model() const {return model_;}
private:
  ptc_core::CloudRegistrationModel model_;
};
}
