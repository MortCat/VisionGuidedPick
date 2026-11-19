#include "ptc_core/target_selection.hpp"
#include "ptc_core/grasp_offset_ramp.hpp"
#include <chrono>
#include <iostream>
#include <limits>

#include "ptc_core/arm_state.hpp"
#include "ptc_core/approach_timing.hpp"
#include "ptc_core/move_to_pose.hpp"

namespace
{

bool expect(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  bool passed = true;
  passed &= expect(std::abs(*ptc_core::estimate_joint_travel_duration(.25,1.,1.)-1.)<1e-9,"triangular rest-to-rest duration");
  passed &= expect(std::abs(*ptc_core::estimate_joint_travel_duration(2.,1.,1.)-3.)<1e-9,"trapezoidal rest-to-rest duration");
  passed &= expect(!ptc_core::estimate_joint_travel_duration(1.,0.,1.),"invalid velocity has no estimate");
  passed &= expect(!ptc_core::observation_entry_follow_elapsed(std::chrono::seconds(2),std::chrono::milliseconds(2499),.5),"follow is not completed early");
  passed &= expect(ptc_core::observation_entry_follow_elapsed(std::chrono::seconds(2),std::chrono::milliseconds(2500),.5),"follow uses source-time elapsed");
  passed &= expect(!ptc_core::observation_entry_follow_elapsed(std::chrono::seconds(2),std::chrono::seconds(1),.5),"clock rewind cannot finish follow");
  passed &= expect(ptc_core::observation_entry_follow_elapsed(std::chrono::seconds(2),std::chrono::seconds(2),0),"zero follow skips dwell");

  ptc_core::MoveToPoseRequest request;
  request.target_pose.reference_frame = "robot_base";
  request.timeout = std::chrono::seconds(10);

  passed &= expect(
    ptc_core::is_valid_move_to_pose_request(request),
    "a complete request should be valid");

  request.max_velocity_scale = 0.0;
  passed &= expect(
    !ptc_core::is_valid_move_to_pose_request(request),
    "a zero velocity scale should be invalid");

  request.max_velocity_scale = 0.5;
  request.target_pose.pose.orientation.w = 0.0;
  passed &= expect(
    !ptc_core::is_valid_move_to_pose_request(request),
    "a zero quaternion should be invalid");

  request.target_pose.pose.orientation.w = 1.0;
  request.target_pose.pose.position.x_m = std::numeric_limits<double>::infinity();
  passed &= expect(
    !ptc_core::is_valid_move_to_pose_request(request),
    "a non-finite position should be invalid");

  request.target_pose.pose.position.x_m=0;
  request.target_pose.timestamp=std::chrono::seconds(1);
  request.target_linear_velocity={0.06,0,0};request.prediction_horizon=std::chrono::seconds(2);
  const auto predicted=ptc_core::predict_move_target(request,std::chrono::milliseconds(1500));
  passed &= expect(predicted && std::abs(predicted->pose.position.x_m-.03)<1e-9,"moving target uses source time once");
  const auto with_execution_overhead=ptc_core::predict_move_target(request,std::chrono::milliseconds(2900));
  passed &= expect(with_execution_overhead&&std::abs(with_execution_overhead->pose.position.x_m-.114)<1e-9,"arrival includes travel and execution overhead exactly once");
  passed &= expect(request.target_pose.timestamp==std::chrono::seconds(1)&&request.target_pose.pose.position.x_m==0,"prediction never rewrites its source anchor");
  passed &= expect(!ptc_core::predict_move_target(request,std::chrono::milliseconds(3100)),"expired moving target is rejected");
  passed &= expect(!ptc_core::predict_move_target(request,std::chrono::milliseconds(900)),"backward source time is rejected");
  request.prediction_horizon=std::chrono::seconds(0);
  passed &= expect(ptc_core::predict_move_target(request,std::chrono::seconds(10))->pose.position.x_m==0,"fixed pose is unchanged by velocity");
  request.prediction_horizon=std::chrono::seconds(1);request.target_pose.timestamp=std::chrono::seconds(0);
  passed &= expect(!ptc_core::is_valid_move_to_pose_request(request),"moving target requires nonzero source stamp");
  ptc_core::ArmState arm_state;
  passed &= expect(
    arm_state.motion_state == ptc_core::ArmMotionState::kUnknown,
    "a default arm state should have an unknown motion state");
  passed &= expect(!arm_state.status.ready, "a default device should not be ready");
  passed &= expect(arm_state.status.data_stale, "default device data should be stale");

  ptc_core::TaskConfig selection;
  passed &= expect(ptc_core::is_in_target_selection_roi({.6,0.,.85},selection),"omitted ROI preserves selection");
  selection.target_selection_roi=ptc_core::TargetSelectionRoiConfig{{.45,-1.1,.75},{.775,-.8,1.1}};
  passed &= expect(ptc_core::is_in_target_selection_roi({.6,-1.,.85},selection),"upstream target selected");
  passed &= expect(ptc_core::is_in_target_selection_roi({.6,-.8,.85},selection),"ROI boundary inclusive");
  passed &= expect(!ptc_core::is_in_target_selection_roi({.6,-.75,.85},selection),"centerline target not newly selected");
  passed &= expect(!ptc_core::is_in_target_selection_roi({.9,-1.,.85},selection),"outside belt not selected");
  ptc_core::GraspOffsetRamp ramp;ramp.rebase(0.,{0,.02,0},{0,0,0},.2);
  const auto before=ramp.position(.05),velocity_before=ramp.velocity(.05);
  ramp.rebase(.05,{0,.03,0},{0,0,0},.2);
  passed &= expect(std::abs(ramp.position(.05).y_m-before.y_m)<1e-12,"new visual frame does not jump the commanded position offset");
  passed &= expect(std::abs(ramp.velocity(.05).y_m-velocity_before.y_m)<1e-12,"new visual frame preserves offset velocity continuity");
  passed &= expect(std::abs(ramp.position(.25).y_m-.03)<1e-12,"live offset reaches its updated forecast within the existing preview horizon");
  return passed ? 0 : 1;
}
