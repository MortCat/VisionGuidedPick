#include "state_test_support.hpp"
#include "ptc_core/bin_recovery_policy.hpp"
using namespace ptc_core;
using namespace ptc_test;
void tests()
{
  Pose3d preferred{{.1,.2,.3},{0,0,0,1}},configured=preferred;
  configured.orientation={0,0,1,0};
  constexpr double position_tolerance=.001,angle_tolerance=.01;
  auto target=select_bin_recovery_target(preferred,configured,{},position_tolerance,angle_tolerance);
  CHECK(target && pose_near(*target,preferred,position_tolerance,angle_tolerance));
  target=select_bin_recovery_target(preferred,configured,{preferred},position_tolerance,angle_tolerance);
  CHECK(target && pose_near(*target,configured,position_tolerance,angle_tolerance));
  CHECK(!select_bin_recovery_target(preferred,configured,{preferred,configured},position_tolerance,angle_tolerance));
  // A configured-orientation fallback arrival advances to release instead of
  // endlessly issuing the same successful approach under the preferred label.
  CHECK(bin_recovery_arrived(configured,preferred,configured,position_tolerance,angle_tolerance));
  auto not_above=configured;not_above.position.x_m+=.1;
  CHECK(!bin_recovery_arrived(not_above,preferred,configured,position_tolerance,angle_tolerance));
  // Do not retry an identical candidate under a different label or tiny jitter.
  CHECK(!select_bin_recovery_target(preferred,preferred,{preferred},position_tolerance,angle_tolerance));
  auto jitter=preferred;jitter.position.z_m+=.0001;
  CHECK(!select_bin_recovery_target(jitter,preferred,{preferred},position_tolerance,angle_tolerance));
  auto invalid=preferred;invalid.position.x_m=std::numeric_limits<double>::quiet_NaN();
  CHECK(!select_bin_recovery_target(invalid,invalid,{},position_tolerance,angle_tolerance));
  // A genuinely different release position remains available after approach.
  auto release=preferred;release.position.z_m-=.1;
  CHECK(select_bin_recovery_target(release,release,{preferred,configured},position_tolerance,angle_tolerance));
}
int main(){return run(tests);}
