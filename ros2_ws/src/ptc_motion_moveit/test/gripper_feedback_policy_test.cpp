#include "ptc_motion_moveit/gripper_feedback_policy.hpp"
#include <cassert>
using namespace ptc_motion_moveit;
int main()
{
  const std::vector<FingerPositionSample> fixed{{0.,{.04,.04}},{.05,{.04,.04}},{.10,{.04,.04}}};
  auto motion=estimate_finger_motion(fixed,.01);assert(motion && motion->stopped);
  assert(motion->velocity_mps[0]==0. && motion->velocity_mps[1]==0.);
  auto moving=fixed;moving[1].positions={.03,.05};moving[2].positions={.02,.06};
  motion=estimate_finger_motion(moving,.01);assert(motion && !motion->stopped);
  assert(motion->velocity_mps[0]<0. && motion->velocity_mps[1]>0.);
  moving=fixed;moving[1].positions[1]=.03;
  motion=estimate_finger_motion(moving,.01);assert(motion && !motion->stopped); // reversal
  assert(!estimate_finger_motion({fixed[0],fixed[1]},.01));
  moving=fixed;moving[2].seconds=.05;assert(!estimate_finger_motion(moving,.01));
  moving=fixed;moving[2].seconds=.4;assert(!estimate_finger_motion(moving,.01));
  moving=fixed;moving[1].positions.pop_back();assert(!estimate_finger_motion(moving,.01));
  moving=fixed;moving[1].positions[0]=std::nan("");assert(!estimate_finger_motion(moving,.01));
  assert(measured_full_open_empty(true,true,fixed,.04,.001));
  assert(!measured_full_open_empty(true,false,fixed,.04,.001));
  assert(!measured_full_open_empty(false,true,fixed,.04,.001));
  auto one_jaw_part_open=fixed;
  one_jaw_part_open[1].positions[1]=.038;
  assert(!measured_full_open_empty(true,true,one_jaw_part_open,.04,.001));
  assert(!measured_full_open_empty(true,true,{},.04,.001));
  // Regression: 74.8mm width plus contact was previously also marked Empty.
  assert(!measured_empty(true,true,.0748,.08,.008));
  assert(measured_empty(true,false,.08,.08,.008));
  assert(!measured_empty(false,false,.08,.08,.008));
  assert(!measured_empty(true,false,.04,.08,.008));
  assert(contact_after_command(true,false,true,false)); // Failed open keeps valid holding evidence.
  bool contact=true, unknown=true;
  clear_latched_possession_if_closed_empty(false,contact,unknown);
  assert(contact && unknown);
  clear_latched_possession_if_closed_empty(true,contact,unknown);
  assert(!contact && !unknown);
  contact=contact_after_command(contact,false,true,false);
  clear_latched_possession_if_closed_empty(false,contact,unknown);
  assert(!contact && !unknown); // An interrupted reopening cannot revive a prior grasp.
  assert(!contact_after_command(true,true,true,false)); // Successful open clears it.
  assert(contact_after_command(false,true,false,true));
  assert(!contact_after_command(false,true,false,false));
}
