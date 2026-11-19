#include "ptc_motion_moveit/grasp_contact_policy.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace ptc_motion_moveit;
int main()
{
  try{
    const auto check=[](bool value){if(!value)throw std::runtime_error("grasp contact policy regression");};
    const std::vector<std::string> fingers{"leftfinger","rightfinger"};
    moveit_msgs::msg::AllowedCollisionMatrix matrix;
    matrix.entry_names={"leftfinger","rightfinger","hand","camera","belt","tracked_1","tracked_2"};
    matrix.entry_values.resize(7);for(auto & row:matrix.entry_values)row.enabled.resize(7,false);
    matrix.entry_values[2].enabled[3]=matrix.entry_values[3].enabled[2]=true;
    replace_grasp_touch_permissions(matrix,fingers,"tracked_1");
    check(matrix.entry_values[0].enabled[5]&&matrix.entry_values[5].enabled[1]);
    check(!matrix.entry_values[2].enabled[5]&&!matrix.entry_values[3].enabled[5]&&!matrix.entry_values[0].enabled[4]);
    replace_grasp_touch_permissions(matrix,fingers,"tracked_2");
    check(!matrix.entry_values[0].enabled[5]&&!matrix.entry_values[5].enabled[1]);
    check(matrix.entry_values[0].enabled[6]&&matrix.entry_values[6].enabled[1]);
    check(matrix.entry_values[2].enabled[3]);
    replace_grasp_touch_permissions(matrix,fingers,"");
    check(!matrix.entry_values[0].enabled[6]&&!matrix.entry_values[1].enabled[5]);
    replace_grasp_touch_permissions(matrix,fingers,"tracked_3");check(matrix.entry_names.size()==8);
    check(matrix.entry_values[7].enabled[0]&&matrix.entry_values[1].enabled[7]);
    const std::vector<GraspContactPatchModel> patches{{{{-.02,0,0}},.005},{{{.02,0,0}},.005}};
    check(expected_grasp_contact("tracked_3","leftfinger",{-.02,0,0},"tracked_3",fingers,patches));
    check(expected_grasp_contact("rightfinger","tracked_3",{.02,0,0},"tracked_3",fingers,patches));
    check(!expected_grasp_contact("hand","tracked_3",{.02,0,0},"tracked_3",fingers,patches));
    check(!expected_grasp_contact("leftfinger","belt",{-.02,0,0},"tracked_3",fingers,patches));
    check(!expected_grasp_contact("leftfinger","tracked_2",{-.02,0,0},"tracked_3",fingers,patches));
    check(!expected_grasp_contact("leftfinger","tracked_3",{0,0,.04},"tracked_3",fingers,patches));
    check(!expected_grasp_contact("leftfinger","tracked_3",{std::numeric_limits<double>::quiet_NaN(),0,0},"tracked_3",fingers,patches));
    // Demo contact trials include the palm and all observed rabbit IDs only.
    replace_grasp_touch_permissions(matrix,std::vector<std::string>{"leftfinger","rightfinger","hand"},std::vector<std::string>{"tracked_1","tracked_2"});
    check(matrix.entry_values[2].enabled[5]&&matrix.entry_values[2].enabled[6]);
    check(matrix.entry_values[0].enabled[5]&&matrix.entry_values[1].enabled[6]);
    check(!matrix.entry_values[3].enabled[5]&&!matrix.entry_values[2].enabled[4]);
    check(!matrix.entry_values[0].enabled[1]&&matrix.entry_values[2].enabled[3]);
    replace_grasp_touch_permissions(matrix,std::vector<std::string>{"leftfinger","rightfinger","hand"},std::vector<std::string>{});
    check(!matrix.entry_values[2].enabled[5]&&!matrix.entry_values[2].enabled[6]);
    check(!matrix.entry_values[0].enabled[5]&&!matrix.entry_values[1].enabled[6]);
    check(matrix.entry_values[2].enabled[3]);
    // The rigid wrist-camera assembly can join the demo contact list; the
    // formal wrapper above still excludes it, and static geometry stays checked.
    replace_grasp_touch_permissions(matrix,std::vector<std::string>{"leftfinger","rightfinger","hand","camera"},std::vector<std::string>{"tracked_1","tracked_2"});
    check(matrix.entry_values[3].enabled[5]&&matrix.entry_values[3].enabled[6]);
    check(!matrix.entry_values[3].enabled[4]&&!matrix.entry_values[0].enabled[4]);
    replace_grasp_touch_permissions(matrix,std::vector<std::string>{"leftfinger","rightfinger","hand","camera"},std::vector<std::string>{});
    check(!matrix.entry_values[3].enabled[5]&&!matrix.entry_values[2].enabled[6]);
    matrix.entry_values[0].enabled.clear();bool rejected=false;
    try{replace_grasp_touch_permissions(matrix,fingers,"tracked_1");}catch(const std::runtime_error &){rejected=true;}check(rejected);
    std::cout<<"grasp target switching, stop revocation and patch boundaries passed\n";return 0;
  }catch(const std::exception & error){std::cerr<<error.what()<<'\n';return 1;}
}
