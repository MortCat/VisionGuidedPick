#include <limits>
#include <cmath>
#include "test_support.hpp"
#include "ptc_core/vision/cloud_registration_model.hpp"
#include "ptc_core/pose_math.hpp"
#include "ptc_core/vision/rgbd_image_frame.hpp"

int main()
{
  return ptc_test::run([] {
    using namespace ptc_core; using namespace ptc_test;
    auto s = settings_data(); CHECK(validate_settings(s).empty());
    const auto frozen = settings(); s.task.cycle_timeout_sec = 1;
    CHECK(frozen.get().task.cycle_timeout_sec == 100);
    s = settings_data(); s.configuration_state = "draft"; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.motion.position_tolerance_m = kUnset; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.motion.observation_pose.orientation.w = 0; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.task.candidate_retry_interval_sec = 0; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.task.max_candidate_admission_attempts_per_observation = 0; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.task.max_candidate_admission_attempts_per_observation = 6; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.max_prediction_horizon_sec = 2; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.velocity_min_samples = 2; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.velocity_max_samples = 65; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.velocity_max_samples = 2; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.velocity_min_span_sec = 0.6; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.max_velocity_fit_residual_m = 0; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.tracking.max_velocity_uncertainty_mps = 0; CHECK(!validate_settings(s).empty());
    s = settings_data(); s.inspection.inference.backend = "efficient_ad";
    s.inspection.inference.anomaly_score_threshold = 12; s.inspection.inference.efficient_ad_min_positive_frames = 2;
    CHECK(validate_settings(s).empty());
    CHECK(!seconds_to_duration(-1)); CHECK(!seconds_to_duration(kUnset)); CHECK(!seconds_to_duration(1e20));
    CHECK(!deadline_after(Nanoseconds(std::numeric_limits<std::int64_t>::max()), 1));
    CHECK(expired(Nanoseconds(10), {Nanoseconds(0), Nanoseconds(10), 1, false}));
    CHECK(!source_is_fresh(Nanoseconds(-1), time(), 1, 0));
    CHECK(!source_is_fresh(Nanoseconds(0), time(), 1, 0));
    CHECK(!source_is_fresh(*seconds_to_duration(1.1), time(), 1, 0.01));
    auto a = arm(); CHECK(arm_snapshot_ready(a, frozen, time()));
    a.state.joints[0].position = kUnset; CHECK(!arm_snapshot_ready(a, frozen, time()));
    a = arm(); a.epoch = 2; CHECK(!arm_snapshot_ready(a, frozen, time()));
    auto g = gripper(); CHECK(gripper_snapshot_ready(g, frozen, time()));
    g.width_m = -1; CHECK(!gripper_snapshot_ready(g, frozen, time()));
    auto f = frame(); CHECK(rgbd_frame_valid(f, frozen, time()));
    f.transform_valid = false; CHECK(!rgbd_frame_valid(f, frozen, time()));
    f = frame(); f.depth_timestamp += *seconds_to_duration(0.1); CHECK(!rgbd_frame_valid(f, frozen, time()));
    CHECK(same_operation(key(), key())); auto old = key(); old.epoch = 2; CHECK(!same_operation(old, key()));
    EventQueue queue{1, false, {}}; CHECK(queue.push(key())); CHECK(!queue.push(key())); CHECK(queue.overflowed);
    CHECK(queue.pop()); CHECK(!queue.pop());
    std::deque<TransitionRecord> history;
    CHECK(append_history(history, {}, 1)); CHECK(append_history(history, {}, 1)); CHECK(history.size() == 1);
    CHECK(!append_history(history, {}, 0));
    // Direct cloud logic only: no camera/ROS runtime or performance benchmark.
    GeometryConfig geometry;geometry.registration={0.01,0.001,128,20,0.1,0.5,0.000001,0.000001,0.05,0.2};
    geometry.fusion={0.001,512};geometry.min_registration_points=3;
    geometry.yaw_search_step_rad=0.2;geometry.registration_max_error_m=0.02;
    TrackingConfig tracking;tracking.max_position_stddev_m=0.02;tracking.max_yaw_stddev_rad=0.2;tracking.yaw_ambiguity_margin_m=0.0001;
    const std::vector<Vector3d> nominal{{0,0,0},{0.1,0,0.05},{0,0.2,0.08},{0.06,0.07,0.1},{-0.04,0.03,0.14}};
    CloudRegistrationModel reference(nominal,geometry.registration);
    Pose3d truth;truth.position={0.3,0.2,0.1};truth.orientation={0,0,std::sin(0.15),std::cos(0.15)};
    std::vector<Vector3d> observed;for(const auto & p:nominal){observed.push_back(transform_point(truth,p));}
    const auto local=register_cloud(observed,reference,geometry,tracking,truth);
    CHECK(local.valid);CHECK(pose_near(local.object_world,truth,0.00001,0.00001));CHECK(local.error_m<0.00001);
    observed.push_back({9,9,9});const auto with_defect=register_cloud(observed,reference,geometry,tracking,truth);
    CHECK(with_defect.valid);CHECK(observed.size()==6); // Pose outlier trimming never deletes evidence.
    RgbdImageFrame sparse;sparse.camera={100,100,100,100,50,50,{}};sparse.rgb.resize(30000);sparse.depth_m.resize(10000,0.f);
    std::vector<std::uint8_t> selected(10000,0);
    for(std::size_t i=101;i<9999;i+=101){sparse.depth_m[i]=.1f;selected[i]=1;}
    const auto all_sparse=region_points_world(sparse,{0,0,100,100},4096,selected);
    CHECK(all_sparse.size()==98); // Previously an ROI stride silently lost these valid samples.
    const auto bounded_sparse=region_points_world(sparse,{0,0,100,100},12,selected);
    CHECK(bounded_sparse.size()==12);CHECK(bounded_sparse.front().x_m<bounded_sparse.back().x_m);
    CHECK(std::abs(all_sparse.front().z_m-.1)<1e-6); // Close points survive processing.
    RgbdImageFrame local_grasp;
    local_grasp.camera={8,4,40,40,4,2,{}};
    local_grasp.rgb.resize(8*4*3);
    local_grasp.depth_m.resize(8*4);
    std::vector<std::uint8_t> target_mask(8*4);
    target_mask[20]=1;
    local_grasp.depth_m[20]=1.F; // Selected surface.
    local_grasp.depth_m[21]=1.F; // Adjacent object, outside the target ROI.
    local_grasp.depth_m[22]=.75F; // Belt.
    const auto neighbors=grasp_neighbor_offsets(local_grasp,{4,2,1,1},target_mask,{0.,0.,1.},.10,.75);
    CHECK(neighbors.size()==1);
    CHECK(neighbors.front().x_m>=.02 && neighbors.front().x_m<=.03);
    CHECK(grasp_neighbor_offsets(local_grasp,{4,2,1,1},{},{0.,0.,1.},.10,.75).empty());
  });
}
