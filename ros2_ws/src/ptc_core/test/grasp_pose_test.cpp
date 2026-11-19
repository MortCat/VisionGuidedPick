#include <limits>
#include "state_test_support.hpp"
#include "ptc_core/vision/grasp_pose_model.hpp"
#include "ptc_core/grasp_offset_ramp.hpp"
#include <cmath>
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <vector>
using namespace ptc_core;using namespace ptc_test;
int main(){return run([]{
  auto data=settings_data();data.motion.gripper_joint="panda_finger_joint1";data.grasp.approach_tcp_in_object.orientation={1,0,0,0};data.grasp.closing_axis_tcp={0,1,0};data.grasp.finger_center_tcp={0,0,0};
  data.grasp.intercept_lead_sec=.08;data.grasp.contact_band_fraction=.45;data.grasp.min_contact_points=6;data.inspection.geometry.fusion.evidence_voxel_size_m=.001;
  data.grasp.pad_contact_size_m=std::array<double,2>{.038,.022};
  const auto cfg=settings(data);auto object=track();object.velocity_mps={0,0,0};
  std::vector<Vector3d> cloud,flat;
  for(int i=0;i<=30;++i)for(int j=0;j<=30;++j){
    const double a=i/30.,b=j/30.;
    cloud.push_back({-.025,.06*(a-.5),.06*b});cloud.push_back({.025,.06*(a-.5),.06*b});
    cloud.push_back({.05*(a-.5),-.03,.06*b});cloud.push_back({.05*(a-.5),.03,.06*b});
    flat.push_back({0,.06*(a-.5),.06*b});}
  GraspSurfaceSnapshot surface;surface.key=object.context.key;surface.revision=7;surface.object_frame="object_1";surface.settings_revision=cfg.revision();
  for(const auto & point:cloud){GraspSurfacePoint sample;sample.position_object_m=point;sample.capture_timestamp=object.observation.timestamp;
    if(std::abs(std::abs(point.x_m)-.025)<1e-8)sample.view_direction_object=Vector3d{point.x_m>0?1.:-1.,0,0};
    else if(std::abs(std::abs(point.y_m)-.03)<1e-8)sample.view_direction_object=Vector3d{0,point.y_m>0?1.:-1.,0};
    surface.points.push_back(sample);}
  const auto gripper=configured_gripper_geometry(cfg,"sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
  // Single-view commissioning uses measured silhouette bounds, never labels
  // estimated horizontal directions as observed opposed normals.
  for(const double yaw:{0.,.7,1.8,2.9}){
    auto top=surface;top.points.clear();
    for(int i=0;i<40;++i)for(int j=0;j<10;++j){
      const double a=i*2*3.14159265358979323846/40.,rr=.65+.35*j/9.;
      const double x=.03*rr*std::cos(a),y=.015*rr*std::sin(a);
      top.points.push_back({{std::cos(yaw)*x-std::sin(yaw)*y,std::sin(yaw)*x+std::cos(yaw)*y,.04+.006*(1-rr*rr)},Vector3d{0,0,1},object.observation.timestamp});
    }
    const auto slice=propose_top_view_grasp_candidates(top,object,gripper,cfg,time(),.04);
    CHECK(!slice.candidates.empty());
    for(const auto & candidate:slice.candidates){CHECK(candidate.valid);CHECK(candidate.slice_normal_alignment==0.);CHECK(!candidate.pad_contact_coverage[0]);CHECK(!candidate.pad_contact_coverage[1]);
      CHECK(candidate.source=="measured_top_view_slice_commissioning");CHECK(candidate.opening_width_m<gripper.maximum_opening_m);
      for(const auto & sample:top.points)CHECK(candidate.pregrasp_in_object.position.z_m>=sample.position_object_m.z_m+.04-1e-8);}
    top.points.clear();CHECK(propose_top_view_grasp_candidates(top,object,gripper,cfg,time(),.04).candidates.empty());
  }
  // Two visible sloped sides support horizontal closing, but their shared
  // upward component prevents a pair from meeting the main search's .72 threshold.
  auto sloped=surface;sloped.points.clear();
  for(int side:{-1,1})for(int row=0;row<21;++row)for(int col=0;col<21;++col){
    const double z=.034+.012*row/20.;
    const double x=side*(.026-(4./3.)*(z-.04));
    sloped.points.push_back({{x,.028*(col/20.-.5),z},
      Vector3d{side*.6,0,.8},object.observation.timestamp});
  }
  const auto measured_slice=propose_top_view_grasp_candidates(sloped,object,gripper,cfg,time(),.04);
  std::cout<<"Sloped contact fixture: "<<measured_slice.reason<<'\n';
  CHECK(std::any_of(measured_slice.candidates.begin(),measured_slice.candidates.end(),[](const auto & candidate){
    return candidate.source=="measured_top_view_slice_commissioning" && candidate.slice_normal_alignment>.55;
  }));
  // The unseen side must leave the measured side available for ranking.
  auto occluded_sloped=sloped;
  for(auto & point:occluded_sloped.points){
    if(point.position_object_m.x_m<0.)point.view_direction_object.reset();
  }
  const auto occluded=propose_top_view_grasp_candidates(occluded_sloped,object,gripper,cfg,time(),.04);
  CHECK(std::any_of(occluded.candidates.begin(),occluded.candidates.end(),[](const auto & candidate){
    const auto & pads=candidate.pad_contact_coverage;
    return static_cast<bool>(pads[0])!=static_cast<bool>(pads[1]);
  }));
  const auto proposals=propose_grasp_candidates(surface,object,gripper,cfg,time(),.04);
  if(proposals.candidates.empty())std::cerr<<"proposal status="<<static_cast<int>(proposals.status)<<" reason="<<proposals.reason<<" elapsed_ms="<<proposals.elapsed_ms<<'\n';
  CHECK(!proposals.candidates.empty());CHECK(proposals.candidates.size()<=16);CHECK(proposals.surface_revision==7);
  // Wrist proximity chooses between equivalent poses of one contact pair. It
  // must not replace the proposal's contact order with the nearest wrist yaw.
  {
    auto top=surface;top.points.clear();
    for(int i=0;i<40;++i)for(int j=0;j<10;++j){
      const double a=i*2*3.14159265358979323846/40.,rr=.65+.35*j/9.;
      top.points.push_back({{.03*rr*std::cos(a),.015*rr*std::sin(a),.04+.006*(1-rr*rr)},Vector3d{0,0,1},object.observation.timestamp});
    }
    const auto original=propose_top_view_grasp_candidates(top,object,gripper,cfg,time(),.04).candidates;
    const auto same_pair=[](const GraspCandidate & a,const GraspCandidate & b){
      const auto near=[](const Vector3d & p,const Vector3d & q){return std::hypot(p.x_m-q.x_m,p.y_m-q.y_m,p.z_m-q.z_m)<1e-6;};
      return (near(a.left_contact.center_object_m,b.left_contact.center_object_m)&&near(a.right_contact.center_object_m,b.right_contact.center_object_m))||
        (near(a.left_contact.center_object_m,b.right_contact.center_object_m)&&near(a.right_contact.center_object_m,b.left_contact.center_object_m));
    };
    const auto pair_size=static_cast<std::size_t>(std::count_if(original.begin(),original.end(),[&](const auto & c){return same_pair(c,original.front());}));
    const auto other=std::find_if(original.begin(),original.end(),[&](const auto & c){return !same_pair(c,original.front());});
    CHECK(pair_size>=2);CHECK(other!=original.end());
    auto ordered=original;
    order_grasp_candidates_for_wrist(ordered,other->tcp_world.orientation);
    CHECK(ordered.size()==original.size());
    for(std::size_t i=0;i<pair_size;++i)CHECK(same_pair(ordered[i],original.front()));
    const auto facing=[](const GraspCandidate & a,const GraspCandidate & b){
      const auto & p=a.tcp_world.orientation;const auto & q=b.tcp_world.orientation;
      return std::abs(p.x*q.x+p.y*q.y+p.z*q.z+p.w*q.w);};
    CHECK(same_pair(ordered[pair_size],*other));CHECK(facing(ordered[pair_size],*other)>.999);
    const auto twin=std::find_if(original.begin(),original.end(),[&](const auto & c){
      return same_pair(c,original.front()) && facing(c,original.front())<.5;});
    CHECK(twin!=original.end());
    order_grasp_candidates_for_wrist(ordered,twin->tcp_world.orientation);
    CHECK(same_pair(ordered.front(),*twin));CHECK(facing(ordered.front(),*twin)>.999);
    // Measured bilateral normals move their pair ahead; its wrist twin follows.
    ordered=original;
    for(auto & c:ordered)if(same_pair(c,*other))c.slice_normal_alignment=.8;
    order_grasp_candidates_for_wrist(ordered,original.front().tcp_world.orientation);
    CHECK(same_pair(ordered.front(),*other));
    // Distinct opposed-surface candidates keep the core's priority order.
    auto ranked=proposals.candidates;
    order_grasp_candidates_for_wrist(ranked,proposals.candidates.back().tcp_world.orientation);
    for(std::size_t i=0;i<ranked.size();++i)CHECK(ranked[i].candidate_id==proposals.candidates[i].candidate_id);
  }
  Pose3d priority_rotation;priority_rotation.orientation=object.observation.pose_world.orientation;
  CHECK(transform_point(priority_rotation,proposals.candidates.front().approach_direction_object).z_m<-.5);
  auto tiered=surface;tiered.points.clear();
  for(const auto & point:surface.points){
    auto lower=point;lower.position_object_m.x_m*=.8;lower.position_object_m.y_m*=.8;lower.position_object_m.z_m*=.5;
    auto upper=point;upper.position_object_m.x_m*=1.1;upper.position_object_m.y_m*=1.1;upper.position_object_m.z_m=upper.position_object_m.z_m*.5+.07;
    tiered.points.push_back(lower);tiered.points.push_back(upper);
  }
  const auto upper_supported=propose_grasp_candidates(tiered,object,gripper,cfg,time(),.04);
  CHECK(!upper_supported.candidates.empty());
  const auto & preferred=upper_supported.candidates.front();
  const double contact_height=(preferred.left_contact.center_object_m.z_m+preferred.right_contact.center_object_m.z_m)*.5;
  if(contact_height<=.06)std::cerr<<"bounded selection discarded supported upper contacts; retained height="<<contact_height<<'\n';
  CHECK(contact_height>.06);
  CHECK(proposals.collision_geometry_digest==gripper.collision_geometry_digest);
  CHECK(proposals.candidates.front().collision_geometry_digest==gripper.collision_geometry_digest);
  CHECK(proposals.sampled_point_count<=4096);CHECK(!proposals.approach_direction_coverage.empty());
  // G11 baseline only: warm the pure geometry path, then measure 100 synthetic
  // variants. This is deliberately not reported as a sensor/ROS round-trip.
  std::vector<double> proposal_wall_ms,proposal_compute_ms;proposal_wall_ms.reserve(100);proposal_compute_ms.reserve(100);
  for(unsigned sample_index=0;sample_index<100;++sample_index){
    auto variant=surface;variant.revision=100+sample_index;
    const double angle=.003*sample_index;Pose3d rotation;rotation.orientation={0,0,std::sin(angle*.5),std::cos(angle*.5)};
    for(auto & point:variant.points){point.position_object_m=transform_point(rotation,point.position_object_m);
      if(point.view_direction_object){point.view_direction_object=transform_point(rotation,*point.view_direction_object);}}
    const auto began=std::chrono::steady_clock::now();
    const auto result=propose_grasp_candidates(variant,object,gripper,cfg,time(),.04);
    proposal_wall_ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count());
    proposal_compute_ms.push_back(result.elapsed_ms);CHECK(!result.candidates.empty());
  }
  const auto latency=[&](std::vector<double> values,double q){std::sort(values.begin(),values.end());
    return values[std::min(static_cast<std::size_t>(std::ceil(q*values.size())-1.),values.size()-1)];};
  std::cout<<"synthetic_grasp_proposal_100 n="<<proposal_wall_ms.size()
    <<" wall_p50_ms="<<latency(proposal_wall_ms,.50)<<" wall_p95_ms="<<latency(proposal_wall_ms,.95)
    <<" wall_p99_ms="<<latency(proposal_wall_ms,.99)<<" wall_max_ms="<<latency(proposal_wall_ms,1.)
    <<" compute_p50_ms="<<latency(proposal_compute_ms,.50)<<" compute_p95_ms="<<latency(proposal_compute_ms,.95)
    <<" compute_p99_ms="<<latency(proposal_compute_ms,.99)<<" compute_max_ms="<<latency(proposal_compute_ms,1.)
    <<" input_points="<<surface.points.size()<<" source=synthetic_geometry_fixture (not RGB-D/ROS round-trip)\n";
  CHECK(std::count_if(proposals.approach_direction_coverage.begin(),proposals.approach_direction_coverage.end(),[](auto n){return n>0;})>=3);
  const auto dot3=[](const Vector3d & a,const Vector3d & b){return a.x_m*b.x_m+a.y_m*b.y_m+a.z_m*b.z_m;};
  for(const auto & candidate:proposals.candidates){CHECK(candidate.valid);CHECK(candidate.left_contact.support_point_count>=cfg.get().grasp.min_contact_points);
    Pose3d rotation;rotation.orientation=candidate.tcp_in_object.orientation;
    const auto closing=transform_point(rotation,gripper.closing_axis_tcp),approach=transform_point(rotation,gripper.approach_axis_tcp);
    CHECK(std::abs(vector_norm(closing)-1.)<1e-6);CHECK(std::abs(dot3(approach,candidate.approach_direction_object)-1.)<1e-5);
    const auto offset=Vector3d{candidate.pregrasp_in_object.position.x_m-candidate.tcp_in_object.position.x_m,
      candidate.pregrasp_in_object.position.y_m-candidate.tcp_in_object.position.y_m,
      candidate.pregrasp_in_object.position.z_m-candidate.tcp_in_object.position.z_m};
    CHECK(dot3(offset,candidate.approach_direction_object)<=-.04+1e-8);}
  GraspSurfaceSnapshot cube;cube.key=object.context.key;cube.revision=8;cube.object_frame="object_1";cube.settings_revision=cfg.revision();
  for(int face=0;face<6;++face)for(int i=-12;i<=12;++i)for(int j=-12;j<=12;++j){
    GraspSurfacePoint sample;const double u=i*.002,v=j*.002;Vector3d normal;
    if(face<2){sample.position_object_m={face==0?-.025:.025,u,v};normal={face==0?-1.:1.,0,0};}
    else if(face<4){sample.position_object_m={u,face==2?-.025:.025,v};normal={0,face==2?-1.:1.,0};}
    else{sample.position_object_m={u,v,face==4?-.025:.025};normal={0,0,face==4?-1.:1.};}
    sample.view_direction_object=normal;sample.capture_timestamp=object.observation.timestamp;cube.points.push_back(sample);}
  const auto cube_proposal=propose_grasp_candidates(cube,object,gripper,cfg,time(),.04);
  CHECK(!cube_proposal.candidates.empty());CHECK(cube_proposal.sampled_point_count<=4096);
  CHECK(std::any_of(cube_proposal.candidates.begin(),cube_proposal.candidates.end(),[](const auto & candidate){
    const auto & pads=candidate.pad_contact_coverage;
    return pads[0] && pads[1] && std::min(*pads[0],*pads[1])>=.5;
  }));
  bool top_approach=false,side_approach=false,oblique_approach=false;
  for(const auto & candidate:cube_proposal.candidates){const auto & direction=candidate.approach_direction_object;
    top_approach=top_approach||direction.z_m<-.9;side_approach=side_approach||std::abs(direction.x_m)>.9;
    oblique_approach=oblique_approach||(direction.x_m>.5&&direction.z_m<-.5);}
  CHECK(top_approach);CHECK(side_approach);CHECK(oblique_approach);
  Pose3d yaw; yaw.orientation={0,0,std::sin(.31),std::cos(.31)};
  GraspSurfaceSnapshot rotated=cube;for(auto & sample:rotated.points){sample.position_object_m=transform_point(yaw,sample.position_object_m);
    if(sample.view_direction_object){Pose3d rotation;rotation.orientation=yaw.orientation;sample.view_direction_object=transform_point(rotation,*sample.view_direction_object);}}
  const auto rotated_proposal=propose_grasp_candidates(rotated,object,gripper,cfg,time(),.04);CHECK(!rotated_proposal.candidates.empty());
  for(const auto & target:std::array<Vector3d,3>{{{0,0,-1},{-1,0,0},{.70710678,0,-.70710678}}}){
    const auto expected=transform_point(Pose3d{{0,0,0},yaw.orientation},target);
    CHECK(std::any_of(rotated_proposal.candidates.begin(),rotated_proposal.candidates.end(),[&](const auto & candidate){return dot3(candidate.approach_direction_object,expected)>.82;}));}
  auto flat_surface=surface;flat_surface.points.clear();for(const auto & point:flat){GraspSurfacePoint sample;sample.position_object_m=point;
    sample.view_direction_object=Vector3d{1,0,0};sample.capture_timestamp=object.observation.timestamp;flat_surface.points.push_back(sample);}
  const auto flat_proposal=propose_grasp_candidates(flat_surface,object,gripper,cfg,time(),.04);
  CHECK(flat_proposal.candidates.empty());CHECK(flat_proposal.status==GraspSearchStatus::kNoCandidates);
  GraspSurfaceSnapshot same_facing; same_facing.key=object.context.key;same_facing.revision=9;
  same_facing.object_frame="object_1";same_facing.settings_revision=cfg.revision();
  for(const auto & point:cloud){if(std::abs(std::abs(point.x_m)-.025)>1e-8)continue;
    same_facing.points.push_back({point,Vector3d{1,0,0},object.observation.timestamp});}
  const auto same_facing_proposal=propose_grasp_candidates(same_facing,object,gripper,cfg,time(),.04);
  CHECK(same_facing_proposal.candidates.empty());
  auto temporary=object;temporary.phase=TrackPhase::kTemporarilyLost;
  CHECK(!track_is_usable(temporary,cfg,time()));
  CHECK(track_is_usable(temporary,cfg,time(),true));
  const auto predicted_proposal=propose_grasp_candidates(surface,temporary,gripper,cfg,time(),.04);
  CHECK(!predicted_proposal.candidates.empty());
  CHECK(predicted_proposal.surface_revision==surface.revision);
  auto beyond_prediction=time();
  beyond_prediction.source_now=temporary.observation.timestamp+*seconds_to_duration(cfg.get().tracking.max_prediction_horizon_sec+.01);
  CHECK(propose_grasp_candidates(surface,temporary,gripper,cfg,beyond_prediction,.04).candidates.empty());
  auto stale=object;stale.observation.timestamp=Nanoseconds{0};
  const auto stale_proposal=propose_grasp_candidates(surface,stale,gripper,cfg,time(),.04);
  CHECK(stale_proposal.candidates.empty());CHECK(stale_proposal.reason.find("track")!=std::string::npos);
  auto shifted=object;shifted.observation.pose_world.position={.12,.08,.1};shifted.observation.pose_world.orientation=yaw.orientation;
  const auto moved=propose_grasp_candidates(surface,shifted,gripper,cfg,time(),.04);CHECK(!moved.candidates.empty());
  const auto expected_world=compose_pose(shifted.observation.pose_world,moved.candidates.front().tcp_in_object);
  CHECK(expected_world.has_value());CHECK(pose_near(moved.candidates.front().tcp_world,*expected_world,1e-8,1e-8));
  auto wide_cube=cube;for(auto & sample:wide_cube.points){sample.position_object_m.x_m*=3;sample.position_object_m.y_m*=3;sample.position_object_m.z_m*=3;}
  CHECK(propose_grasp_candidates(wide_cube,object,gripper,cfg,time(),.04).candidates.empty());
  const auto unidentified_gripper=configured_gripper_geometry(cfg,"");
  const auto unidentified=propose_grasp_candidates(surface,object,unidentified_gripper,cfg,time(),.04);
  CHECK(unidentified.candidates.empty());CHECK(unidentified.reason.find("collision geometry digest")!=std::string::npos);
  auto tilted=data;tilted.grasp.finger_center_tcp={0,0,.012};const auto tilted_cfg=settings(tilted);
  const auto tilted_gripper=configured_gripper_geometry(tilted_cfg,gripper.collision_geometry_digest);
  const auto tilted_proposal=propose_grasp_candidates(surface,object,tilted_gripper,tilted_cfg,time(),.04);CHECK(!tilted_proposal.candidates.empty());
  for(const auto & candidate:tilted_proposal.candidates){
    Pose3d rotation;rotation.orientation=candidate.tcp_in_object.orientation;
    const auto closing_axis=transform_point(rotation,tilted_gripper.closing_axis_tcp);
    const auto tool_approach=transform_point(rotation,tilted_gripper.approach_axis_tcp);
    const auto contact_axis=Vector3d{candidate.right_contact.center_object_m.x_m-candidate.left_contact.center_object_m.x_m,
      candidate.right_contact.center_object_m.y_m-candidate.left_contact.center_object_m.y_m,
      candidate.right_contact.center_object_m.z_m-candidate.left_contact.center_object_m.z_m};
    CHECK(std::abs(dot3(closing_axis,candidate.approach_direction_object))<1e-5);
    CHECK(std::abs(dot3(closing_axis,contact_axis)/vector_norm(contact_axis))>.8);
    CHECK(std::abs(dot3(tool_approach,candidate.approach_direction_object)-1.)<1e-5);
    const auto entry_center=transform_point(candidate.pregrasp_in_object,tilted.grasp.finger_center_tcp);
    for(const auto & sample:surface.points){
      const auto & p=sample.position_object_m;const auto & a=candidate.approach_direction_object;
      CHECK((p.x_m-entry_center.x_m)*a.x_m+(p.y_m-entry_center.y_m)*a.y_m+(p.z_m-entry_center.z_m)*a.z_m>=.04-1e-8);
    }
  }
  Pose3d invalid_pose;invalid_pose.orientation={0,0,0,0};CHECK(!make_grasp_preapproach_pose(invalid_pose,Vector3d{0,0,1},.04));
  CHECK(!make_grasp_preapproach_pose(Pose3d{},Vector3d{},.04));
  CHECK(!make_grasp_preapproach_pose(Pose3d{},Vector3d{0,0,1},0));
  Pose3d aligned,behind;behind.position.y_m=-.08;
  const Vector3d center_offset{0,0,.1};
  CHECK(grasp_center_aligned(aligned,aligned,center_offset,.02,.05,.008,.04));
  CHECK(!grasp_center_aligned(behind,aligned,center_offset,.02,.05,.008,.04));
  CHECK(!grasp_center_aligned(aligned,aligned,center_offset,.2,.05,.008,.04));
  CHECK(!grasp_center_aligned(aligned,aligned,center_offset,kUnset,.05,.008,.04));
  auto above_contact=aligned;above_contact.position.z_m+=.04;
  CHECK(!grasp_descent_height_reached(above_contact,aligned,center_offset,.008));
  above_contact.position.z_m=aligned.position.z_m+.007;
  above_contact.position.x_m+=.1; // Do not reintroduce an XY alignment gate.
  CHECK(grasp_descent_height_reached(above_contact,aligned,center_offset,.008));
  above_contact.position.z_m=aligned.position.z_m-.001;
  CHECK(grasp_descent_height_reached(above_contact,aligned,center_offset,.008));
  CHECK(!grasp_descent_height_reached(above_contact,aligned,center_offset,-1.));
  // Equal TCP origins can still put the finger midpoint outside the tolerance.
  auto rotated_center=aligned;rotated_center.orientation={0,std::sin(.05),0,std::cos(.05)};
  CHECK(!grasp_center_aligned(rotated_center,aligned,center_offset,0,.05,.008,.2));
  auto moving_aligned=aligned;moving_aligned.position.y_m=.3;
  CHECK(grasp_center_aligned(moving_aligned,moving_aligned,center_offset,.02,.05,.008,.04));
  {
    Pose3d initial,object_pose,pregrasp,contact;
    initial.position={0,0,.5};pregrasp.position={.1,0,.2};contact.position={.1,0,.16};
    const Vector3d moving{0,.06,0};
    const auto recipe=make_grasp_motion_reference(initial,object_pose,moving,time().source_now,pregrasp,contact,.04,1.,4.,.4,.5);
    CHECK(recipe.has_value());
    // The configured relative speed must bound descent/probe velocity even
    // while the object contributes conveyor feedforward in another axis.
    const auto bounded=make_grasp_motion_reference(initial,object_pose,moving,time().source_now,pregrasp,contact,.04,.11,4.,.4,.5);
    CHECK(bounded.has_value());CHECK(bounded->descent_sec>=1.5-1e-8);
    const auto descending=sample_grasp_motion_reference(*bounded,time().source_now+*seconds_to_duration(3),
      bounded->descent_start()+bounded->descent_sec*.5,1.);
    CHECK(descending.has_value());CHECK(std::abs(descending->linear_world.z_m)<=.05+1e-8);
    const auto sample=[&](double source,double cursor,double rate=1.){return sample_grasp_motion_reference(*recipe,time().source_now+*seconds_to_duration(source),cursor,rate);};
    CHECK(pose_near(sample(0,0)->tcp_world,initial,1e-8,1e-8));
    CHECK(vector_norm(sample(0,0)->linear_world)<1e-8);
    // Pausing the path clock never pauses conveyor compensation.
    const auto a=sample(2,recipe->descent_start(),0),b=sample(3,recipe->descent_start(),0);
    CHECK(std::abs(b->tcp_world.position.y_m-a->tcp_world.position.y_m-.06)<1e-8);
    CHECK(std::abs(b->linear_world.y_m-.06)<1e-8);
    CHECK(std::abs(b->linear_world.z_m)<1e-8);
    // Join velocities match on both sides of every section boundary.
    for(double join:{recipe->approach_sec,recipe->descent_start(),recipe->contact_start()}){
      const auto before=sample(2,join-1e-6),after=sample(2,join+1e-6);
      CHECK(vector_norm({before->linear_world.x_m-after->linear_world.x_m,before->linear_world.y_m-after->linear_world.y_m,before->linear_world.z_m-after->linear_world.z_m})<1e-5);
    }
    // Probe tiny offsets around all quintic joins, including fractions whose
    // polynomial evaluation rounds above one. Valid paths must stay valid.
    for(double join:{recipe->approach_sec,recipe->descent_start(),recipe->contact_start(),recipe->lift_end()}){
      for(double epsilon:{-1e-6,-1e-7,-1e-9,0.,1e-9}){
        const auto endpoint_sample=sample(3,join+epsilon);
        CHECK(endpoint_sample.has_value());CHECK(is_valid_pose(endpoint_sample->tcp_world));
      }
    }
    const auto raised=sample(3,recipe->lift_end());
    CHECK(std::abs(raised->tcp_world.position.z_m-.2)<1e-8);
    const auto shifted=sample_grasp_motion_reference(*recipe,time().source_now+*seconds_to_duration(3),recipe->descent_start(),1.,{.02,0,0},{});
    CHECK(std::abs(shifted->tcp_world.position.x_m-b->tcp_world.position.x_m-.02)<1e-8);
    // Once Picking stops new visual rebases, the existing ramp settles
    // without a pose/velocity reset. A frozen closure cursor still follows
    // the conveyor after the visual offset endpoint has become constant.
    GraspOffsetRamp committed_offset;
    committed_offset.rebase(0.,{.02,-.01,0.},{},.2);
    const auto committed_sample=[&](double t){return sample_grasp_motion_reference(*recipe,
      recipe->source_start+*seconds_to_duration(t),recipe->contact_start(),0.,
      committed_offset.position(t),committed_offset.velocity(t));};
    const auto settling=committed_sample(.2-1e-7),settled=committed_sample(.2+1e-7);
    CHECK(settling.has_value());CHECK(settled.has_value());
    CHECK(pose_near(settling->tcp_world,settled->tcp_world,1e-6,1e-6));
    CHECK(vector_norm({settling->linear_world.x_m-settled->linear_world.x_m,
      settling->linear_world.y_m-settled->linear_world.y_m,
      settling->linear_world.z_m-settled->linear_world.z_m})<1e-5);
    const auto later=committed_sample(.3);
    CHECK(std::abs(later->tcp_world.position.x_m-.12)<1e-8);
    CHECK(std::abs(later->tcp_world.position.y_m-settled->tcp_world.position.y_m-.06*(.3-.2000001))<1e-8);
    CHECK(std::abs(later->linear_world.x_m)<1e-8);
    CHECK(std::abs(later->linear_world.y_m-.06)<1e-8);
    CHECK(std::abs(later->linear_world.z_m)<1e-8);
    // After lift clearance, braking the nominal feedforward must preserve
    // position/velocity at the join and then stop planar drift entirely.
    const auto before_brake=committed_sample(.3);
    const auto p=committed_offset.position(.3),v=committed_offset.velocity(.3);
    const Vector3d exit_velocity{-moving.x_m,-moving.y_m,-moving.z_m};
    committed_offset.rebase(.3,{p.x_m+.1*(v.x_m+exit_velocity.x_m),
      p.y_m+.1*(v.y_m+exit_velocity.y_m),p.z_m+.1*(v.z_m+exit_velocity.z_m)},exit_velocity,.2);
    const auto brake_start=committed_sample(.3),brake_end=committed_sample(.5),after_brake=committed_sample(.8);
    CHECK(pose_near(before_brake->tcp_world,brake_start->tcp_world,1e-8,1e-8));
    CHECK(std::abs(before_brake->linear_world.y_m-brake_start->linear_world.y_m)<1e-8);
    CHECK(vector_norm(brake_end->linear_world)<1e-8);
    CHECK(pose_near(brake_end->tcp_world,after_brake->tcp_world,1e-8,1e-8));
    CHECK(!sample(0,-1));
  }
  Pose3d start,goal;start.position={0,0,.15};goal.position={.04,.02,.03};goal.orientation={0,0,std::sin(.25),std::cos(.25)};
  const Vector3d feed{0,.06,0};const auto reference=make_grasp_approach_reference(start,goal,time().source_now,.5,2.,feed);CHECK(reference.has_value());
  const auto first=sample_grasp_approach(*reference,object.observation.pose_world,time().source_now);CHECK(first.has_value());
  CHECK(first->progress==0.);
  CHECK(pose_near(first->tcp_world,start,1e-8,1e-8));CHECK(vector_norm(first->relative_linear_world)<1e-8);
  const auto middle_stamp=reference->source_start+*seconds_to_duration(reference->duration_sec*.5);
  const auto middle=sample_grasp_approach(*reference,object.observation.pose_world,middle_stamp);CHECK(middle.has_value());
  CHECK(middle->progress>0. && middle->progress<1.);
  CHECK(vector_norm(middle->relative_linear_world)+vector_norm(feed)<=.500001);CHECK(vector_norm(middle->relative_angular_world)<=2.000001);
  auto shifted_pose=object.observation.pose_world;shifted_pose.position.y_m=.3;
  const auto shifted_sample=sample_grasp_approach(*reference,shifted_pose,middle_stamp);CHECK(shifted_sample.has_value());CHECK(std::abs(shifted_sample->tcp_world.position.y_m-middle->tcp_world.position.y_m-.3)<1e-8);
  const auto last=sample_grasp_approach(*reference,object.observation.pose_world,reference->source_start+*seconds_to_duration(reference->duration_sec+1));CHECK(last.has_value());
  CHECK(last->progress==1.);
  const auto moving_last=sample_grasp_approach(*reference,shifted_pose,reference->source_start+*seconds_to_duration(reference->duration_sec+1));
  CHECK(moving_last && std::abs(moving_last->tcp_world.position.y_m-last->tcp_world.position.y_m-.3)<1e-8);
  CHECK(pose_near(last->tcp_world,goal,1e-8,1e-8));CHECK(vector_norm(last->relative_linear_world)<1e-8);CHECK(vector_norm(last->relative_angular_world)<1e-8);
  CHECK(!make_grasp_approach_reference(start,goal,time().source_now,.01,2.,feed));
  const auto paced=make_grasp_approach_reference(start,goal,time().source_now,.5,2.,feed,3.);
  CHECK(paced&&paced->duration_sec>=3.);
  CHECK(!make_grasp_approach_reference(start,goal,time().source_now,.5,2.,feed,-1.));
  data.runtime.operation_mode="assumed_ng_demo";data.inspection.inference.backend="assumed_ng";data.grasp.moving_intercept=true;
  const auto demo=settings(data);
  TaskDecision decision;
  auto & state=decision.state;
  state.phase=TaskPhase::kPreparingPick;state.context.key=key();state.expected_operation_id=1;
  state.context.deadline=state.cycle_deadline=*seconds_to_duration(100);
  state.grasp.phase=GraspPhase::kEmpty;state.grasp.possession=Possession::kEmpty;state.selected_track=object;
  auto preparation=task_event(TaskEventKind::kMotionFinished,decision.state,3);preparation.motion_succeeded=true;preparation.grasp_event.grasp_window_valid=true;
  CHECK(std::none_of(decision.commands.begin(),decision.commands.end(),[](const auto & command){return command.kind==CommandKind::kStopArm||command.kind==CommandKind::kStopGripper;}));
  preparation.arm.stopped=false;preparation.arm.state.motion_state=ArmMotionState::kFollowing;
  preparation.track.phase=TrackPhase::kTemporarilyLost;
  CHECK(!track_is_usable(preparation.track,demo,time()));CHECK(track_is_usable(preparation.track,demo,time(),true));
  preparation.readiness.vision_ready=false;preparation.readiness.inference_ready=false;
  const auto predicted_pick=advance_task(decision.state,preparation,demo,time());CHECK(predicted_pick.state.phase==TaskPhase::kPicking);
  preparation.arm.stopped=true;preparation.arm.state.motion_state=ArmMotionState::kIdle;
  CHECK(advance_task(decision.state,preparation,demo,time()).state.phase==TaskPhase::kPicking);
  preparation.motion_succeeded=false;
  CHECK(advance_task(decision.state,preparation,demo,time()).state.phase==TaskPhase::kRecovering);
  preparation.motion_succeeded=true;preparation.arm.stopped=false;preparation.arm.state.motion_state=ArmMotionState::kFollowing;
  CHECK(predicted_pick.state.grasp.phase==GraspPhase::kApproaching);
  CHECK(advance_task(decision.state,preparation,cfg,time()).state.phase==TaskPhase::kRecovering);
  preparation.track.observation.timestamp=Nanoseconds{0};
  auto expired_time=time();expired_time.source_now=*seconds_to_duration(10);
  CHECK(!track_is_usable(preparation.track,demo,expired_time,true));
  // A committed short motion model survives old vision without refreshing the
  // visual timestamp. It is scoped to this cycle and cannot establish Holding.
  GraspEvent predicted_event;predicted_event.key=key();
  StampedPose3d predicted_object;predicted_object.pose=object.observation.pose_world;
  predicted_object.reference_frame=data.frames.world;predicted_object.timestamp=time().source_now;
  predicted_event.motion_prediction=GraspMotionPrediction{key(),predicted_object,time().source_now};
  CHECK(committed_grasp_object_pose(predicted_event,demo,time()));
  CHECK(!committed_grasp_object_pose(predicted_event,cfg,time()));
  ++predicted_event.motion_prediction->key.cycle_id;
  CHECK(!committed_grasp_object_pose(predicted_event,demo,time()));
  --predicted_event.motion_prediction->key.cycle_id;
  auto later=time();later.source_now+=*seconds_to_duration(data.task.preparing_pick_timeout_sec+data.grasp.approach_timeout_sec+1);
  predicted_event.motion_prediction->object_world.timestamp=later.source_now;
  CHECK(!committed_grasp_object_pose(predicted_event,demo,later));
  // Descent/probe speed is independent of the unchanged alignment tolerance.
  {
    Pose3d object{{.3,0,.8},{}};Pose3d pre{{0,0,.07},{}};Pose3d contact{};
    const Vector3d belt{.03,0,0};const auto initial=*compose_pose(object,pre);
    const auto slow=make_grasp_motion_reference(initial,object,belt,Nanoseconds{0},pre,contact,.04,.08,4.,.2,.5);
    const auto fast=make_grasp_motion_reference(initial,object,belt,Nanoseconds{0},pre,contact,.04,.08,4.,.2,.5,.23,.23);
    CHECK(slow && fast);CHECK(std::abs(fast->approach_sec-slow->approach_sec)<1e-12);
    CHECK(fast->follow_sec==.5);CHECK(std::abs(fast->descent_sec*4-slow->descent_sec)<1e-12);
    CHECK(std::abs(fast->lift_sec*4-slow->lift_sec)<1e-12);
    const double mid=fast->descent_start()+fast->descent_sec*.5;
    const auto peak=sample_grasp_motion_reference(*fast,*seconds_to_duration(mid),mid);
    CHECK(peak);CHECK(vector_norm(peak->linear_world)<=.23+1e-9);
    CHECK(std::abs(peak->linear_world.x_m-.03)<1e-9);CHECK(std::abs(peak->linear_world.z_m+.20)<1e-9);
    const double lift_mid=fast->contact_start()+fast->lift_sec*.5;
    const auto lift_peak=sample_grasp_motion_reference(*fast,*seconds_to_duration(lift_mid),lift_mid);
    CHECK(lift_peak);CHECK(std::abs(lift_peak->linear_world.x_m-.03)<1e-9);
    CHECK(std::abs(lift_peak->linear_world.z_m-.20)<1e-9);
    // A frozen contact cursor still advances absolute-time conveyor feedforward.
    const double at=fast->contact_start();
    const auto a=sample_grasp_motion_reference(*fast,*seconds_to_duration(at),at);
    const auto b=sample_grasp_motion_reference(*fast,*seconds_to_duration(at+.5),at);
    CHECK(a && b);CHECK(std::abs(b->tcp_world.position.x_m-a->tcp_world.position.x_m-.015)<1e-9);
    CHECK(std::abs(b->tcp_world.position.z_m-a->tcp_world.position.z_m)<1e-9);
    for(double invalid:{0.,-.1,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
      CHECK(!make_grasp_motion_reference(initial,object,belt,Nanoseconds{0},pre,contact,.04,.08,4.,.2,.5,invalid,.23));
      CHECK(!make_grasp_motion_reference(initial,object,belt,Nanoseconds{0},pre,contact,.04,.08,4.,.2,.5,.23,invalid));
    }
  }
  // Calibrated robot geometry: orientation changes low-tip clearance.
  {
    auto fingers=gripper;fingers.fingertip_box_tcp=std::array<Vector3d,2>{Vector3d{-.010502,-.066346,-.044867},Vector3d{.010502,.066346,.0089}};
    Pose3d down{{.5,0,.80},{1,0,0,0}};
    const auto gap=fingertip_support_clearance(down,fingers,.75);CHECK(gap);CHECK(std::abs(*gap-.0411)<1e-9);
    const auto retreat=grasp_recovery_retreat(down,fingers,.75,.01,.04);CHECK(retreat);
    CHECK(std::abs(*fingertip_support_clearance(*retreat,fingers,.75)-.05)<1e-9);
    CHECK(retreat->orientation.x==down.orientation.x && retreat->position.x_m==down.position.x_m);
    Pose3d sideways=down;sideways.orientation={std::sqrt(.5),0,0,std::sqrt(.5)};
    CHECK(*fingertip_support_clearance(sideways,fingers,.75)<0);
    CHECK(!fingertip_support_clearance(down,gripper,.75));
    auto bad=fingers;(*bad.fingertip_box_tcp)[0].z_m=1;CHECK(!fingertip_support_clearance(down,bad,.75));
    CHECK(!grasp_recovery_retreat(down,fingers,.75,.01,-.1));
    auto guarded_data=data;guarded_data.grasp.fingertip_box_tcp=fingers.fingertip_box_tcp;
    guarded_data.vision.depth_workspace.support_plane_z_m=object.observation.pose_world.position.z_m+.15;
    const auto blocked=settings(guarded_data);const auto guarded_gripper=configured_gripper_geometry(blocked,gripper.collision_geometry_digest);
    CHECK(propose_top_view_grasp_candidates(surface,object,guarded_gripper,blocked,time(),.04).candidates.empty());
  }
  {
    // Reconstructed failures: the new relative path must not turn centimetres
    // of remaining gap into multi-second conveyor-chasing iterations.
    const std::array<Vector3d,3> gaps{{{-.082343721,-.002699154,.093989513},
      {-.000035186,.003261847,-.000355976},{-.002132632,.029775019,-.001426323}}};
    const std::array<double,3> seeds{{.639807,.568136,1.112412}};
    const Vector3d belt{0,.06,0};Pose3d object_pose;object_pose.position={.3,-.7,.8};
    for(std::size_t i=0;i<gaps.size();++i)for(bool following:{false,true}){
      GraspEntryState entry;entry.timestamp=time().source_now;entry.tcp_world=object_pose;
      entry.linear_world=following?belt:Vector3d{};
      Pose3d pre;pre.position=gaps[i];Pose3d contact=pre;contact.position.z_m-=.04;
      std::string failure;
      const auto ref=make_relative_grasp_motion_reference(entry,object_pose,belt,pre,contact,.04,1.,4.,.2,.5,.26,.26,seeds[i],0.,&failure);
      CHECK(ref.has_value());CHECK(failure.empty());CHECK(ref->relative_approach.has_value());
      CHECK(ref->approach_sec<2.);CHECK(ref->relative_approach->linear_bound_mps<=1.+1e-8);
      const auto sample=[&](double cursor){return sample_grasp_motion_reference(*ref,
        entry.timestamp+*seconds_to_duration(cursor),cursor);};
      const auto start_sample=sample(0),end_sample=sample(ref->approach_sec);
      CHECK(start_sample.has_value());CHECK(end_sample.has_value());
      CHECK(pose_near(start_sample->tcp_world,entry.tcp_world,1e-8,1e-8));
      CHECK(vector_norm({start_sample->linear_world.x_m-entry.linear_world.x_m,
        start_sample->linear_world.y_m-entry.linear_world.y_m,start_sample->linear_world.z_m-entry.linear_world.z_m})<1e-8);
      CHECK(vector_norm({end_sample->linear_world.x_m-belt.x_m,end_sample->linear_world.y_m-belt.y_m,end_sample->linear_world.z_m-belt.z_m})<1e-8);
      for(double join:{ref->approach_sec,ref->descent_start(),ref->contact_start(),ref->lift_end()}){
        const auto before=sample(join-1e-7),after=sample(join+1e-7);
        CHECK(before.has_value());CHECK(after.has_value());CHECK(pose_near(before->tcp_world,after->tcp_world,1e-5,1e-5));
        CHECK(vector_norm({before->linear_world.x_m-after->linear_world.x_m,
          before->linear_world.y_m-after->linear_world.y_m,before->linear_world.z_m-after->linear_world.z_m})<1e-5);
      }
      const auto closed_a=sample_grasp_motion_reference(*ref,entry.timestamp+*seconds_to_duration(3),ref->contact_start(),0.);
      const auto closed_b=sample_grasp_motion_reference(*ref,entry.timestamp+*seconds_to_duration(3.4),ref->contact_start(),0.);
      CHECK(std::abs(closed_b->tcp_world.position.y_m-closed_a->tcp_world.position.y_m-.024)<1e-8);
      CHECK(std::abs(closed_b->tcp_world.position.z_m-closed_a->tcp_world.position.z_m)<1e-8);
    }
    // Non-axis-aligned orientation/twist: independently differentiate the
    // quaternion samples and compare the reported WORLD angular velocity.
    object_pose.orientation={0,0,std::sin(.35),std::cos(.35)};
    GraspEntryState entry;entry.timestamp=time().source_now;entry.tcp_world=object_pose;
    entry.tcp_world.orientation={std::sin(.3),0,0,std::cos(.3)};
    entry.linear_world={.02,.03,-.01};entry.angular_world={.1,.2,-.1};
    Pose3d pre;pre.position={.02,.03,.1};pre.orientation={0,std::sin(.5),0,std::cos(.5)};
    Pose3d contact=pre;contact.position.z_m-=.04;
    const auto ref=make_relative_grasp_motion_reference(entry,object_pose,belt,pre,contact,.04,1.,4.,.2,.5,.26,.26,.2,.3);
    CHECK(ref.has_value());
    const auto at=[&](double t){return sample_grasp_motion_reference(*ref,entry.timestamp+*seconds_to_duration(t+.3),t);};
    const auto first=at(0);CHECK(pose_near(first->tcp_world,entry.tcp_world,1e-8,1e-8));
    CHECK(vector_norm({first->angular_world.x_m-entry.angular_world.x_m,
      first->angular_world.y_m-entry.angular_world.y_m,first->angular_world.z_m-entry.angular_world.z_m})<1e-8);
    CHECK(vector_norm({first->linear_world.x_m-entry.linear_world.x_m,
      first->linear_world.y_m-entry.linear_world.y_m,first->linear_world.z_m-entry.linear_world.z_m})<1e-8);
    const double h=1e-5;
    for(double fraction:{.1,.3,.5,.7,.9}){
      const double t=ref->approach_sec*fraction;const auto a=at(t-h),b=at(t+h),m=at(t);
      CHECK(a.has_value());CHECK(b.has_value());CHECK(m.has_value());
      const Vector3d derivative{(b->tcp_world.position.x_m-a->tcp_world.position.x_m)/(2*h),
        (b->tcp_world.position.y_m-a->tcp_world.position.y_m)/(2*h),(b->tcp_world.position.z_m-a->tcp_world.position.z_m)/(2*h)};
      CHECK(vector_norm({derivative.x_m-m->linear_world.x_m,derivative.y_m-m->linear_world.y_m,derivative.z_m-m->linear_world.z_m})<1e-5);
      const auto delta=compose_pose(b->tcp_world,*inverse_pose(a->tcp_world));auto q=delta->orientation;
      if(q.w<0)q={-q.x,-q.y,-q.z,-q.w};const double sine=std::hypot(q.x,q.y,q.z);
      const double factor=sine>1e-12?std::atan2(sine,q.w)/(sine*h):1./h;
      CHECK(vector_norm({q.x*factor-m->angular_world.x_m,q.y*factor-m->angular_world.y_m,q.z*factor-m->angular_world.z_m})<1e-5);
      CHECK(vector_norm(m->linear_world)<=1.+1e-8);CHECK(vector_norm(m->angular_world)<=4.+1e-8);
      auto effective=object_pose;effective.position.y_m+=.06*(t+.3);
      const auto local=compose_pose(*inverse_pose(effective),m->tcp_world);
      const auto & box=ref->relative_approach->position_bounds_object;
      CHECK(local->position.x_m>=box[0].x_m-1e-8&&local->position.x_m<=box[1].x_m+1e-8);
      CHECK(local->position.y_m>=box[0].y_m-1e-8&&local->position.y_m<=box[1].y_m+1e-8);
      CHECK(local->position.z_m>=box[0].z_m-1e-8&&local->position.z_m<=box[1].z_m+1e-8);
    }
    const auto angular_before=at(ref->approach_sec-1e-7),angular_after=at(ref->approach_sec+1e-7);
    CHECK(vector_norm({angular_before->angular_world.x_m-angular_after->angular_world.x_m,
      angular_before->angular_world.y_m-angular_after->angular_world.y_m,angular_before->angular_world.z_m-angular_after->angular_world.z_m})<1e-5);
    for(double speed:{0.,.03,.06}){
      Pose3d origin,nearby,near_contact;nearby.position={0,.03,0};near_contact=nearby;near_contact.position.z_m=-.04;
      GraspEntryState aligned;aligned.timestamp=entry.timestamp;aligned.linear_world={0,speed,0};
      const auto short_ref=make_relative_grasp_motion_reference(aligned,origin,{0,speed,0},nearby,near_contact,.04,1.,4.,.2,.5,speed+.2,speed+.2);
      CHECK(short_ref.has_value());CHECK(short_ref->approach_sec<.5);
      nearby.position={};near_contact=nearby;near_contact.position.z_m=-.04;
      const auto zero_gap=make_relative_grasp_motion_reference(aligned,origin,{0,speed,0},nearby,near_contact,.04,1.,4.,.2,.5,speed+.2,speed+.2);
      CHECK(zero_gap.has_value());CHECK(std::abs(zero_gap->approach_sec-.2)<1e-8);
    }
    // Lead advances ONLY object translation; it cannot descend early.
    const auto now_contact=sample_grasp_motion_reference(*ref,entry.timestamp+*seconds_to_duration(2),ref->descent_start());
    const auto lead_contact=sample_grasp_motion_reference(*ref,entry.timestamp+*seconds_to_duration(2.3),ref->descent_start());
    CHECK(std::abs(now_contact->tcp_world.position.z_m-lead_contact->tcp_world.position.z_m)<1e-8);
    CHECK(std::abs(lead_contact->tcp_world.position.y_m-now_contact->tcp_world.position.y_m-.018)<1e-8);
    for(double sign:{-1.,1.}){
      Pose3d origin,pi_pre,pi_contact;pi_pre.orientation={sign,0,0,0};pi_contact=pi_pre;pi_contact.position.z_m=-.04;
      GraspEntryState stopped;stopped.timestamp=entry.timestamp;
      const auto pi_ref=make_relative_grasp_motion_reference(stopped,origin,{},pi_pre,pi_contact,.04,1.,4.,.2,.5,.2,.2);
      CHECK(pi_ref.has_value());
      const auto end=sample_grasp_motion_reference(*pi_ref,entry.timestamp+*seconds_to_duration(pi_ref->approach_sec),pi_ref->approach_sec);
      CHECK(end.has_value());CHECK(pose_near(end->tcp_world,pi_pre,1e-8,1e-8));
    }
    const auto predicted=extrapolate_grasp_entry(entry,entry.timestamp+*seconds_to_duration(.1));
    CHECK(predicted.has_value());CHECK(std::abs(predicted->tcp_world.position.x_m-entry.tcp_world.position.x_m-.002)<1e-8);
    CHECK(!extrapolate_grasp_entry(entry,entry.timestamp-Nanoseconds{1}));
    auto invalid=entry;invalid.linear_world={2,0,0};std::string reason;
    CHECK(!make_relative_grasp_motion_reference(invalid,object_pose,belt,pre,contact,.04,1.,4.,.2,.5,.26,.26,0.,0.,&reason));CHECK(!reason.empty());
  }

});}
