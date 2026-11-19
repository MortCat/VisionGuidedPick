#include "ptc_core/state/track_state.hpp"
#include <algorithm>
#include <cmath>

namespace ptc_core
{
namespace
{
struct VelocityFit
{
  Vector3d velocity,position;
  double residual_m{kUnset},uncertainty_mps{kUnset};
  bool valid{false};
};
void clear_velocity_fit(TrackedObjectState & track)
{
  track.velocity_mps={};track.velocity_valid=false;
  track.velocity_fit_residual_m=kUnset;track.velocity_uncertainty_mps=kUnset;
  track.velocity_samples.clear();
}
VelocityFit fit_velocity(const std::vector<ObjectObservation> & samples,const TrackingConfig & config)
{
  VelocityFit fit;
  if(samples.size()<config.velocity_min_samples){return fit;}
  const double span=std::chrono::duration<double>(samples.back().timestamp-samples.front().timestamp).count();
  if(!std::isfinite(span)||span<config.velocity_min_span_sec){return fit;}
  double total_weight=0,mean_time=0;Vector3d mean_position{};
  for(const auto & sample:samples){
    if(!std::isfinite(sample.position_stddev_m)||sample.position_stddev_m<0||!is_valid_pose(sample.pose_world)){return fit;}
    const double weight=1.0/std::max(1e-8,sample.position_stddev_m*sample.position_stddev_m);
    const double relative_time=std::chrono::duration<double>(sample.timestamp-samples.back().timestamp).count();
    total_weight+=weight;mean_time+=weight*relative_time;
    mean_position.x_m+=weight*sample.pose_world.position.x_m;
    mean_position.y_m+=weight*sample.pose_world.position.y_m;
    mean_position.z_m+=weight*sample.pose_world.position.z_m;
  }
  if(!std::isfinite(total_weight)||total_weight<=0){return fit;}
  mean_time/=total_weight;mean_position.x_m/=total_weight;mean_position.y_m/=total_weight;mean_position.z_m/=total_weight;
  double time_variance=0;Vector3d covariance{};
  for(const auto & sample:samples){
    const double weight=1.0/std::max(1e-8,sample.position_stddev_m*sample.position_stddev_m);
    const double centered_time=std::chrono::duration<double>(sample.timestamp-samples.back().timestamp).count()-mean_time;
    time_variance+=weight*centered_time*centered_time;
    covariance.x_m+=weight*centered_time*(sample.pose_world.position.x_m-mean_position.x_m);
    covariance.y_m+=weight*centered_time*(sample.pose_world.position.y_m-mean_position.y_m);
    covariance.z_m+=weight*centered_time*(sample.pose_world.position.z_m-mean_position.z_m);
  }
  if(!std::isfinite(time_variance)||time_variance<=1e-12){return fit;}
  fit.velocity={covariance.x_m/time_variance,covariance.y_m/time_variance,covariance.z_m/time_variance};
  // Fitted position at the latest measured timestamp; samples remain raw measurements.
  fit.position={mean_position.x_m-fit.velocity.x_m*mean_time,
    mean_position.y_m-fit.velocity.y_m*mean_time,mean_position.z_m-fit.velocity.z_m*mean_time};
  double residual_sum=0;
  for(const auto & sample:samples){
    const double weight=1.0/std::max(1e-8,sample.position_stddev_m*sample.position_stddev_m);
    const double centered_time=std::chrono::duration<double>(sample.timestamp-samples.back().timestamp).count()-mean_time;
    const double ex=sample.pose_world.position.x_m-mean_position.x_m-fit.velocity.x_m*centered_time;
    const double ey=sample.pose_world.position.y_m-mean_position.y_m-fit.velocity.y_m*centered_time;
    const double ez=sample.pose_world.position.z_m-mean_position.z_m-fit.velocity.z_m*centered_time;
    residual_sum+=weight*(ex*ex+ey*ey+ez*ez);
  }
  fit.residual_m=std::sqrt(std::max(0.0,residual_sum/total_weight));
  const double time_spread=std::sqrt(time_variance/total_weight);
  if(time_spread<=1e-9){return fit;}
  // Combine the position-measurement contribution and unexplained fit error.
  // This is an engineering quality bound, not a calibrated covariance.
  const double measurement_velocity_uncertainty=std::sqrt(3.0/time_variance);
  const double model_velocity_uncertainty=fit.residual_m/time_spread;
  fit.uncertainty_mps=std::hypot(measurement_velocity_uncertainty,model_velocity_uncertainty);
  fit.valid=std::isfinite(vector_norm(fit.velocity))&&std::isfinite(fit.residual_m)&&
    std::isfinite(fit.uncertainty_mps)&&fit.residual_m<=config.max_velocity_fit_residual_m&&
    fit.uncertainty_mps<=config.max_velocity_uncertainty_mps;
  return fit;
}
}
bool track_is_usable(const TrackedObjectState & s, const SettingsSnapshot & snapshot, const TimeContext & t, bool prediction)
{
  const auto & c = snapshot.get();
  if (c.schema_version==2 && (!s.observation.yaw_valid || !std::isfinite(s.observation.yaw_stddev_rad) ||
    s.observation.yaw_stddev_rad<0 || s.observation.yaw_stddev_rad>c.tracking.max_yaw_stddev_rad)) {return false;}
  if (!valid_time(t) || t.paused || s.context.key.epoch != t.epoch || s.context.key.settings_revision != snapshot.revision() ||
    !s.observation.valid || !s.velocity_valid || !is_valid_pose(s.observation.pose_world) ||
    !std::isfinite(vector_norm(s.velocity_mps)) || !std::isfinite(s.velocity_fit_residual_m) ||
    s.velocity_fit_residual_m<0 || s.velocity_fit_residual_m>c.tracking.max_velocity_fit_residual_m ||
    !std::isfinite(s.velocity_uncertainty_mps) || s.velocity_uncertainty_mps<0 ||
    s.velocity_uncertainty_mps>c.tracking.max_velocity_uncertainty_mps || s.observation.reference_frame != c.frames.world ||
    !std::isfinite(s.observation.position_stddev_m) || s.observation.position_stddev_m < 0 || s.observation.position_stddev_m > c.tracking.max_position_stddev_m) {return false;}
  if (s.phase == TrackPhase::kTracking) {
    if (source_is_fresh(s.observation.timestamp, t, c.tracking.observation_max_age_sec, c.time.sensor_future_tolerance_sec)) {return true;}
    // Missing-frame delivery can lag the control timer. Prediction has the
    // same bounded age for a stale Tracking state as for TemporarilyLost;
    // capture and grasp callers still require a fresh observation.
    return prediction && c.tracking.max_prediction_horizon_sec > 0 &&
      source_is_fresh(s.observation.timestamp, t, c.tracking.max_prediction_horizon_sec, c.time.sensor_future_tolerance_sec);
  }
  return prediction && s.phase == TrackPhase::kTemporarilyLost && c.tracking.max_prediction_horizon_sec > 0 &&
    source_is_fresh(s.observation.timestamp, t, c.tracking.max_prediction_horizon_sec, c.time.sensor_future_tolerance_sec);
}
std::optional<double> predicted_position_uncertainty_m(const TrackedObjectState & s,const SettingsSnapshot & snapshot,
  const TimeContext & t,double horizon)
{
  if(!track_is_usable(s,snapshot,t,true)||!std::isfinite(horizon)||horizon<0||
    horizon>snapshot.get().tracking.max_prediction_horizon_sec){return std::nullopt;}
  const double age=t.source_now>s.observation.timestamp?
    std::chrono::duration<double>(t.source_now-s.observation.timestamp).count():0;
  if(age+horizon>snapshot.get().tracking.max_prediction_horizon_sec){return std::nullopt;}
  // A conservative additive bound, not a statistical covariance.
  const double uncertainty=s.observation.position_stddev_m+s.velocity_uncertainty_mps*(age+horizon);
  return std::isfinite(uncertainty)?std::optional<double>{uncertainty}:std::nullopt;
}
std::optional<Pose3d> predict_object_pose(const TrackedObjectState & s, const SettingsSnapshot & snapshot,
  const TimeContext & t, double horizon)
{
  if (!track_is_usable(s, snapshot, t, true) || !std::isfinite(horizon) || horizon < 0 ||
    horizon > snapshot.get().tracking.max_prediction_horizon_sec) {return std::nullopt;}
  const double age = t.source_now > s.observation.timestamp ?
    std::chrono::duration<double>(t.source_now - s.observation.timestamp).count() : 0;
  if (age + horizon > snapshot.get().tracking.max_prediction_horizon_sec) {return std::nullopt;}
  auto p = s.observation.pose_world;
  p.position.x_m += s.velocity_mps.x_m * (age+horizon);
  p.position.y_m += s.velocity_mps.y_m * (age+horizon);
  p.position.z_m += s.velocity_mps.z_m * (age+horizon);
  return is_valid_pose(p) ? std::optional<Pose3d>(p) : std::nullopt;
}
std::optional<Pose3d> track_pose_at_observation(const TrackedObjectState & s,const SettingsSnapshot & snapshot,
  const TimeContext & time,Nanoseconds capture_timestamp)
{
  const auto & c=snapshot.get();
  if(!valid_time(time)||time.paused||s.context.key.epoch!=time.epoch||s.context.key.settings_revision!=snapshot.revision()||
    s.phase==TrackPhase::kLost||s.phase==TrackPhase::kRetired||!s.confirmations||!s.observation.valid||
    !is_valid_pose(s.observation.pose_world)||s.observation.reference_frame!=c.frames.world||
    !std::isfinite(s.observation.position_stddev_m)||s.observation.position_stddev_m<0||
    s.observation.position_stddev_m>c.tracking.max_position_stddev_m||
    !source_is_fresh(capture_timestamp,time,c.tracking.observation_max_age_sec,c.time.sensor_future_tolerance_sec)){
    return std::nullopt;
  }
  if(c.schema_version==2&&(!s.observation.yaw_valid||!std::isfinite(s.observation.yaw_stddev_rad)||
    s.observation.yaw_stddev_rad<0||s.observation.yaw_stddev_rad>c.tracking.max_yaw_stddev_rad)){return std::nullopt;}
  const double elapsed=std::chrono::duration<double>(capture_timestamp-s.observation.timestamp).count();
  if(!std::isfinite(elapsed)||std::abs(elapsed)>c.tracking.temporary_loss_timeout_sec){return std::nullopt;}
  auto pose=s.observation.pose_world;
  if(s.velocity_valid){
    if(!std::isfinite(vector_norm(s.velocity_mps))||!std::isfinite(s.velocity_fit_residual_m)||
      s.velocity_fit_residual_m<0||s.velocity_fit_residual_m>c.tracking.max_velocity_fit_residual_m||
      !std::isfinite(s.velocity_uncertainty_mps)||s.velocity_uncertainty_mps<0||
      s.velocity_uncertainty_mps>c.tracking.max_velocity_uncertainty_mps){return std::nullopt;}
    // A processed wrist frame may precede a newer source-camera update.
    // Transform its cloud at capture time without applying an out-of-order track event.
    if(elapsed<0&&-elapsed>c.tracking.max_prediction_horizon_sec){return std::nullopt;}
    if(std::abs(elapsed)<=c.tracking.max_prediction_horizon_sec){
      pose.position.x_m+=s.velocity_mps.x_m*elapsed;
      pose.position.y_m+=s.velocity_mps.y_m*elapsed;
      pose.position.z_m+=s.velocity_mps.z_m*elapsed;
    }
  }else if(s.phase==TrackPhase::kTracking){return std::nullopt;}
  return is_valid_pose(pose)?std::optional<Pose3d>{pose}:std::nullopt;
}
TrackDecision advance_track(const TrackedObjectState & original, const TrackEvent & e,
  const SettingsSnapshot & snapshot, const TimeContext & t)
{
  TrackDecision d{original, {}, {}}; auto & s = d.state; const auto & c = snapshot.get();
  const auto finish = [&]() {
      d.record = record_decision(s.context, "track", static_cast<int>(original.phase), static_cast<int>(s.phase),
        e.key, t, d.status, c.diagnostics.transition_history_capacity); return d;
    };
  const auto reject = [&](const char * reason) {d.status = {Disposition::kRejected, reason}; return finish();};
  if (static_cast<int>(e.kind) < 0 || static_cast<int>(s.phase) < 0 || e.kind >= TrackEventKind::kCount || s.phase >= TrackPhase::kCount) {return reject("invalid event or phase");}
  if (!valid_time(t) || t.monotonic_now < s.context.last_monotonic) {return reject("invalid or backward monotonic time");}
  if (e.kind == TrackEventKind::kClockReset || (s.context.key.epoch && s.context.key.epoch != t.epoch)) {
    s.phase = TrackPhase::kLost; clear_velocity_fit(s);
    d.status = {Disposition::kApplied, "clock epoch invalidated track"}; return finish();
  }
  if (!valid_observation_key(e.key, t, snapshot.revision())) {return reject("invalid track identity/revision");}
  if (s.context.key.track_id && (!same_track_identity(e.key, s.context.key) || e.key.sequence <= s.context.key.sequence)) {
    return reject("mismatched or duplicate/out-of-order track event");
  }
  if (s.phase == TrackPhase::kLost || s.phase == TrackPhase::kRetired) {return finish();}
  if (t.paused) {s.phase = TrackPhase::kLost; clear_velocity_fit(s); d.status = {Disposition::kApplied, "paused source"}; return finish();}
  if (!s.context.key.track_id) {s.context.key = e.key;}
  s.context.last_monotonic = t.monotonic_now;
  if (s.confirmations && !source_is_fresh(s.observation.timestamp, t,
      c.tracking.temporary_loss_timeout_sec, c.time.sensor_future_tolerance_sec)) {
    s.phase = TrackPhase::kLost; clear_velocity_fit(s);
    d.status = {Disposition::kApplied, "loss deadline reached; old track cannot revive"}; return finish();
  }
  if (e.kind == TrackEventKind::kRetire) {
    s.phase = TrackPhase::kRetired; clear_velocity_fit(s); d.status = {Disposition::kApplied, "track retired"};
  } else if (e.kind == TrackEventKind::kObservation) {
    const auto & o = e.observation;
    if (c.schema_version==2 && (!o.yaw_valid || !std::isfinite(o.yaw_stddev_rad) || o.yaw_stddev_rad<0 ||
      o.yaw_stddev_rad>c.tracking.max_yaw_stddev_rad)) {return reject("upright yaw is not identifiable");}
    if (!o.valid || !is_valid_pose(o.pose_world) || o.reference_frame != c.frames.world ||
      !std::isfinite(o.position_stddev_m) || o.position_stddev_m < 0 ||
      o.position_stddev_m > c.tracking.max_position_stddev_m ||
      !source_is_fresh(o.timestamp, t, c.tracking.observation_max_age_sec, c.time.sensor_future_tolerance_sec)) {
      return reject("invalid/stale world observation");
    }
    if (s.confirmations) {
      if (o.timestamp <= s.observation.timestamp) {return reject("non-increasing observation timestamp");}
      const auto & a = s.observation.pose_world.position; const auto & b = o.pose_world.position;
      const Vector3d delta{b.x_m-a.x_m, b.y_m-a.y_m, b.z_m-a.z_m};
      if (vector_norm(delta) > c.tracking.association_distance_m) {return reject("association distance exceeded");}

    }
    if(s.velocity_samples.empty()&&s.confirmations){s.velocity_samples.push_back(s.observation);}
    s.velocity_samples.push_back(o);
    while(s.velocity_samples.size()>c.tracking.velocity_max_samples ||
      (s.velocity_samples.size()>1 && std::chrono::duration<double>(o.timestamp-s.velocity_samples.front().timestamp).count()>c.tracking.velocity_window_sec)){
      s.velocity_samples.erase(s.velocity_samples.begin());}
    const auto velocity=fit_velocity(s.velocity_samples,c.tracking);
    s.velocity_mps=velocity.velocity;s.velocity_fit_residual_m=velocity.residual_m;
    s.velocity_uncertainty_mps=velocity.uncertainty_mps;s.velocity_valid=velocity.valid;
    s.observation = o;
    // The existing conveyor motion model has translation and no angular velocity.
    // Keep its confirmed axes stable; raw detector poses remain in velocity_samples.
    if(original.confirmations>=c.tracking.min_confirmed_observations){
      s.observation.pose_world.orientation=original.observation.pose_world.orientation;
      s.observation.yaw_stddev_rad=original.observation.yaw_stddev_rad;
      s.observation.yaw_valid=original.observation.yaw_valid;}
    if(velocity.valid)s.observation.pose_world.position=velocity.position;
    if (s.confirmations < c.tracking.min_confirmed_observations) {++s.confirmations;}
    s.phase = s.confirmations >= c.tracking.min_confirmed_observations && s.velocity_valid ? TrackPhase::kTracking : TrackPhase::kCandidate;
    d.status = {Disposition::kApplied, "world observation accepted"};
  } else if (e.kind == TrackEventKind::kMissing || e.kind == TrackEventKind::kTick) {
    if (!s.confirmations || !source_is_fresh(s.observation.timestamp, t,
        c.tracking.temporary_loss_timeout_sec, c.time.sensor_future_tolerance_sec)) {
      s.phase = TrackPhase::kLost; clear_velocity_fit(s);
    } else if (e.kind == TrackEventKind::kMissing || !source_is_fresh(s.observation.timestamp, t,
        c.tracking.observation_max_age_sec, c.time.sensor_future_tolerance_sec)) {
      s.phase = TrackPhase::kTemporarilyLost;
    }
    d.status = {Disposition::kApplied, "visibility/age evaluated"};
  }
  if (d.status.disposition == Disposition::kApplied) {s.context.key.sequence = e.key.sequence;}
  return finish();
}
}  // namespace ptc_core
