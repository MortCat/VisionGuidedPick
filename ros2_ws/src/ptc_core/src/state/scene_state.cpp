#include "ptc_core/state/scene_state.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace ptc_core
{
bool scene_ready(const SceneState & s, const SettingsSnapshot & snapshot, const TimeContext & t, std::uint64_t predicted_track_id)
{
  if (!s.coherent || s.static_geometry.empty() || s.context.inhibited || s.attachment == AttachmentPhase::kUnknown ||
    s.attachment == AttachmentPhase::kAttachPending || s.attachment == AttachmentPhase::kDetachPending ||
    s.context.key.epoch != t.epoch || s.context.key.settings_revision != snapshot.revision()) {return false;}
  for (const auto & o : s.dynamic_objects) {
    if (o.key.track_id == s.attached_track_id && s.attachment == AttachmentPhase::kAttached) {continue;}
    const auto & c=snapshot.get();
    // Physical moving-pick commissioning accepts any rabbit. While a tracked
    // motion is admitted, rabbit image ages must not cancel its output. The
    // existing operation deadline, fresh robot feedback, coherent scene and
    // collision handling remain; ordinary admission/formal mode stay fresh.
    // A confirmed attachment retains this policy during bin transfer, when
    // the wrist camera naturally leaves the conveyor.
    const bool carrying=s.attachment==AttachmentPhase::kAttached && s.attached_track_id;
    if((predicted_track_id || carrying) && c.runtime.operation_mode=="assumed_ng_demo" &&
      c.grasp.moving_intercept && o.key.track_id)continue;
    const double max_age=o.key.track_id==predicted_track_id?std::max(c.scene.dynamic_object_max_age_sec,c.tracking.max_prediction_horizon_sec):c.scene.dynamic_object_max_age_sec;
    if (!source_is_fresh(o.observation.timestamp,t,max_age,
      snapshot.get().time.sensor_future_tolerance_sec)) {return false;}
  }
  return true;
}
SceneDecision advance_scene(const SceneState & original, const SceneEvent & e,
  const SettingsSnapshot & snapshot, const TimeContext & t)
{
  SceneDecision d{original, {}, {}, {}}; auto & s = d.state; const auto & c = snapshot.get();
  const auto finish = [&]() {
      if (s.context.inhibited) {s.coherent = false;}
      d.record = record_decision(s.context,"scene",static_cast<int>(original.attachment),static_cast<int>(s.attachment),
        e.key,t,d.status,c.diagnostics.transition_history_capacity); return d;
    };
  const auto reject = [&](const char * reason) {d.status = {Disposition::kRejected,reason}; return finish();};
  const auto change = [&](CommandKind kind) {
      s.coherent = false;
      if (s.revision == std::numeric_limits<std::uint64_t>::max()) {s.context.inhibited = true; return;}
      ++s.revision;
      s.context.deadline = deadline_after(t.monotonic_now,c.motion.planning_timeout_sec).value_or(t.monotonic_now);
      emit_command(s.context,d.commands,kind,"scene");
      if (!d.commands.empty()) {s.pending_command_id = d.commands.back().command_id; d.commands.back().scene_revision = s.revision;}
    };
  if (static_cast<int>(e.kind) < 0 || e.kind >= SceneEventKind::kCount || s.attachment < AttachmentPhase::kDetached ||
    s.attachment > AttachmentPhase::kUnknown || !valid_time(t) || t.monotonic_now < s.context.last_monotonic) {return reject("invalid scene event/time/attachment");}
  if (e.kind == SceneEventKind::kResetRequested) {
    if (!valid_event(e.key,t,snapshot.revision()) || !arm_snapshot_ready(e.arm,snapshot,t) || !e.arm.stopped ||
      !gripper_snapshot_ready(e.gripper,snapshot,t) || !e.gripper.stopped || !e.gripper.empty_confirmed) {return reject("scene reset requires manual measured stops and Empty");}
    s.context.key = e.key; s.context.last_monotonic = t.monotonic_now; s.context.inhibited = false;
    s.attachment = AttachmentPhase::kDetached; s.attached_track_id = s.pending_command_id = 0;
    s.dynamic_objects.clear(); s.coherent = false; d.status = {Disposition::kApplied,"manual scene reset; static reload/acknowledgment required"}; return finish();
  }
  // A measured physical release can reconcile a failed/unknown planning
  // attachment. Use the existing detach command and require its backend ACK;
  // never reset an attached model merely because opening was requested.
  if(e.kind==SceneEventKind::kDetachRequested && s.attachment!=AttachmentPhase::kDetached && s.attached_track_id &&
    valid_event(e.key,t,snapshot.revision()) && e.key.track_id==s.attached_track_id &&
    arm_snapshot_ready(e.arm,snapshot,t) && e.arm.stopped &&
    gripper_snapshot_ready(e.gripper,snapshot,t) && e.gripper.stopped && e.gripper.empty_confirmed){
    s.context.key=e.key;s.context.last_monotonic=t.monotonic_now;s.context.inhibited=false;
    // Recovery uploads only static geometry plus the removal. Stale free
    // tracks must not veto cleanup; fresh vision repopulates them after reset.
    s.dynamic_objects.clear();
    s.attachment=AttachmentPhase::kDetachPending;change(CommandKind::kDetachObject);
    d.status={Disposition::kApplied,"measured Empty recovery detach requested; backend acknowledgment required"};return finish();
  }
  // Reconcile an uncertain, non-empty failed pickup without inventing new
  // visual evidence. The supplied object is the original measured snapshot;
  // a fresh backend ACK is still required before any motion.
  if(e.kind==SceneEventKind::kAttachRequested && e.object.key.track_id &&
    e.object.key.track_id==e.key.track_id && c.runtime.operation_mode=="assumed_ng_demo" &&
    c.grasp.moving_intercept && valid_event(e.key,t,snapshot.revision()) &&
    arm_snapshot_ready(e.arm,snapshot,t) && e.arm.stopped && gripper_snapshot_ready(e.gripper,snapshot,t) &&
    !e.gripper.empty_confirmed && !s.static_geometry.empty() && e.object.observation.valid &&
    e.object.observation.reference_frame==c.frames.world && is_valid_pose(e.object.observation.pose_world)){
    if(s.attached_track_id && s.attached_track_id!=e.key.track_id)return reject("recovery attachment identity conflicts with existing object");
    s.dynamic_objects.erase(std::remove_if(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & o){
      return o.key.track_id!=e.key.track_id && !source_is_fresh(o.observation.timestamp,t,c.scene.dynamic_object_max_age_sec,c.time.sensor_future_tolerance_sec);
    }),s.dynamic_objects.end());
    auto found=std::find_if(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & o){return o.key.track_id==e.key.track_id;});
    if(found==s.dynamic_objects.end())s.dynamic_objects.push_back(e.object);
    s.context.key=e.key;s.context.last_monotonic=t.monotonic_now;s.context.inhibited=false;
    s.attachment=AttachmentPhase::kAttachPending;s.attached_track_id=e.key.track_id;s.pending_command_id=0;
    change(CommandKind::kAttachObject);
    d.status={Disposition::kApplied,"conservative recovery attachment requested; backend acknowledgment required"};return finish();
  }
  if (s.pending_command_id && expired(s.context.deadline,t)) {
    s.context.inhibited = true; s.coherent = false;
    if (s.attachment != AttachmentPhase::kDetached) {s.attachment = AttachmentPhase::kUnknown;}
    d.status = {Disposition::kApplied,"scene acknowledgment deadline unconfirmed"}; return finish();
  }
  if (e.kind == SceneEventKind::kClockReset || (s.context.key.epoch && s.context.key.epoch != t.epoch)) {
    s.dynamic_objects.clear(); s.coherent = false;
    if (s.attachment != AttachmentPhase::kDetached) {s.attachment = AttachmentPhase::kUnknown;}
    s.context.inhibited = true; d.status = {Disposition::kApplied,"epoch reset invalidates dynamic scene; reload required"}; return finish();
  }
  if (!valid_event(e.key,t,snapshot.revision())) {return reject("invalid scene identity/revision");}
  if (s.context.inhibited) {return reject("scene fault requires manual reset");}
  // Scene-wide receive sequence is assigned by its single owner, not by each vision publisher.
  if (s.context.key.operation_id && e.key.sequence <= s.context.key.sequence) {return reject("duplicate/out-of-order scene event");}
  s.context.key = e.key; s.context.last_monotonic = t.monotonic_now; s.expired_track_ids.clear();
  if (e.kind == SceneEventKind::kStaticLoaded) {
    if (s.attachment != AttachmentPhase::kDetached || !s.dynamic_objects.empty()) {return reject("static reload requires cleared, detached scene");}
    std::set<std::string> ids;
    if (e.static_geometry.empty()) {return reject("empty static geometry");}
    for (const auto & g : e.static_geometry) {
      if (g.id.empty() || !ids.insert(g.id).second || g.reference_frame != c.frames.world || !is_valid_pose(g.pose) ||
        !std::isfinite(vector_norm(g.dimensions_m)) ||
        (g.kind == GeometryKind::kBox && (g.dimensions_m.x_m <= 0 || g.dimensions_m.y_m <= 0 || g.dimensions_m.z_m <= 0)) ||
        (g.kind == GeometryKind::kSphere && g.dimensions_m.x_m <= 0) ||
        (g.kind == GeometryKind::kMesh && g.asset_digest.empty()) || g.kind < GeometryKind::kBox || g.kind > GeometryKind::kMesh) {return reject("invalid static geometry");}
    }
    s.static_geometry = e.static_geometry; s.context.inhibited = false; change(CommandKind::kSyncScene);
    d.status = {Disposition::kApplied,"static geometry loaded; backend acknowledgment required"};
  } else if (e.kind == SceneEventKind::kObservation) {
    const auto & o = e.object; const auto & v = o.observation;
    if (e.source != ObservationSource::kVision || o.key.epoch != t.epoch || o.key.settings_revision != snapshot.revision() ||
      o.key.track_id == 0 || o.key.sequence == 0 || !v.valid || !is_valid_pose(v.pose_world) || v.reference_frame != c.frames.world ||
      !std::isfinite(v.position_stddev_m) || v.position_stddev_m < 0 || v.position_stddev_m > c.tracking.max_position_stddev_m ||
      !source_is_fresh(v.timestamp,t,c.scene.dynamic_object_max_age_sec,c.time.sensor_future_tolerance_sec)) {return reject("invalid/non-vision dynamic observation");}
    if ((!c.scene.object_cloud_enabled && !o.object_cloud.empty()) ||
      (c.scene.object_cloud_enabled && o.object_cloud.size() > c.scene.max_points_per_object) ||
      std::any_of(o.object_cloud.begin(),o.object_cloud.end(),[](const auto & p) {return !std::isfinite(vector_norm(p));})) {return reject("invalid/beyond-capacity object cloud");}
    if (o.key.track_id == s.attached_track_id && s.attachment != AttachmentPhase::kDetached) {return reject("attached object cannot be updated as free obstacle");}
    auto found = std::find_if(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & x) {return x.key.track_id == o.key.track_id;});
    if (found == s.dynamic_objects.end()) {
      if (s.dynamic_objects.size() >= c.tracking.max_tracked_objects) {return reject("dynamic object capacity reached");}
      s.dynamic_objects.push_back(o);
    } else {
      if (o.key.sequence <= found->key.sequence || v.timestamp <= found->observation.timestamp) {return reject("duplicate/out-of-order object observation");}
      *found = o;
    }
    change(CommandKind::kSyncScene); d.status = {Disposition::kApplied,"dynamic scene update requires acknowledgment"};
  } else if (e.kind == SceneEventKind::kRefresh) {
    if (!s.coherent || s.pending_command_id || s.attachment != AttachmentPhase::kDetached ||
      std::none_of(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & object) {
        return same_track_identity(object.key,e.key) && source_is_fresh(object.observation.timestamp,t,
          c.scene.dynamic_object_max_age_sec,c.time.sensor_future_tolerance_sec);
      })) {return reject("scene refresh requires a fresh free target in an acknowledged scene");}
    change(CommandKind::kSyncScene);
    d.status = {Disposition::kApplied,"scene geometry refresh requires acknowledgment"};
  } else if (e.kind == SceneEventKind::kExpire) {
    s.dynamic_objects.erase(std::remove_if(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & o) {
        if (o.key.track_id == s.attached_track_id && s.attachment != AttachmentPhase::kDetached) {return false;}
        // Keep the active commissioning target's original collision model
        // through camera occlusion and the contact-to-attachment handoff.
        // Its visual timestamp is unchanged. Cycle deadlines still bound the
        // motion; after the active target is cleared, ordinary expiry resumes.
        if(c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept &&
          e.key.track_id && e.key.track_id==o.key.track_id)return false;
        const double max_age=c.runtime.operation_mode=="assumed_ng_demo" && e.key.track_id==o.key.track_id?
          std::max(c.scene.dynamic_object_max_age_sec,c.tracking.max_prediction_horizon_sec):c.scene.dynamic_object_max_age_sec;
        if (source_is_fresh(o.observation.timestamp,t,max_age,c.time.sensor_future_tolerance_sec)) {return false;}
        s.expired_track_ids.push_back(o.key.track_id); return true;
      }),s.dynamic_objects.end());
    if (!s.expired_track_ids.empty()) {change(CommandKind::kSyncScene); d.status = {Disposition::kApplied,"expired visual obstacles removed; scene invalid until acknowledgment"};}
    else {d.status = {Disposition::kApplied,"dynamic scene expiry checked"};}
  } else if (e.kind == SceneEventKind::kAttachRequested && s.attachment == AttachmentPhase::kDetached) {
    const auto found = std::find_if(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & o) {return o.key.track_id == e.key.track_id;});
    // In a failed moving-pick, non-Empty may mean an object is held even if
    // contact is unconfirmed. Attach the measured geometry conservatively for
    // collision planning; this does not assert physical Holding.
    const bool possible_demo_carry=c.runtime.operation_mode=="assumed_ng_demo" && c.grasp.moving_intercept &&
      gripper_snapshot_ready(e.gripper,snapshot,t) && !e.gripper.empty_confirmed;
    if (!scene_ready(s,snapshot,t,possible_demo_carry?e.key.track_id:0) || found == s.dynamic_objects.end()) {return reject("attachment requires coherent observed object and current grasp evidence");}
    s.attached_track_id = e.key.track_id; s.attachment = AttachmentPhase::kAttachPending; change(CommandKind::kAttachObject);
    d.status = {Disposition::kApplied,"attachment requested, not yet attached"};
  } else if (e.kind == SceneEventKind::kDetachRequested && s.attachment == AttachmentPhase::kAttached) {
    if (e.key.track_id != s.attached_track_id || !s.coherent) {return reject("detach target/scene mismatch");}
    s.attachment = AttachmentPhase::kDetachPending; change(CommandKind::kDetachObject); d.status = {Disposition::kApplied,"detach requested"};
  } else if (e.kind == SceneEventKind::kUpdateFailed) {
    if (e.scene_revision != s.revision || e.command_id != s.pending_command_id) {return reject("failure acknowledgment mismatch");}
    s.coherent = false; s.context.inhibited = true;
    if (s.attachment != AttachmentPhase::kDetached) {s.attachment = AttachmentPhase::kUnknown;}
    d.status = {Disposition::kApplied,"scene acknowledgment failed; inhibit motion"};
  } else if (e.kind == SceneEventKind::kUpdateConfirmed || e.kind == SceneEventKind::kAttachConfirmed || e.kind == SceneEventKind::kDetachConfirmed) {
    if (!s.pending_command_id || e.scene_revision != s.revision || e.command_id != s.pending_command_id || s.context.inhibited ||
      (s.attachment == AttachmentPhase::kAttachPending && (e.kind != SceneEventKind::kAttachConfirmed || e.key.track_id != s.attached_track_id)) ||
      (s.attachment == AttachmentPhase::kDetachPending && (e.kind != SceneEventKind::kDetachConfirmed || e.key.track_id != s.attached_track_id)) ||
      (e.kind == SceneEventKind::kAttachConfirmed && s.attachment != AttachmentPhase::kAttachPending) ||
      (e.kind == SceneEventKind::kDetachConfirmed && s.attachment != AttachmentPhase::kDetachPending) ||
      (s.attachment == AttachmentPhase::kDetached && e.kind != SceneEventKind::kUpdateConfirmed)) {return reject("scene acknowledgment mismatch");}
    if (s.attachment == AttachmentPhase::kAttachPending) {s.attachment = AttachmentPhase::kAttached;}
    else if (s.attachment == AttachmentPhase::kDetachPending) {
      const auto id = s.attached_track_id;
      s.dynamic_objects.erase(std::remove_if(s.dynamic_objects.begin(),s.dynamic_objects.end(),[&](const auto & o) {return o.key.track_id == id;}),s.dynamic_objects.end());
      s.attachment = AttachmentPhase::kDetached; s.attached_track_id = 0;
    }
    s.pending_command_id = 0; s.coherent = true; d.status = {Disposition::kApplied,"scene backend acknowledged"};
  }
  return finish();
}
}  // namespace ptc_core
