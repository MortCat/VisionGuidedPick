#pragma once
#include "ptc_core/state/track_state.hpp"

namespace ptc_core
{
enum class GeometryKind {kBox, kSphere, kMesh};
struct StaticGeometry
{std::string id, reference_frame; GeometryKind kind{GeometryKind::kBox}; Pose3d pose; Vector3d dimensions_m; std::string asset_digest,mesh_path;};
struct DynamicObject
{EventKey key; ObjectObservation observation; std::vector<Vector3d> object_cloud;};
enum class AttachmentPhase {kDetached, kAttachPending, kAttached, kDetachPending, kUnknown};
enum class SceneEventKind
{kStaticLoaded, kObservation, kExpire, kAttachRequested, kAttachConfirmed, kDetachRequested,
  kDetachConfirmed, kUpdateConfirmed, kUpdateFailed, kClockReset, kResetRequested, kRefresh, kCount};
enum class ObservationSource {kVision, kSimulationTruth};
struct SceneState
{
  OperationContext context; std::vector<StaticGeometry> static_geometry; std::vector<DynamicObject> dynamic_objects;
  AttachmentPhase attachment{AttachmentPhase::kDetached}; std::uint64_t attached_track_id{0}, revision{0}, pending_command_id{0};
  bool coherent{false}; std::vector<std::uint64_t> expired_track_ids;
};
struct SceneEvent
{
  SceneEventKind kind{SceneEventKind::kExpire}; EventKey key; std::vector<StaticGeometry> static_geometry;
  DynamicObject object; ObservationSource source{ObservationSource::kVision};
  std::uint64_t scene_revision{0}, command_id{0};
  ArmSnapshot arm; GripperState gripper;
};
struct SceneDecision {SceneState state; DecisionStatus status; std::vector<Command> commands; TransitionRecord record;};
// A selected moving-target operation may retain its object through the
// existing visual prediction horizon. Other obstacles use normal freshness.
bool scene_ready(const SceneState &, const SettingsSnapshot &, const TimeContext &, std::uint64_t predicted_track_id=0);
SceneDecision advance_scene(const SceneState &, const SceneEvent &, const SettingsSnapshot &, const TimeContext &);
}  // namespace ptc_core
