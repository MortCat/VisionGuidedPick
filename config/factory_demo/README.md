# Factory assumed-NG demo deployment

This is a separate, explicit demo configuration. `config/settings.yaml` remains the unresolved production draft. No RF-DETR model is loaded, and no defect boxes/scores are fabricated. Every target is assumed NG only after all five actual RGB-D views, requested geometry/point-cloud processing, and the complete saved-report acknowledgment succeed.

The wrist calibration and nominal Stanford reference are exported from the authored assets. Runtime object pose, yaw and velocity come from measured RGB-D and capture-time TF. Static workspace/support geometry is installation calibration, not a conveyor speed or moving-object input.

`arm_01.yaml` and `arm_02.yaml` validate resource manifests, frame identities and settings. Output directories must exist before startup: `artifacts/inspections/arm_01`, `arm_02` and `artifacts/state_history/arm_01`, `arm_02`. Parameters are development limits and do not establish achieved throughput.

The factory currently uses its existing MoveIt/Servo stack and unique joint-command owner. Use `external_motion_stack:=true` when launching against that stack, avoiding duplicate MoveIt/Servo nodes. Both host and container use ROS domain 0 and Fast DDS UDPv4. The simulator and existing controllers must already be playing/ready.

The body mount Euler rotation handles the pitch singularity; optical +Z is forward. Camera and TCP fixed links must match the model used by the running motion stack. URDF mesh paths remain relative on disk and are verified against identities before conversion to in-memory absolute file URIs.

Current evidence: static five-view wrist-camera workspace verified; C++ moving-rabbit perception observed 73 RGB-D frames and four visual IDs in a 35-second read-only probe. Complete moving five-view acquisition, physical grasp/binning and dual-arm throughput remain in progress. These settings must not be described as an accepted four-phase demo yet.
