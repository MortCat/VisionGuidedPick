# Observe Web API technical reference

Date: 2026-11-02. Implementation: `tools/teach_target/service.py`, `observe_runtime.py`; [machine contract](teach-target.openapi.json). The same version-controlled OpenAPI is served at `/openapi.json`; `/docs` reads it. Same8088 origin and precise source-IP list as Teach Target; no login/session. Observe never emits Task/Arm/Gripper commands or presses Isaac Play. Five-view/defect processing stays disabled.

## Endpoints

| Method | Path | Request → response |
| --- | --- | --- |
| GET | `/api/teach/models?target_id=…&state=published` | Published manifest catalogue; no checkpoint bytes |
| POST | `/api/observe/model-activations` | `{model_version}` →202 loading activation |
| GET | `/api/observe/model-activations/{activation_id}` | loading/active/failed; exact version/hash ACK and actual inference contract |
| POST | `/api/observe/runs` | `{activation_id}` →202 run; requires resident matching ACK |
| POST | `/api/observe/runs/{run_id}/stop` | `{}` →202 stopping /200 already stopped |
| GET | `/api/observe/status` | Current `run` or null, resident model or null, source/health statistics |
| GET | `/api/observe/runs/{run_id}/objects` | Confirmed detection and outcome totals and up to 100 recent object records, newest first |
| GET | `/api/observe/runs/{run_id}/objects/{track_id}/point-cloud` | Cached gantry surface for one recent confirmed target |
| GET | `/api/observe/runs/{run_id}/snapshot` | Latest exact-frame snapshot; before first frame: `{run_id,frame_key:null,message}` |
| GET | `/api/observe/runs/{run_id}/pick-preview` | Read-only `{run_id,frame_key,motion_control:false,candidates}` for the latest live frame |
| GET | `/api/observe/frames/{frame_key}/preview?layer=rgb\|mask` | Same-key JPEG /RGBA PNG;410 if evicted |

`pick-preview` returns `candidates` only while `run_id` is the current run, its cached snapshot is the latest frame, `source_state` is `live` and the display age is below each candidate's `valid_for_sec`. Otherwise it returns an empty list. Each candidate has `track_key`, `confidence`, `position_world_m`, `velocity_world_mps`, `measurement_stamp_ns` (decimal string) and `valid_for_sec`. A candidate requires the same ID's `pick_confirmed` streak and measured velocity. It never sends motion. Errors are 404 for an unknown run and 410 for an expired run.

POST requires Idempotency-Key. Repeat method/path/key/body replays the original receipt; same key/different body409. Only named fields are accepted; no arbitrary checkpoint paths, topics, thresholds or shell commands. These async request bodies are JSON objects, including `{}` for Stop.

## UI and acknowledgement sequence

Teach `Open Observe` only navigates, carrying the fixed published model_version. `Start Observe` requests activation, polls until the worker acknowledges the exact activation/version/SHA256 after smoke inference, then creates a run.202 is receipt, not completion. `Stop Observe` stops input processing and releases its owned worker/GPU; stopped is reported only after process exit. No detections still counts as a processed frame; quality is independent of successful load. Stop preserves the last bounded preview and labels it frozen. Historical activation ACK is not a resident-model claim; GET status is authoritative. Browser closure/navigation does not stop a run; API restart defaults stopped.

Run states: starting →waiting_for_frame →observing →stopping →stopped, or failed. Status contains `run`, `active_model_version`, `activation_id`, `last_frame_key`, nullable `display_age_sec`, `source_state`, `health`, and `motion_control:false`. Health carries measured input/inference Hz, inference milliseconds, dropped frames, processing age, clock epoch and depth/TF availability when measured. No source means waiting; aged results become stale/frozen, not Live. The API uses monotonic age, not subtraction between simulator and system clocks.

First version uses Stop→select→Start. A new activation while a model owner exists returns409; failed activation after stopped leaves no active model. No in-place/hot model switch is implemented, so an old running model is never replaced by a failed load. Training can run concurrently with Observe and Auto Pick on the shared GPU. Training neither stops Observe nor activates the candidate model. Auto Contour requests during Observe return409. Stop only affects this Observe owner, not an externally running pick backend.

## Frame and object contract

`snapshot` includes run_id, model_version, clock_epoch, frame_key, capture_stamp_ns (decimal string), original_size, preview_size, rgb_url, mask_url, objects, message. A frame key binds RGB, overlay and all object geometry. Preview preserves the full image composition, at360px height. Native inference/annotation images are not cropped or preview-downsampled. Clients fetch both preview layers before swapping the displayed frame.410 means fetch a new snapshot, never substitute latest RGB for an older mask.

Each object includes track_key (`run:epoch:id`), semantic label, confidence, phase (candidate/tracking), measurement_stamp_ns, original-pixel bbox_xyxy, nullable position_world_m/velocity_world_mps/observed_bounds_world, box_edges_rgb_pixels and geometry_reason. Coordinates/velocity stay null without measured support. World position is the visible surface centre; bounds are world-axis-aligned visible-surface bounds (`minimum_m`, `maximum_m`, `orientation`), not physical centre-of-mass, full CAD volume or inferred yaw. Projected edges contain pairs of original-RGB pixel endpoints. The overlay fits its display bounds to trimmed visible depth and projects them into the original RGB image. The fresh fit and the displayed projection must both agree with the current mask extent. Dimensions use a rolling median of up to five accepted fits of the same ID, including comparable measurements after confirmation. `box_estimated` is true for a dashed projection based on an older accepted depth sample. The estimate expires after 250 ms from that sample, even if the RGB ID stays visible. Display retention is independent of raw geometry and control data.

The optional `pick_confirmed` field records whether the same gantry ID has maintained qualifying confidence and geometry for `config/observe.yaml` `pick_confirmation_sec` (1.0 s). Time uses the RGB capture stamp, and consecutive frames of the ID must be at most 250 ms apart. One missing, low-confidence or depth-less frame is bridged without restarting the time; it cannot confirm by itself. Two such frames in a row, a longer gap or an out-of-order observation interrupt the streak. Before the third transit confirmation, the transit check accepts gaps of up to 500 ms between confirmed measurements, which covers one bridged frame. Admission also requires measured velocity and the existing transit checks. The wrist checks the shared ID without adding a second timed confirmation. `display_confirmed` controls the contour, label, 3D box and object table. Once confirmed, the same visible ID remains annotated through a brief geometry loss. `pick_confirmed` still governs pick candidacy.

The display fit preserves more of the measured silhouette and rejects clearly separated depth surfaces or large disconnected mask regions. The projected extent must cover the mask on both image axes within a small pixel allowance and stay close to its size. A confirmed object may show only its contour when the frame cannot support a plausible box. Adjacent objects with one continuous mask and depth surface can still be ambiguous.

RGB-D uses source-time TF and nearest-visible-depth registration; invalid/nonpositive depth is excluded, without a0.18m floor. Missing/unsynchronized depth or missing capture-time TF degrades to2D and a concrete reason. A single C++ ObserveTrackModel owns gantry IDs for2D→3D upgrades; Web shared records are routed into each arm's wrist association. Fast local prototype transport is API↔Python inference worker↔pure C++ geometry/track helper over owned stdio. ROS input is standard Image/CameraInfo/Clock/TF only; no new custom ROS message is claimed delivered. This replaces the draft's proposed additional ROS node/DTO with fewer processes and no legacy five-view dependencies.

One latest pending RGB, eight depth samples, sixteen Web frames, bounded256 tracking records, and32 activation/run summaries. Clock rollback clears pending/depth/track and prior-epoch preview caches. Run/version identities prevent late old-run results from updating a new run. Settings: `config/observe.yaml`; camera/world and timing reuse `config/factory_demo/arm_01.yaml` tracking-camera contract; ROS/DDS environment reuses the deployment runtime profile. No simulator object truth or Rabbit CAD is used for detection.

## Errors and quality

400 malformed JSON;404 unknown model/activation/run;409 unpublished model, `MODEL_BUNDLE_LEGACY` (published before self-contained bundles; migrate explicitly, see [Teach Target storage](teach-target-api.md)), no resident ACK, conflicting owner or idempotency conflict;410 discarded run/frame;422 malformed body/layer;503 missing/corrupt checkpoint or `MODEL_BUNDLE_MISMATCH`. Async worker failures appear on activation/run with their actual error. A load ACK does not guarantee detections or accuracy. No foreground-based label substitutes for model detections.

Training Roboflow COCO categories are remapped to contiguous output labels: the current dataset category1 becomes model label0. The mapping is resolved once from the frozen sorted category catalogue when training succeeds and stored as the bundle's `class_mapping`. Never treat background/output slot1 as Rabbit just because the COCO category is1. Activation reads only the model directory. The worker takes `preprocessing.resolution` from the verified bundle and verifies the checkpoint checksum again before load. Smoke inference uses a generated neutral frame at that resolution, so a custom storage root, a moved checkout or removed training images do not affect activation.

See [approved design](../architecture/product-workflow.md) and [ROS contract](../architecture/ros2-interface-contract.md). Auto Pick consumes the shared records and retains the existing arm executor.

## Implementation method map

| Method | Responsibility |
| --- | --- |
| `list_published_models` / `resolve_published_model` | Catalogue and self-contained bundle resolution through `model_bundle.verify` (checkpoint and contract hashes; no job reads) |
| `ObserveRuntime.request_activation` | Owned loader request and exact-identity ACK |
| `ObserveModelAdapter.__init__` / `predict` | Shared RF-DETR loading, frozen input contract and semantic instance masks |
| `ObserveRuntime.start_run` / `stop_run` / `status` | Observe lifecycle, verified process exit and current owner state |
| `prepare_observe_frame` | ROS stride/color decode, original pixels retained |
| `extract_target_surface` | Source-time registration/TF and bounded visible object points |
| `estimate_observed_geometry` / `ObserveTrackModel.update` | Pure C++ observed bounds, measured velocity and one central ID |
| `ObserveRuntime._accept_frame` | Identity-bound preview/snapshot and bounded frame-cache commit |

No additional motion admission flags or five-view actions are attached to these methods.

## Preview pacing / read-only settings — API0.4.0

GET status adds confidence_threshold, preview_hz and inference_hz_limit from the loaded Observe configuration. The current experiment uses0.10confidence and10Hz inference/preview caps. These settings do not overwrite frozen training evaluation thresholds. Status/telemetry/object-table polling stays1Hz; snapshot/RGB+mask rendering uses a separate configured10Hz loop with a single in-flight preview request. Each displayed RGB/mask/3D record must share the exact frame_key. No old mask is painted on newer raw RGB. Browser display FPS is measured from unique rendered frames; input/inference Hz are backend measurements. A configured10Hz limit cannot create10frames when the source/model supplies fewer. Sixteen bounded cached frames support this rate. On stale/paused source the last frame remains frozen.

## Auto Pick API

The published Auto Pick transit ID can continue across a short image-track split when the new fragment uniquely matches one established trajectory. An established route that still has a valid observation in the same frame does not compete for the fragment.

After a short observation gap, a detection with measured depth can keep its existing gantry ID when the belt projection uniquely matches in both directions. The matching history retains the last depth sample during 2D-only frames, while the API continues to report only the current frame’s geometry.

The Auto Pick handoff publishes a gantry ID after three successive measurements support the same belt trajectory. Short-lived detections can appear in Observe without reserving an arm target. Hints retain the timestamp of the last plausible measurement, and wrist RGB-D confirms identity before selection.

Auto Pick uses the Observe worker and a separate ROS task service for each arm. Run `bash start.sh` on native Linux, start Observe with a published model, then open `/auto-pick`. Both arms start idle.

| Method and path | Request | Result |
| --- | --- | --- |
| GET `/api/auto-pick/status` | None | `available`, `task`, `command`, `outcome`, `gripper`, `arm_feedback`, `wrist_camera`, and optional `placement` (cycle identity, `state`, `reason`, and nullable `delta_n`). Also includes nullable `bin_load` with `contents_g` (fresh, stable load less the configured empty-bin tare) and `last_drop_g` (latest completed release's measured change). Either field can be null. Empty-bin calibration is stored per arm in `config/observe.yaml` and survives restarts. IDs are decimal strings. |
| GET `/api/auto-pick/wrist-preview` | None | With `frame`, returns the exact cached wrist RGB JPEG, or `410` after eviction. Without it, returns the latest JPEG. `404` before the first frame. |
| POST `/api/auto-pick/start` | `{"run_id":"…"}` | `202` with the pending command. The worker requests one cycle from the selected arm. |
| POST `/api/auto-pick/continuous` | `{"run_id":"…"}` | `202` with the pending command. The ROS task coordinator starts the next cycle after each completed cycle while Observe data remains fresh. |
| POST `/api/auto-pick/pause` | `{"run_id":"…"}` | `202` with the pending command. Stop taking new targets and let an assigned cycle finish. |
| POST `/api/auto-pick/cancel` | `{"run_id":"…"}` | `202` with the pending command. The worker requests the existing ROS cancellation and recovery. |
| POST `/api/auto-pick/home` | `{"run_id":"…"}` | `202` with the pending Return Home command. |
| POST `/api/auto-pick/clear-bin` | `{"run_id":"…"}` | `202` with the pending Clear Bin command. |

`joint_feedback` is nullable. When present it carries `fresh`, `issue`, and `joints` from the arm’s `JointState` stream. Each row has `name`, `type`, `position`, and `effort`. Revolute values use radians and N·m; prismatic values use meters and N. Missing or nonfinite measurements are `null`. Stale feedback carries no rows.

When the latest joint sample is ahead of the received simulation clock, the worker may display an earlier sample that still meets the same freshness checks. A rewind clears the buffered joint samples.

Use the string `track_id` from object history to query a point cloud. Responses contain `run_id`, `track_id`, `gantry_cloud_revision` and `gantry_cloud`. The cloud contains latest and motion-aligned accumulated samples, capped at 450 world XYZ points per layer, and `capture_stamp_ns` (capture time in the source clock, or null when unavailable). Send the last revision as `cloud_revision`. A matching revision with a null cloud means the drawing is unchanged. Two null values mean no cloud is available in the current clock epoch. A target near the image edge retains its last accepted cloud revision and capture time until fusion resumes. Targets outside recent history return `404 OBJECT_NOT_FOUND`. Superseded runs return `410 RUN_EXPIRED`.

`GET /api/auto-pick/status?point_cloud=true` includes the assigned target's last measured `gantry_cloud` when available, with `track_id`, `scans`, `latest` and `accumulated`. Each array holds at most 450 world XYZ points in meters. Pass the previously received `gantry_cloud_revision` as `cloud_revision` to reuse a cached preview. A matching non-null revision with `gantry_cloud: null` means the preview is unchanged. A null revision with a null cloud means no preview is available. `wrist_camera.grasp` is null or carries the plan identity and three normalized image points in center, left jaw, right jaw order. Use it only with that status's `wrist_camera.frame_id`. Both previews are read-only.

Bin status also includes `overflow`, `clearing` and `clear_available`. An overflow is latched for that station across Observe restarts. It requests PAUSE_CONTINUOUS, lets assigned work finish and blocks new Start requests. Clear Bin requires a live stopped arm, an idle task, a stopped empty gripper and an available simulator clear service. A successful clear response means fresh stable load returned to the empty tare. Clearing does not start motion. Use Start after clearing.

The Auto Pick page maps **Start** to `/api/auto-pick/continuous` and **Stop** to `/api/auto-pick/cancel`. The one-cycle route remains available to API clients.

All Auto Pick routes accept `?arm=arm_01` or `?arm=arm_02`, defaulting to arm_01. Each POST requires an `Idempotency-Key`. Poll status for the matching command `request_id` and actual ROS `accepted` reply. A null acknowledgement after timeout means the command status is unknown. `task.continuous_operation` reports the arm's current mode. `task.last_result` preserves the last saved result with its `epoch`, `cycle_id`, `track_id`, and `outcome` after the next cycle starts. Outcome 1 means placement reported, 3 means inconclusive, 4 means canceled, and 5 means failed. The top-level `outcome` is `task.last_result.outcome` while the idle task (phase 0) carries the same `epoch`, `cycle_id` and `track_id`, otherwise null. It comes only from live TaskStatus. It is not read from diagnostic files, so missing or unwritable diagnostic directories do not change it. Outcome 1 is a controller release report. Without a matching bin-load placement, the object row stays `unverified`, not `placed`.

The `gripper` object has `fresh`, `ready`, `stopped`, `width_m` and `possession` (`empty`, `holding` or `unknown`). Holding combines Task possession with current gripper contact. The `arm_feedback` object has `fresh`, `ready`, `command_ready` and `stopped`. Each feedback object includes a nullable `issue` describing why it is stale. Either object can be null when the worker is unavailable.

Return Home pauses new cycles for the selected arm. The existing cycle and recovery owner handles any held or uncertain object. Home moves require fresh stopped Empty and a detached planning scene. `task.home_state` is 0 none, 1 queued, 2 resetting, 3 moving, 4 measured arrival, 5 failed or 6 canceled. `task.home_message` describes the last transition. Observe blocks stopping while a semantic task, Home request or command acknowledgement is pending.

`POST /api/auto-pick/home` is sent whenever an Observe run exists. It does not require the arm's reported availability, fresh feedback or the reply to an earlier command; the worker forwards it as soon as the arm's command service is ready and any in-flight request has returned.

The object history response contains `run_id`, `detected_count`, `outcome_counts`, and up to 100 `objects`. The total counts confirmed tracks across the current Observe run and clock epoch, including records outside the visible history. The outcome_counts fields placed, failed, and unverified each count the last settled outcome per confirmed ID across the same scope, including evicted rows. Active or canceled attempts do not add a result. A verified placement replaces a previous failure and is retained after an identity merge. The overlay and history share the consecutive `display_id` assigned upon confirmation. Late association of two counted fragments keeps the earlier number and adjusts the total and later display numbers. ROS identity stays in `track_id`. Each row also includes `label`, `state`, `first_seen_ns` and optional assignment fields `arm`, `epoch`, `cycle_id`. States are `tracked`, `picking`, `holding`, `placing`, `placed`, `unverified`, `failed` and `canceled`. A stable net bin-load increase after release marks the keyed cycle as `placed`. A reported release with insufficient load evidence is `unverified`. The controller outcome remains available separately. When both arms are in continuous Auto Pick, a confirmed, unassigned track measured crossing the downstream selection boundary is counted as `failed` and includes `reason`. Missing observations alone do not settle the target.


### Transit association configuration — 2026-11-04

The live gantry geometry helper and published-transit continuation/rejoin use the same existing `tracking.association_distance_m` from `config/factory_demo/arm_01.yaml` (currently0.075m). There is no additional35mm transit-specific rejection range. The value is loaded at Observe startup, not mutated during a run. Unique one-to-one matching, co-visible separation and actual measurement timestamps remain unchanged. No Web endpoint, request field or UI control was added.
