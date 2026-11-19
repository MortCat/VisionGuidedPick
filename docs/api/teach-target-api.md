# Teach Target Web API — contract v0.3.0

The frontend/backend contract for image review, fresh training, publication and
Observe integration. [OpenAPI](teach-target.openapi.json) and
[workflow](../architecture/product-workflow.md) define the companion contracts.

## 1. Decisions

- Teach Target is a GUI upload → mask review → fresh fine-tuning → result preview → publish workflow.
- No SQLite, database server, distributed queue, automatic resume or resume endpoint. Metadata uses ordinary JSON files with atomic replacement and one application-side writer.
- A cancelled/interrupted run does not become a usable model. Retrain creates a new job and starts from the configured pretrained base, not the previous cancelled checkpoint. Changing images/configuration creates a new immutable dataset/configuration snapshot.
- A separate local worker process prevents HTTP/GUI blocking and provides a clean cancel boundary. It does not imply long training or resumable jobs. Small sample count alone does not guarantee a fixed wall time.
- `config/teaching.yaml` is the only operator-facing source of teaching/training parameters. Hyperparameters are not ordinary UI controls and cannot be overridden in the Train request.
- Product entry remains one origin on port8088. The new UI/API is independent of the legacy read-only HTTP server: no mandatory legacy proxy, port8089 or old UI compatibility layer. Old UI may be archived/removed during product cutover; reuse ROS telemetry adapters only where useful. Normal startup now opens Teach Target; existing pick startup is an explicit checkpoint.
- No arm command, Isaac Play or Auto Pick start is issued by Teach Target. Initially do not train concurrently with actual Auto Pick on the same GPU; report a resource conflict rather than silently stopping an arm. No running simulator is required just to teach/train.

## 2. Frontend controls and handshakes

| Control | Operation ID | Outcome |
| --- | --- | --- |
| New Target | `create_target_profile` | Server-generated `target_id` |
| Target list / reopen | `list_target_profiles`, `get_target_profile` | Draft revision, images and latest job references |
| Training Images / Validation Images | `import_target_image` | Multipart purpose=teaching/validation; optional source_label; original filename retained |
| Next-run split summary | `get_dataset_summary` | Counts, pending reviews, exact image IDs/source records; no training |
| Add/Remove Instance; No Target | `save_target_annotation` | Save every visible target separately, or explicitly review instances=[] for a negative image |
| Remove | `remove_target_image` | Remove draft membership; immutable training versions remain unchanged |
| Auto Contour | `generate_mask_proposal` | User box/points → a local segmentation suggestion, not an approved annotation |
| Save Contour / Adjust | `save_target_annotation` | Optional Include/Exclude points and correction Undo; explicit save returns annotation and target revisions |
| Train | `create_training_job` | 202 + persistent `job_id` and dataset/config versions. Observe and Auto Pick keep running |
| Progress / reload page | `get_training_job`, `list_training_jobs` | Server state is authoritative; poll once per second |
| Cancel Training | `cancel_training_job` | `cancelling` until the owned worker exits; then `cancelled` |
| Results | `get_training_results` | Held-out metrics where available, image overlays, candidate model reference |
| Publish Model | `publish_target_model` | Immutable published model manifest; not Observe activation |
| Use in Observe | `request_model_activation`, `get_model_activation` | P2 integration: actual runtime load ACK required before `active` |

All labels are English. Drafts/images/annotations survive browser refresh. HTTP upload progress means bytes transferred, not stored/decoded completion. Image cards show server-confirmed upload errors individually; retry one image without re-uploading the entire set. Six–twelve is a suggested teaching size, not a hard file-count gate.

## 3. API conventions

- Same-origin relative paths; Teach endpoints under `/api/teach`, runtime handoff under `/api/observe`. Version0.1.0 refers to this contract, not an existing application release.
- UTF-8 JSON, multipart/form-data for a single image upload. IDs are opaque strings, not file paths. No browser-supplied shell command, checkpoint path or training hyperparameter is accepted.
- Single-user installation: no login, Access Code, accounts, session cookies or `/api/session` endpoints. Source IP access remains restricted to explicitly authorized clients. Browser mutations reject mismatched Origin or cross-site Fetch Metadata. Same-origin UI and trusted local API clients require no CSRF-token exchange.
- POST operations that create a resource/work item use `Idempotency-Key`; one key + same installation/method/path/key/body returns the same original response. Key reuse with a different body returns409. Persist only the small request→resource mapping in JSON; no login or cookie participates in deduplication. This prevents duplicate jobs after a network timeout.
- Annotation PUT supplies its annotation revision; Train supplies the target draft revision. Conflicts return409 with the current revision, not silent overwrites. After upload/remove/save, refetch the target summary before training. The UI exposes **Remove Image** for the selected Training or Validation image, distinct from Remove Instance (one mask only). It shows the filename in an inline Remove/Cancel panel (no browser confirm dialog), reports deletion progress, deletes draft membership with the displayed target revision, refreshes counts and selects a remaining image.409 refreshes the target without retrying deletion automatically. Removing the last image clears the editor. Existing frozen jobs, source files and published models remain intact; a new Train uses the updated draft.
- Times are UTC RFC3339 wall time. Teaching jobs do not use `/clock` or simulator time. No automatic retrain/retry/resume after server restart.
- Acknowledgement is not completion: 202 means accepted; GET job/activation state identifies the final result. Polling continues after a temporary network error, and reconnect queries the existing ID rather than starting again.

### Error envelope

```json
{"error":{"code":"DRAFT_REVISION_CONFLICT","message":"Reload the target before training.","details":{"current_revision":8}}}
```

| HTTP | Examples / meaning |
| --- | --- |
| 400 | `INVALID_MASK`, `INVALID_IMAGE`: readable input but invalid content/geometry |
| 403 | `ACCESS_DENIED`: unauthorized source IP or cross-origin/cross-site mutation |
| 404 | `TARGET_NOT_FOUND`, `IMAGE_NOT_FOUND`, `JOB_NOT_FOUND`, `MODEL_NOT_FOUND` |
| 409 | `REVISION_CONFLICT`, `IDEMPOTENCY_CONFLICT`, `TRAINING_BUSY`, `JOB_NOT_READY`, `MODEL_NOT_READY` |
| 413 / 415 | Configured upload size exceeded / unsupported file format |
| 422 | Request fields/type/shape invalid |
| 503 | `OBSERVE_UNAVAILABLE`, `TRAINING_ENVIRONMENT_UNAVAILABLE`: runtime consumer or configured interpreter unavailable; no pretend success |

Errors include actionable reasons. Full stack traces stay in local diagnostics, not the ordinary UI. These are input/ownership/conflict checks, not a new collection of motion gates.

## 4. Resources and endpoints

OpenAPI is authoritative for exact fields, required headers and responses. Tables here explain intent.

| Method | Path | Responsibility |
| --- | --- | --- |
| GET | `/api/health` | Service readiness/version; source-IP authorization still applies |
| GET / POST | `/api/teach/targets` | List/create target profiles |
| GET | `/api/teach/targets/{target_id}` | Target draft and image references |
| POST | `/api/teach/targets/{target_id}/images` | Upload one JPEG/PNG with `purpose=teaching|validation` |
| DELETE | `/api/teach/targets/{target_id}/images/{image_id}` | Remove from draft using expected target revision |
| GET | `/api/teach/targets/{target_id}/images/{image_id}/content` | `variant=raw|full|preview` image content |
| GET / PUT | `/api/teach/targets/{target_id}/images/{image_id}/annotation` | Read/save reviewed instance masks |
| POST | `/api/teach/targets/{target_id}/images/{image_id}/mask-proposals` | Optional box/point-guided local mask suggestion |
| POST | `/api/teach/targets/{target_id}/training-jobs` | Freeze input/config and start one fresh training job |
| GET | `/api/teach/training-jobs?target_id=…` | Find previous/latest jobs after reload |
| GET | `/api/teach/training-jobs/{job_id}` | Current stage, epochs, elapsed time and result links |
| POST | `/api/teach/training-jobs/{job_id}/cancel` | Request cancel of this job only |
| GET | `/api/teach/training-jobs/{job_id}/results` | Metrics/overlays or409 if no completed result |
| GET | `/api/teach/training-jobs/{job_id}/artifacts/{artifact_id}` | Serve a manifest-listed overlay/metrics file; no checkpoint download |
| GET | `/api/teach/models/{model_version}` | Candidate/published model manifest |
| POST | `/api/teach/models/{model_version}/publish` | Publish a candidate after bundle/hash verification; 409 `MODEL_BUNDLE_LEGACY` for pre-bundle manifests |
| POST | `/api/observe/model-activations` | P2: load a published version, never start Auto Pick |
| GET | `/api/observe/model-activations/{activation_id}` | P2: pending/active/failed with acknowledged version |

Suggestion is a bounded local operation in P1; no separate annotation-job orchestration is required. The implementation uses pretrained SAM 2.1 Small, followed by GrabCut only within a narrow boundary band. It accepts a rough bbox, positive/negative points and optional saved RLE to derive a box. It returns `method=sam2_grabcut`, `elapsed_ms`, `embedding_reused`, `sam_ms`, `refinement` and `approved=false`; approval occurs only on annotation PUT. Invalid prompts/failed separation return400 with a readable correction request. The UI offers box redraw or point correction instead of a full brush workspace. Content references in results are same-origin URLs, never arbitrary local paths. Image/results serving routes authorize the owning target/job and resolve only manifest-listed asset IDs within the artifact root. They never accept arbitrary filesystem paths.

## 5. Image resolution and masks

1. Preserve uploaded source bytes unchanged (`raw`), including their original resolution. Never cap source files at360p or automatically reduce1280×720 to1024 pixels.
2. Create a full-resolution working image with EXIF orientation normalized (`full`), without geometric resizing. All API dimensions, masks, boxes and training annotations refer to this working image.
3. A lightweight preview/editor derivative may use **maximum long edge1024px**, keep aspect ratio, never upscale, and map editing coordinates back to the full image. This is what1024 means in this contract; it is not an ambiguous “1024p” cap on originals.
4. Model input resolution is a separate `config/teaching.yaml` parameter, validated against the selected model. Preserving high-resolution sources does not force all training/inference inputs to1024 or claim that enlarging a small object recovers detail.
5. Request/response masks use uncompressed COCO-style RLE: `size=[height,width]`, column-major order, alternating background/foreground runs beginning with background, and run counts summing to width×height. The editor can encode a canvas bitmap without implementing compressed COCO strings. Backend converts to the training format losslessly.
6. `instances` contains one mask per physical target in the image. Masks can represent holes; all instances use the profile's target class. Reviewed non-target-only validation images legitimately use `instances=[]`; do not erase visible target labels to fabricate negatives.

## 6. Training lifecycle and cancellation

```text
POST training-jobs(target_id, draft_revision, Idempotency-Key)
→ freeze dataset manifest + annotation versions + resolved YAML
→ save job.json
→ 202(job_id, preparing)
→ owned subprocess: prepare → train → evaluate
→ successful artifacts/manifest written
→ job.json = succeeded + candidate model_version
```

States: `preparing → training → evaluating → succeeded`; active jobs may become `failed`, `cancelling → cancelled`, or `interrupted` after unrecoverable process/service loss. One authoritative application owner writes job state; worker progress is consumed as events, not a competing writer to job.json. Atomic temp-file replacement prevents half-written status responses.

```text
POST cancel(job_id)
→ 202(cancelling)
→ ask only this verified owned worker/process group to stop
→ wait for actual exit
→ remove incomplete weights/temp products
→ cancelled (no model_version)
```

Preserve originals, reviewed annotations, frozen configuration and a short cancellation/error summary. **Default: do not retain cancelled/incomplete weights and do not offer Continue/Resume.** Already successful/published models are not deleted when a later job is cancelled. A new Train request with a new idempotency key takes the latest saved data/config and starts fresh from the configured pretrained base, not random weights and not the cancelled run.

Cancellation races are deterministic: if the job is already terminal, return its terminal state with200; if cancellation wins before success is committed, no candidate is published. Closing a tab does not cancel. Server shutdown requests owned worker termination; an unexpected restart reconciles ownership and records interrupted runs without launching duplicates. No guarantee of recovering arbitrary mid-epoch state.

When only6–12 images are available, show actual train/validation counts. Use provided independent validation images when present; otherwise reserve original-image/view groups from teaching inputs according to YAML. Augmented copies never cross the split. Evaluation labels its source (`held_out_views`, `deployment_holdout` or `preview_only`); previews/training fit are never presented as deployment accuracy. Runtime success is evaluated later in Observe/Auto Pick, not inferred from a Train success flag.

### Job response example

```json
{"job_id":"job_01","target_id":"target_01","dataset_version":"data_01","config_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","state":"training","epoch":8,"total_epochs":40,"elapsed_sec":25.2,"updated_at":"2026-11-02T03:00:00Z","model_version":null,"message":"Training epoch 8 of 40","error":null}
```

40epochs and25.2seconds are illustrative, not a required default or a performance claim. The GUI polls every second; no new training WebSocket/SSE protocol is necessary initially.

## 7. Storage: files only

```text
config/teaching.yaml                         loaded operator-managed parameters
artifacts/teach_target/
  targets/<target_id>/profile.json           name, draft revision, image references
  targets/<target_id>/images/<image_id>/      raw/full/preview + metadata + annotation revisions
  targets/<target_id>/datasets/<version>/    immutable manifest and selected annotation references
  jobs/<job_id>/job.json                     lifecycle and progress summary
  jobs/<job_id>/resolved_config.yaml         immutable settings snapshot
  jobs/<job_id>/events.jsonl                 bounded/rotated training progress
  jobs/<job_id>/evaluation/                  held-out results/overlays
  models/<model_version>/manifest.json       self-contained bundle: class map, preprocessing, evaluation, checksums
  models/<model_version>/checkpoint.pth      successful best checkpoint
  models/<model_version>/manifest.legacy.json  original manifest kept by explicit migration only
  requests/                                 small single-installation idempotency-key mappings
```

The root shown is the default `storage_root` in `config/teaching.yaml`; a custom root uses the same layout. A model directory is self-contained. Observe activation reads only `manifest.json` and `checkpoint.pth`. It does not read jobs, training images or `resolved_config.yaml`. The job ID and `config_sha256` remain for traceability. A model directory can be moved to another storage root or checkout, and its job and images can be removed, without changing activation.

Bundle fields (`bundle_format=1`) are written when training succeeds. They contain `class_mapping` (model output index to label, from the sorted COCO categories), `preprocessing` (`resolution`, RGB order, mask space), the held-out `evaluation` summary and `source` (resolved training section, plus worker-recorded `rfdetr`/`torch`/`torchvision` versions). `contract_sha256` covers the activation fields: version, checkpoint hash, class names/mapping, preprocessing and evaluation. Publish and activation both verify the checkpoint hash and contract hash. Mismatches return 503 `CHECKPOINT_HASH_MISMATCH`/`MODEL_BUNDLE_MISMATCH`.

Models created before bundles exist are not rewritten automatically. Publish and activation return 409 `MODEL_BUNDLE_LEGACY`. An operator migrates them explicitly while the original job records still exist:

```bash
python tools/teach_target/model_bundle.py check   [--storage-root PATH] [MODEL_VERSION ...]
python tools/teach_target/model_bundle.py migrate [--storage-root PATH] [MODEL_VERSION ...]
```

Migration verifies the checkpoint hash, rebuilds bundle fields from the retained job, keeps `state`, saves the original as `manifest.legacy.json` and sets `migrated_from_legacy=true`. If the job's category file, `result.json` or `resolved_config.yaml` is missing, it reports `MODEL_BUNDLE_SOURCE_MISSING` with the missing paths and leaves the manifest unchanged. Such a model must be retrained. Library versions are null for migrated models because older jobs did not record them.

No SQLite stores metadata or weights. Draft removal does not delete assets referenced by frozen versions. Filesystem manifests are the catalog for this small single-host workflow. Keep one training worker, no general job queue. Job/data/model IDs do not reuse live `track_id`. Retention policy for cancelled outputs is explicit, not a presumed continuation feature.

## 8. Observe handoff

Published versions are fixed validation-selected checkpoints. Actual activation, runtime ACK, start/stop/status, frame and shared-object queries are implemented in the [Observe API technical reference](observe-api.md) and the shared [OpenAPI contract](teach-target.openapi.json). Loading verifies the exact version/hash and runs smoke inference on a generated neutral frame at the bundle resolution; it does not read training images, assert accuracy or start Auto Pick. Stop releases only the Observe owner. Ordinary UI parameters stay in `config/observe.yaml`.

## 9. Acceptance cases

- Upload native1280×720 or larger: raw unchanged; full retains native resolution; preview longest edge≤1024; masks map exactly to full pixels.
- Mixed upload success/failure: individual retries preserve already stored images. Revision conflicts and repeated Train requests cannot create duplicate jobs.
- Train with reviewed arbitrary-view data: snapshot settings/data, publish actual progress and independent evaluation labels; no UI hyperparameter controls.
- Cancel during preparation/training/evaluation: worker exits, no unfinished model published, no Resume control, new job starts from configured base.
- Refresh/disconnect: reopen the same job; neither automatic cancel nor retrain.
- Model publish: shows Ready for Observe; Active requires actual matching worker ACK, not a button click.
- Preserve existing arm code/parameters and simulator scene. The Teach Target HTTP entry is deployed; no robot operation or GPU training is performed by its launch.

OpenAPI format reference: [OpenAPI3.1.0](https://spec.openapis.org/oas/v3.1.0.html). Implementation must update this contract and the human guide together when changing request fields or handshake semantics.

Training progress revision: `completed_epochs`, nullable `eta_sec` and Unix `eta_updated_at` are reported after measured epochs. ETA uses the last three epoch durations and excludes final evaluation; preparation/first epoch display Estimating. Failures expose the short terminal error in the UI while full tracebacks remain in worker.log. RF-DETR trainer single-GPU selection uses devices=1 (the installed builder rejects list-valued devices).

Training results now include actual CSV-based epoch history and UI charts for train/validation total, classification, box and mask losses, plus trainer box/mask mAP. Sparse train/validation rows are merged by epoch without inventing missing values. Reloaded-checkpoint IoU remains a separate result; zero IoU is labelled Needs review, not model acceptance. Available through existing job results API; no retraining needed for saved metrics.csv.

## Specified validation and provenance — v0.3.0

- Upload remains POST `/api/teach/targets/{target_id}/images` with multipart `file`, `purpose` (`teaching` default or `validation`) and optional `source_label` (trimmed, ≤256characters). ImageAsset adds original_filename and source_label. Preserve native raw/full pixels. Source label is supplied by the operator; blank/legacy unknown stays unspecified, never silently called Gantry. The UI reports Added / Already present / Failed separately using server-returned image IDs and refreshes current dataset counts. Identical-content retries do not increment membership; there is no40-image cap. Duplicate same bytes/same purpose returns the stored asset; same bytes across different purposes returns409 DATA_SPLIT_CONFLICT.
- GET `/api/teach/targets/{target_id}/dataset-summary` returns target_id/draft_revision/train_count/validation_count/validation_mode/pending_count and training_images/validation_images source records. It uses the same deterministic split selector as the worker. If explicit validation exists, every teaching image is used for training:12+6 remains12train/6validation. Without validation, retain configured seed/fraction fallback, currently12uploaded→9train/3validation. This is the **next** run; historical job counts/results remain unchanged.
- Existing annotation PUT accepts instances=[] as explicit reviewed No Target and computes outcome=no_target; instances with masks compute outcome=instances. Pending empty annotations remain pending. Saved assets expose instance_count/annotation_outcome. UI Add Instance keeps all draft masks; Save Contour saves every instance; Remove Instance is persisted on Save. No Target explicitly replaces this image's saved instances with an empty list. No automatic segmentation prediction is treated as a negative label.
- New jobs freeze source_label, original_filename, purposes and annotations in dataset.json. Results add validation_images with source/image ID, truth/predicted instance counts, union IoU, maximum retained target confidence and overlay URL; training_images also preserves source membership. Source metadata is captured only for new uploads; historical filenames/labels are not fabricated.
- metrics_source is specified_validation or automatic_holdout for new jobs. Keep historical held_out_views/deployment_holdout/preview_only values readable. Evaluation reports positive_mean_union_iou and positive_image_hit_rate_at_iou_0_5 separately from negative_false_positive_images/rate; missing positive/negative cases are null, not a fabricated perfect score. mean_union_iou remains the legacy combined metric and can be inflated by correctly empty negatives; do not use it alone. These are image-level metrics, not instance recall or physical pick success.
- Validation participates in epoch/checkpoint selection without gradient updates; it is not an untouched test set. Keep separate capture groups/time segments for final test; identical adjacent frames must not be split across training/validation/test. Exact-byte duplicate checks do not detect all near-duplicates.
- Ordinary Train request still accepts only draft_revision. No model weight, existing annotation, published version or historical job is rewritten by adding Validation uploads. No training begins automatically.

User supplies the validation source images. Suggested initial collection: six native gantry frames—two target-only, two target+Bear/cube clutter, one non-target-only and one empty belt. Annotate all visible target instances; confirm negatives explicitly. These are selection guidelines, not images already captured/imported. Use images from the intended deployment conditions.

UI simplification: source/session text boxes are removed. Ordinary uploads send file and purpose only; the server retains original_filename automatically. Optional source_label remains available to API clients and stored historical records for compatibility; it is not a user-required input. Upload labels are Training Image and Validation Source.

Training job detail includes training_history during execution. Job progress can include train_count and augmented_train_count after dataset preparation. Results include selected_epoch when the checkpoint records an epoch, or null otherwise.

### Default demonstration catalogue order

Setup registers the pinned demo model without activating it. The published-model
catalogue lists that model first when present; an explicit target filter or model
selection remains authoritative. Catalogue reads and UI preselection do not start
Observe or arm tasks. Validation metrics/artifacts remain available even though
the Teach page no longer renders a validation-prediction image gallery.
