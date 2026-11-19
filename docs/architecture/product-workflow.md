# Product workflow

Teach Target → Observe → Auto Pick is the active application workflow.

1. Upload target views covering expected poses, distances, lighting and backgrounds. Review every instance mask and mark negative images No Target. Use separate validation images.
2. Train from config/teaching.yaml in an owned subprocess. Inspect held-out results and publish a model. Cancellation discards incomplete weights; the next Train starts fresh.
3. Observe loads the published checkpoint with its class map and preprocessing contract. Gantry RGB-D supplies masks, localization and shared object identities.
4. Start either arm. A routed target approaching that arm increases wrist inference cadence. One wrist top-view measurement refines the same identity and supplies the grasp surface.
5. Existing task/motion owners plan moving-target grasp, lift, transport and release. Empty failures return through Home; possible possession uses bin-release recovery before new assignment.

Observe alone does not start arm tasks. Action acknowledgement, measured Holding and bin placement evidence are separate facts. Stop remains authoritative. Five-face orbit inspection and defect classification are outside this application workflow.

P1–P5 acceptance follows upload/review, fresh training/publication, real model activation, shared gantry/wrist identity and existing dual-arm operation respectively. Simulation truth can score results or generate labels, never supply formal detections/control.
