# VisionGuidedPick contributor rules

Read README.md, docs/getting-started.md, docs/architecture/product-workflow.md,
docs/architecture/blueprint.md, docs/architecture/state-machines-and-nodes.md,
docs/architecture/ros2-interface-contract.md and docs/configuration/settings.md before changes.

- C++17 core decisions stay in ptc_core, independent of ROS, Isaac Sim and YAML.
- Keep ROS nodes thin; optional MoveIt dependencies stay in the backend packages.
- Preserve internal ptc_ packages and the existing ROS contract when changing display branding.
- Use capture-time TF for RGB-D. Simulator truth is only for annotation/evaluation.
- Keep one motion owner per arm. Retain explicit Stop, collision checks and measured possession recovery.
- Reuse state machines; do not add duplicate flags, readiness gates or speculative timeouts.
- Config is loaded at boundaries. Teach parameters belong to config/teaching.yaml; no SQL or training Resume.
- Update CHANGELOG.md for meaningful changes and corresponding API/architecture docs when contracts change.
- Run tests appropriate to the change and git diff --check. Never commit models, datasets, acquired NVIDIA assets or local settings.
- Expected origin: https://github.com/MortCat/VisionGuidedPick.git. Preserve others' changes; no force pushes.

- Public release content must not include development logs, test results or validation/acceptance reports. Keep regression test source separate from execution records; model Release bundles contain inference content and notices only.
