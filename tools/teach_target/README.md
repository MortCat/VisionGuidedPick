# Teach Target and Observe

Follow [Getting started](../../docs/getting-started.md) and the
[workflow](../../docs/architecture/product-workflow.md). Start the product with
`bash start.sh` on Linux or `./start.ps1` on Windows. Setup registers the default
demo model; Observe and arm operation still require explicit UI actions.

API and ML workers use separate Python environments. Configuration resolves at
the boundary from config/teaching.yaml or a host-local override. Jobs and models
are file-backed. No database, login or training Resume is required. The Teach
page shows training progress/results without a validation-prediction gallery.
