# Project status

VisionGuidedPick implements Teach Target → Observe → Auto Pick.

The application provides model training, RGB-D localization, shared IDs, per-arm
wrist measurements and continuous dual-arm tasks. Original source and authored
scene content use MIT; third-party content retains its own terms.

Setup prepares platform-specific dependencies and registers a separately acquired
Stanford Bunny demo inference model. Start opens backend/Web and an optional App;
Isaac Sim is launched separately. Windows uses a separate WSL application checkout.

Datasets, model binaries, acquired NVIDIA payloads, local settings, logs and
development/validation reports are excluded from source. The model Release holds
inference content and notices separately. See README.md for usage and licensing.
