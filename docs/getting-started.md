# Installation and first run

Run commands from your checkout root. Host paths, environments and build outputs
belong to each checkout; no fixed project directory is required.

## Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy, initialized rosdep, MoveIt 2 and Servo.
- Python 3.12 or newer, Git, CMake, a C++17 compiler, colcon and python3-venv.
- NVIDIA Isaac Sim, a supported GPU and a driver compatible with your simulator
  and CUDA PyTorch wheels. Native configuration targets Isaac Sim 6.2; Windows
  profiles target 6.1. The downloadable scene dependency set uses Isaac 6.1 assets.
- Windows hosts also need PowerShell 7 and an Ubuntu WSL distribution containing
  the Linux application prerequisites. Simulator and App run on Windows;
  ROS, Web and ML run inside WSL.

Install the simulator, ROS and OS packages before Setup. Setup does not install
drivers, run sudo or change WSL networking. Typical ROS development dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros2_ws/src deployment/linux/runtime/gripper_adapter \
  --ignore-src -r -y --skip-keys openusd
sudo apt install python3-venv python3-colcon-common-extensions python3-yaml \
  ros-jazzy-moveit ros-jazzy-moveit-servo \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-moveit-resources-panda-description ros-jazzy-control-msgs \
  ros-jazzy-realtime-tools libboost-system-dev libjsoncpp-dev libopencv-dev
```

## Setup and daily operation

Native Linux:

```bash
bash setup.sh --isaac-root /path/to/isaacsim
bash setup.sh --check
bash deployment/linux/start_isaac.sh
# In another terminal:
bash start.sh
bash start.sh --no-app
bash stop.sh --status
bash stop.sh
```

Windows:

```powershell
./setup.ps1 --isaac-root C:/isaacsim --wsl-distribution Ubuntu-24.04
./setup.ps1 --check
./deployment/windows/start_isaac.ps1
./start.ps1
./start.ps1 --no-app
./stop.ps1 --status
./stop.ps1
```

Setup obtains the scene assets and demo model, prepares isolated API/ML Python
environments, installs SAM and builds the native ROS/Servo stack. Repeated Setup
preserves existing target drafts and models. `--check` checks an installation
without installing or downloading; `--dry-run` prints planned work.

Start never installs, builds, trains or presses Play. Open Isaac Sim with the
project launcher and press Play yourself. Start Observe and each arm explicitly
in the UI. `--no-app` runs backend/Web for a regular browser at the printed URL.
Closing the dedicated App requests shutdown of its matching backend, with the
existing simulator pause lifecycle. Closing a regular browser tab does not stop
the backend. Stop/status remain available even when source copies differ.

## Platform configuration

Setup creates ignored files under `deployment/runtime/local/`. Simulator and ROS
paths belong in `locations.json`; the Windows WSL destination is in `launcher.json`.
Windows and WSL need separate source directories, configurations and builds.
Do not use a Windows `/mnt/...` directory as the WSL application checkout.

The WSL destination defaults to `$HOME/workspace/VisionGuidedPick`; choose another
with `--wsl-project-root /home/user/workspace/VisionGuidedPick` during Setup.
Start and Setup --check compare deployable sources on both sides, excluding local
settings, datasets, models and outputs. Existing WSL source is never blindly reset.
For an upgrade, stop the old deployment, clone the desired version into a parallel
directory and run Setup with an explicit new WSL destination. Preserve old data
until migration is complete; do not copy venvs, builds, PID records or owner locks.

Local Web/training overrides go in `deployment/runtime/local/teaching.yaml`.
Use exact bind/client addresses for LAN access; the default is loopback.
Training settings are operator YAML settings, not ordinary UI inputs.
Windows/WSL remains an experimental deployment profile.

## Scene assets

The main scene is
[FactoryRabbitDemo.camera-bracket-v2.usda](../deployment/factory_rabbit_20261005/scene/FactoryRabbitDemo.camera-bracket-v2.usda).
Keep its relative dependencies together. Setup acquires 54 pinned NVIDIA files,
including material-indirect textures, directly from the official asset server.
To acquire only the scene assets:

```bash
python3 prepare_scene_assets.py
python3 prepare_scene_assets.py --check
```

On Windows use `python`, or Isaac Sim's bundled `kit/python/python.exe`. Prepare
assets in the Windows checkout opened by Isaac Sim as well as the application
checkout as requested by Setup. A WSL-only download does not populate Windows USD
references. Offline authorized local asset roots are supported:

```bash
python3 prepare_scene_assets.py /path/to/asset-root
```

Valid files are reused; interrupted downloads can be retried. Mismatched existing
files are reported and preserved. Network bytes must match pinned official hashes;
only explicitly listed legacy hashes are accepted for offline USD files.
NVIDIA payloads are ignored by Git and retain their own terms. See
[asset sources](release/asset-provenance.md) and [third-party notices](../THIRD_PARTY_NOTICES.md).

## Default inference model

Setup installs the standalone Stanford Bunny model from the
[model Release](https://github.com/MortCat/VisionGuidedPick/releases/tag/demo-model-2cdd6434-r1).
`config/default_model.json` pins the archive, checkpoint and inference-contract
checksums. The bundle contains inference weights, class/preprocessing metadata,
a minimal target profile and content notices. It contains no dataset, training
job, evaluation report or historical metrics.

The model is preselected in Observe but is not automatically loaded or run.
For another object, use Teach Target with images covering the expected poses,
distances, lighting and backgrounds, plus independent validation images.
The Teach page displays training progress and result summaries without a
validation-prediction image gallery.

Public Release attachments are downloadable without credentials once the
repository is public. While the repository is private, existing GitHub access
or an authorized offline bundle is required:

```bash
bash setup.sh --isaac-root /path/to/isaacsim --demo-model-bundle /path/to/stanford-bunny-2cdd6434.bundle.zip
```

Windows accepts the same option. Bundles are cached outside Git. Existing target
drafts are retained; conflicting model bytes are refused rather than overwritten.
ML dependencies and model content retain their respective upstream terms.

## Configuration and updates

- `config/teaching.yaml`: annotation, training and Web settings.
- `config/observe.yaml`: inference scheduling, preview and cache limits.
- `config/factory_demo/variants/camera_bracket_v2/`: per-arm application settings.
- `deployment/runtime/local/`: ignored host configuration.

Rebuild after changing source or authored scene dependencies to refresh the
installed source fingerprint. Restart backend participants together when ROS
bindings change. API and ML workers use separate environments; keep one pip
OpenCV provider per environment and do not mutate shared live environments.
See [startup contract](architecture/startup-setup-contract.md) and
[model bundle API](api/teach-target-api.md) for lifecycle and storage details.
