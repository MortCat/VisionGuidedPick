# Setup and startup contract

Setup is explicit provisioning. Start never installs, builds, trains, starts arm
tasks or changes Isaac Play/Stop. Linux exposes setup.sh / start.sh / stop.sh;
Windows exposes setup.ps1 / start.ps1 / stop.ps1. Both require Python 3.12+.

Native Linux owns simulator, backend and App locally. Windows owns simulator and
App; a separate WSL checkout owns ROS, Web, ML and its native build. Windows Start
compares deployable source identity before launch. Stop/status do not require
source equality. Existing source/configuration is preserved by Setup.

Setup verifies pinned official assets and the demo bundle, prepares isolated
Python environments, installs SAM and builds the selected ROS/Servo packages.
It registers the default model with the Web storage's existing exclusive owner
lock, preserving target drafts. It does not activate Observe. Driver, simulator
and OS/ROS development packages remain host prerequisites.

Start launches backend/Web before opening the optional App. --no-app omits the
desktop window. App close stops only its matching Web instance. Explicit Stop
closes only the dedicated browser identified by process identity/profile;
unrelated browser windows are retained. Stop does not recursively invoke itself.

The project simulator launcher enables its Python Server on 127.0.0.1:8226 for
the existing pause/close lifecycle unless local settings select an explicit MCP
owner. Host paths, WSL destination and interpreter overrides remain in ignored
deployment/runtime/local files. No network access policy is broadened.

While an arm is stopped at Home with no assigned target, transient background
scene/inference/motion readiness gaps do not fail an empty cycle. Device feedback
must remain fresh, stopped and unpaused. Full readiness is checked when selecting
a target. Moving or assigned tasks retain existing recovery and safety handling.
