# Configuration

config/teaching.yaml owns annotation, training and Web settings; local overrides go in
deployment/runtime/local/teaching.yaml. Train snapshots the resolved settings and dataset.
The Web service uses exact source-IP and same-origin write checks, without login/session.

config/observe.yaml owns confidence, preview/inference cadence, cache and process limits.
Model resolution, class map and preprocessing come from the verified published bundle.

config/factory_demo/variants/camera_bracket_v2/ contains the active per-arm YAML and
product overlays. Application settings schema v2 is validated at the boundary into immutable
typed configuration. Core code does not read YAML or mutate global settings during a tick.
No draft/null active setting may start work. Changing ROS bindings requires restarting all
participants from the same source/configuration.

Wrist standby inference defaults to 2 Hz; a routed approaching target or active pick restores
the working interval. Camera streaming continues. This changes processing cadence, not camera
publication. Joint limits, collision protection, controller ownership, Stop and measured
possession recovery remain in force.

Host-specific simulator/ROS paths belong to deployment/runtime/local/locations.json.
See ../getting-started.md for installation, path resolution and environment setup.
