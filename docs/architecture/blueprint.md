# System blueprint

The product Web service owns teaching files, training jobs, publication and Observe process lifecycle. The Observe worker owns model inference and shared gantry tracking. Each arm's vision node refines associated wrist geometry; its task coordinator owns assignment, grasp/place and recovery.

ptc_core contains pure C++17 decisions. ptc_app supplies boundary configuration/type conversions. ptc_motion owns the common gateway; ptc_motion_moveit and ptc_moveit_config supply the optional MoveIt backend. No ROS, YAML or Isaac API belongs in ptc_core.

The UI issues high-level per-arm task commands. Planning, Servo and HOLD share one command owner; vision and the browser never publish joint commands. RGB-D is transformed at capture time into the fixed frame. Object velocity comes from visual measurements.

Both native Linux and Windows/WSL profiles use the same application contracts. Host paths and vendor libraries belong to local deployment settings; internal ptc_ names remain unchanged.

[Setup and startup contract](startup-setup-contract.md) defines platform entrypoints and App ownership.
