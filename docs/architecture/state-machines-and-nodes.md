# Nodes, owners and state machines

| Responsibility | Owner |
| --- | --- |
| Images, annotations, jobs, publication | tools/teach_target application |
| Real model inference and shared IDs | Observe worker and observe_geometry |
| Wrist association and measured grasp surface | per-arm ptc_vision |
| Continuous task, grasp/place and recovery | per-arm ptc_task |
| Motion/gripper command ownership | ptc_motion gateway with optional MoveIt backend |
| Pure decisions | ptc_core |

These are responsibility boundaries, not one-process-per-row requirements.
Reuse existing state and measured feedback instead of adding duplicate booleans.

Each station starts Idle. A Start request admits continuous dispatch; PreparingPick,
Picking, Transporting and Placing precede reporting or recovery. Empty failure recovery
returns Home before the next cycle. Possible possession retains bin-release recovery;
unavailable equipment or exhausted recovery stops the station rather than claiming success.
Explicit Stop/Cancel and clock invalidation take priority over automatic dispatch.

A stopped arm at Home with no assigned target tolerates transient background readiness gaps; full readiness is required when selecting a target. Fresh device feedback, Stop and clock validity remain authoritative.
