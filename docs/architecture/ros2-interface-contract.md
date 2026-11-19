# ROS 2 interface contract

Message, service and action definitions live in ros2_ws/src/ptc_interfaces. ROS names are
relative, bound through each selected settings profile; /clock, /tf and /tf_static are
global exceptions. The gateway remains the unique motion owner.

- Observe publishes shared pick_preview_tracks and pick_preview_tracks_arm_02 under
  /factory/observe. Each arm's vision owner associates the wrist observation with that shared ID.
- vision/capture_grasp_surface uses CaptureGraspSurface. The measured, masked wrist cloud
  carries capture pose/time and optional neighboring occupancy into existing grasp evaluation.
- TaskCommand controls one arm's task mode. TaskStatus carries cycle/track identity,
  phase, grasp preview and last terminal outcome. GripperStatus and ArmStatus carry actual feedback.
- Bin-load placement events are independent of command acknowledgement and software terminal status.
- /factory/observe/gantry_cloud is a read-only ObjectCloudPreview. Up to four recent surfaces
  within 350 ms are motion-aligned from visual velocity, capped at 4096 points. It is not a ground-truth input.

RGB/depth masks use original image pixels. TF must resolve at capture time; current-time
TF cannot substitute for an old image. Settings schema v2 binds names; retired five-view
endpoints are rejected. Some shared interface types remain for internal library compatibility,
without re-enabling five-view execution.

See [Teach Target API](../api/teach-target-api.md) and [Observe API](../api/observe-api.md) for the separate Web contract.
