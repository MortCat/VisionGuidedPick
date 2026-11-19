cmake_minimum_required(VERSION 3.16)
project(ptc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "action/CheckMotion.action"
  "msg/EventKey.msg" "msg/TrackedObject.msg" "msg/TrackedObjectArray.msg" "msg/CapabilityStatus.msg"
  "msg/Detection.msg" "msg/RgbdFrame.msg" "msg/DetectionArray.msg"
  "msg/ObjectCloudPreview.msg" "msg/GraspNeighborhood.msg" "msg/ObjectSurfaceGeometry.msg"
  "msg/TargetUpdate.msg" "msg/ArmStatus.msg" "msg/GripperStatus.msg"
  "msg/GraspCandidate.msg" "msg/GraspFeasibility.msg"
  "msg/InspectionViewStatus.msg" "msg/InspectionProgress.msg" "msg/TaskStatus.msg" "msg/CollisionGeometry.msg"
  "srv/TaskCommand.srv" "srv/StopArm.srv"
  "action/FollowTarget.action" "action/CaptureGraspSurface.action"
  "action/GripperCommand.action" "action/SyncScene.action"
  "action/EvaluateGraspCandidates.action"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
