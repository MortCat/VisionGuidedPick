# Robot-specific MoveIt configuration

This optional Linux package launches `move_group`, Jazzy `servo_node`, and the common gateway with a MoveIt backend. It contains no fabricated URDF, SRDF, controllers or measured robot parameters.

Supply `motion.moveit.srdf_path` (`robot_semantic` identity matching calibration id) and `moveit_config_path` (`moveit_config` identity). The verified parameter YAML has exactly three maps: `move_group`, `servo`, `gateway`. Include robot-specific kinematics/joint limits/planning pipelines/controller mappings in the appropriate maps. They are immutable deployment assets referenced by `config/settings.yaml`, not a second application Settings singleton.

Configure the Servo group/TCP/world frames, SI speed commands, input `servo_twist`, status `servo_status`, collision checking, and the robot controller output topic. `gateway` needs robot-description kinematics parameters for MoveGroupInterface. Controllers must expose the YAML `controller_trajectory` FollowJointTrajectory and `controller_gripper` GripperCommand Actions; actual `/joint_states` and capture-time TF are mandatory. Controller/Isaac deployment is supplied by the user. Missing velocity measurements do not prove stopped.

Jazzy Servo uses `pause_servo` (`std_srvs/SetBool`) and `switch_command_type` (`moveit_msgs/ServoCommandType`). Pausing is acknowledged only after a zero-twist halt and measured joint stop. Grasp contact permission is limited to the selected track and declared gripper touch links; other collision pairs remain unchanged. Planning and Servo commands share the gateway owner.

API references: [Jazzy Servo implementation](https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/moveit_servo/src/servo_node.cpp), [MoveGroupInterface](https://moveit.picknik.ai/main/api/html/classMoveGroupInterface.html).
