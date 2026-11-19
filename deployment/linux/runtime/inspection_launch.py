"""Primary MoveIt scenes and Servo use exactly the same robot model."""
import json,yaml,sys,subprocess
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
sys.path.insert(0,str(Path(__file__).parent))
from robot_config import build_config, settings_file

def generate_launch_description():
 directory=Path(__file__).parent;scene=json.loads((directory/'scene_config.json').read_text());nodes=[]
 for name,base in scene['bases'].items():
  cfg=build_config(name,base,directory);params=cfg.to_dict()
  params.update({'use_sim_time':True,'publish_robot_description':True,'publish_robot_description_semantic':True,
   'allow_trajectory_execution':True,'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
   'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
   'moveit_simple_controller_manager':{'controller_names':['arm_controller'],'arm_controller':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':[f'panda_joint{i}' for i in range(1,8)]}},
   'trajectory_execution.allowed_execution_duration_scaling':2.,'trajectory_execution.allowed_goal_duration_margin':5.})
  nodes.extend([Node(package='moveit_ros_move_group',executable='move_group',namespace=name,output='screen',parameters=[params]),
    Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=name,output='screen',parameters=[cfg.robot_description,{'use_sim_time':True}],remappings=[('/tf','/'+name+'/model/tf'),('/tf_static','/'+name+'/model/tf_static')])])
  # Use the same validated per-arm asset as the gateway. Loading MoveIt's
  # Panda demo here silently shadowed our collision, joint and timing settings.
  settings_path=settings_file(directory.parents[2],name)
  checker=Path(get_package_prefix('ptc_app'))/'lib/ptc_app/settings_check'
  settings=json.loads(subprocess.check_output([str(checker),str(settings_path),'--export-json'],text=True))
  asset=yaml.safe_load(Path(settings['moveit_config_path']).read_text(encoding='utf-8'))
  servo=dict(asset['servo']['moveit_servo'])
  period=float(servo['publish_period'])
  if not 0.<period<=1.:raise ValueError(f'invalid configured Servo period for {name}')
  print(f"{name} Servo asset={settings['moveit_config_path']} period={period} scene_proximity={servo['scene_collision_proximity_threshold']} check_collisions={servo['check_collisions']}")
  servo.update({'is_primary_planning_scene_monitor':False,'joint_topic':'/'+name+'/joint_states',
   'monitored_planning_scene_topic':'/'+name+'/monitored_planning_scene',
   'command_out_topic':'/'+name+'/servo/joint_trajectory','cartesian_command_in_topic':'/'+name+'/servo/twist_command',
   'pose_command_in_topic':'/'+name+'/servo/pose_target','status_topic':'/'+name+'/servo/status'})
  nodes.append(Node(executable=str(directory.parents[2]/'native_build/sim_servo/agent_sim_servo'),namespace=name,remappings=[('servo_node:__node','servo')],output='screen',parameters=[cfg.robot_description,cfg.robot_description_semantic,cfg.robot_description_kinematics,cfg.joint_limits,{'use_sim_time':True,'update_period':period,'planning_group_name':'panda_arm','moveit_servo':servo}]))
 return LaunchDescription(nodes)
