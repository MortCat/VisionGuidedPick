"""Native device interfaces; does not start models, tasks, MoveIt, or joint owners."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    bindings = json.loads((directory / 'tcp_bindings.json').read_text())
    nodes = [Node(package='factory_gripper_adapter', executable='gripper_adapter',
                  parameters=[{'use_sim_time': True}], output='screen')]
    for arm, binding in bindings.items():
        p, q = binding['translation_m'], binding['rotation_xyzw']
        arguments = ['--x', str(p[0]), '--y', str(p[1]), '--z', str(p[2]),
                     '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                     '--frame-id', binding['parent'], '--child-frame-id', binding['child']]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name=arm + '_tcp_binding', arguments=arguments,
                          parameters=[{'use_sim_time': True}], output='screen'))
    return LaunchDescription(nodes)
