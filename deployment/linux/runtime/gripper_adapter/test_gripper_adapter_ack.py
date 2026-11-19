"""Exercise the real adapter against a fake controller in an isolated ROS domain.

No simulator, real robot, possession inference or production ROS domain is used.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def wait(future, timeout=5.):
    end=time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:
        time.sleep(.005)
    if not future.done():
        raise TimeoutError('Isolated adapter action did not complete')
    return future.result()


class Fixture(Node):
    def __init__(self):
        super().__init__('gripper_ack_fixture', namespace='/arm_01')
        self.positions=(.04,.04)
        self.enable_updates=True
        self.case='blocked'
        group=ReentrantCallbackGroup()
        self.joints=self.create_publisher(JointState,'joint_states',qos_profile_sensor_data)
        self.timer=self.create_timer(.01,self.publish_joints,callback_group=group)
        self.server=ActionServer(self,FollowJointTrajectory,'arm_controller/follow_joint_trajectory',
            self.execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.client=ActionClient(self,GripperCommand,'gripper_controller/gripper_cmd',callback_group=group)

    def publish_joints(self):
        if not self.enable_updates:
            return
        msg=JointState()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=['panda_finger_joint1','panda_finger_joint2']
        msg.position=list(self.positions)
        self.joints.publish(msg)

    def execute(self,handle):
        time.sleep(.03)
        # Recorded closure: both fingers moved inward, but the adapter's
        # separate50ms stability clock has not yet elapsed on the latest sample.
        self.positions=(.0271,.0354)
        self.publish_joints()
        time.sleep(.02)
        result=FollowJointTrajectory.Result()
        if self.case in ('cancel','stale'):
            if self.case=='stale':
                self.enable_updates=False
            end=time.monotonic()+2.5
            while not handle.is_cancel_requested and time.monotonic()<end:
                time.sleep(.01)
            if not handle.is_cancel_requested:
                raise AssertionError('Cancellation did not reach the fake controller')
            handle.canceled()
            result.error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            return result
        handle.abort()
        result.error_code=(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED if self.case=='blocked'
                           else FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED)
        return result


def main():
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID']=='0':
        raise RuntimeError('Use an isolated non-production ROS_DOMAIN_ID, e.g.119')
    root=Path(__file__).resolve().parents[4]
    binary=root/'native_install/factory_gripper_adapter/lib/factory_gripper_adapter/gripper_adapter'
    rclpy.init()
    node=Fixture()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    adapter=subprocess.Popen([str(binary)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        if not node.client.wait_for_server(timeout_sec=8.):
            raise TimeoutError('Isolated native adapter server unavailable')
        for case in ('blocked','path','cancel','stale'):
            node.case=case
            node.positions=(.04,.04)
            node.enable_updates=True
            time.sleep(.25)
            goal=GripperCommand.Goal()
            goal.command.position=0.
            goal.command.max_effort=0.
            handle=wait(node.client.send_goal_async(goal))
            assert handle.accepted,case
            if case=='cancel':
                time.sleep(.08)
                wait(handle.cancel_goal_async())
            result=wait(handle.get_result_async())
            expected=(GoalStatus.STATUS_SUCCEEDED if case=='blocked' else
                      GoalStatus.STATUS_CANCELED if case=='cancel' else GoalStatus.STATUS_ABORTED)
            print(json.dumps(dict(case=case,status=result.status,position=result.result.position,
                                  stalled=result.result.stalled,reached_goal=result.result.reached_goal)),flush=True)
            assert result.status==expected,case
            if case=='blocked':
                assert result.result.stalled and not result.result.reached_goal
                assert abs(result.result.position-sum(node.positions)/2)<.001
        print('Four real-adapter ACK cases passed; no Holding claim or production commands.')
    finally:
        adapter.send_signal(signal.SIGINT)
        try:
            adapter.wait(timeout=4)
        except subprocess.TimeoutExpired:
            adapter.kill()
            adapter.wait(timeout=3)
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=3)


if __name__=='__main__':
    main()
