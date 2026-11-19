"""Focused owner/opening checks; no ROS node or simulator is started."""
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace as S
from rclpy.action import GoalResponse
from control_msgs.action import FollowJointTrajectory
from inspection_bridge import InspectionBridgeNode, JOINTS
from stream_adapter import StreamAdapter


def fixture():
    node=InspectionBridgeNode.__new__(InspectionBridgeNode)
    node.mode_lock=threading.RLock();node.sim_time=1.;node.clock_count=1
    node.owners={'arm_01':'HOLD'};node.conflicting_source={'arm_01':False}
    node.command_epoch={'arm_01':1};node.finger_epoch={'arm_01':1};node.finger_operation={'arm_01':None}
    node.feedback={'arm_01':S(name=JOINTS,position=[0.]*7+[.02,.02],header=S(stamp=S(sec=1,nanosec=0)))}
    node.feedback_wall={'arm_01':time.monotonic()};node.finger_targets={'arm_01':[.02,.02]}
    node.holds={};node.errors={};node.active={}
    node.streams={'arm_01':S(reset=lambda:None)}
    node.get_logger=lambda:S(warning=lambda message:None)
    return node


def goal(target):
    return S(trajectory=S(joint_names=JOINTS[-2:],points=[
        S(positions=[.02,.02],time_from_start=S(sec=0,nanosec=0)),
        S(positions=[target,target],time_from_start=S(sec=1,nanosec=0))]))


class OpeningOwnerTest(unittest.TestCase):
    def test_servo_keeps_rolling_velocities_with_delayed_measured_positions(self):
        node=fixture();node.owners['arm_01']='SERVO';node.streams['arm_01']=StreamAdapter()
        q=[0.,-.5,0.,-2.3,0.,1.8,.785];v=[.1]+[0.]*6
        node.feedback['arm_01'].position=q+[.04,.04];node.feedback['arm_01'].velocity=v+[0.,0.]
        def message(stamp,start):
            end=start.copy();end[0]+=.02
            return S(header=S(stamp=S(sec=1,nanosec=int(round((stamp-1)*1e9)))),joint_names=JOINTS[:7],points=[
                S(time_from_start=S(sec=0,nanosec=0),positions=start,velocities=v),
                S(time_from_start=S(sec=0,nanosec=200000000),positions=end,velocities=v)])
        node.on_stream('arm_01',message(1.,q))
        self.assertTrue(all(len(p)==3 for p in node.streams['arm_01'].points))
        node.sim_time=1.1;node.feedback['arm_01'].header.stamp.nanosec=83333333
        delayed=q.copy();delayed[0]=-.03;node.feedback['arm_01'].position=delayed+[.04,.04]
        commanded=q.copy();commanded[0]=.01
        node.on_stream('arm_01',message(1.1,commanded))
        self.assertEqual(node.owners['arm_01'],'SERVO')
        self.assertTrue(all(len(p)==3 for p in node.streams['arm_01'].points))
        self.assertAlmostEqual(node.streams['arm_01'].sample(1.15,time.monotonic())[0],.015)

    def test_rejected_velocity_stream_does_not_kill_bridge(self):
        for has_velocity in [False,True]:
            for disk_failure in [False,True]:
                with self.subTest(velocities=has_velocity,disk_failure=disk_failure):
                    node=fixture();node.owners['arm_01']='SERVO'
                    node.feedback['arm_01'].velocity=[0.]*9
                    node.streams['arm_01'].accept=lambda *a,**k:(_ for _ in ()).throw(ValueError('Trajectory discontinuity from measured state'))
                    node.streams['arm_01'].points=[]
                    msg=S(header=S(stamp=S(sec=1,nanosec=0)),joint_names=JOINTS[:7],points=[
                        S(time_from_start=S(sec=0,nanosec=0),positions=[0.]*7,velocities=[0.]*7 if has_velocity else [])])
                    with patch('inspection_bridge.Path.write_text',side_effect=OSError('fixture disk full') if disk_failure else None) as report:
                        node.on_stream('arm_01',msg)
                    self.assertEqual(node.owners['arm_01'],'HOLD')
                    self.assertEqual(node.errors['arm_01'],'Trajectory discontinuity from measured state')
                    self.assertEqual(node.finger_targets['arm_01'],[.02,.02])
                    self.assertFalse(report.called)  # Command rejection never writes diagnostic files.

    def test_stop_invalidates_opening_and_holds_measured_fingers(self):
        node=fixture();node.accept('arm_01',goal(.04));token=node.finger_operation['arm_01']
        node.finger_targets['arm_01']=[.04,.04];node.hold('arm_01')
        self.assertNotEqual(node.finger_epoch['arm_01'],token)
        self.assertEqual(node.finger_targets['arm_01'],[.02,.02])
        self.assertEqual(node.accept('arm_01',goal(.04)),GoalResponse.REJECT)  # Wait for old Action to unwind.

    def test_servo_release_preserves_completed_closure_through_repeated_handoffs(self):
        node=fixture();node.owners['arm_01']='SERVO'
        node.finger_targets['arm_01']=[0.,0.]
        for _ in range(2):
            response=S();node.set_servo('arm_01',S(data=False),response)
            self.assertTrue(response.success)
            self.assertEqual(node.owners['arm_01'],'HOLD')
            self.assertEqual(node.finger_targets['arm_01'],[0.,0.])
            self.assertEqual(node.holds['arm_01'][-2:],[0.,0.])
        # Explicit opening is still a new command and replaces the closure.
        self.assertEqual(node.accept('arm_01',goal(.04)),GoalResponse.ACCEPT)

    def test_completed_arm_trajectory_preserves_grip_until_explicit_release(self):
        node=fixture();node.owners['arm_01']='PLAN'
        node.finger_targets['arm_01']=[0.,0.];node.plan_context=threading.local()
        handle=S(status=4,request=S(trajectory=S(joint_names=JOINTS[:7],points=[S(positions=[.01]*7)])))
        with patch('joint_trajectory_bridge.JointTrajectoryBridgeNode.execute',return_value='arrived'):
            self.assertEqual(node.execute('arm_01',handle),'arrived')
        self.assertEqual(node.owners['arm_01'],'HOLD')
        self.assertEqual(node.finger_targets['arm_01'],[0.,0.])
        self.assertEqual(node.holds['arm_01'][-2:],[0.,0.])
        self.assertEqual(node.holds['arm_01'][:7],[.01]*7)


class MeasuredContactStopTest(unittest.TestCase):
    def run_close(self, startup_delay=0., speed=.1, contact=.028):
        node=fixture();node.sim_time=1.
        node.feedback['arm_01'].position=[0.]*7+[.04,.04]
        node.finger_targets['arm_01']=[.04,.04]
        node.finger_epoch={'arm_01':1};node.finger_operation={'arm_01':1}
        node.holds={'arm_01':[0.]*7+[.04,.04]}
        start=node.sim_time
        trajectory=S(joint_names=JOINTS[-2:],points=[
            S(positions=[.04,.04],time_from_start=S(sec=0,nanosec=0)),
            S(positions=[0.,0.],time_from_start=S(sec=1,nanosec=0))])
        class ActionHandle:
            def __init__(self):
                self.request=S(trajectory=trajectory);self.is_cancel_requested=False
                self.aborted=False;self.succeeded=False
            def publish_feedback(self,message): pass
            def abort(self): self.aborted=True
            def succeed(self): self.succeeded=True
            def canceled(self): pass
        handle=ActionHandle()
        def advance(dt):
            node.sim_time+=dt
            previous=node.feedback['arm_01'].position[-2:]
            command=node.finger_targets['arm_01']
            actual=previous if node.sim_time-start<startup_delay else [
                max(contact,commanded,last-speed*dt) for commanded,last in zip(command,previous)]
            node.feedback['arm_01'].position=[0.]*7+actual
            node.feedback_wall['arm_01']=time.monotonic()
        with patch('inspection_bridge.time.sleep',side_effect=advance):
            result=node.execute_fingers('arm_01',handle)
        return node,handle,result,node.sim_time-start

    def assert_contact_hold(self,node,handle,result):
        self.assertTrue(handle.aborted)
        self.assertFalse(handle.succeeded)
        self.assertEqual(result.error_code,FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
        self.assertIn('closing setpoint retained',result.error_string)
        self.assertEqual(node.finger_targets['arm_01'],[0.,0.])
        for value in node.feedback['arm_01'].position[-2:]:self.assertAlmostEqual(value,.028,delta=.0005)
        self.assertEqual(node.holds['arm_01'][-2:],node.finger_targets['arm_01'])

    def test_closing_stall_retains_closure_without_claiming_target_arrival(self):
        node,handle,result,elapsed=self.run_close()
        self.assert_contact_hold(node,handle,result)
        self.assertLess(elapsed,1.)

    def test_startup_latency_is_not_contact_at_fully_open_aperture(self):
        node,handle,result,elapsed=self.run_close(startup_delay=.25)
        self.assert_contact_hold(node,handle,result)
        self.assertGreater(elapsed,.25)

    def test_slow_continuous_motion_is_not_a_stall(self):
        node,handle,result,elapsed=self.run_close(speed=.01)
        self.assert_contact_hold(node,handle,result)
        self.assertGreater(elapsed,1.15)

    def test_empty_closure_reaches_target_without_false_contact(self):
        node,handle,result,elapsed=self.run_close(speed=.01,contact=0.)
        self.assertTrue(handle.succeeded)
        self.assertFalse(handle.aborted)
        self.assertEqual(result.error_code,0)
        self.assertLess(max(node.feedback['arm_01'].position[-2:]),.002)


class FinalCommandVelocityTest(unittest.TestCase):
    def test_arm_arrival_uses_validated_tolerance_and_respects_explicit_goal(self):
        from joint_trajectory_bridge import JointTrajectoryBridgeNode
        node=JointTrajectoryBridgeNode.__new__(JointTrajectoryBridgeNode)
        node.arm_position_tolerance={'arm_01':.015}
        request=S(trajectory=S(joint_names=['panda_joint3'],points=[S(positions=[0.])]),goal_tolerance=[])
        self.assertTrue(node.trajectory_goal_reached('arm_01',request,[.0038]))
        self.assertFalse(node.trajectory_goal_reached('arm_01',request,[.016]))
        request.goal_tolerance=[S(name='panda_joint3',position=.001)]
        self.assertFalse(node.trajectory_goal_reached('arm_01',request,[.0038]))

    def test_velocity_reaches_joint_command_and_hold_clears_it(self):
        from joint_trajectory_bridge import JointTrajectoryBridgeNode
        node=JointTrajectoryBridgeNode.__new__(JointTrajectoryBridgeNode)
        messages=[]
        node.get_clock=lambda:S(now=lambda:S(to_msg=lambda:S(sec=1,nanosec=0)))
        node.command_publishers={'arm_01':S(publish=messages.append)}
        node.command_count={'arm_01':0}
        JointTrajectoryBridgeNode.publish_command(node,'arm_01',JOINTS,[0.]*9,[.2]+[0.]*8)
        self.assertEqual(list(messages[-1].velocity),[.2]+[0.]*8)
        JointTrajectoryBridgeNode.publish_command(node,'arm_01',JOINTS,[0.]*9)
        self.assertEqual(list(messages[-1].velocity),[0.]*9)


if __name__=='__main__':unittest.main()
