"""One output owner per arm: HOLD, planned Action, or stamped Servo stream."""
import json
import math
import threading
import time
import subprocess
import sys
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'runtime'))
from model_variant import settings_file

import rclpy
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from joint_trajectory_bridge import JointTrajectoryBridgeNode, JOINTS, NAMES
from stream_adapter import StreamAdapter
from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus


class InspectionBridgeNode(JointTrajectoryBridgeNode):
    def __init__(self):
        super().__init__()
        checker = Path(get_package_prefix('ptc_app'))/'lib/ptc_app/settings_check'
        settings_root = Path(__file__).resolve().parents[3]
        self.arm_position_tolerance = {name: json.loads(subprocess.check_output(
            [str(checker), str(settings_file(settings_root,name)), '--export-json'], text=True))[
                'joint_position_tolerance_rad'] for name in NAMES}
        self.mode_lock = threading.RLock()
        self.owners = dict.fromkeys(NAMES, 'HOLD')
        self.holds = {}
        self.streams = {n: StreamAdapter(heartbeat_timeout_wall=1.5) for n in NAMES}
        self.last_output = {}
        self.errors = dict.fromkeys(NAMES, '')
        self.enabled_wall = {}
        self.last_sim = None
        self.finger_operation = dict.fromkeys(NAMES, None)
        self.finger_epoch = dict.fromkeys(NAMES, 0)
        self.finger_targets = {}
        self.command_epoch = dict.fromkeys(NAMES, 0)
        self.plan_context = threading.local()
        self.conflicting_source = dict.fromkeys(NAMES, False)
        self.create_timer(1., self.check_command_sources, callback_group=self.group)
        self.mode_pub = self.create_publisher(String, '/factory/control_owner', 10)
        for n in NAMES:
            self.create_subscription(JointTrajectory, '/'+n+'/servo/joint_trajectory',
                lambda msg, n=n: self.on_stream(n, msg), qos_profile_sensor_data,
                callback_group=self.group)
            self.create_service(SetBool, '/'+n+'/control/servo_enable',
                lambda req, res, n=n: self.set_servo(n, req, res), callback_group=self.group)

    def on_feedback_clock_reset(self):
        # Invalidate output immediately; old pre-reset hold targets must not be reused.
        with self.mode_lock:
            super().on_feedback_clock_reset()
            for name in NAMES:
                self.hold(name, 'Simulation clock reset')
                self.holds.pop(name, None)
                self.finger_targets.pop(name, None)
            self.last_sim = self.sim_time

    def measured(self, name):
        state = dict(zip(self.feedback[name].name, self.feedback[name].position))
        return [state[j] for j in JOINTS]

    def feedback_fresh(self, name):
        if not self.clock_count or name not in self.feedback or time.monotonic()-self.feedback_wall.get(name, 0.) > 1.5:
            return False
        stamp = self.feedback[name].header.stamp
        age = self.sim_time-(stamp.sec+stamp.nanosec*1e-9)
        return -.03 <= age <= .25

    def hold(self, name, reason='', *, preserve_fingers=False):
        self.command_epoch[name] += 1
        if not preserve_fingers:
            self.finger_epoch[name] += 1
        self.streams[name].reset()
        self.streams[name].acceleration_limits=None
        self.streams[name].max_tracking_error=.12
        self.owners[name], self.active[name], self.errors[name] = 'HOLD', False, reason
        if name in self.feedback:
            self.holds[name] = self.measured(name)
            if preserve_fingers and not reason and name in self.finger_targets:
                self.holds[name][-2:] = self.finger_targets[name]
            else:
                self.finger_targets[name] = self.holds[name][-2:]

    def check_command_sources(self):
        with self.mode_lock:
            for name in NAMES:
                publishers = self.get_publishers_info_by_topic('/'+name+'/joint_command')
                conflict = len(publishers) > 1 or any(item.node_name != self.get_name() or
                    item.node_namespace != self.get_namespace() for item in publishers)
                if conflict and not self.conflicting_source[name]:
                    self.hold(name, 'Another ROS joint command publisher is active')
                self.conflicting_source[name] = conflict

    def set_servo(self, name, req, res):
        with self.mode_lock:
            if self.conflicting_source[name]:
                res.success, res.message = False, 'Another ROS joint command publisher is active'
            elif self.owners[name] == 'PLAN':
                res.success, res.message = False, 'Cancel and finish the active planned Action first'
            elif req.data:
                if not self.feedback_fresh(name):
                    res.success, res.message = False, 'Fresh measured joint feedback required'
                else:
                    self.streams[name].reset()
                    # Native simulation commissioning uses the observation
                    # policy: record measured lag instead of latching HOLD at
                    # 0.12 rad. Panda bounds, stream expiry, owner and Servo
                    # collision/limit handling remain active.
                    self.streams[name].max_tracking_error = math.inf
                    self.owners[name], self.active[name], self.errors[name] = 'SERVO', True, ''
                    self.enabled_wall[name] = time.monotonic()
                    res.success, res.message = True, 'Servo owns output; planned Action commands blocked'
            else:
                self.hold(name, preserve_fingers=True)
                res.success, res.message = True, 'Held measured position; Servo output disconnected'
        return res

    def on_stream(self, name, msg):
        with self.mode_lock:
            if self.owners[name] != 'SERVO':
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            # Preserve Servo's velocities so overlapping windows remain one
            # Hermite command history instead of splicing delayed measurements.
            points = [(p.time_from_start.sec+p.time_from_start.nanosec*1e-9, p.positions, p.velocities)
                      if p.velocities else (p.time_from_start.sec+p.time_from_start.nanosec*1e-9, p.positions)
                      for p in msg.points]
            measured_sim = None
            try:
                measured_stamp = self.feedback[name].header.stamp
                measured_sim = measured_stamp.sec+measured_stamp.nanosec*1e-9
                velocities = dict(zip(self.feedback[name].name, self.feedback[name].velocity))
                measured_velocity = [velocities[j] for j in JOINTS[:7]] if all(j in velocities for j in JOINTS[:7]) else None
                self.streams[name].accept(msg.joint_names, points, stamp, self.sim_time,
                    time.monotonic(), self.measured(name)[:7], measured_sim, measured_velocity)
            except (ValueError, KeyError) as exc:
                self.hold(name, str(exc))

    def accept(self, name, goal):
        with self.mode_lock:
            trajectory = goal.trajectory
            def reject(reason):
                feedback = self.feedback.get(name)
                stamp = feedback.header.stamp if feedback else None
                measured_stamp = stamp.sec+stamp.nanosec*1e-9 if stamp else None
                self.get_logger().warning(
                    f'Trajectory goal rejected: {reason}; arm={name} owner={self.owners[name]} '
                    f'finger_operation={self.finger_operation[name]} conflict={self.conflicting_source[name]} '
                    f'source={self.sim_time:.9f} measured_stamp={measured_stamp} '
                    f'feedback_wall_age={time.monotonic()-self.feedback_wall.get(name,0.):.6f} '
                    f'joints={list(trajectory.joint_names)} positions={[list(p.positions) for p in trajectory.points]}')
                return GoalResponse.REJECT
            if self.conflicting_source[name] or not self.feedback_fresh(name):
                return reject('conflicting publisher or feedback source time not fresh')
            if trajectory.joint_names and set(trajectory.joint_names).issubset(JOINTS[-2:]):
                if self.owners[name] not in ('HOLD', 'SERVO') or self.finger_operation[name] is not None:
                    return reject('finger owner unavailable or another finger goal active')
                stamps = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points]
                if (len(set(trajectory.joint_names)) != len(trajectory.joint_names) or not stamps or stamps[0] < 0
                    or any(b <= a for a,b in zip(stamps,stamps[1:]))
                    or any(len(p.positions) != len(trajectory.joint_names) or
                        any(not math.isfinite(q) or q < 0 or q > .04 for q in p.positions) for p in trajectory.points)
                    or time.monotonic()-self.feedback_wall.get(name,0.) > .5):
                    return reject('finger trajectory bounds/shape/time or feedback wall age')
                self.finger_epoch[name] += 1
                self.finger_operation[name] = self.finger_epoch[name]
                self.finger_targets[name] = self.measured(name)[-2:]
                return GoalResponse.ACCEPT
            if self.owners[name] != 'HOLD' or self.finger_operation[name] is not None:
                return reject('arm owner unavailable or finger operation active')
            result = super().accept(name, goal)
            if result == GoalResponse.ACCEPT:
                self.owners[name] = 'PLAN'
            return result

    def execute(self, name, handle):
        if set(handle.request.trajectory.joint_names).issubset(JOINTS[-2:]):
            return self.execute_fingers(name, handle)
        issued = self.command_epoch[name]
        self.plan_context.command = (name, issued)
        try:
            return super().execute(name, handle)
        finally:
            self.plan_context.command = None
            with self.mode_lock:
                if self.command_epoch[name] == issued:
                    self.hold(name, preserve_fingers=True)
                    if handle.status == GoalStatus.STATUS_SUCCEEDED:
                        # Controller tolerance acknowledges completion; keep
                        # converging to the commanded endpoint for the gateway's
                        # measured TCP arrival check instead of freezing residuals.
                        for joint,position in zip(handle.request.trajectory.joint_names,
                                handle.request.trajectory.points[-1].positions):
                            self.holds[name][JOINTS.index(joint)] = float(position)

    def publish_command(self, name, names, values, velocities=None):
        command = getattr(self.plan_context, 'command', None)
        with self.mode_lock:
            if self.conflicting_source[name]:
                raise RuntimeError('Another ROS joint command publisher is active')
            if command and (command != (name, self.command_epoch[name]) or self.owners[name] != 'PLAN'):
                raise RuntimeError('Planned output invalidated by stop or clock reset')
            output_key = (self.sim_time, self.owners[name], self.command_epoch[name])
            if self.last_output.get(name) != output_key:
                self.last_output[name] = output_key
                return super().publish_command(name, names, values, velocities)

    def execute_fingers(self, name, handle):
        """Only tick publishes output; finger goals share the same arm owner."""
        trajectory = handle.request.trajectory
        stamps = np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points])
        values = np.array([p.positions for p in trajectory.points])
        start = self.sim_time
        issued = self.finger_operation[name]
        deadline = time.monotonic()+max(8.,float(stamps[-1])*5+3.)
        result = FollowJointTrajectory.Result()
        previous = None
        initial_actual = None
        stable_since = [None] * len(trajectory.joint_names)
        closing = any(start-target > .002 for start, target in zip(values[0], values[-1]))
        try:
            while True:
                if handle.is_cancel_requested:
                    with self.mode_lock:
                        self.finger_targets[name] = self.measured(name)[-2:]
                    handle.canceled();result.error_string = 'Finger goal canceled; measured fingers held'
                    return result
                if issued != self.finger_epoch[name] or self.sim_time < start-.001:
                    raise RuntimeError('Finger goal invalidated by control/clock reset')
                if time.monotonic()>deadline or time.monotonic()-self.feedback_wall.get(name,0.)>.5:
                    raise RuntimeError('Finger goal timed out or feedback became stale')
                elapsed = self.sim_time-start
                desired = [float(np.interp(elapsed,stamps,values[:,i])) for i in range(values.shape[1])]
                state = dict(zip(self.feedback[name].name,self.feedback[name].position))
                actual = [state[j] for j in trajectory.joint_names]
                with self.mode_lock:
                    if issued != self.finger_epoch[name]:
                        raise RuntimeError('Finger goal invalidated before output update')
                    merged = dict(zip(JOINTS[-2:],self.finger_targets[name]))
                    merged.update(zip(trajectory.joint_names,desired))
                    self.finger_targets[name] = [merged[j] for j in JOINTS[-2:]]
                feedback = FollowJointTrajectory.Feedback();feedback.joint_names=trajectory.joint_names
                feedback.desired.positions=desired;feedback.actual.positions=actual
                feedback.error.positions=[a-b for a,b in zip(desired,actual)];handle.publish_feedback(feedback)
                error=max(abs(a-b) for a,b in zip(actual,values[-1]))
                if elapsed>=stamps[-1] and error<.002:
                    handle.succeed();result.error_code=0;return result
                if previous is None:
                    previous=list(actual)
                    initial_actual=list(actual)
                    stable_since=[self.sim_time]*len(actual)
                else:
                    for i,(measured,last) in enumerate(zip(actual,previous)):
                        if abs(measured-last)>.0003:
                            stable_since[i]=self.sim_time
                            previous[i]=measured
                stalled=[i for i,(measured,command,stopped_at,initial) in enumerate(zip(actual,desired,stable_since,initial_actual))
                         if closing and initial-measured>.0003 and elapsed>=.05 and stopped_at is not None and
                         self.sim_time-stopped_at>=.10 and measured-command>.002]
                if stalled:
                    with self.mode_lock:
                        if issued != self.finger_epoch[name]:
                            raise RuntimeError('Finger goal invalidated before contact hold')
                        # A blocked position drive needs its closing setpoint
                        # to maintain compression. Completion reports measured
                        # stall; it must not replace the command with feedback.
                        merged = dict(zip(JOINTS[-2:], self.finger_targets[name]))
                        merged.update(zip(trajectory.joint_names, map(float, values[-1])))
                        self.finger_targets[name] = [merged[j] for j in JOINTS[-2:]]
                    result.error_code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string='Measured finger motion stalled during closing; closing setpoint retained'
                    handle.abort();return result
                time.sleep(.01)
        except Exception as exc:
            result.error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string=str(exc);handle.abort();return result
        finally:
            with self.mode_lock:
                if self.finger_operation[name] == issued:
                    self.finger_operation[name]=None
                    if name in self.holds and name in self.finger_targets:
                        self.holds[name][-2:]=self.finger_targets[name]

    def tick(self):
        now = time.monotonic()
        with self.mode_lock:
            reset = self.last_sim is not None and self.sim_time < self.last_sim-.001
            self.last_sim = self.sim_time
            for n in NAMES:
                if self.conflicting_source[n]:
                    continue
                if reset:
                    self.hold(n, 'Simulation clock reset')
                if not self.feedback_fresh(n):
                    if self.owners[n] == 'SERVO':
                        self.hold(n, 'Measured joint feedback lost')
                    continue
                if n not in self.holds:
                    self.holds[n] = self.measured(n)
                if self.owners[n] == 'HOLD':
                    self.publish_command(n, JOINTS, self.holds[n][:7]+self.finger_targets.get(n,self.holds[n][-2:]))
                elif self.owners[n] == 'SERVO':
                    try:
                        if self.streams[n].stamp is None and now-self.enabled_wall[n] < 1.5:
                            continue
                        q = self.streams[n].sample(self.sim_time, now)
                        current = self.measured(n)[:7]
                        stamp = self.feedback[n].header.stamp
                        measured_sim = stamp.sec+stamp.nanosec*1e-9
                        if self.sim_time-measured_sim > .25 or measured_sim > self.sim_time+.03:
                            raise ValueError('Measured joint capture time is not fresh')
                        expected = self.streams[n].interpolate(self.streams[n].points, measured_sim)
                        self.streams[n].last_tracking_error=max(abs(a-b) for a,b in zip(expected,current))
                        if self.streams[n].last_tracking_error > self.streams[n].max_tracking_error:
                            raise ValueError(f'Stream tracking error exceeds {self.streams[n].max_tracking_error:.2f} rad')
                        self.publish_command(n, JOINTS, list(q)+self.finger_targets.get(n,self.measured(n)[-2:]), self.streams[n].velocity(self.sim_time)+[0.,0.])
                    except (TimeoutError, ValueError) as exc:
                        self.hold(n, str(exc))
        if now-self.last_report < .1:
            return
        self.last_report = now
        report = {'owners': self.owners, 'errors': self.errors,
            'finger_targets':self.finger_targets,'finger_operation':self.finger_operation,
            'tracking_error_rad':{n:self.streams[n].last_tracking_error for n in NAMES},
            'interpolated_acceleration_peak_rad_s2':{n:self.streams[n].last_acceleration_peak for n in NAMES},
            'feedback_count': self.feedback_count, 'command_count': self.command_count,
            'clock_count': self.clock_count, 'sim_time': self.sim_time,
            'feedback': {n: dict(zip(msg.name, msg.position)) for n, msg in self.feedback.items()},
            'feedback_age_seconds': {n: now-self.feedback_wall.get(n, 0.) for n in NAMES},
            'command_source_conflict': self.conflicting_source, 'wall_time': time.time()}
        self.mode_pub.publish(String(data=json.dumps(report)))



if __name__ == '__main__':
    rclpy.init()
    node = InspectionBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
