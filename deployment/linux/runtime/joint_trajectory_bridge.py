"""ROS-only trajectory/feedback adapter; the inspection owner chooses output mode."""
import math
import threading
import time
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

NAMES = ['arm_01', 'arm_02']
JOINTS = [f'panda_joint{i}' for i in range(1, 8)] + ['panda_finger_joint1', 'panda_finger_joint2']


class JointTrajectoryBridgeNode(Node):
    def __init__(self):
        super().__init__('factory_runtime_bridge', automatically_declare_parameters_from_overrides=True)
        self.group = ReentrantCallbackGroup()
        self.feedback, self.feedback_wall = {}, {}
        self.pending_feedback = {}
        self.command_count, self.feedback_count = dict.fromkeys(NAMES, 0), dict.fromkeys(NAMES, 0)
        self.active = dict.fromkeys(NAMES, False)
        self.goal_lock = threading.Lock()
        self.sim_time, self.clock_count = 0., 0
        self.tf_frames = set()
        self.command_publishers = {name: self.create_publisher(JointState, '/' + name + '/joint_command', 10)
                                   for name in NAMES}
        for name in NAMES:
            self.create_subscription(JointState, '/' + name + '/joint_states',
                                     lambda msg, name=name: self.on_joint(name, msg), qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.servers = [ActionServer(self, FollowJointTrajectory, '/' + name + '/arm_controller/follow_joint_trajectory',
                       execute_callback=lambda goal, name=name: self.execute(name, goal), callback_group=self.group,
                       goal_callback=lambda goal, name=name: self.accept(name, goal),
                       cancel_callback=lambda goal: CancelResponse.ACCEPT) for name in NAMES]
        self.last_report = 0.
        self.create_timer(.01, self.tick, callback_group=self.group)

    def on_joint(self, name, msg):
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or not set(JOINTS).issubset(msg.name) or not all(math.isfinite(q) for q in msg.position)):
            return
        received = time.monotonic()
        self.feedback_count[name] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current = self.feedback.get(name)
        if current is not None:
            current_stamp = current.header.stamp.sec + current.header.stamp.nanosec * 1e-9
            if stamp <= current_stamp:
                return
        pending = self.pending_feedback.get(name, [])
        if pending:
            old = pending[-1][0].header.stamp
            if stamp <= old.sec + old.nanosec * 1e-9:
                return
        self.pending_feedback[name] = (pending + [(msg, received)])[-8:]
        self.promote_clock_eligible_feedback(name)

    def promote_clock_eligible_feedback(self, name):
        pending = self.pending_feedback.get(name, [])
        if not self.clock_count or not pending:
            return
        future = []
        for msg, received in pending:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.sim_time - stamp
            if age < -.03:
                future.append((msg, received))
            elif age <= .25 and time.monotonic() - received <= 1.5:
                self.feedback[name], self.feedback_wall[name] = msg, received
        if future:
            self.pending_feedback[name] = future
        else:
            self.pending_feedback.pop(name, None)

    def on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        reset = self.clock_count and now < self.sim_time - .001
        self.sim_time = now
        self.clock_count += 1
        if reset:
            self.on_feedback_clock_reset()
        for name in NAMES:
            self.promote_clock_eligible_feedback(name)

    def on_feedback_clock_reset(self):
        self.feedback.clear()
        self.feedback_wall.clear()
        self.pending_feedback.clear()

    def on_tf(self, msg):
        self.tf_frames.update((item.header.frame_id, item.child_frame_id) for item in msg.transforms)

    def publish_command(self, name, joints, positions, velocities=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = list(joints), list(map(float, positions))
        msg.velocity = list(map(float, velocities)) if velocities is not None else [0.] * len(msg.name)
        self.command_publishers[name].publish(msg)
        self.command_count[name] += 1

    def accept(self, name, goal):
        trajectory = goal.trajectory
        if (self.active[name] or not trajectory.points or not trajectory.joint_names
                or len(set(trajectory.joint_names)) != len(trajectory.joint_names)
                or not all(joint in JOINTS for joint in trajectory.joint_names)
                or not self.feedback_fresh(name)):
            return GoalResponse.REJECT
        stamps = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in trajectory.points]
        if (stamps[0] < 0 or any(b <= a for a, b in zip(stamps, stamps[1:]))
                or any(len(point.positions) != len(trajectory.joint_names)
                       or not all(math.isfinite(q) for q in point.positions) for point in trajectory.points)):
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.active[name]:
                return GoalResponse.REJECT
            self.active[name] = True
        return GoalResponse.ACCEPT

    def trajectory_goal_reached(self, name, goal, actual):
        default = getattr(self, 'arm_position_tolerance', {}).get(name, .002)
        requested = {t.name: t.position for t in goal.goal_tolerance if t.position > 0.}
        return all(abs(a-b) < requested.get(joint, default) for joint,a,b in zip(
            goal.trajectory.joint_names, actual, goal.trajectory.points[-1].positions))

    def execute(self, name, handle):
        result = FollowJointTrajectory.Result()
        trajectory = handle.request.trajectory
        stamps = np.array([point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                           for point in trajectory.points])
        values = np.array([point.positions for point in trajectory.points])
        start = self.sim_time
        deadline = time.monotonic() + max(15., float(stamps[-1]) * 5 + 10)
        try:
            while True:
                if handle.is_cancel_requested:
                    current = self.feedback[name]
                    self.publish_command(name, current.name, current.position)
                    handle.canceled()
                    result.error_string = 'Canceled and held measured joint state'
                    return result
                if time.monotonic() > deadline:
                    raise RuntimeError('execute: wall deadline expired')
                if not self.feedback_fresh(name):
                    sample = self.feedback.get(name)
                    stamp = (sample.header.stamp.sec + sample.header.stamp.nanosec * 1e-9
                             if sample is not None else float('nan'))
                    wall_age = time.monotonic() - self.feedback_wall.get(name, 0.)
                    raise RuntimeError(f'execute: feedback_fresh rejected; '
                                       f'wall_age={wall_age:.6f} source_age={self.sim_time-stamp:.6f} '
                                       f'clock={self.sim_time:.9f} joint_stamp={stamp:.9f}')
                elapsed = self.sim_time - start
                desired = [float(np.interp(elapsed, stamps, values[:, i])) for i in range(values.shape[1])]
                self.publish_command(name, trajectory.joint_names, desired)
                state = dict(zip(self.feedback[name].name, self.feedback[name].position))
                actual = [state[joint] for joint in trajectory.joint_names]
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = trajectory.joint_names
                feedback.desired.positions, feedback.actual.positions = desired, actual
                feedback.error.positions = [a - b for a, b in zip(desired, actual)]
                handle.publish_feedback(feedback)
                if elapsed >= stamps[-1] and self.trajectory_goal_reached(name, handle.request, actual):
                    handle.succeed()
                    result.error_code = 0
                    return result
                time.sleep(.01)
        except Exception as error:
            handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(error)
            return result
        finally:
            self.active[name] = False
