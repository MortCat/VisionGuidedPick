"""Validate and interpolate Servo's stamped rolling trajectories without ROS."""
import math

LIMITS = [(-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973),
          (-3.0718, -.0698), (-2.8973, 2.8973), (-.0175, 3.7525), (-2.8973, 2.8973)]
JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
VELOCITY_LIMITS = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]


class StreamAdapter:
    def __init__(self, heartbeat_timeout_wall=.5, acceleration_limits=None, max_tracking_error=.12):
        self.acceleration_limits = acceleration_limits
        self.max_tracking_error = max_tracking_error
        self.heartbeat_timeout_wall = heartbeat_timeout_wall
        self.reset()

    def reset(self):
        self.stamp = None
        self.points = []
        self.arrival_wall = None
        self.heartbeat_end = None
        self.heartbeat_sim = None
        self.last_sim = None
        self.last_tracking_error = 0.
        self.last_acceleration_peak = 0.

    def check_time(self, now_sim, now_wall):
        if not math.isfinite(now_sim) or not math.isfinite(now_wall):
            raise ValueError('Nonfinite controller clock')
        if self.last_sim is not None and now_sim < self.last_sim-1e-6:
            raise ValueError('Simulation clock reset; reset stream before reuse')

    def accept(self, joints, points, stamp, now_sim, now_wall, measured, measured_sim=None, measured_velocity=None):
        self.check_time(now_sim, now_wall)
        measured_sim = now_sim if measured_sim is None else measured_sim
        if not math.isfinite(measured_sim) or measured_sim > now_sim+.03 or now_sim-measured_sim > .25:
            raise ValueError('Measured joint capture time is not fresh')
        if len(measured) != 7 or not all(math.isfinite(q) for q in measured):
            raise ValueError('Seven finite measured arm positions required')
        if len(joints) != 7 or set(joints) != set(JOINTS):
            raise ValueError('Expected each of the seven arm joints exactly once')
        if not points or not math.isfinite(stamp) or stamp > now_sim+.25+1e-6 or now_sim-stamp > .5:
            raise ValueError('Missing, future or expired trajectory')
        if self.stamp is not None and stamp < self.stamp-1e-6:
            raise ValueError('Out-of-order trajectory')
        order = [joints.index(j) for j in JOINTS]
        ordered = []
        last_t = -1.
        for item in points:
            duration, positions = item[:2]
            velocity = item[2] if len(item)>2 else None
            if velocity is not None and (len(velocity)!=7 or not all(math.isfinite(v) for v in velocity)):
                raise ValueError("Seven finite joint velocities required")
            if not math.isfinite(duration) or duration < 0 or duration <= last_t:
                raise ValueError('Trajectory times must increase')
            if len(positions) != 7 or not all(math.isfinite(q) for q in positions):
                raise ValueError('Nonfinite or incomplete positions')
            q = [positions[i] for i in order]
            if any(not lo <= x <= hi for x, (lo, hi) in zip(q, LIMITS)):
                raise ValueError('Position exceeds Panda joint limit')
            ordered.append((stamp+duration, q, [velocity[i] for i in order]) if velocity is not None else (stamp+duration, q))
            last_t = duration
        if ordered[-1][0]-ordered[0][0] > .75 or ordered[-1][0] > now_sim+.5:
            raise ValueError('Trajectory exceeds bounded rolling horizon')
        if ordered[-1][0] < now_sim-.02:
            raise ValueError('Trajectory is already exhausted')
        # A rolling Hermite window continues the already commanded path. A
        # delayed measured sample is evidence about that path, not a new knot.
        # Re-inserting measured sensor noise on every packet creates artificial
        # accelerations between the old capture time and the new window head.
        stamp_rounding = max(.0001, min(.001, 2.**(math.frexp(max(abs(measured_sim), 1.))[1]-25)))
        # ROS duration addition can put a shared rolling endpoint a few
        # floating-point ULPs past the previous tail. Reuse the existing
        # capture-time precision instead of splicing a delayed measurement
        # into an already commanded path and inventing a high-speed segment.
        overlap = (self.points and len(ordered[0]) == 3 and
                   self.points[0][0]-stamp_rounding <= ordered[0][0] <= self.points[-1][0]+stamp_rounding)
        if overlap:
            if self.points[0][0]-stamp_rounding <= measured_sim <= self.points[-1][0]+stamp_rounding:
                expected = self.interpolate(self.points, measured_sim)
                self.last_tracking_error=max(abs(a-b) for a,b in zip(expected,measured))
                if self.last_tracking_error > self.max_tracking_error:
                    raise ValueError('Trajectory discontinuity from measured state')
            # Keep bounded past command history for capture-time verification.
            # The new window owns its head onward; no measured anchor is spliced.
            head=ordered[0][0]; cutoff=now_sim-.30
            past=[point for point in self.points if point[0]<head-stamp_rounding]
            keep=[point for point in past if point[0]>=cutoff]
            earlier=[point for point in past if point[0]<cutoff]
            if earlier: keep.insert(0,earlier[-1])
            ordered=keep+ordered
        else:
            q_measured = self.interpolate(ordered, measured_sim)
            if measured_sim >= ordered[0][0]-stamp_rounding and max(abs(a-b) for a,b in zip(q_measured,measured)) > self.max_tracking_error:
                raise ValueError('Trajectory discontinuity from measured state')
            if measured_sim < ordered[0][0]-stamp_rounding:
                ordered.insert(0,(measured_sim,list(measured),list(measured_velocity)) if measured_velocity is not None else (measured_sim,list(measured)))
        if ordered[-1][0]-ordered[0][0] > .75:
            raise ValueError('Combined command history exceeds rolling horizon')
        for a, b in zip(ordered, ordered[1:]):
            ta, qa = a[:2]; tb, qb = b[:2]
            if len(a)==3 and len(b)==3:
                self.validate_cubic(a, b)
            if any(abs(b-a)/(tb-ta) > limit+1e-6
                   for a, b, limit in zip(qa, qb, VELOCITY_LIMITS)):
                raise ValueError('Trajectory segment exceeds Panda joint velocity limit')
        # Repeated simulation stamps can occur while a wall timer runs between
        # physics ticks. They must not keep a frozen stream heartbeat alive.
        # A rolling window may retain its oldest stamp while its tail advances.
        # Credit actual simulation-time and tail progress together; replaying a
        # frozen window (or changing it under a frozen clock) is not a heartbeat.
        if self.heartbeat_end is None or (ordered[-1][0] > self.heartbeat_end+1e-6 and now_sim > self.heartbeat_sim+1e-6):
            self.arrival_wall = now_wall
            self.heartbeat_end = ordered[-1][0]
            self.heartbeat_sim = now_sim
        self.stamp, self.points, self.last_sim = stamp, ordered, now_sim

    @staticmethod
    def interpolate(points, now):
        if now <= points[0][0]:
            return list(points[0][1])
        for a, b in zip(points, points[1:]):
            ta, qa = a[:2]; tb, qb = b[:2]
            if now <= tb:
                alpha = (now-ta)/(tb-ta)
                if len(a)==3 and len(b)==3:
                    u=alpha; dt=tb-ta
                    return [(2*u**3-3*u*u+1)*q0+(u**3-2*u*u+u)*dt*v0+(-2*u**3+3*u*u)*q1+(u**3-u*u)*dt*v1
                            for q0,q1,v0,v1 in zip(qa,qb,a[2],b[2])]
                return [q0+alpha*(q1-q0) for q0,q1 in zip(qa,qb)]
        return list(points[-1][1])

    def velocity(self, now):
        # Derivative of the same validated position reference. A held tail has
        # zero velocity; never keep driving past an exhausted rolling segment.
        if not self.points or now < self.points[0][0] or now >= self.points[-1][0]:
            return [0.] * 7
        for a, b in zip(self.points, self.points[1:]):
            ta, qa = a[:2]; tb, qb = b[:2]
            if now <= tb:
                dt=tb-ta; u=(now-ta)/dt
                if len(a)==3 and len(b)==3:
                    return [(6*u*u-6*u)*q0/dt+(3*u*u-4*u+1)*v0+
                            (-6*u*u+6*u)*q1/dt+(3*u*u-2*u)*v1
                            for q0,q1,v0,v1 in zip(qa,qb,a[2],b[2])]
                return [(q1-q0)/dt for q0,q1 in zip(qa,qb)]
        return [0.] * 7

    def validate_cubic(self, a, b):
        dt=b[0]-a[0]
        for j,(q0,q1,v0,v1) in enumerate(zip(a[1],b[1],a[2],b[2])):
            A=2*q0-2*q1+dt*(v0+v1); B=-3*q0+3*q1-dt*(2*v0+v1); C=dt*v0
            roots=[]
            if abs(A)>1e-12:
                disc=4*B*B-12*A*C
                if disc>=0: roots=[(-2*B+math.sqrt(disc))/(6*A),(-2*B-math.sqrt(disc))/(6*A)]
            elif abs(B)>1e-12: roots=[-C/(2*B)]
            lo,hi=LIMITS[j]
            if any(not lo-1e-9<=((A*u+B)*u+C)*u+q0<=hi+1e-9 for u in roots if 0<u<1):
                raise ValueError('Interpolated position exceeds joint limit')
            candidates=[0.,1.]+([-B/(3*A)] if abs(A)>1e-12 and 0<-B/(3*A)<1 else [])
            if any(abs((3*A*u*u+2*B*u+C)/dt)>VELOCITY_LIMITS[j]+1e-6 for u in candidates):
                raise ValueError('Interpolated velocity exceeds joint limit')
            acceleration=max(abs(2*B),abs(6*A+2*B))/dt**2
            self.last_acceleration_peak=max(self.last_acceleration_peak,acceleration)
            if self.acceleration_limits is not None and acceleration>self.acceleration_limits[j]+1e-6:
                raise ValueError('Interpolated acceleration exceeds joint limit')

    def sample(self, now_sim, now_wall):
        self.check_time(now_sim, now_wall)
        if self.stamp is None or now_wall-self.arrival_wall > self.heartbeat_timeout_wall:
            raise TimeoutError('Servo output heartbeat lost')
        # Clock and trajectory callbacks arrive independently. The old 30 ms
        # tail allowance was shorter than the measured 83 ms source step, so
        # one callback ordering gap latched the bridge into HOLD permanently.
        # Briefly keep the last validated position; never extrapolate a joint
        # command or refresh the stream heartbeat from a held sample.
        period = self.points[-1][0]-self.points[-2][0] if len(self.points)>1 else .03
        tail_allowance = max(.03, min(.25, 2*period))
        if now_sim > self.points[-1][0]+tail_allowance:
            raise TimeoutError('Servo trajectory expired')
        self.last_sim = now_sim
        return self.interpolate(self.points, now_sim)
