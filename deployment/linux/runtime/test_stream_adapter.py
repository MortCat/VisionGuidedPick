"""Boundary checks for stamped stream acceptance and expiry."""
import unittest
from stream_adapter import StreamAdapter, JOINTS

Q = [0., -.5, 0., -2.3, 0., 1.8, .785]


class AdapterChecks(unittest.TestCase):
    def test_future_points_interpolate_from_measured_without_jump(self):
        s = StreamAdapter()
        q = Q.copy(); q[0] = .04
        s.accept(JOINTS, [(.1, q)], 10., 10., 50., Q)
        self.assertAlmostEqual(s.sample(10.05, 50.05)[0], .02)

    def test_future_endpoint_is_checked_by_rate_not_mistaken_for_instant_jump(self):
        s = StreamAdapter()
        q=Q.copy();q[0]=.2
        s.accept(JOINTS, [(.2,q)], 10.,10.,50.,Q)
        self.assertAlmostEqual(s.sample(10.1,50.1)[0],.1)

    def test_reversed_joint_order_is_mapped_correctly(self):
        s = StreamAdapter()
        q = Q.copy(); q[2] = .02
        s.accept(JOINTS[::-1], [(.1, q[::-1])], 10., 10., 50., Q)
        self.assertAlmostEqual(s.sample(10.1, 50.1)[2], .02)

    def test_expired_and_out_of_order_streams_are_rejected(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(.1, Q)], 10., 10., 50., Q)
        with self.assertRaises(ValueError):
            s.accept(JOINTS, [(.1, Q)], 9.99, 10., 50., Q)
        with self.assertRaises(TimeoutError):
            s.sample(10.31, 50.31)
        with self.assertRaises(TimeoutError):
            s.sample(10.01, 50.51)

    def test_source_tick_can_precede_next_servo_packet_without_expiring(self):
        s = StreamAdapter()
        step = 1./12.
        q=Q.copy();q[0]+=.005
        s.accept(JOINTS,[(0.,Q),(step,q)],10.,10.,50.,Q)
        # A clock callback can run one full source tick before the new window.
        self.assertEqual(s.sample(10.+2*step,50.1),q)
        next_q=q.copy();next_q[0]+=.005
        s.accept(JOINTS,[(0.,q),(step,next_q)],10.+step,10.+2*step,50.11,q)
        self.assertAlmostEqual(s.sample(10.+2*step,50.12)[0],next_q[0])
        with self.assertRaises(TimeoutError):s.sample(10.+6*step,50.3)

    def test_nonfinite_and_discontinuous_targets_are_rejected(self):
        s = StreamAdapter()
        q = Q.copy(); q[0] = float('nan')
        with self.assertRaises(ValueError):
            s.accept(JOINTS, [(.1, q)], 10., 10., 50., Q)
        q[0] = .5
        with self.assertRaises(ValueError):
            s.accept(JOINTS, [(.1, q)], 10., 10., 50., Q)

    def test_future_segment_velocity_limit_is_checked(self):
        s = StreamAdapter()
        q = Q.copy(); q[0] = 1.
        with self.assertRaisesRegex(ValueError, 'velocity'):
            s.accept(JOINTS, [(0., Q), (.01, q)], 10., 10., 50., Q)

    def test_short_future_anchor_cannot_make_an_instantaneous_jump(self):
        s = StreamAdapter()
        q = Q.copy(); q[0] = .1
        with self.assertRaisesRegex(ValueError, 'velocity'):
            s.accept(JOINTS, [(.001, q)], 10., 10., 50., Q)

    def test_repeated_stamp_does_not_extend_wall_heartbeat(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(.1, Q)], 10., 10., 50., Q)
        s.accept(JOINTS, [(.1, Q)], 10., 10., 50.49, Q)
        with self.assertRaises(TimeoutError):
            s.sample(10., 50.51)

    def test_advancing_rolling_tail_refreshes_heartbeat_with_fixed_head(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(0., Q), (.1, Q)], 10., 10., 50., Q)
        s.accept(JOINTS, [(0., Q), (.2, Q)], 10., 10.1, 50.4, Q)
        self.assertEqual(s.sample(10.15, 50.6), Q)

    def test_frozen_clock_with_advancing_tail_does_not_extend_heartbeat(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(0., Q), (.1, Q)], 10., 10., 50., Q)
        s.accept(JOINTS, [(0., Q), (.2, Q)], 10., 10., 50.4, Q)
        with self.assertRaises(TimeoutError):
            s.sample(10., 50.6)

    def test_bounded_old_head_is_valid_when_rolling_window_covers_now(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(0., Q), (.4, Q)], 9.7, 10., 50., Q)
        self.assertEqual(s.sample(10.05, 50.1), Q)
        with self.assertRaisesRegex(ValueError, 'horizon'):
            s.accept(JOINTS, [(0., Q), (1., Q)], 10.05, 10.05, 50.2, Q)

    def test_clock_reset_requires_reset_before_new_epoch(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(.1, Q)], 10., 10., 50., Q)
        with self.assertRaisesRegex(ValueError, 'clock reset'):
            s.sample(0., 50.01)
        with self.assertRaisesRegex(ValueError, 'clock reset'):
            s.accept(JOINTS, [(.1, Q)], 0., 0., 50.02, Q)
        s.reset()
        s.accept(JOINTS, [(.1, Q)], 0., 0., 50.03, Q)
        self.assertEqual(s.sample(.05, 50.04), Q)

    def test_measured_state_and_controller_clocks_must_be_finite(self):
        s = StreamAdapter()
        for measured in (Q[:-1], [float('nan')]+Q[1:]):
            with self.assertRaises(ValueError):
                s.accept(JOINTS, [(.1, Q)], 10., 10., 50., measured)
        with self.assertRaises(ValueError):
            s.accept(JOINTS, [(.1, Q)], 10., float('nan'), 50., Q)
        s.accept(JOINTS, [(.1, Q)], 10., 10., 50., Q)
        with self.assertRaises(ValueError):
            s.sample(10., float('inf'))

    def test_duplicate_joint_and_exhausted_points_are_rejected(self):
        s = StreamAdapter()
        with self.assertRaises(ValueError):
            s.accept(JOINTS[:-1]+[JOINTS[0]], [(.1, Q)], 10., 10., 50., Q)
        with self.assertRaises(ValueError):
            s.accept(JOINTS, [(.1, Q)], 9.85, 10., 50., Q)

    def test_rejected_packet_does_not_replace_the_last_valid_stream(self):
        s = StreamAdapter()
        s.accept(JOINTS, [(.1, Q)], 10., 10., 50., Q)
        with self.assertRaises(ValueError):
            s.accept(JOINTS, [(0., Q), (.01, [1.]+Q[1:])], 10.01, 10.01, 50.01, Q)
        self.assertEqual(s.stamp, 10.)
        self.assertEqual(s.sample(10.02, 50.02), Q)

    def test_discontinuity_compares_at_measured_capture_time(self):
        s = StreamAdapter()
        end = list(Q); end[0] += .2
        s.accept(JOINTS, [(0., Q), (.2, end)], 10., 10.15, 50., Q, 10.)
        self.assertAlmostEqual(s.sample(10.15, 50.01)[0], Q[0]+.15)
        with self.assertRaisesRegex(ValueError, 'discontinuity'):
            s.accept(JOINTS, [(0., Q), (.2, end)], 10., 10.15, 50.02, Q, 10.15)

    def test_stale_measured_capture_time_is_rejected(self):
        s = StreamAdapter()
        with self.assertRaisesRegex(ValueError, 'capture time'):
            s.accept(JOINTS, [(0., Q), (.2, Q)], 10., 10.15, 50., Q, 9.8)

    def test_float32_stamp_rounding_does_not_create_microsecond_segment(self):
        s = StreamAdapter()
        near = list(Q); near[0] += .009
        s.accept(JOINTS, [(0., near), (.083333, near)], 315.016666666,
                 315.016666666, 50., Q, 315.016662597)
        self.assertEqual(len(s.points), 2)


class HermiteChecks(unittest.TestCase):
    def test_observation_soft_errors_are_metrics_not_motion_vetoes(self):
        s=StreamAdapter(max_tracking_error=float('inf'))
        head=Q.copy();head[0]+=.3;tail=head.copy();tail[0]+=.05
        s.accept(JOINTS,[(0.,head,[0.]*7),(.1,tail,[0.]*7)],10.,10.,50.,Q)
        self.assertGreater(s.last_acceleration_peak,20.)
        self.assertAlmostEqual(s.sample(10.05,50.05)[0],Q[0]+.325)


    def test_rolling_windows_keep_commands_not_delayed_measured_knots(self):
        s=StreamAdapter(acceleration_limits=[3.75]*7);v=[.1]+[0.]*6
        end=Q.copy();end[0]+=.03
        s.accept(JOINTS,[(0.,Q,v),(.3,end,v)],10.,10.,50.,Q,10.,v)
        head=Q.copy();head[0]+=.02;tail=Q.copy();tail[0]+=.04
        measured=Q.copy();measured[0]+=.001
        s.accept(JOINTS,[(0.,head,v),(.2,tail,v)],10.2,10.2,50.2,measured,10.05,[0.]*7)
        self.assertAlmostEqual(s.sample(10.25,50.25)[0],Q[0]+.025)
        self.assertTrue(all(abs(point[0]-10.05)>1e-6 for point in s.points))

    def test_rounded_shared_tail_keeps_command_history_not_delayed_anchor(self):
        s=StreamAdapter();v=[.1]+[0.]*6
        end=Q.copy();end[0]+=.02
        s.accept(JOINTS,[(0.,Q,v),(.2,end,v)],10.,10.,50.,Q,10.,v)
        head=10.200000000000001
        tail=end.copy();tail[0]+=.02
        lagging=Q.copy();lagging[0]-=.04
        s.accept(JOINTS,[(0.,end,v),(.2,tail,v)],head,10.25,50.25,lagging,10.183333333,[0.]*7)
        self.assertAlmostEqual(s.sample(10.3,50.3)[0],Q[0]+.03)
        self.assertAlmostEqual(s.velocity(10.3)[0],.1)
        self.assertTrue(all(abs(point[0]-10.183333333)>1e-6 for point in s.points))

    def test_actual_gap_still_uses_measured_anchor_and_checks_speed(self):
        s=StreamAdapter();v=[.1]+[0.]*6
        end=Q.copy();end[0]+=.02
        s.accept(JOINTS,[(0.,Q,v),(.2,end,v)],10.,10.,50.,Q)
        head=end.copy();head[0]+=.10
        tail=head.copy();tail[0]+=.02
        with self.assertRaisesRegex(ValueError,'velocity'):
            s.accept(JOINTS,[(0.,head,v),(.2,tail,v)],10.23,10.25,50.25,Q,10.22,[0.]*7)

    def test_running_window_still_checks_actual_measured_error(self):
        s=StreamAdapter();v=[.1]+[0.]*6;end=Q.copy();end[0]+=.03
        s.accept(JOINTS,[(0.,Q,v),(.3,end,v)],10.,10.,50.,Q)
        wrong=Q.copy();wrong[0]-=.3
        with self.assertRaisesRegex(ValueError,'discontinuity'):
            s.accept(JOINTS,[(0.,Q,v),(.2,end,v)],10.1,10.1,50.1,wrong,10.05,[0.]*7)


    def test_relaxed_observation_threshold_does_not_change_legacy_default(self):
        q=Q.copy();q[0]+=.3
        with self.assertRaisesRegex(ValueError,'discontinuity'):
            StreamAdapter().accept(JOINTS,[(0.,q),(.2,q)],10.,10.,50.,Q)
        StreamAdapter(max_tracking_error=.4).accept(JOINTS,[(0.,q),(.2,q)],10.,10.,50.,Q)

    def test_velocity_matched_constant_speed(self):
        s=StreamAdapter(acceleration_limits=[3.75]*7); q=Q.copy(); end=q.copy();end[0]+=.04
        v=[.2]+[0.]*6;s.accept(JOINTS,[(0.,q,v),(.2,end,v)],10.,10.,50.,q)
        self.assertAlmostEqual(s.sample(10.1,50.1)[0],q[0]+.02)

    def test_internal_position_overshoot_is_rejected(self):
        s=StreamAdapter();q=Q.copy();q[0]=2.895;v=[2.]+[0.]*6
        with self.assertRaisesRegex(ValueError,'Interpolated position'):
            s.accept(JOINTS,[(0.,q,v),(.2,q,[-2.]+[0.]*6)],10.,10.,50.,q)

    def test_acceleration_excess_is_rejected(self):
        s=StreamAdapter(acceleration_limits=[3.75]*7);q=Q.copy();end=q.copy();end[0]+=.05
        with self.assertRaisesRegex(ValueError,'Interpolated acceleration'):
            s.accept(JOINTS,[(0.,q,[0.]*7),(.1,end,[0.]*7)],10.,10.,50.,q)

class VelocityReferenceTest(unittest.TestCase):
    def test_constant_speed_and_exhausted_tail(self):
        stream=StreamAdapter();end=Q.copy();end[0]+=.04;v=[.2]+[0.]*6
        stream.accept(JOINTS,[(0.,Q,v),(.2,end,v)],10.,10.,50.,Q)
        self.assertAlmostEqual(stream.velocity(10.1)[0],.2)
        self.assertEqual(stream.velocity(10.25),[0.]*7)
        self.assertEqual(stream.velocity(9.9),[0.]*7)

    def test_cubic_derivative_matches_position(self):
        stream=StreamAdapter();end=Q.copy();end[0]+=.04
        stream.accept(JOINTS,[(0.,Q,[0.]*7),(.2,end,[0.]*7)],10.,10.,50.,Q)
        t=10.08;epsilon=1e-5
        numerical=(stream.interpolate(stream.points,t+epsilon)[0]-stream.interpolate(stream.points,t-epsilon)[0])/(2*epsilon)
        self.assertAlmostEqual(stream.velocity(t)[0],numerical,places=6)

    def test_position_only_segment_keeps_its_slope(self):
        stream=StreamAdapter();end=Q.copy();end[0]+=.02
        stream.accept(JOINTS,[(0.,Q),(.2,end)],10.,10.,50.,Q)
        self.assertAlmostEqual(stream.velocity(10.1)[0],.1)


if __name__ == '__main__':
    unittest.main()
