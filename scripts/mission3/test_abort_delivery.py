"""Failed-flight delivery must precede any bounded observation completion."""
import unittest
from copy import deepcopy
from .abort_delivery import AbortDelivery, finite_control_state


class AbortContract(unittest.TestCase):
    def own(self):
        return dict(position_world_m=[1.,2.,1.], velocity_world_m_s=[0.,0.,0.],
                    heading_rad=0., rpy_rad=[0.,0.,0.], gyro_body_rad_s=[0.,0.,0.],
                    range_accurate=True, flow_valid=True, flow_fusion_usable=True,
                    flow_fusion_reason='floor_patch_clear')

    def delivered(self, start=10.):
        d = AbortDelivery(start, 'search_timeout_without_metric_pose')
        d.note_wire(start)
        d.note_physics_step(start+.001)
        return d

    def test_completion_requires_wire_and_physics_after_failure(self):
        d = AbortDelivery(10., 'failure')
        d.note_wire(9.99); d.note_physics_step(10.)
        self.assertIsNone(d.observe(10., self.own(), 10.))
        for i in range(1,31):
            now = 10.+i*.02
            self.assertIsNone(d.observe(now, self.own(), now))
        d.note_wire(10.6)
        self.assertIsNone(d.observe(10.62, self.own(), 10.62))
        d.note_physics_step(10.621)
        self.assertEqual(d.observe(10.64, self.own(), 10.64), 'sensor_stationary_hold_observed')
        self.assertFalse(d.summary()['safe_recovery_or_motor_stop_confirmed'])

    def test_motion_or_surface_uncertainty_resets_continuous_dwell(self):
        for key, value in [('velocity_world_m_s', [.051,0.,0.]), ('flow_fusion_reason', 'pad_patch_contained'),
                           ('flow_valid', False), ('range_accurate', False)]:
            with self.subTest(key=key):
                d = self.delivered()
                d.observe(10., self.own(), 10.)
                bad = deepcopy(self.own()); bad[key] = value
                self.assertIsNone(d.observe(10.4, bad, 10.4))
                for i in range(25):
                    now = 10.6+i*.02
                    self.assertIsNone(d.observe(now, self.own(), now))
                self.assertEqual(d.observe(11.10, self.own(), 11.10), 'sensor_stationary_hold_observed')

    def test_future_stale_and_nonfinite_samples_cannot_certify_hold(self):
        for stamp in [10.1,9.9,float('nan')]:
            d = self.delivered()
            self.assertIsNone(d.observe(10., self.own(), stamp))
            self.assertFalse(d.latest['stable'])
        bad = self.own(); bad['position_world_m'][0] = float('nan')
        self.assertFalse(finite_control_state(bad))
        d = self.delivered()
        self.assertIsNone(d.observe(10., bad, 10.))
        self.assertFalse(d.latest['stable'])
        # A fresh sample after a long gap starts a new dwell, not a held gap.
        d = self.delivered(); d.observe(10., self.own(), 10.)
        self.assertIsNone(d.observe(10.6, self.own(), 10.6))
        self.assertEqual(d.latest['stable_dwell_s'], 0.)

    def test_unobserved_stationarity_has_bounded_failure_exit(self):
        d = self.delivered(); bad = self.own(); bad['flow_valid'] = False
        self.assertIsNone(d.observe(12.99, bad, 12.99))
        self.assertEqual(d.observe(13., bad, 13.), 'bounded_delivery_elapsed_without_stationary_evidence')
        self.assertEqual(d.summary()['failure_latched'], 'search_timeout_without_metric_pose')

    def test_duplicate_or_reverse_observation_time_is_rejected(self):
        d = self.delivered(); d.observe(10., self.own(), 10.)
        for stamp in [10.,9.99,float('nan')]:
            with self.assertRaises(ValueError): d.observe(stamp, self.own(), stamp)


if __name__ == '__main__': unittest.main()
