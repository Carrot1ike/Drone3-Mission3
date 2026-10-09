from copy import deepcopy
import math
import unittest
import numpy as np
from .approach_reference import CoordinatedApproach


class ApproachReference(unittest.TestCase):
    def test_endpoints_monotonic_path_and_analytic_derivatives(self):
        r=CoordinatedApproach(10.,10.5,.65,.3,.7,.4)
        rows=[r.at(t) for t in np.linspace(10.,20.5,1001)]
        for row in (rows[0],rows[-1]):
            self.assertEqual(row['height_rate_m_s'],0.)
            self.assertEqual(row['standoff_rate_m_s'],0.)
            self.assertEqual(row['height_accel_m_s2'],0.)
        self.assertAlmostEqual(rows[0]['height_m'],.7)
        self.assertAlmostEqual(rows[-1]['height_m'],.4)
        self.assertAlmostEqual(rows[-1]['standoff_m'],.3)
        self.assertTrue(np.all(np.diff([x['height_m'] for x in rows])<=0))
        self.assertLess(max(abs(x['height_rate_m_s']) for x in rows),.06)
        self.assertLess(max(abs(x['standoff_rate_m_s']) for x in rows),.063)
        t=14.;eps=1e-4;a=r.at(t);lo=r.at(t-eps);hi=r.at(t+eps)
        self.assertAlmostEqual(a['height_rate_m_s'],(hi['height_m']-lo['height_m'])/(2*eps),places=8)
        self.assertAlmostEqual(a['height_accel_m_s2'],(hi['height_rate_m_s']-lo['height_rate_m_s'])/(2*eps),places=8)

    def test_invalid_reference_and_nonfinite_clock_rejected(self):
        for args in ((0.,0.,.65,.3,.7,.4),(0.,10.,.1,.3,.7,.4),(0.,10.,.65,0.,.7,.4),
                     (0.,10.,.65,.3,.3,.4),(0.,float('nan'),.65,.3,.7,.4)):
            with self.assertRaises(ValueError):CoordinatedApproach(*args)
        with self.assertRaises(ValueError):CoordinatedApproach(0.,10.,.65,.3,.7,.4).at(float('nan'))

    def mission(self,velocity_policy='vehicle_relative'):
        from .test_straight_preparation import StraightPreparation
        mission,own,vehicle=StraightPreparation().fixture(x=22.,y=6.25)
        mission.c.update(terminal_policy='last_fix_bridge',lower_align_world_height_m=.4,
                         last_fix_standoff_m=.3,approach_motion_policy='coordinated_quintic',
                         coordinated_velocity_feedback_policy=velocity_policy,
                         coordinated_approach_duration_s=10.5,max_descent_speed_m_s=.06)
        from .controller import Mission3
        updated=Mission3(mission.cfg,mission.cfg['entry']['positions'][0])
        updated.phase='LOWER_ALIGN';updated.discovery_time=10.;updated.last_good=20.
        updated.straight_start=18.
        return updated,own,vehicle

    def test_reference_moves_forward_and_down_without_completing_early(self):
        mission,own,vehicle=self.mission()
        command=mission.update(20.,own,vehicle)
        self.assertEqual(command.phase,'LOWER_ALIGN')
        self.assertEqual(command.approach_reference['progress'],0.)
        # Evaluate perfect reference tracking; the target creates simultaneous
        # forward catch-up and descent rather than a height-only first segment.
        ref=mission.coordinated_approach.at(25.25)
        own=deepcopy(own);vehicle=deepcopy(vehicle)
        own['position_world_m'][2]=ref['height_m']
        vehicle['relative_pad_world_m']=[ref['standoff_m'],0.,.3-ref['height_m']]
        command=mission.update(25.25,own,vehicle)
        self.assertGreater(command.velocity_world_m_s[0],.25)
        self.assertLess(command.velocity_world_m_s[2],0.)
        self.assertEqual(command.phase,'LOWER_ALIGN')
        self.assertFalse(command.physical_success)
        self.assertFalse(command.motors_off)

    def test_original_straight_clearance_still_required(self):
        from .test_straight_preparation import StraightPreparation
        fixture=StraightPreparation()
        for args in ({'rate':math.radians(3.8)},{'x':34.}):
            mission,own,vehicle=fixture.fixture(**args)
            mission.c.update(approach_motion_policy='coordinated_quintic',coordinated_approach_duration_s=10.5)
            commands=fixture.ticks(mission,own,vehicle)
            self.assertTrue(all(x.phase=='LINEUP' for x in commands))

    def test_concurrent_confirmation_keeps_dwell_completion_and_cross_gates(self):
        for cross in (0.,.04):
            with self.subTest(cross=cross):
                mission,own,vehicle=self.mission()
                mission.coordinated_confirm_policy='concurrent'
                mission.update(20.,own,vehicle)
                own=deepcopy(own);vehicle=deepcopy(vehicle)
                own['position_world_m'][2]=.4
                vehicle['relative_pad_world_m']=[.3,cross,-.1]
                for t in (29.,30.,30.49):
                    self.assertEqual(mission.update(t,own,vehicle).phase,'LOWER_ALIGN')
                result=mission.update(30.5,own,vehicle)
                self.assertEqual(result.phase,'APPROACH' if cross==0. else 'LOWER_ALIGN')
                self.assertFalse(result.physical_success)
                self.assertFalse(result.motors_off)

    def test_concurrent_confirmation_resets_after_height_failure(self):
        mission,own,vehicle=self.mission()
        mission.coordinated_confirm_policy='concurrent'
        mission.update(20.,own,vehicle)
        own=deepcopy(own);vehicle=deepcopy(vehicle)
        own['position_world_m'][2]=.4
        vehicle['relative_pad_world_m']=[.3,0.,-.1]
        mission.update(29.,own,vehicle)
        own['position_world_m'][2]=.44
        self.assertEqual(mission.update(30.,own,vehicle).phase,'LOWER_ALIGN')
        own['position_world_m'][2]=.4
        self.assertEqual(mission.update(30.5,own,vehicle).phase,'LOWER_ALIGN')
        self.assertEqual(mission.update(31.49,own,vehicle).phase,'LOWER_ALIGN')
        self.assertEqual(mission.update(31.5,own,vehicle).phase,'APPROACH')

    def test_perfect_reference_velocity_is_not_damped(self):
        mission,own,vehicle=self.mission('reference_relative')
        mission.update(20.,own,vehicle)
        ref=mission.coordinated_approach.at(25.25)
        own=deepcopy(own);vehicle=deepcopy(vehicle)
        own['position_world_m'][2]=ref['height_m']
        vehicle['relative_pad_world_m']=[ref['standoff_m'],0.,.3-ref['height_m']]
        vehicle['relative_velocity_world_m_s']=[ref['standoff_rate_m_s'],0.,-ref['height_rate_m_s']]
        command=mission.update(25.25,own,vehicle)
        # Perfect tracking requires no D correction; only the vehicle and
        # moving-reference feedforward terms remain. Ordinary slew is inactive
        # at this five-second fixture interval, and speed remains below .4.
        expected=vehicle['vehicle_velocity_world_m_s'][0]-ref['standoff_rate_m_s']
        self.assertAlmostEqual(command.velocity_world_m_s[0],expected,places=10)
        self.assertEqual(command.phase,'LOWER_ALIGN')

    def test_default_legacy_velocity_feedback_is_preserved(self):
        mission,own,vehicle=self.mission()
        mission.update(20.,own,vehicle)
        ref=mission.coordinated_approach.at(25.25)
        own=deepcopy(own);vehicle=deepcopy(vehicle)
        own['position_world_m'][2]=ref['height_m']
        vehicle['relative_pad_world_m']=[ref['standoff_m'],0.,.3-ref['height_m']]
        vehicle['relative_velocity_world_m_s']=[ref['standoff_rate_m_s'],0.,-ref['height_rate_m_s']]
        command=mission.update(25.25,own,vehicle)
        expected=vehicle['vehicle_velocity_world_m_s'][0]-(1.-.35)*ref['standoff_rate_m_s']
        self.assertAlmostEqual(command.velocity_world_m_s[0],expected,places=10)

    def test_unknown_velocity_feedback_policy_rejected(self):
        with self.assertRaises(ValueError):self.mission('invalid')


if __name__=='__main__':unittest.main()
