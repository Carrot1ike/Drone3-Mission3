"""Touchdown negative controls, independent of a particular successful flight."""
import unittest
from .touchdown import TouchdownObserver

C={"assumed_mass_kg":.035,"assumed_motor_response_s":.02,"filter_window_s":.15,
   "support_residual_min_m_s2":1.2,"supported_accel_tolerance_m_s2":1.5,
   "gyro_limit_rad_s":.3,"tilt_limit_deg":10.,"support_confirm_s":.25,"hold_bad_grace_s":.25}


class ContactEvidence(unittest.TestCase):
    def feed(self, obs, duration, rotor_accel, measured_accel, enabled=True, roll=0.):
        result=False
        for k in range(int(duration/.001)):
            now=k*.001
            obs.predict_motor([rotor_accel*.035/4]*4,.001)
            obs.imu({"stamp_s":now,"specific_force_body_m_s2":[0,0,measured_accel],"gyro_body_rad_s":[0,0,0]})
            result=obs.contact_candidate(now,roll,0,enabled)
        return result

    def test_hover_specific_force_is_not_contact(self):
        obs=TouchdownObserver(C)
        self.assertFalse(self.feed(obs,2.,9.81,9.81))
        self.assertAlmostEqual(obs.latest["support_residual_m_s2"],0.,places=5)

    def test_supported_reduced_thrust_can_confirm(self):
        self.assertTrue(self.feed(TouchdownObserver(C),1.,7.,9.81))

    def test_free_fall_not_contact_or_supported_hold(self):
        obs=TouchdownObserver(C)
        self.assertFalse(self.feed(obs,1.,0.,0.))
        self.assertEqual(obs.hold_supported(1.,0,0),(False,False))
        self.assertEqual(obs.hold_supported(1.3,0,0),(False,True))

    def test_impact_or_tilt_or_disabled_descent_not_confirmation(self):
        self.assertFalse(self.feed(TouchdownObserver(C),1.,7.,20.))
        self.assertFalse(self.feed(TouchdownObserver(C),1.,7.,9.81,roll=.3))
        self.assertFalse(self.feed(TouchdownObserver(C),1.,7.,9.81,enabled=False))

    def test_model_error_can_fake_contact_must_remain_conditional(self):
        # Explicit counterexample: bad force/mass calibration causes a false
        # support residual even in hover. Never claim hardware validation.
        self.assertTrue(self.feed(TouchdownObserver(C),1.,7.,9.81))

    def test_uncalibrated_model_is_not_accepted_by_strict_policy(self):
        c={**C,"require_freeflight_calibration":True,"probe_motor_accel_max_m_s2":7.3}
        self.assertFalse(self.feed(TouchdownObserver(c),1.,7.,9.81))

    def test_freeflight_calibration_rejects_mass_gain_hover_counterexample(self):
        obs=TouchdownObserver({**C,"require_freeflight_calibration":True})
        for k in range(3000):
            sample={"stamp_s":k*.001,"specific_force_body_m_s2":[0,0,9.81],"gyro_body_rad_s":[0,0,0]}
            obs.predict_motor([7*.035/4]*4,.001)
            obs.calibrate_freeflight(sample,True);obs.imu(sample)
        self.assertTrue(obs.calibration_valid)
        self.assertAlmostEqual(obs.force_to_accel*sum(obs.force),9.81,delta=.02)
        self.assertFalse(obs.contact_candidate(3.,0,0,True))


if __name__=="__main__":unittest.main()
