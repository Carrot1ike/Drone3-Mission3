"""Stationary observation and unobserved motion must have different uncertainty."""
import unittest
import numpy as np
from .uncertainty import MotionUncertainty

C={"initial_velocity_std_m_s":.03,"initial_gyro_bias_std_deg_s":.05,"initial_accel_bias_std_m_s2":.005,
   "accel_sample_std_m_s2":.02,"gyro_sample_std_deg_s":.08,
   "gyro_bias_walk_deg_s_sqrt_s":.001,"accel_bias_walk_m_s2_sqrt_s":.0001,"floor_flow_velocity_std_m_s":.02}
E={"initial_position_std_m":.1,"initial_yaw_std_deg":2.}


class Covariance(unittest.TestCase):
    def test_stationary_qualified_floor_does_not_reach_one_meter_on_timer(self):
        u=MotionUncertainty(E,C)
        for _ in range(30000):u.predict(.01,0,[0,0]);u.qualified_floor_flow([0,0],.13)
        self.assertLess(u.state()["position_std_m"],.3)
        self.assertGreater(u.state()["heading_std_deg"],2.)

    def test_sensor_loss_uncertainty_grows_without_static_timer(self):
        u=MotionUncertainty(E,C);start=u.state()["position_std_m"]
        for _ in range(1000):u.predict(.01,0,[0,0])
        self.assertGreater(u.state()["position_std_m"],start+.25)

    def test_initial_heading_uncertainty_survives_floor_updates_during_travel(self):
        u=MotionUncertainty(E,C)
        for _ in range(2000):u.predict(.01,0,[0,0]);u.qualified_floor_flow([.7,0],.13)
        self.assertGreater(u.state()["position_std_m"],.4)

    def test_joseph_is_positive_and_gauge_rotation_does_not_remove_information(self):
        u=MotionUncertainty(E,C)
        for _ in range(100):u.predict(.01,.4,[.2,-.1]);u.qualified_floor_flow([.3,.1],.13)
        old=np.linalg.eigvalsh(u.P);u.change_heading_gauge(.8)
        self.assertGreater(np.linalg.eigvalsh(u.P).min(),-1e-12)
        np.testing.assert_allclose(np.linalg.eigvalsh(u.P),old,atol=1e-12)

    def test_known_straight_heading_retains_systematic_floor(self):
        u=MotionUncertainty(E,C)
        for _ in range(1000):u.predict(.05,0,[0,0]);u.straight_heading_update(.02,-.0001)
        self.assertGreaterEqual(u.state()["heading_std_deg"],2.)
        self.assertGreater(np.linalg.eigvalsh(u.P).min(),-1e-12)
        for _ in range(100):u.predict(.01,0,[0,0])
        self.assertGreaterEqual(u.state()["heading_std_deg"],2.)


if __name__=="__main__":unittest.main()
