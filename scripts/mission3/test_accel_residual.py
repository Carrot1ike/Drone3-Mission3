"""Sensor-only bias identification and unusable-surface isolation."""
import math
import unittest
import numpy as np
from .settings import load,controller_settings
from .estimator import SelfMotion
from .track import rotation


class FloorResidualTests(unittest.TestCase):
    def own(self,heading=0.):
        cfg=controller_settings(load())
        cfg["observer"]["floor_accel_bias_observer"]={"integral_gain_per_s2":4.,"bias_limit_m_s2":.15}
        entry=dict(cfg["entry"]["positions"][0]);entry["heading_deg"]=math.degrees(heading)
        return SelfMotion(cfg,entry)

    def feed(self,own,t,acc=(.01,-.006,0.),flow_valid=True):
        return own.update({"stamp_s":t,"gyro_z_rad_s":0.,"accel_heading_m_s2":acc,
            "range_m":1.,"range_valid":True,"range_accurate":True,"range_new":True,
            "flow_angular_rad_s":[0.,0.],"flow_valid":flow_valid})

    def test_floor_reference_identifies_gravity_leak_before_blind_propagation(self):
        own=self.own()
        for k in range(2001):self.feed(own,k*.01)
        np.testing.assert_allclose(own.accel_bias_heading[:2],[.01,-.006],atol=.0001)
        bias=own.accel_bias_heading.copy()
        for k in range(2001,2501):self.feed(own,k*.01,flow_valid=False)
        np.testing.assert_array_equal(own.accel_bias_heading,bias)
        self.assertLess(np.linalg.norm(own.velocity[:2]),.001)

    def test_bias_is_heading_frame_and_rotates_once_into_world(self):
        own=self.own(math.pi/2)
        for k in range(2001):self.feed(own,k*.01,acc=(.01,0.,0.))
        self.assertGreater(own.accel_bias_heading[0],.0099)
        self.assertLess(abs(own.accel_bias_heading[1]),1e-10)
        for k in range(2001,2501):self.feed(own,k*.01,acc=(.01,0.,0.),flow_valid=False)
        self.assertLess(np.linalg.norm(own.velocity[:2]),.001)

    def test_moving_or_mixed_surface_cannot_fit_floor_bias(self):
        own=self.own();own.position[2]=.4
        own.last_visual={"stamp_s":0.,"relative_pad_world_m":np.array([0.,0.,-.1]),
                         "vehicle_velocity_world_m_s":np.array([.25,0.,0.]),"vehicle_heading_rad":0.}
        for k in range(20):
            own.last_visual["stamp_s"]=k*.01
            own.update({"stamp_s":k*.01,"gyro_z_rad_s":0.,"accel_heading_m_s2":[.1,0.,0.],
                "range_m":.1,"range_valid":True,"range_accurate":True,"flow_angular_rad_s":[0.,0.],"flow_valid":True})
        self.assertEqual(own.accel_bias_floor_updates,0)
        np.testing.assert_array_equal(own.accel_bias_heading,np.zeros(3))


if __name__=="__main__":unittest.main()
