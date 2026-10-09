"""Vehicle FF must not acquire speed from a changing position gauge."""
import unittest
import numpy as np
from .settings import load,controller_settings
from .estimator import VehicleState


class RelativeVelocity(unittest.TestCase):
    def fixture(self):
        c=controller_settings(load());c["observer"]["vehicle_velocity_method"]="relative_kinematics"
        return VehicleState(c)

    def feed(self,v,count=100,qualified=True,offset=0):
        for i in range(offset,offset+count):
            t=i*.05
            # Gauge drifts at .40m/s, but raw relative motion .11m/s and
            # independently measured floor speed .20m/s give car speed .31.
            v.observe([20+.40*t,6.25,.3],0,t,t+.05,
                      relative_pad_vehicle=[.8+.11*t,0,-.2],
                      self_velocity_vehicle=[.20,0,0],floor_velocity_usable=qualified)

    def test_changing_global_gauge_cannot_become_vehicle_ff(self):
        v=self.fixture();self.feed(v)
        self.assertAlmostEqual(v.velocity[0],.31,places=6)
        self.assertNotAlmostEqual(v.velocity[0],.40,places=3)

    def test_pad_flow_cannot_refresh_floor_referenced_speed(self):
        v=self.fixture();self.feed(v);before=v.velocity.copy()
        for i in range(100,200):
            t=i*.05
            v.observe([20+.40*t,6.25,.3],0,t,t+.05,
                      relative_pad_vehicle=[.8+3*t,0,-.2],
                      self_velocity_vehicle=[5,0,0],floor_velocity_usable=False)
        np.testing.assert_allclose(v.velocity,before)
        self.assertIn("held",v.velocity_source)

    def test_no_qualified_baseline_does_not_invent_speed(self):
        v=self.fixture();self.feed(v,qualified=False)
        np.testing.assert_allclose(v.velocity,0)
        self.assertIsNone(v.local_velocity)

    def test_common_position_anchor_does_not_change_relative_history(self):
        v=self.fixture();self.feed(v)
        old=[r[1].copy() for r in v.kinematics]
        v.change_anchor(np.array([2.,-1.,0.]),.1)
        for a,r in zip(old,v.kinematics):np.testing.assert_allclose(a,r[1])


if __name__=="__main__":unittest.main()
