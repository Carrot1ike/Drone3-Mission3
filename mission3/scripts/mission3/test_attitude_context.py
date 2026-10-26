"""Exercise actual C state separation, including independent gyro evolution."""
import unittest
import numpy as np
from .firmware import FirmwareCore


class AttitudeContexts(unittest.TestCase):
    def test_pc_initialization_and_updates_cannot_reset_onboard_state(self):
        onboard=FirmwareCore((1.,2.,1.),.5)
        pc=FirmwareCore((9.,8.,.7),-.2,role="pc_imu")
        np.testing.assert_allclose(onboard.state()["position_world_m"],[1.,2.,1.])
        for tick in range(10,1010,10):
            onboard.process_noise(tick);onboard.predict(tick,(0.,0.,9.81),(0.,0.,.2));onboard.finalize()
            pc.process_noise(tick);pc.predict(tick,(0.,0.,9.81),(0.,0.,-.1));pc.finalize()
        onboard_q=np.array(onboard.state()["quaternion_wxyz"])
        before=pc.state()
        for tick in range(1010,1210,10):
            onboard.tof(tick,.31,.0025);onboard.flow(tick,[30.,-20.],.01);onboard.finalize()
        self.assertEqual(before,pc.state())
        self.assertGreater(np.linalg.norm(onboard_q-np.array(before["quaternion_wxyz"])),.2)

    def test_pc_rejects_surface_measurements(self):
        pc=FirmwareCore(role="pc_imu")
        with self.assertRaises(ValueError):pc.tof(10,.3)
        with self.assertRaises(ValueError):pc.flow(10,[0.,0.])


if __name__=="__main__":unittest.main()
