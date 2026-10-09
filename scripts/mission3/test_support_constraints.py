"""Wrong-surface identity/freshness negatives; no contact truth inputs."""
import json
import unittest
import numpy as np
from .settings import ROOT
from .touchdown import TouchdownObserver


class SupportConstraints(unittest.TestCase):
    def observer(self,require_fresh=False):
        settings=json.loads((ROOT/"config/mission3-rpyt-paired-pc-imu-margin.json").read_text())["contact_observer"]
        settings.update(supported_accel_tolerance_m_s2=.2,probe_motor_accel_max_m_s2=6.5,
                        support_body_height_above_pad_m=.0145,support_height_tolerance_m=.012)
        if require_fresh:settings["calibration_max_age_s"]=45.
        observer=TouchdownObserver(settings);observer.calibration_valid=True
        return observer

    def test_vehicle_body_floor_and_missing_height_cannot_complete_support(self):
        for height in (-.026,-.286,None):
            o=self.observer();o.force=np.ones(4)*(6.*.035/4)
            for k in range(101):
                t=k*.01;o.imu({"stamp_s":t,"specific_force_body_m_s2":[0.,0.,9.81],"gyro_body_rad_s":[0.,0.,0.]})
                self.assertFalse(o.contact_candidate(t,0.,0.,True,height))

    def test_matching_estimated_height_allows_confirmed_support(self):
        o=self.observer();o.force=np.ones(4)*(6.*.035/4)
        result=False
        for k in range(101):
            t=k*.01;o.imu({"stamp_s":t,"specific_force_body_m_s2":[0.,0.,9.81],"gyro_body_rad_s":[0.,0.,0.]})
            result=o.contact_candidate(t,0.,0.,True,.0145)
        self.assertTrue(result)

    def test_valid_latch_does_not_imply_recent_calibration(self):
        o=self.observer(True)
        self.assertFalse(o.calibration_is_fresh(1.))
        o.calibration_last_valid_s=1.
        self.assertTrue(o.calibration_is_fresh(45.9))
        self.assertFalse(o.calibration_is_fresh(46.01))
        self.assertFalse(o.calibration_is_fresh(.9))

    def test_numpy_estimator_height_has_a_serializable_log_boundary(self):
        o=self.observer();o.force=np.ones(4)*(6.*.035/4)
        o.imu({"stamp_s":1.,"specific_force_body_m_s2":[0.,0.,9.81],"gyro_body_rad_s":[0.,0.,0.]})
        o.contact_candidate(1.,0.,0.,True,np.float64(.0145))
        saved=json.loads(json.dumps(o.latest))
        self.assertIs(saved["support_height_gate_valid"],True)
        self.assertEqual(saved["estimated_body_height_above_pad_m"],.0145)


if __name__=="__main__":unittest.main()
