"""Analytic accelerating heights and causal-history boundary tests."""
import unittest
import numpy as np

from .estimator import SelfMotion
from .settings import load, controller_settings
from .vertical_fit import instantaneous_range_velocity


class VerticalFit(unittest.TestCase):
    def test_irregular_range_timestamps_recover_current_accelerating_speed(self):
        ranges_t=np.array([.025,.05,.075,.125,.20,.275,.35,.375])
        imu_t=np.arange(0.,.401,.01)
        for origin in (0.,123.):
            for acceleration in (-.04,0.,.06):
                height=.70-.03*ranges_t+.5*acceleration*ranges_t**2
                fit=instantaneous_range_velocity(list(zip(origin+ranges_t,height)),
                    list(zip(origin+imu_t,np.full(len(imu_t),acceleration))),origin+.4)
                self.assertAlmostEqual(fit,-.03+acceleration*.4,places=10)

    def test_future_uncovered_duplicate_and_gapped_inputs_rejected(self):
        t=np.arange(0.,.401,.05); ranges=list(zip(t,.7-.03*t))
        imu=list(zip(np.arange(0.,.401,.01),np.zeros(41)))
        for r,i,now in ((ranges,imu,.3),(ranges,imu[6:],.4),
                        (ranges+[(.4,.6)],imu,.4),(ranges,[(0.,0.),(.4,0.)],.4)):
            with self.assertRaises(ValueError):instantaneous_range_velocity(r,i,now)

    def test_default_keeps_linear_filter_and_missing_imu_is_explicit_fallback(self):
        cfg=controller_settings(load());entry=cfg['entry']['positions'][0]
        own=SelfMotion(cfg,entry)
        self.assertEqual(own.range_velocity_model,'linear_window')
        own.range_heights.extend((t,.7-.03*t) for t in np.arange(0.,.401,.05))
        self.assertAlmostEqual(own.fitted_range_velocity(.4),-.03)
        own.range_velocity_model='imu_compensated'
        self.assertAlmostEqual(own.fitted_range_velocity(.4),-.03)
        self.assertEqual(own.vertical_fit_fallbacks,1)

    def test_future_interval_average_imu_cannot_change_current_velocity(self):
        # Even zero future acceleration must be rejected: a right-end stamped
        # interval average is not available before the interval has ended.
        t=np.arange(6)*.01; ranges=list(zip(t,.7-.02*t))
        imu_t=np.arange(7)*.01
        for future_acceleration in (0.,1.):
            acceleration=np.zeros(7);acceleration[-1]=future_acceleration
            with self.assertRaises(ValueError):
                instantaneous_range_velocity(ranges,list(zip(imu_t,acceleration)),.055)
        exact=instantaneous_range_velocity(ranges,list(zip(imu_t,np.zeros(7))),.06)
        self.assertAlmostEqual(exact,-.02,places=10)
        imu_t[-1]=np.nextafter(.06,np.inf)
        with self.assertRaises(ValueError):
            instantaneous_range_velocity(ranges,list(zip(imu_t,np.zeros(7))),.06)


if __name__=='__main__':unittest.main()
