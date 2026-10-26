"""Native image depth acceptance and qualified-range velocity noise/lag checks."""
from copy import deepcopy
import math
import unittest
import numpy as np
from . import test_search
from .estimator import SelfMotion
from .vision import Observer


class MetricHeightQuality(unittest.TestCase):
    def test_marginal_real_pixel_pose_cannot_replace_earlier_reliable_fix(self):
        helper=test_search.Search();helper.setUp();cfg=deepcopy(helper.cfg)
        cfg['observer']['metric_pose_min_depth_m']=.32
        own=SelfMotion(cfg,cfg['entry']['positions'][0]);own.position=np.array([17.,6.25,.4])
        own.heading=0.;own.history.append((0.,own.position.copy(),0.))
        observer=Observer(cfg,own)
        _,good=helper.projection(own.position,0.,[17.+.40-.25,6.25,.3])
        first=observer.frame(good,0.,.05)
        self.assertTrue(first['valid'],first)
        stamp=observer.vehicle.stamp;quality=deepcopy(observer.vehicle.quality)
        own.history.append((.1,own.position.copy(),0.))
        _,close=helper.projection(own.position,0.,[17.+.315-.25,6.25,.3])
        next_frame=observer.frame(close,.1,.15)
        self.assertFalse(next_frame['valid'])
        self.assertEqual(next_frame['reason'],'metric_pose_depth_guard')
        self.assertEqual(observer.vehicle.stamp,stamp)
        self.assertEqual(observer.vehicle.quality,quality)

    def motion(self,window):
        helper=test_search.Search();helper.setUp();cfg=deepcopy(helper.cfg)
        cfg['observer']['vertical_range_velocity_window_s']=window
        return SelfMotion(cfg,cfg['entry']['positions'][0])

    def sample(self,t,r,new=True,accurate=True):
        return {'stamp_s':t,'gyro_z_rad_s':0.,'accel_heading_m_s2':[0.,0.,0.],
                'range_m':r,'range_stamp_s':t,'range_new':new,'range_accurate':accurate,
                'flow_angular_rad_s':[0.,0.],'flow_valid':True}

    def test_affine_range_descent_is_recovered_without_duplicate_updates(self):
        for window in (.4,.8):
            own=self.motion(window)
            for i in range(161):own.update(self.sample(i*.025,1.-.01*i*.025))
            self.assertAlmostEqual(own.velocity[2],-.01,delta=.0001)
            history=list(own.range_heights)
            own.update(self.sample(4.01,.90,new=False))
            self.assertEqual(own.range_heights[-1],history[-1])
            self.assertTrue(all(row in history for row in own.range_heights))
            own.update(self.sample(4.02,.02,accurate=False))
            self.assertEqual(len(own.range_heights),0)

    def test_longer_window_reduces_stationary_range_noise_but_has_finite_response_lag(self):
        noise=np.random.default_rng(71).normal(0,.004,1200)
        std=[]
        for window in (.4,.8):
            own=self.motion(window);velocity=[]
            for i,value in enumerate(noise):
                own.update(self.sample(i*.025,1.+value))
                if i>100:velocity.append(own.velocity[2])
            std.append(float(np.std(velocity)))
        self.assertLess(std[1],std[0]*.7)
        own=self.motion(.8)
        for i in range(121):own.update(self.sample(i*.025,1.))
        own.update(self.sample(3.025,1.-.02*.025))
        self.assertGreater(own.velocity[2],-.01)
        for i in range(1,81):own.update(self.sample(3.025+i*.025,1.-.02*(.025+i*.025)))
        self.assertAlmostEqual(own.velocity[2],-.02,delta=.001)

    def test_invalid_quality_settings_are_rejected(self):
        for window in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.motion(window)
        helper=test_search.Search();helper.setUp()
        for depth in (-1.,float('nan'),float('inf')):
            cfg=deepcopy(helper.cfg);cfg['observer']['metric_pose_min_depth_m']=depth
            with self.assertRaises(ValueError):Observer(cfg,SelfMotion(cfg,cfg['entry']['positions'][0]))


if __name__=='__main__':unittest.main()
