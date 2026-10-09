"""Native decoded bearing translation keeps existing fresh-image and path budgets."""
from copy import deepcopy
import math
import unittest
import numpy as np
from .controller import Mission3
from .settings import ROOT,load,controller_settings


class CoarseDirection(unittest.TestCase):
    def fixture(self,axis='fresh_decoded_bearing',age=.05,side=10.,maximum=12.,angle=30.):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-cell8-coarse-v6.json'))
        cfg['control'].update(coarse_translation_axis=axis,coarse_translate_turn_error_deg=40.,coarse_turn_translation_speed_m_s=.4)
        m=Mission3(cfg,cfg['entry']['positions'][0])
        own={'position_world_m':[4.,2.5,1.],'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':0.,'position_std_m':.1}
        v={'kind':'coarse_image_bearing_only','valid':True,'age_s':age,
           'capture_stamp_s':20.-age,'bearing_world_rad':math.radians(angle),
           'min_side_px':side,'max_side_px':maximum}
        return m,own,v

    def test_fresh_bearing_motion_and_gaze_are_separate_with_unchanged_caps(self):
        m,own,v=self.fixture();previous=np.zeros(3);previous_yaw=0.
        for i in range(180):
            v['capture_stamp_s']=20.+i*.02-.05
            cmd=m.update(20.+i*.02,own,v);velocity=np.array(cmd.velocity_world_m_s)
            self.assertLessEqual(np.linalg.norm(velocity-previous)/.02,.15+1e-9)
            self.assertLessEqual(np.linalg.norm(velocity),.4+1e-9)
            self.assertLessEqual(abs(cmd.yaw_rate_rad_s),math.radians(10)+1e-9)
            self.assertLessEqual(abs(cmd.yaw_rate_rad_s-previous_yaw)/.02,math.radians(20)+1e-9)
            previous=velocity;previous_yaw=cmd.yaw_rate_rad_s
        np.testing.assert_allclose(previous[:2],.4*np.array([math.cos(math.radians(30)),math.sin(math.radians(30))]))
        self.assertTrue(cmd.coarse_guard['fresh_decoded_translation_used'])
        self.assertIsNone(m.discovery_time);self.assertIsNone(m.last_vehicle)
        self.assertFalse(cmd.motors_off);self.assertFalse(cmd.physical_success)
        body,own,v=self.fixture(axis='body_forward')
        cmd=body.update(20.,own,v);self.assertGreater(cmd.velocity_world_m_s[0],0.)
        self.assertEqual(cmd.velocity_world_m_s[1],0.)

    def test_stale_large_or_out_of_sector_images_cannot_open_turn_translation(self):
        for age,maximum,angle in ((.5,12.,30.),(.05,40.,30.),(.05,12.,45.)):
            m,own,v=self.fixture(age=age,maximum=maximum,angle=angle)
            np.testing.assert_allclose(m.update(20.,own,v).velocity_world_m_s,0.)
        m,own,v=self.fixture(age=.5,angle=10.)
        cmd=m.update(20.,own,v)
        self.assertFalse(cmd.coarse_guard['fresh_decoded_translation_used'])
        self.assertEqual(cmd.velocity_world_m_s[1],0.)

    def test_path_expiry_and_original_uncertainty_still_stop_or_abort(self):
        m,own,v=self.fixture();m.update(20.,own,v)
        m.last_v*=0.;m.coarse_total_distance_m=4.
        cmd=m.update(20.02,own,v)
        self.assertEqual(cmd.coarse_guard['policy'],'total_path_exhausted')
        np.testing.assert_allclose(cmd.velocity_world_m_s,0.)
        m,own,v=self.fixture();own['position_std_m']=1.001
        self.assertEqual(m.update(20.,own,v).failure,'odometry_uncertainty')
        cfg=deepcopy(m.cfg);cfg['control']['coarse_translation_axis']='truth_velocity'
        with self.assertRaises(ValueError):Mission3(cfg,cfg['entry']['positions'][0])


if __name__=='__main__':unittest.main()
