"""이동 중 시선 분리의 단계 인계·좌표·포화·관측 입력 계약."""
from copy import deepcopy
import math
import unittest
import numpy as np
from .controller import Mission3
from .settings import ROOT, load, controller_settings
from .track import wrap


class SearchTranslationGaze(unittest.TestCase):
    def config(self, enabled=True):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-search-reverse005-flow-slew-v9.json'))
        if enabled:cfg['control']['search_translation_gaze_policy']='entry_forward'
        return cfg

    def test_wait_and_initial_scan_match_original_at_every_entry(self):
        a=self.config();b=self.config(False)
        for entry in a['entry']['positions']:
            m=Mission3(a,entry);n=Mission3(b,entry)
            own={'position_world_m':[*entry['xy_m'],1.], 'velocity_world_m_s':[0.,0.,0.],
                 'heading_rad':math.radians(entry['heading_deg']), 'position_std_m':.1}
            for i in range(3000):
                t=i*.02
                x=m.update(t,own,None);y=n.update(t,own,None)
                self.assertEqual(x.phase,y.phase)
                self.assertIn(x.phase,('WAIT','ENTRY_SCAN'))
                np.testing.assert_array_equal(x.velocity_world_m_s,y.velocity_world_m_s)
                self.assertEqual(x.yaw_rate_rad_s,y.yaw_rate_rad_s)
                own['heading_rad']=wrap(own['heading_rad']+.02*x.yaw_rate_rad_s)
            self.assertIsNone(m.discovery_time)

    def test_scan_to_translation_retains_yaw_acceleration_and_position_slew(self):
        cfg=self.config();m=Mission3(cfg,cfg['entry']['positions'][0])
        m.phase='ENTRY_SCAN';m.entry_scan_started=15.;m.last_time=59.98
        m.last_yaw_rate=math.radians(10.);m.last_v=np.array([.01,0.,0.])
        own={'position_world_m':[1.,2.,1.], 'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':math.radians(70.), 'position_std_m':.1}
        old_v=m.last_v.copy();old_yaw=m.last_yaw_rate
        for i in range(3000):
            cmd=m.update(60.+i*.02,own,None);v=np.array(cmd.velocity_world_m_s);yaw=cmd.yaw_rate_rad_s
            self.assertEqual(cmd.phase,'ENTRY_SEARCH')
            self.assertLessEqual(np.linalg.norm(v-old_v)/.02,.15+1e-9)
            self.assertLessEqual(np.linalg.norm(v),.15+1e-9)
            self.assertLessEqual(abs(yaw),math.radians(10.)+1e-9)
            self.assertLessEqual(abs(yaw-old_yaw)/.02,math.radians(20.)+1e-9)
            self.assertAlmostEqual(v[2],0.)
            own['position_world_m']=(np.array(own['position_world_m'])+.02*v).tolist()
            own['heading_rad']=wrap(own['heading_rad']+.02*yaw)
            old_v,old_yaw=v,yaw
        self.assertLess(abs(wrap(own['heading_rad']-m.entry_heading)),1e-6)
        self.assertIsNone(m.last_vehicle);self.assertIsNone(m.discovery_time)
        self.assertFalse(cmd.motors_off);self.assertFalse(cmd.physical_success)

    def test_translation_direction_is_initial_gaze_independent_of_track_and_scan_clock(self):
        cfg=self.config();changed=deepcopy(cfg);changed['track']['center_world_xy_m']=[100.,100.]
        for entry in cfg['entry']['positions']:
            a=Mission3(cfg,entry);b=Mission3(changed,entry)
            np.testing.assert_array_equal(a.entry_search_targets,b.entry_search_targets)
            a.phase=b.phase='ENTRY_SEARCH';a.search_gaze=.7;b.search_gaze=-2.
            a.entry_scan_started=15.;b.entry_scan_started=150.
            for delta in np.linspace(-3.,3.,17):
                yaw=wrap(a.entry_heading+delta)
                self.assertAlmostEqual(a.entry_search_yaw_rate(200.,yaw),-1.2*delta)
                self.assertEqual(a.entry_search_yaw_rate(200.,yaw),b.entry_search_yaw_rate(600.,yaw))

    def test_coarse_guidance_and_hard_failure_are_unchanged(self):
        cfg=self.config();original=self.config(False)
        a=Mission3(cfg,cfg['entry']['positions'][0]);b=Mission3(original,original['entry']['positions'][0])
        own={'position_world_m':[4.,2.,1.], 'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':0., 'position_std_m':.1}
        coarse={'kind':'coarse_image_bearing_only','valid':True,'age_s':.05,
                'bearing_world_rad':0.,'min_side_px':10.,'max_side_px':12.}
        for i in range(20):
            x=a.update(80.+i*.02,own,coarse);y=b.update(80.+i*.02,own,coarse)
            np.testing.assert_array_equal(x.velocity_world_m_s,y.velocity_world_m_s)
            self.assertEqual(x.yaw_rate_rad_s,y.yaw_rate_rad_s);self.assertEqual(x.coarse_guard,y.coarse_guard)
        own['position_std_m']=1.001
        self.assertEqual(a.update(81.,own,coarse).failure,'odometry_uncertainty')
        self.assertEqual(cfg['observer']['min_side_px'],18.)
        for bad in ('track_target','vehicle_heading'):
            invalid=self.config();invalid['control']['search_translation_gaze_policy']=bad
            with self.assertRaises(ValueError):Mission3(invalid,invalid['entry']['positions'][0])
        invalid=self.config();invalid['control']['search_motion_policy']='stationary'
        with self.assertRaises(ValueError):Mission3(invalid,invalid['entry']['positions'][0])


if __name__=='__main__':unittest.main()
