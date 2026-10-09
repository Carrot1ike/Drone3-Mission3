"""역방향 탐색 속도 스케줄의 좌표 불변성과 기존 포화·관측 계약."""
from copy import deepcopy
import math
import unittest
import numpy as np
from .controller import Mission3
from .settings import ROOT, load, controller_settings


class SearchReverseSpeed(unittest.TestCase):
    def config(self, limit=.05):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-cell8-coarse-v6.json'))
        if limit is not None:cfg['control']['search_reverse_heading_speed_m_s']=limit
        return cfg

    def mission(self, limit=.05):
        cfg=self.config(limit)
        return Mission3(cfg,cfg['entry']['positions'][0])

    def test_rotation_equivariance_and_unaffected_forward_lateral_directions(self):
        m=self.mission()
        for heading in np.linspace(-math.pi,math.pi,17):
            R=np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
            for direction in np.linspace(-math.pi,math.pi,65):
                v=.15*np.array([math.cos(direction),math.sin(direction)])
                scheduled,guard=m.entry_search_horizontal(R@v,heading)
                reference,_=m.entry_search_horizontal(v,0.)
                np.testing.assert_allclose(scheduled,R@reference,atol=1e-12)
                self.assertLessEqual(np.linalg.norm(scheduled),np.linalg.norm(v)+1e-12)
                self.assertGreaterEqual(guard['scheduled_desired_heading_projection_m_s'],-.05-1e-12)
                self.assertGreater(guard['scale'],0.)
                if v[0]>=-.05:np.testing.assert_allclose(scheduled,R@v,atol=1e-12)
        scheduled,guard=m.entry_search_horizontal([-.15,0.],0.)
        np.testing.assert_allclose(scheduled,[-.05,0.]);self.assertAlmostEqual(guard['scale'],1/3)

    def test_search_schedule_uses_entry_heading_not_track_or_vehicle_state(self):
        cfg=self.config();other=deepcopy(cfg)
        other['track']['center_world_xy_m']=[20.,5.]
        m=Mission3(cfg,cfg['entry']['positions'][0]);n=Mission3(other,other['entry']['positions'][0])
        np.testing.assert_allclose(m.entry_search_targets,n.entry_search_targets)
        own={'position_world_m':[1.,2.,1.],'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':0.,'position_std_m':.1}
        for now in (0.,14.98,15.,59.98):
            cmd=m.update(now,own,None);self.assertIn(cmd.phase,('WAIT','ENTRY_SCAN'))
            self.assertIsNone(cmd.search_guard);np.testing.assert_allclose(cmd.velocity_world_m_s,0.)
        full=load(ROOT/'config/mission3-35x12-cell8-coarse-v6.json')
        full['control']['search_reverse_heading_speed_m_s']=.05
        full['vehicle_plant_only']['speed_m_s']=99.
        copy=Mission3(controller_settings(full),full['entry']['positions'][0])
        np.testing.assert_allclose(m.entry_search_targets,copy.entry_search_targets)
        with self.assertRaises(ValueError):Mission3(full,full['entry']['positions'][0])

    def test_schedule_retains_slew_during_reversal_and_moving_gaze(self):
        m=self.mission();m.phase='ENTRY_SEARCH';m.entry_scan_started=15.;m.entry_search_index=2
        own={'position_world_m':[6.,2.75,1.],'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':0.,'position_std_m':.1}
        previous=np.array([-.15,0.,0.]);previous_yaw=0.;m.last_v=previous.copy();m.last_time=59.98
        first=m.update(60.,own,None)
        self.assertLess(first.search_guard['output_heading_projection_m_s'],-.05)
        self.assertGreaterEqual(first.search_guard['scheduled_desired_heading_projection_m_s'],-.05-1e-12)
        # The desired schedule does not instantly overwrite a pre-existing
        # command. Ordinary acceleration constraints still take priority.
        for i in range(3000):
            cmd=first if i==0 else m.update(60.+i*.02,own,None)
            v=np.array(cmd.velocity_world_m_s);rate=cmd.yaw_rate_rad_s
            self.assertEqual(cmd.phase,'ENTRY_SEARCH')
            self.assertLessEqual(np.linalg.norm(v-previous)/.02,.15+1e-9)
            self.assertLessEqual(np.linalg.norm(v),.15+1e-9)
            self.assertLessEqual(abs(rate),math.radians(10)+1e-9)
            self.assertLessEqual(abs(rate-previous_yaw)/.02,math.radians(20)+1e-9)
            own['position_world_m']=(np.array(own['position_world_m'])+.02*v).tolist()
            own['heading_rad']=math.atan2(math.sin(own['heading_rad']+.02*rate),math.cos(own['heading_rad']+.02*rate))
            previous=v;previous_yaw=rate
        self.assertIsNone(m.discovery_time);self.assertIsNone(m.last_vehicle)
        self.assertFalse(cmd.physical_success);self.assertFalse(cmd.motors_off)

    def test_metric_coarse_and_hard_gates_unchanged_and_invalid_limits_rejected(self):
        a=self.mission(.05);b=self.mission(None)
        own={'position_world_m':[4.,2.,1.],'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':0.,'position_std_m':.1}
        coarse={'kind':'coarse_image_bearing_only','valid':True,'age_s':.05,
                'bearing_world_rad':0.,'min_side_px':10.,'max_side_px':12.}
        for i in range(20):
            x=a.update(80.+i*.02,own,coarse);y=b.update(80.+i*.02,own,coarse)
            np.testing.assert_equal(x.velocity_world_m_s,y.velocity_world_m_s)
            self.assertEqual(x.yaw_rate_rad_s,y.yaw_rate_rad_s)
            self.assertEqual(x.coarse_guard,y.coarse_guard);self.assertIsNone(x.search_guard)
        own['position_std_m']=1.001
        self.assertEqual(a.update(81.,own,coarse).failure,'odometry_uncertainty')
        for limit in (0.,-.01,.151,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.mission(limit)
        cfg=self.config();cfg['control']['search_motion_policy']='stationary'
        with self.assertRaises(ValueError):Mission3(cfg,cfg['entry']['positions'][0])


if __name__=='__main__':unittest.main()
