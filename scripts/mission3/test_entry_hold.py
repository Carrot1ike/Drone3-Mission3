"""New user scenario: observation-triggered travel and command envelopes."""
from copy import deepcopy
import math
import unittest
import numpy as np
from .settings import ROOT, load, controller_settings
from .controller import Mission3


class EntryHoldTests(unittest.TestCase):
    def setUp(self):
        self.cfg = controller_settings(load(ROOT/'config/mission3-35x12-entry-hold.json'))
        self.entry = self.cfg['entry']['positions'][0]

    def own(self, position=None):
        return {'position_world_m': position or [1.,2.,1.], 'heading_rad': 0.,
                'velocity_world_m_s': [0.,0.,0.], 'position_std_m': .1}

    def test_unobserved_search_stays_at_entry_for_multiple_scan_cycles(self):
        mission = Mission3(self.cfg,self.entry)
        for now in np.arange(0.,110.,.02):
            command = mission.update(float(now),self.own(),None)
            self.assertIn(command.phase,('WAIT','ENTRY_SCAN'))
            np.testing.assert_allclose(command.velocity_world_m_s,0.)
            if now < 15.: self.assertEqual(command.yaw_rate_rad_s,0.)
        self.assertEqual(mission.phase,'ENTRY_SCAN')
        self.assertIsNone(mission.discovery_time)

    def test_hold_target_does_not_follow_changed_track_prior(self):
        changed = deepcopy(self.cfg)
        changed['track']['center_world_xy_m']=[20.,5.]
        changed['track']['straight_length_m']=12.
        for cfg in (self.cfg,changed):
            mission=Mission3(cfg,self.entry)
            np.testing.assert_allclose(mission.search_target,[1.,2.,1.])
            command=mission.update(16.,self.own([1.1,2.1,1.]),None)
            self.assertLess(command.velocity_world_m_s[0],0.)
            self.assertLess(command.velocity_world_m_s[1],0.)

    def test_only_decoded_image_bearing_can_start_forward_travel(self):
        mission=Mission3(self.cfg,self.entry)
        mission.update(15.,self.own(),None)
        coarse={'kind':'coarse_image_bearing_only','age_s':0.,'bearing_world_rad':0.,
                'min_side_px':8.,'max_side_px':9.}
        command=mission.update(15.02,self.own(),coarse)
        self.assertEqual(command.phase,'COARSE_ACQUIRE')
        self.assertGreater(command.velocity_world_m_s[0],0.)
        self.assertIsNone(mission.discovery_time)

    def test_lost_coarse_marker_holds_current_pose_instead_of_track_station(self):
        mission=Mission3(self.cfg,self.entry)
        coarse={'kind':'coarse_image_bearing_only','age_s':0.,'bearing_world_rad':0.,
                'min_side_px':8.,'max_side_px':9.}
        mission.update(15.,self.own(),coarse)
        command=mission.update(21.,self.own([1.6,2.2,1.]),None)
        self.assertEqual(command.phase,'ENTRY_SCAN')
        np.testing.assert_allclose(mission.search_target,[1.6,2.2,1.])

    def test_speed_vertical_acceleration_and_yaw_limits_under_reversal(self):
        mission=Mission3(self.cfg,self.entry)
        previous=np.zeros(3);previous_yaw=0.
        for i in range(1000):
            desired=[5.,-4.,3.] if i < 500 else [-5.,4.,-3.]
            command=mission.output(desired,7. if i < 500 else -7.,.02)
            v=np.array(command.velocity_world_m_s)
            self.assertLessEqual(np.linalg.norm(v),.4+1e-12)
            self.assertLessEqual(v[2],.10+1e-12)
            self.assertGreaterEqual(v[2],-.06-1e-12)
            self.assertLessEqual(np.linalg.norm(v-previous)/.02,.15+1e-10)
            self.assertLessEqual(abs(command.yaw_rate_rad_s),math.radians(10.)+1e-12)
            self.assertLessEqual(abs(command.yaw_rate_rad_s-previous_yaw)/.02,math.radians(20.)+1e-10)
            previous=v;previous_yaw=command.yaw_rate_rad_s

    def test_new_arena_contains_track_and_all_entry_poses(self):
        from .track import Stadium
        track=Stadium.from_config(self.cfg['track'])
        self.assertEqual(self.cfg['arena']['length_m'],35.)
        self.assertEqual(self.cfg['arena']['width_m'],12.)
        self.assertAlmostEqual(track.straight+2*track.radius,27.5)
        self.assertAlmostEqual(2*track.radius,7.5)
        for entry in self.cfg['entry']['positions']:
            x,y=entry['xy_m']
            self.assertTrue(.5 <= x <= 34.5 and .5 <= y <= 11.5)

    def test_entry_coverage_has_no_track_goal_and_no_motion_before_full_scan(self):
        cfg=deepcopy(self.cfg)
        cfg['control'].update(search_motion_policy='bounded_entry_raster',search_pretranslate_scan_s=45.,
            search_forward_extent_m=4.,search_lateral_extent_m=.75,search_translation_speed_m_s=.15)
        mission=Mission3(cfg,self.entry)
        changed=deepcopy(cfg);changed['track']['center_world_xy_m']=[20.,5.]
        other=Mission3(changed,self.entry)
        np.testing.assert_allclose(mission.entry_search_targets,other.entry_search_targets)
        for now in np.arange(0.,60.,.02):
            command=mission.update(float(now),self.own(),None)
            np.testing.assert_allclose(command.velocity_world_m_s,0.)
        command=mission.update(60.02,self.own(),None)
        self.assertEqual(command.phase,'ENTRY_SEARCH')
        self.assertGreater(command.velocity_world_m_s[0],0.)
        self.assertLessEqual(np.linalg.norm(command.velocity_world_m_s),.15+1e-9)

    def test_entry_coverage_all_entries_stay_inside_known_net_clearance(self):
        cfg=deepcopy(self.cfg)
        cfg['control'].update(search_motion_policy='bounded_entry_raster',search_pretranslate_scan_s=45.,
            search_forward_extent_m=4.,search_lateral_extent_m=.75,search_translation_speed_m_s=.15)
        for entry in cfg['entry']['positions']:
            mission=Mission3(cfg,entry)
            points=np.array(mission.entry_search_targets)
            self.assertTrue(np.all(points[:,:2]>=1.))
            self.assertTrue(np.all(points[:,:2]<=[34.,11.]))
            np.testing.assert_allclose(points[:,2],1.)

    def test_observed_coarse_loss_disables_unobserved_raster_resume(self):
        cfg=deepcopy(self.cfg)
        cfg['control'].update(search_motion_policy='bounded_entry_raster',search_pretranslate_scan_s=45.,
            search_forward_extent_m=4.,search_lateral_extent_m=.75,search_translation_speed_m_s=.15)
        mission=Mission3(cfg,self.entry)
        coarse={'kind':'coarse_image_bearing_only','age_s':0.,'bearing_world_rad':0.,
                'min_side_px':8.,'max_side_px':9.}
        mission.update(65.,self.own(),coarse)
        mission.update(72.,self.own([2.,2.,1.]),None)
        command=mission.update(140.,self.own([2.,2.,1.]),None)
        self.assertEqual(command.phase,'ENTRY_SCAN')
        np.testing.assert_allclose(command.velocity_world_m_s,0.)

    def sector_config(self):
        cfg=deepcopy(self.cfg)
        cfg['control'].update(search_gaze_policy='entry_sector',search_sector_amplitude_deg=70.,
                              search_sector_period_s=56.)
        return cfg

    def test_sector_wait_and_continuous_search_respect_yaw_envelope(self):
        cfg=self.sector_config()
        for entry in cfg['entry']['positions']:
            mission=Mission3(cfg,entry)
            yaw=math.radians(entry['heading_deg']);previous_rate=0.
            for now in np.arange(0.,128.,.02):
                own=self.own([*entry['xy_m'],1.]);own['heading_rad']=yaw
                command=mission.update(float(now),own,None)
                rate=command.yaw_rate_rad_s
                if now<15.:self.assertAlmostEqual(rate,0.)
                self.assertLessEqual(abs(rate),math.radians(10.)+1e-10)
                self.assertLessEqual(abs(rate-previous_rate)/.02,math.radians(20.)+1e-10)
                yaw=math.atan2(math.sin(yaw+rate*.02),math.cos(yaw+rate*.02))
                self.assertLessEqual(abs(math.atan2(math.sin(yaw-mission.entry_heading),
                                                   math.cos(yaw-mission.entry_heading))),math.radians(71.))
                previous_rate=rate

    def test_sector_target_and_rate_depend_on_entry_search_clock_not_absolute_time(self):
        cfg=self.sector_config()
        for start in (0.,123.):
            mission=Mission3(cfg,self.entry,start_time_s=start)
            mission.enter('ENTRY_SCAN',start+15.,'test_clock')
            mission.search_gaze=math.radians(179.)
            for elapsed in (0.,7.,14.,28.,42.,56.):
                target=mission.search_gaze+math.radians(70.)*math.sin(2*math.pi*elapsed/56.)
                wrapped=math.atan2(math.sin(target),math.cos(target))
                rate=mission.entry_search_yaw_rate(start+15.+elapsed,wrapped)
                expected=math.radians(70.)*2*math.pi/56.*math.cos(2*math.pi*elapsed/56.)
                self.assertAlmostEqual(rate,expected)

    def test_sector_loss_holds_current_pose_and_restarts_at_measured_heading(self):
        mission=Mission3(self.sector_config(),self.entry)
        coarse={'kind':'coarse_image_bearing_only','age_s':0.,'bearing_world_rad':0.,
                'min_side_px':8.,'max_side_px':9.}
        mission.update(15.,self.own(),coarse)
        own=self.own([1.6,2.2,1.]);own['heading_rad']=.5
        mission.update(21.,own,None)
        self.assertEqual(mission.phase,'ENTRY_SCAN')
        self.assertEqual(mission.entry_scan_started,21.)
        self.assertEqual(mission.search_gaze,.5)
        np.testing.assert_allclose(mission.search_target,[1.6,2.2,1.])

    def test_sector_rejects_invalid_parameters(self):
        for key,value in (('search_sector_period_s',0.),('search_sector_period_s',float('nan')),
                          ('search_sector_amplitude_deg',180.),('search_sector_amplitude_deg',float('inf'))):
            cfg=self.sector_config();cfg['control'][key]=value
            with self.assertRaises(ValueError):Mission3(cfg,self.entry)


if __name__=='__main__': unittest.main()
