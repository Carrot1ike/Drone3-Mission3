"""단일 실제 decoded ID는 확인 전 제동/시선만, 확정/착륙 승격 금지."""
from copy import deepcopy
import math
import unittest
import cv2
import numpy as np
from .controller import Mission3
from .estimator import SelfMotion
from .settings import ROOT,load,controller_settings
from .vision import Observer


class CandidateGaze(unittest.TestCase):
    def fixture(self,enabled=True):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-entry-coverage-forward8-v4.json'))
        if enabled:cfg['observer']['decoded_candidate_gaze']={'enabled':True,'max_age_s':.35}
        m=Mission3(cfg,cfg['entry']['positions'][0])
        own={'position_world_m':[3.,2.,1.],'heading_rad':.5,
             'velocity_world_m_s':[-.15,0.,0.],'position_std_m':.1}
        candidate={'kind':'unconfirmed_decoded_bearing','valid':False,'bearing_valid':True,
                   'capture_stamp_s':70.,'age_s':.05,'bearing_world_rad':0.}
        return m,own,candidate

    def test_real_native_single_decode_remains_nonmetric_and_requires_second_id(self):
        m,_,_=self.fixture();motion=SelfMotion(m.cfg,m.cfg['entry']['positions'][0])
        motion.history.extend((t,motion.position.copy(),motion.heading) for t in (0.,.1,.8))
        observer=Observer(m.cfg,motion)
        dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,m.cfg['marker']['dictionary']))
        gray=np.full((320,320),255,np.uint8)
        gray[152:168,152:168]=cv2.aruco.generateImageMarker(dictionary,m.cfg['marker']['id'],16)
        first=observer.frame(gray,0.,.05)
        self.assertEqual(first['reason'],'marker_too_small',first)
        state=observer.state(.05)
        self.assertEqual(state['kind'],'unconfirmed_decoded_bearing')
        self.assertFalse(state['valid']);self.assertNotIn('vehicle_velocity_world_m_s',state)
        self.assertIsNone(observer.vehicle.stamp)
        self.assertIsNone(observer.state(.36))
        second=observer.frame(gray,.1,.15)
        self.assertTrue(second.get('coarse_guidance_available'),second)
        self.assertEqual(observer.state(.15)['kind'],'coarse_image_bearing_only')
        self.assertIsNone(observer.vehicle.stamp)
        wrong=np.full((320,320),255,np.uint8)
        wrong[152:168,152:168]=cv2.aruco.generateImageMarker(dictionary,(m.cfg['marker']['id']+1)%50,16)
        other=Observer(m.cfg,motion);other.frame(wrong,0.,.05)
        self.assertIsNone(other.state(.05))

    def test_candidate_brakes_with_slew_without_feedforward_or_motor_stop(self):
        m,own,candidate=self.fixture();m.phase='ENTRY_SEARCH';m.entry_scan_started=15.
        m.last_v=np.array([-.15,0.,0.]);m.last_yaw_rate=math.radians(7.)
        previous=m.last_v.copy();previous_yaw=m.last_yaw_rate
        for i in range(16):
            candidate['age_s']=.02*i
            command=m.update(70.+.02*i,own,candidate)
            self.assertEqual(command.phase,'VERIFY_BEARING')
            v=np.array(command.velocity_world_m_s)
            self.assertLessEqual(np.linalg.norm(v-previous)/.02,.15+1e-9)
            self.assertLessEqual(abs(command.yaw_rate_rad_s-previous_yaw)/.02,math.radians(20)+1e-9)
            self.assertGreater(v[0],previous[0]);self.assertEqual(v[2],0.)
            self.assertIsNone(m.discovery_time);self.assertIsNone(m.last_vehicle)
            self.assertFalse(command.motors_off);self.assertFalse(command.physical_success)
            previous=v;previous_yaw=command.yaw_rate_rad_s

    def test_expiry_resumes_interrupted_scan_or_raster_without_resetting_clock(self):
        for phase in ('WAIT','ENTRY_SCAN','ENTRY_SEARCH'):
            m,own,candidate=self.fixture();m.phase=phase;m.entry_scan_started=15.
            index=m.entry_search_index
            m.update(10. if phase=='WAIT' else 30.,own,candidate)
            command=m.update(10.36 if phase=='WAIT' else 30.36,own,None)
            self.assertEqual(command.phase,phase)
            self.assertEqual(m.entry_scan_started,15.)
            self.assertEqual(m.entry_search_index,index)
            self.assertIsNone(m.candidate_interrupt)

    def test_original_coarse_and_metric_evidence_are_required_for_advancing(self):
        m,own,candidate=self.fixture();m.phase='ENTRY_SEARCH';m.entry_scan_started=15.
        m.update(70.,own,candidate)
        coarse={**candidate,'kind':'coarse_image_bearing_only','valid':True,'min_side_px':10.,'max_side_px':11.}
        command=m.update(70.02,own,coarse)
        self.assertEqual(command.phase,'COARSE_ACQUIRE');self.assertIsNone(m.discovery_time)
        self.assertIsNone(m.candidate_interrupt)
        m,own,candidate=self.fixture();m.update(10.,own,candidate)
        metric={'valid':True,'age_s':.05,'samples':40,
                'relative_pad_world_m':[1.,0.,-.7],'relative_velocity_world_m_s':[.2,0,0],
                'vehicle_velocity_world_m_s':[.2,0,0],'vehicle_heading_rad':0.,
                'vehicle_heading_rate_rad_s':0.,'pad_world_m':[4.,2.,.3]}
        command=m.update(10.02,own,metric)
        self.assertNotEqual(command.phase,'VERIFY_BEARING');self.assertEqual(m.discovery_time,10.02)
        self.assertIsNone(m.candidate_interrupt)

    def test_candidate_cannot_bypass_uncertainty_timeout_or_disabled_option(self):
        for reason in ('odometry_uncertainty','search_timeout_without_metric_pose'):
            m,own,candidate=self.fixture()
            if reason=='odometry_uncertainty':own['position_std_m']=1.001;now=10.
            else:now=m.search_deadline_s
            self.assertEqual(m.update(now,own,candidate).failure,reason)
        m,own,candidate=self.fixture(False)
        command=m.update(10.,own,candidate)
        self.assertEqual(command.phase,'WAIT');self.assertIsNone(m.discovery_time)
        for value in (0.,float('nan'),.71):
            cfg=deepcopy(m.cfg);cfg['observer']['decoded_candidate_gaze']={'enabled':True,'max_age_s':value}
            with self.assertRaises(ValueError):Mission3(cfg,cfg['entry']['positions'][0])
            with self.assertRaises(ValueError):Observer(cfg,SelfMotion(cfg,cfg['entry']['positions'][0]))


if __name__=='__main__':unittest.main()
