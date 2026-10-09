"""Opt-in 디코더는 native 입력과 metric/방위 확인·오류교정 계약을 보존한다."""
from copy import deepcopy
import unittest
import cv2
import numpy as np
from yaw_core import marker_detector_parameters
from .settings import ROOT,load,controller_settings
from .vision import Observer
from .estimator import SelfMotion
from . import test_search


class DetectorProfile(unittest.TestCase):
    def setup_observer(self,profile):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-entry-coverage-forward8-v4.json'))
        if profile is not None:cfg['observer']['detector_profile']=profile
        own=SelfMotion(cfg,cfg['entry']['positions'][0])
        own.history.extend((t,own.position.copy(),own.heading) for t in (0.,.1))
        return Observer(cfg,own),cfg

    def test_fixed_optin_preserves_default_error_correction_border_and_legacy_parameters(self):
        old=cv2.aruco.DetectorParameters();old.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        legacy=marker_detector_parameters();new=marker_detector_parameters('small_code_cell8_subpix')
        permitted={'adaptiveThreshWinSizeMin','adaptiveThreshWinSizeMax','adaptiveThreshWinSizeStep',
                   'adaptiveThreshConstant','minMarkerPerimeterRate','perspectiveRemovePixelPerCell'}
        attributes=[key for key in dir(old) if not key.startswith('_') and not callable(getattr(old,key))]
        for key in attributes:
            self.assertEqual(getattr(legacy,key),getattr(old,key),key)
            if key not in permitted:self.assertEqual(getattr(new,key),getattr(old,key),key)
        self.assertEqual(new.errorCorrectionRate,old.errorCorrectionRate)
        self.assertEqual(new.markerBorderBits,old.markerBorderBits)
        self.assertEqual(new.cornerRefinementMethod,cv2.aruco.CORNER_REFINE_SUBPIX)
        observer,cfg=self.setup_observer('small_code_cell8_subpix')
        self.assertEqual(observer.detector.settings['min_side_px'],cfg['observer']['min_side_px'])
        self.assertEqual(observer.detector.settings['max_rmse_px'],cfg['observer']['max_rmse_px'])
        with self.assertRaises(ValueError):self.setup_observer('arbitrary_weakening')

    def test_small_native_decode_requires_temporal_coarse_and_never_becomes_metric(self):
        observer,cfg=self.setup_observer('small_code_cell8_subpix')
        dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,cfg['marker']['dictionary']))
        frame=np.full((320,320),255,np.uint8)
        frame[152:168,152:168]=cv2.aruco.generateImageMarker(dictionary,cfg['marker']['id'],16)
        first=observer.frame(frame,0.,.05)
        self.assertEqual(first['reason'],'marker_too_small',first)
        self.assertFalse(first['valid']);self.assertIsNone(observer.state(.05))
        second=observer.frame(frame,.1,.15)
        self.assertTrue(second.get('coarse_guidance_available'),second)
        self.assertEqual(observer.state(.15)['kind'],'coarse_image_bearing_only')
        self.assertIsNone(observer.vehicle.stamp)
        self.assertNotIn('vehicle_velocity_world_m_s',observer.state(.15))
        fresh,_=self.setup_observer('small_code_cell8_subpix')
        wrong=np.full((320,320),255,np.uint8)
        wrong[152:168,152:168]=cv2.aruco.generateImageMarker(dictionary,(cfg['marker']['id']+1)%50,16)
        self.assertEqual(fresh.frame(wrong,0.,.05)['reason'],'marker_missing')
        self.assertIsNone(fresh.vehicle.stamp)
        for invalid in (np.zeros((244,324),np.uint8),np.zeros((320,320,3),np.uint8),np.zeros((320,320),np.float32)):
            with self.assertRaises(ValueError):fresh.frame(invalid,0.,.05)

    def test_close_metric_depth_rejection_remains_with_new_profile(self):
        helper=test_search.Search();helper.setUp();cfg=deepcopy(helper.cfg)
        cfg['observer'].update(detector_profile='small_code_cell8_subpix',metric_pose_min_depth_m=.32)
        own=SelfMotion(cfg,cfg['entry']['positions'][0]);own.position=np.array([17.,6.25,.4])
        own.heading=0.;own.history.extend((t,own.position.copy(),0.) for t in (0.,.1))
        observer=Observer(cfg,own)
        _,good=helper.projection(own.position,0.,[17.+.40-.25,6.25,.3])
        first=observer.frame(good,0.,.05);self.assertTrue(first['valid'],first)
        stamp=observer.vehicle.stamp;quality=deepcopy(observer.vehicle.quality)
        _,close=helper.projection(own.position,0.,[17.+.315-.25,6.25,.3])
        next_frame=observer.frame(close,.1,.15)
        self.assertFalse(next_frame['valid']);self.assertEqual(next_frame['reason'],'metric_pose_depth_guard')
        self.assertEqual(observer.vehicle.stamp,stamp);self.assertEqual(observer.vehicle.quality,quality)


if __name__=='__main__':unittest.main()
