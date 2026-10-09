"""Synthetic optical boundary and ray checks; not camera calibration tests."""
import math
import unittest
import cv2
import numpy as np
from .optics import brown_candidate,SyntheticLens
from .settings import load,controller_settings
from .estimator import SelfMotion
from .vision import Observer
from .scene import R_HEADING_CAMERA,R_VEHICLE_MARKER
from .track import rotation,wrap
from yaw_core import rotation_rpy


class Optics(unittest.TestCase):
    def test_attitude_constrained_fit_uses_distorted_camera_rays(self):
        cfg=controller_settings(load());lens=brown_candidate();cfg["camera"]["observer_optics_sensitivity"]=lens
        pad=np.array([25.,10.,.3]);heading=.3;roll,pitch=.08,-.04;yaw=heading+.12
        own=SelfMotion(cfg,cfg["entry"]["positions"][0]);own.position=pad+rotation(heading)@np.array([-.60,-.14,.16]);own.heading=yaw
        own.history.append((0.,own.position.copy(),yaw));own.attitude_at=lambda t:(roll,pitch)
        obs=Observer(cfg,own);Rwb=rotation_rpy(roll,pitch,yaw)
        Rcm=R_HEADING_CAMERA.T@Rwb.T@rotation(heading)@R_VEHICLE_MARKER
        tcm=R_HEADING_CAMERA.T@Rwb.T@(pad+rotation(heading)@np.array([.25,0,.1])-own.position)
        rows,cols=np.mgrid[0:320,0:320];pixels=np.stack([cols,rows],axis=-1).reshape(-1,1,2).astype(float)
        rays=cv2.undistortPointsIter(pixels,np.asarray(lens["K"]),np.asarray(lens["distortion"]),None,None,
                                    (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,80,1e-12)).reshape(-1,2)
        rays=np.c_[rays,np.ones(len(rays))];normal=Rcm[:,2]
        depths=np.dot(normal,tcm)/(rays@normal)
        local=(rays*depths[:,None]-tcm)@Rcm
        side=cfg["marker"]["code_side_m"]
        mapx=((local[:,0]/side+.5)*179).reshape(320,320).astype(np.float32)
        mapy=((.5-local[:,1]/side)*179).reshape(320,320).astype(np.float32)
        marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,180)
        gray=cv2.remap(marker,mapx,mapy,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=255)
        result=obs.frame(gray,0.,.05)
        self.assertTrue(result["valid"],result)
        self.assertLess(np.linalg.norm(np.asarray(result["raw_pad_world_m"])-pad),.015)
        self.assertLess(abs(wrap(result["raw_vehicle_heading_rad"]-heading)),math.radians(3))
        # Isolate geometric refinement from raster corner quantization. These
        # exact corners come from the analytic Brown radial equation, not the
        # implementation's projectPoints call.
        xyz=obs.detector.points@Rcm.T+tcm
        norm=xyz[:,:2]/xyz[:,2:];radial=1+lens["distortion"][0]*np.sum(norm*norm,axis=1)
        exact=norm*radial[:,None]*lens["K"][0][0]+160.
        exact_obs=Observer(cfg,own)
        exact_obs.detector.detector=type("Corners",(),{"detectMarkers":lambda self,frame:([exact.reshape(1,4,2)],np.array([[0]]),[])})()
        refined=exact_obs.frame(gray,0.,.05)
        self.assertTrue(refined["valid"],refined)
        self.assertLess(np.linalg.norm(np.asarray(refined["raw_pad_world_m"])-pad),1e-6)
        self.assertLess(abs(wrap(refined["raw_vehicle_heading_rad"]-heading)),1e-6)
    def test_brown_candidate_matches_three_published_angle_boundaries(self):
        c=brown_candidate();f=c["K"][0][0];k=c["distortion"][0]
        for angle,radius in ((87.,160.),(115.,math.sqrt(2)*160.)):
            r=math.tan(math.radians(angle)/2)
            self.assertAlmostEqual(f*r*(1+k*r*r),radius,places=9)
        self.assertGreater(1+3*k*math.tan(math.radians(115.)/2)**2,0.)

    def test_native_ray_transfer_has_no_resize_or_color_repair(self):
        c=brown_candidate();lens=SyntheticLens(c)
        gray=np.zeros((320,320),np.uint8);gray[158:163,158:163]=200
        out=lens.sample(gray)
        self.assertEqual(out.shape,(320,320));self.assertEqual(out.dtype,np.uint8)
        self.assertEqual(int(out[160,160]),200)
        with self.assertRaises(ValueError):lens.sample(np.zeros((324,324),np.uint8))

    def test_inverse_rays_project_back_to_native_pixels(self):
        c=brown_candidate();lens=SyntheticLens(c)
        source_f=160/math.tan(math.radians(lens.source_fov_deg)/2)
        pixels=np.array([[0,0],[160,160],[319,319],[0,160]],float)
        rays=np.array([[(lens.map_x[int(v),int(u)]-160)/source_f,(lens.map_y[int(v),int(u)]-160)/source_f,1.] for u,v in pixels])
        projected=cv2.projectPoints(rays,np.zeros(3),np.zeros(3),np.asarray(c["K"]),np.asarray(c["distortion"]))[0].reshape(-1,2)
        np.testing.assert_allclose(projected,pixels,atol=3e-5)


if __name__=="__main__":unittest.main()
