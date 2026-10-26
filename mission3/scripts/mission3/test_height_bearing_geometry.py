import math
import unittest
import cv2
import numpy as np
from yaw_core import rotation_rpy
from .scene import R_HEADING_CAMERA, R_VEHICLE_MARKER
from .height_bearing_geometry import relative_marker, conditional_covariance, diagonal_center


class HeightBearingTests(unittest.TestCase):
    K=np.array([[166.,0.,160.],[0.,166.,160.],[0.,0.,1.]])
    offset=np.array([.02,0.,.015])

    def projected(self, marker, roll, pitch, vehicle_heading=0.):
        R=rotation_rpy(roll,pitch,0.)
        own=np.array([0.,0.,1.]);camera=own+R@self.offset
        half=.075;points=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
        Rcm=R_HEADING_CAMERA.T@R.T@rotation_rpy(0.,0.,vehicle_heading)@R_VEHICLE_MARKER
        t=R_HEADING_CAMERA.T@R.T@(marker-camera)
        return cv2.projectPoints(points,cv2.Rodrigues(Rcm)[0],t,self.K,np.zeros(5))[0].reshape(4,2)

    def test_native_small_quad_can_supply_scale_without_vehicle_yaw(self):
        for roll,pitch,vehicle_heading in [(0.,0.,0.),(.04,-.02,.5)]:
            marker=np.array([3.,-.25,.4]);q=self.projected(marker,roll,pitch,vehicle_heading)
            self.assertLess(np.linalg.norm(q[1]-q[0]),18.)
            point,info=relative_marker(q,1.,roll,pitch,.4,self.K,np.zeros(5),R_HEADING_CAMERA,self.offset)
            np.testing.assert_allclose(point,marker-[0.,0.,1.],atol=1e-10)
            self.assertIn('not pad',info['scope'])

    def test_height_datum_uncertainty_does_not_disappear(self):
        q=self.projected(np.array([3.,-.25,.4]),0.,0.)
        a,_=conditional_covariance(q,1.,0.,0.,.4,self.K,np.zeros(5),R_HEADING_CAMERA,self.offset,.5,.01,.5)
        b,_=conditional_covariance(q,1.,0.,0.,.4,self.K,np.zeros(5),R_HEADING_CAMERA,self.offset,1.,.05,2.)
        self.assertGreater(np.linalg.eigvalsh(b).max(),np.linalg.eigvalsh(a).max())
        self.assertGreater(np.linalg.eigvalsh(a).max(),0.)
        self.assertGreaterEqual(np.linalg.eigvalsh(b).min(),-1e-12)

    def test_near_horizon_wrong_plane_and_invalid_native_coords_rejected(self):
        q=np.array([[150.,155.],[160.,155.],[160.,165.],[150.,165.]])
        with self.assertRaises(ValueError):relative_marker(q,1.,0.,0.,.4,self.K,np.zeros(5),R_HEADING_CAMERA,self.offset)
        q=self.projected(np.array([3.,-.25,.4]),0.,0.)
        with self.assertRaises(ValueError):relative_marker(q,0.,0.,0.,.4,self.K,np.zeros(5),R_HEADING_CAMERA,self.offset)
        with self.assertRaises(ValueError):relative_marker(q+500,1.,0.,0.,.4,self.K,np.zeros(5),R_HEADING_CAMERA,self.offset)
        with self.assertRaises(ValueError):diagonal_center(np.array([[0.,0.],[1.,0.],[2.,0.],[3.,0.]]))
