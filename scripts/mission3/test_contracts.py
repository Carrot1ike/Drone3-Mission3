"""Mission-critical geometry, no-truth, stale-image and sensor transition tests."""
from copy import deepcopy
import math
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from .settings import load,controller_settings
from .track import Stadium,rotation,wrap
from .controller import Mission3
from .estimator import SelfMotion,VehicleState
from .vision import Observer
from .scene import R_HEADING_CAMERA,R_VEHICLE_MARKER

class Contracts(unittest.TestCase):
    def setUp(self):
        self.full=load();self.cfg=controller_settings(self.full)
        self.entry=self.cfg["entry"]["positions"][0]

    def test_private_vehicle_parameters_cannot_enter_controller(self):
        other=deepcopy(self.full);other["vehicle_plant_only"]["speed_m_s"]=99.
        self.assertEqual(controller_settings(other),self.cfg)
        with self.assertRaises(ValueError):Mission3(self.full,self.entry)

    def test_native_camera_contract_and_geometry(self):
        self.assertEqual(self.cfg["marker"]["code_side_m"],.15)
        self.assertEqual(self.cfg["marker"]["board_side_m"],.20)
        self.assertEqual(self.cfg["marker"]["known_marker_to_pad_vehicle_m"],[.25,0.,.10])
        own=SelfMotion(self.cfg,self.entry);observer=Observer(self.cfg,own)
        for bad in (np.zeros((324,324),np.uint8),np.zeros((320,320,3),np.uint8),np.zeros((320,320),np.float32)):
            with self.assertRaises(ValueError):observer.frame(bad,0.,0.)

    def test_stadium_continuity_at_all_seams(self):
        track=Stadium.from_config(self.cfg["track"])
        seams=[0.,track.straight,track.straight+math.pi*track.radius,2*track.straight+math.pi*track.radius]
        for s in seams:
            left,h1,_=track.pose(s-1e-7);right,h2,_=track.pose(s+1e-7)
            self.assertLess(np.linalg.norm(left-right),3e-7)
            self.assertLess(abs(wrap(h1-h2)),1e-6)
        p,h,turn=track.pose(2.)
        self.assertFalse(turn);self.assertAlmostEqual(h,0.);self.assertAlmostEqual(p[1],6.25)

    def test_heading_regression_does_not_differentiate_filter_innovation(self):
        v=VehicleState(self.cfg)
        for i in range(120):
            t=i*.05;h=math.radians(4.)*math.sin(2*math.pi*4*t)
            ok,_=v.observe(np.array([20+.31*t,6.25,.3]),h,t,t+.05)
            self.assertTrue(ok)
        self.assertAlmostEqual(v.velocity[0],.31,delta=.001)
        self.assertLess(abs(v.heading_rate),math.radians(.5))

    def test_no_vision_after_discovery_blocks_descent(self):
        own=SelfMotion(self.cfg,self.entry)
        own_state={"position_world_m":[20.,6.25,.5],"velocity_world_m_s":[.25,0.,0.],
                   "heading_rad":0.,"position_std_m":.1}
        m=Mission3(self.cfg,self.entry);m.phase="DESCEND";m.discovery_time=0.;m.last_good=0.
        c=m.update(.4,own_state,None)
        self.assertEqual(c.phase,"RECOVER");self.assertEqual(c.velocity_world_m_s,[0.,0.,0.])
        self.assertFalse(c.physical_success);self.assertFalse(c.motors_off)
        c=m.update(2.1,own_state,None)
        self.assertEqual(c.phase,"ABORT")

    def test_coarse_marker_id_bearing_never_enables_landing(self):
        m=Mission3(self.cfg,self.entry)
        own={"position_world_m":[20.,6.25,.7],"velocity_world_m_s":[0.,0.,0.],
             "heading_rad":0.,"position_std_m":.1}
        v={"kind":"coarse_image_bearing_only","valid":True,"age_s":.1,"bearing_world_rad":.2}
        c=m.update(30.,own,v)
        self.assertEqual(c.phase,"COARSE_ACQUIRE")
        self.assertIsNone(m.discovery_time)
        self.assertFalse(c.physical_success);self.assertFalse(c.motors_off)
        self.assertNotEqual(c.phase,"DESCEND")

    def test_floor_to_moving_pad_range_does_not_apply_floor_height_anchor(self):
        own=SelfMotion(self.cfg,self.entry);own.position=np.array([20.,6.25,.5])
        def sample(t,r,flow):
            return {"stamp_s":t,"gyro_z_rad_s":0.,"accel_heading_m_s2":[0.,0.,0.],
                    "range_m":r,"flow_angular_rad_s":flow}
        own.update(sample(0.,.5,[.5,0.]))
        for i in range(1,11):own.update(sample(i*.02,.5,[.5,0.]))
        own.last_visual={"stamp_s":.2,"relative_pad_world_m":np.array([0.,0.,-.2]),
                         "vehicle_velocity_world_m_s":np.array([.25,0.,0.])}
        state=own.update(sample(.22,.2,[0.,0.]))
        self.assertEqual(state["surface_hypothesis"],"moving_surface_candidate")
        self.assertAlmostEqual(state["position_world_m"][2],.5,delta=.01)
        self.assertNotIn("surface_id",state)

    def test_pnp_recovers_unknown_world_vehicle_yaw_from_image(self):
        # Independent projected image fixtures, no simulation truth sent to observer.
        for vehicle_deg,own_deg in ((0.,8.),(35.,43.),(170.,178.)):
            vh,dh=map(math.radians,(vehicle_deg,own_deg));pad=np.array([25.,10.,.3])
            own=SelfMotion(self.cfg,self.entry)
            own.position=pad+rotation(vh)@np.array([-.80,-.08,.25]);own.heading=dh
            own.history.append((0.,own.position.copy(),dh))
            obs=Observer(self.cfg,own)
            marker=pad+rotation(vh)@np.array([.25,0.,.1])
            rcm=R_HEADING_CAMERA.T@rotation(dh).T@rotation(vh)@R_VEHICLE_MARKER
            tcm=R_HEADING_CAMERA.T@rotation(dh).T@(marker-own.position)
            corners=cv2.projectPoints(obs.detector.points,cv2.Rodrigues(rcm)[0],tcm,obs.detector.K,np.zeros(5))[0].reshape(4,2)
            marker_image=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,180)
            h=cv2.getPerspectiveTransform(np.float32([[0,0],[179,0],[179,179],[0,179]]),np.float32(corners))
            gray=cv2.warpPerspective(marker_image,h,(320,320),flags=cv2.INTER_NEAREST,borderValue=255)
            result=obs.frame(gray,0.,.05)
            self.assertTrue(result["valid"],result)
            estimated=obs.state(.05)
            self.assertLess(abs(wrap(estimated["vehicle_heading_rad"]-vh)),math.radians(7.))
            self.assertLess(np.linalg.norm(np.asarray(result["raw_pad_world_m"])-pad),.06)

if __name__=="__main__":unittest.main()
