"""Boundary checks that distinguish conditional physics from optical points."""
import inspect
import json
import math
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from yaw_core import rotation_rpy
from .settings import ROOT,load,controller_settings
from .scene import level_surface_patch,R_HEADING_CAMERA,R_VEHICLE_MARKER
from .track import rotation,wrap
from .estimator import SelfMotion
from .vision import Observer
from .flight_controller import StockPid
from .physical_scene import PhysicalPlant


class PhysicalBoundaries(unittest.TestCase):
    def test_component_envelope_rejects_close_precise_sensors(self):
        physical=json.loads((ROOT/"config/mission3-physical-sensitivity.json").read_text())
        plant=PhysicalPlant.__new__(PhysicalPlant)
        plant.physical=physical;plant.rng=np.random.default_rng(1)
        plant.data=type("Time",(),{"time":1.})()
        plant.sensor_scope={"conditional":True}
        for distance,flow_ok,tof_ok in ((.03,False,False),(.06,False,True),(.09,True,True)):
            plant.patch_sensor=lambda *a,d=distance:{"valid":True,"angular_rad_s":[0.,0.],"distances_m":[d]}
            self.assertEqual(plant.flow_packet()["flow_valid"],flow_ok)
            r=plant.range_packet()
            self.assertTrue(r["range_valid"])
            self.assertEqual(r["range_accurate"],tof_ok)

    def test_detectable_inaccurate_range_cannot_anchor_height(self):
        cfg=controller_settings(load());own=SelfMotion(cfg,cfg["entry"]["positions"][0])
        own.position[2]=.35;own.terminal_hint={"capture_stamp_s":0.,"relative_pad_world_m":[0.,0.,-.05],"vehicle_velocity_world_m_s":[.25,0,0],"vehicle_heading_rad":0.}
        result=own.update({"stamp_s":0.,"gyro_z_rad_s":0.,"accel_heading_m_s2":[0,0,0],"range_m":.01,"range_valid":True,"range_accurate":False,"flow_angular_rad_s":[99.,0.],"flow_valid":False})
        self.assertAlmostEqual(own.position[2],.35)
        self.assertFalse(result["flow_fusion_usable"])
        self.assertEqual(result["surface_hypothesis"],"ambiguous")

    def test_exact_official_motor_compensation_body_and_stop(self):
        raw=(ROOT/"src/bitcraze-mission3-firmware/src/drivers/src/motors.c").read_text()
        start=raw.index("float motorsCompensateBatteryVoltage(");end=raw.index("/* Public functions */",start)
        self.assertEqual((ROOT/"build/mission3-flight-controller/official_motor_compensation.inc").read_text(),raw[start:end].rstrip()+"\n")
        pid=StockPid()
        for tick in range(1,101):out=pid.update(tick,[0,0,1],[0,0,0],[0,0,0],[0,0,0],[0,0,1],[0,0,0],0.)
        for request,force in zip(out["requested_normalized_thrust"],out["motor_static_force_N"]):
            self.assertAlmostEqual(force,request/65535*.12,delta=1e-5)
        stop=pid.update(101,[0,0,1],[0,0,0],[0,0,0],[0,0,0],[0,0,1],[0,0,0],0.,stop=True)
        self.assertEqual(stop["motor_pwm"],[0]*4);self.assertEqual(stop["motor_static_force_N"],[0.]*4)

    def test_finite_patch_is_not_center_point_probe(self):
        # A moving pad passes below a probe with exactly matching world speed.
        # Some rays see stationary floor: averaged flow must not equal zero.
        angular,ranges=level_surface_patch(np.array([0.,0.,.5]),np.array([.25,0,0]),np.array([0.,0.,.3]),0.,np.array([.25,0,0]),0.,[.1,.1],35.,7)
        self.assertGreater(angular[0],0.)
        self.assertGreater(np.max(ranges)-np.min(ranges),.29)
        pure,_=level_surface_patch(np.array([0.,0.,.34]),np.array([.25,0,0]),np.array([0.,0.,.3]),0.,np.array([.25,0,0]),0.,[.1,.1],21.,7)
        self.assertAlmostEqual(pure[0],0.)

    def test_invalid_flow_is_blocked_on_floor(self):
        cfg=controller_settings(load());own=SelfMotion(cfg,cfg["entry"]["positions"][0])
        own.velocity=np.array([.2,0,0])
        result=own.update({"stamp_s":0.,"gyro_z_rad_s":0.,"accel_heading_m_s2":[0,0,0],"range_m":1.,"flow_angular_rad_s":[-9.,0.],"flow_valid":False})
        self.assertAlmostEqual(own.velocity[0],.2);self.assertFalse(result["flow_fusion_usable"])

    def test_aircraft_has_no_pose_write_during_step_and_falls_under_gravity(self):
        source=inspect.getsource(PhysicalPlant.step)
        self.assertNotIn(".qpos",source);self.assertNotIn(".qvel[",source.split("self.update_vehicle")[1])
        cfg=load();physical=json.loads((ROOT/"config/mission3-physical-sensitivity.json").read_text());physical["motor"]["response_time_constant_s"]=0.
        with tempfile.TemporaryDirectory() as temp:
            plant=PhysicalPlant(cfg,physical,temp,1,1,initial_arc_m=30.)
            z0=plant.position[2]
            try:
                for _ in range(100):plant.step({"motor_static_force_N":[0]*4,"motor_pwm":[0]*4},.046,.00735)
                self.assertLess(plant.position[2],z0-.045)
                self.assertEqual(plant.model.neq,0);self.assertEqual(plant.model.nmocap,0)
            finally:plant.close()

    def test_tilted_body_camera_pose_is_based_on_capture_attitude(self):
        cfg=controller_settings(load());pad=np.array([25.,10.,.3]);roll,pitch=map(math.radians,[8.,-4.])
        offset=np.array([.02,0.,.015]);Rbc=rotation_rpy(*np.deg2rad([2.,-.5,0.]))@R_HEADING_CAMERA
        for vh in (0.,math.radians(60.)):
            dh=vh+math.radians(8.);Rwb=rotation_rpy(roll,pitch,dh)
            own=SelfMotion(cfg,cfg["entry"]["positions"][0]);own.position=pad+rotation(vh)@np.array([-.80,-.08,.25]);own.heading=dh
            own.history.append((0.,own.position.copy(),dh));own.attitude_at=lambda stamp:(roll,pitch)
            own.camera_rotation=Rbc;own.camera_position_body=offset
            obs=Observer(cfg,own)
            marker=pad+rotation(vh)@np.array([.25,0.,.10]);camera=own.position+Rwb@offset
            Rcm=Rbc.T@Rwb.T@rotation(vh)@R_VEHICLE_MARKER;tcm=Rbc.T@Rwb.T@(marker-camera)
            corners=cv2.projectPoints(obs.detector.points,cv2.Rodrigues(Rcm)[0],tcm,obs.detector.K,np.zeros(5))[0].reshape(4,2)
            marker_image=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,180)
            h=cv2.getPerspectiveTransform(np.float32([[0,0],[179,0],[179,179],[0,179]]),np.float32(corners))
            gray=cv2.warpPerspective(marker_image,h,(320,320),flags=cv2.INTER_NEAREST,borderValue=255)
            result=obs.frame(gray,0.,.05)
            self.assertTrue(result["valid"],result)
            self.assertLess(np.linalg.norm(np.asarray(result["raw_pad_world_m"])-pad),.06)
            self.assertLess(abs(wrap(result["raw_vehicle_heading_rad"]-vh)),math.radians(7.))


if __name__=="__main__":unittest.main()
