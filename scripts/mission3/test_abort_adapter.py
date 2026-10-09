"""Exercise the actual adapter dispatch with controlled I/O, not flight physics."""
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from contextlib import redirect_stdout
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import run_mission3_physical as adapter
from mission3.controller import Command
from mission3.settings import ROOT, load


class PlantIO:
    def __init__(self, *args):
        self.time=0.; self.camera_rotation=np.eye(3); self.geom_class_map={}
        self.contact_description={'derived':{'inertia_kg_m2':[1e-5]*3}}
        self.model=SimpleNamespace(opt=SimpleNamespace(timestep=.001)); self.rpy=np.zeros(3)
    def imu(self):return {'stamp_s':self.time,'gyro_body_rad_s':[0.,0.,0.],'specific_force_body_m_s2':[0.,0.,9.81]}
    def range_packet(self):return {'range_stamp_s':self.time}
    def flow_packet(self,dt):return {'flow_dt_s':dt}
    def image(self,strict):return None,'controlled_test_no_camera'
    def step(self,*args):self.time+=.001
    def truth(self):return {'t':self.time,'contacts':[]}
    def close(self):pass


class MotionIO:
    usable=True
    def __init__(self,*args):
        self.position=np.array([1.,2.,1.]); self.velocity=np.zeros(3)
        self.heading=self.roll=self.pitch=0.;self.last_stamp=None
        self.onboard_measurement_counts={};self.events=[]
        self.firmware_parallel={};self.pc_attitude_state={}
    def imu(self,*args):pass
    def state_from_deck(self,deck):
        self.last_stamp=deck['stamp_s']
        own=dict(position_world_m=self.position.tolist(),velocity_world_m_s=self.velocity.tolist(),
                 heading_rad=0.,rpy_rad=[0.,0.,0.],gyro_body_rad_s=[0.,0.,0.],
                 range_accurate=True,flow_valid=self.usable,flow_fusion_usable=self.usable,
                 flow_fusion_reason='floor_patch_clear',position_std_m=.1)
        return own,{}
    def onboard_attitude_state(self):return ([0.,0.,0.],[0.,0.,0.],[0.,0.,1.])


class PidIO:
    arm_length=.046;thrust_to_torque=.005
    def update_rpyt(self,*args,**kwargs):return {'motor_pwm':[10000]*4,'motor_static_force_N':[.08]*4}


class PositionIO:
    def __init__(self,*args):pass
    def update(self,*args):return {'roll_deg':0.,'pitch_deg':0.,'legacy_collective':30000}


class ObserverIO:
    def __init__(self,*args):pass
    def state(self,now):return None


class SupportIO:
    latest=None
    def __init__(self,*args):pass
    def calibrate_freeflight(self,*args):pass
    def imu(self,*args):pass
    def calibration_is_fresh(self,*args):return False
    def predict_motor(self,*args):pass


class FailedMission:
    updates=0
    def __init__(self,*args,**kwargs):
        self.last_v=np.zeros(3);self.last_yaw_rate=0.;self.events=[]
    def update(self,*args):
        type(self).updates+=1
        return Command('ABORT',[0.,0.,0.],0.,done=True,failure='controlled_failure')


class AbortAdapterDispatch(unittest.TestCase):
    def exercise(self,usable):
        MotionIO.usable=usable;FailedMission.updates=0
        cfg=load(ROOT/'config/mission3-35x12-entry-scan15-imu12-v14.json')
        cfg['control']['mission_timeout_s']=1.
        physical=json.loads((ROOT/'config/mission3-rpyt-35x12-entry-hold-limited.json').read_text())
        physical.pop('mission_handoff',None)
        with TemporaryDirectory() as d:
            with patch.multiple(adapter,PhysicalPlant=PlantIO,PhysicalMotion=MotionIO,StockPid=PidIO,
                                PCPositionPid=PositionIO,Observer=ObserverIO,TouchdownObserver=SupportIO,
                                Mission3=FailedMission),redirect_stdout(io.StringIO()):
                result=adapter.run(cfg,physical,Path(d)/'adapter',1,613,record=False)
            wire=[json.loads(l) for l in (Path(d)/'adapter/rpyt_packets.jsonl').read_text().splitlines()]
            self.assertTrue(wire)
            self.assertTrue(all(r['stop'] is False for r in wire))
            self.assertEqual(FailedMission.updates,1,'failed mission must never overwrite the delivery controller')
            self.assertEqual(result['failure'],'controlled_failure')
            self.assertEqual(result['abort_delivery']['started_at_s'],0.)
            self.assertEqual(result['abort_delivery']['packet_count_after_failure'],len(wire))
            self.assertGreater(result['abort_delivery']['physics_steps_after_failure'],0)
            return result

    def test_actual_dispatch_latches_failure_delivers_and_observes_stationary(self):
        r=self.exercise(True)
        self.assertEqual(r['abort_delivery']['finish'],'sensor_stationary_hold_observed')
        self.assertLessEqual(r['duration_s'],.521)

    def test_actual_dispatch_bounds_missing_stationary_evidence_at3s(self):
        r=self.exercise(False)
        self.assertEqual(r['abort_delivery']['finish'],'bounded_delivery_elapsed_without_stationary_evidence')
        self.assertLessEqual(r['duration_s'],3.001+1e-8)


if __name__=='__main__':unittest.main()
