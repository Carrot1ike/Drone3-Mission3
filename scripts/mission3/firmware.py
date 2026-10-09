"""ctypes wrapper around the official estimator, with explicit input units.

The adapter has an onboard context and an isolated PC IMU context. There is
one of each per process; a second object with the same role resets that role.
"""
import ctypes as ct
from types import SimpleNamespace
import numpy as np
from .settings import ROOT

class FirmwareCore:
    def __init__(self, position=(0.,0.,.5), yaw=0., role="onboard"):
        if role not in ("onboard","pc_imu"):raise ValueError("unknown estimator context")
        library=ct.CDLL(str(ROOT/"build/mission3-firmware-host/libmission3_estimator.so"))
        u,f,i=ct.c_uint,ct.c_float,ct.c_int
        signatures={"bench_init":([f]*4,None),"bench_predict":([u]+[f]*6+[i],None),
                    "bench_process_noise":([u],None),
                    "bench_flow":([u]+[f]*8,None),"bench_tof":([u,f,f],None),
                    "bench_finalize":([],None),"bench_state":([i],f),
                    "bench_covariance":([i,i],f),"bench_rotation":([i,i],f),
                    "bench_quaternion":([i],f)}
        self.lib=SimpleNamespace()
        self.library=library
        for name,(args,result) in signatures.items():
            if role=="pc_imu" and name in ("bench_flow","bench_tof"):continue
            fn=getattr(library,("pc_" if role=="pc_imu" else "")+name);fn.argtypes=args;fn.restype=result
            # Per-object binding keeps role selection out of numerical calls.
            setattr(self.lib,name,fn)
        self.role=role
        self.lib.bench_init(*position,yaw)

    def predict(self, now_ms, specific_force_body_m_s2=(0.,0.,9.81),gyro_body_rad_s=(0.,0.,0.),flying=True):
        self.lib.bench_predict(now_ms,*specific_force_body_m_s2,*gyro_body_rad_s,int(flying))

    def flow(self, now_ms, raw_dpixel,dt=.01,std=(2.,2.),gyro_body_deg_s=(0.,0.,0.)):
        if self.role!="onboard":raise ValueError("PC IMU context rejects deck updates")
        self.lib.bench_flow(now_ms,*raw_dpixel,dt,*std,*gyro_body_deg_s)

    def process_noise(self,now_ms):self.lib.bench_process_noise(now_ms)

    def tof(self, now_ms,distance,std=.0025):
        if self.role!="onboard":raise ValueError("PC IMU context rejects deck updates")
        self.lib.bench_tof(now_ms,distance,std)

    def finalize(self):self.lib.bench_finalize()

    def state(self):
        s=np.array([self.lib.bench_state(i) for i in range(9)])
        r=np.array([[self.lib.bench_rotation(i,j) for j in range(3)] for i in range(3)])
        return {"position_world_m":s[:3].tolist(),"velocity_body_m_s":s[3:6].tolist(),
                "velocity_world_m_s":(r@s[3:6]).tolist(),
                "quaternion_wxyz":[self.lib.bench_quaternion(i) for i in range(4)],
                "covariance_diagonal":[self.lib.bench_covariance(i,i) for i in range(9)]}
