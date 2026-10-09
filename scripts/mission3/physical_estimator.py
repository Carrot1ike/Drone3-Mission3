"""Sensor-derived conditional flight state; no contact or live vehicle truth.

Body attitude is propagated in the unmodified official Kalman core at its
declared rates. Mission position/velocity use the project's surface-aware
observer. This is not a drop-in firmware estimator replacement yet.
"""
from collections import deque
import math
import numpy as np
from yaw_core import rotation_rpy,quaternion_rpy,euler_yaw_rate
from .estimator import SelfMotion
from .firmware import FirmwareCore
from .track import rotation
from .sensor_time import available_timestamp


class PhysicalMotion(SelfMotion):
    def __init__(self,cfg,entry,physical,camera_rotation):
        super().__init__(cfg,entry)
        self.physical=physical
        self.camera_position_body=np.asarray(physical["mounts"]["camera_position_body_m"])
        self.camera_rotation=camera_rotation
        self.core=FirmwareCore(self.position,self.heading)
        self.roll,self.pitch=0.,0.
        self.attitude_history=deque(maxlen=3000)
        self.latest_imu=None
        self.sum_acc=np.zeros(3);self.sum_gyro=np.zeros(3);self.imu_count=0
        self.control_acc=np.zeros(3);self.control_count=0
        self.firmware_parallel=self.core.state()
        self.full_onboard_ekf=physical.get("control_architecture",{}).get("onboard_estimator")=="official_full_flow_tof"
        self.onboard_measurement_counts={"flow":0,"tof":0}
        self.onboard_tof_min_raw_m=physical.get("control_architecture",{}).get("onboard_tof_min_raw_m",0.)
        if not math.isfinite(self.onboard_tof_min_raw_m) or not 0<=self.onboard_tof_min_raw_m<5.:
            raise ValueError("onboard raw ToF minimum must be finite in [0,5)m")
        if self.onboard_tof_min_raw_m>0:
            # Research software gate based only on measured raw distance.
            # It does not certify RangeStatus/reflectance or identify a pad.
            self.onboard_measurement_counts["tof_near_raw_rejected"]=0
        self.pc_independent_attitude=physical.get("control_architecture",{}).get("pc_attitude")=="independent_imu_100hz"
        self.pc_core=FirmwareCore(self.position,self.heading,role="pc_imu") if self.pc_independent_attitude else None
        self.pc_attitude_state=self.pc_core.state() if self.pc_core else None

    def refresh_attitude(self,stamp):
        source=self.pc_attitude_state if self.pc_core else self.firmware_parallel
        q=np.asarray(source["quaternion_wxyz"])
        self.roll,self.pitch,_=quaternion_rpy(np.r_[q[1:],q[0]])
        self.attitude_history.append((stamp,self.roll,self.pitch))

    def imu(self,tick,sample):
        self.latest_imu=sample
        acc=np.asarray(sample["specific_force_body_m_s2"]);gyro=np.asarray(sample["gyro_body_rad_s"])
        self.sum_acc+=acc;self.sum_gyro+=gyro;self.imu_count+=1
        self.control_acc+=acc;self.control_count+=1
        self.core.process_noise(tick)
        if tick%10==0:
            avg_acc=self.sum_acc/self.imu_count;avg_gyro=self.sum_gyro/self.imu_count
            self.core.predict(tick,avg_acc,avg_gyro)
            if self.pc_core:
                # A conditional PC telemetry packet: averaged body IMU at
                # 100Hz. The PC context does not see 1kHz deck/plant states.
                self.pc_core.process_noise(tick)
                self.pc_core.predict(tick,avg_acc,avg_gyro)
                self.pc_core.finalize();self.pc_attitude_state=self.pc_core.state()
            self.sum_acc*=0;self.sum_gyro*=0;self.imu_count=0
        self.core.finalize();self.firmware_parallel=self.core.state()
        self.refresh_attitude(sample["stamp_s"])

    def attitude_at(self,stamp):
        if not self.attitude_history or stamp<self.attitude_history[0][0]-1e-6 or stamp>self.attitude_history[-1][0]+.025:return None
        times=[p[0] for p in self.attitude_history]
        return tuple(float(np.interp(stamp,times,[p[i] for p in self.attitude_history])) for i in (1,2))

    def state_from_deck(self,deck):
        # This preflight must precede the official EKF measurement calls and
        # consumption of the PC acceleration accumulator, not only super.update.
        stamp,_=self.validate_sample_time(deck)
        available_timestamp(self.latest_imu['stamp_s'],stamp,'IMU')
        if 'flow_stamp_s' in deck:available_timestamp(deck['flow_stamp_s'],stamp,'flow')
        gyro=np.asarray(self.latest_imu["gyro_body_rad_s"])
        if self.full_onboard_ekf:
            # Stock firmware model interprets range against its stationary
            # ground datum. It does not receive PC pad/world corrections.
            distance=deck.get("range_m")
            fresh_range=deck.get("range_new",True)
            if (fresh_range and distance is not None and 0<=distance<self.onboard_tof_min_raw_m):
                self.onboard_measurement_counts["tof_near_raw_rejected"]+=1
            if (fresh_range and distance is not None and self.onboard_tof_min_raw_m<=distance<5.):
                distance=round(distance*1000)/1000
                coefficient=math.log(.2/.0025)/(4.-2.5)
                std=.0025*(1+math.exp(coefficient*(distance-2.5)))
                self.core.tof(int(round(deck["range_stamp_s"]*1000)),distance,std)
                self.onboard_measurement_counts["tof"]+=1
            if deck["flow_valid"]:
                counts=np.rint(deck["flow_raw_dpixel"])
                if np.max(np.abs(counts))<100:
                    self.core.flow(int(round(deck["stamp_s"]*1000)),counts,deck["flow_dt_s"],(2.,2.),np.rad2deg(gyro))
                    self.onboard_measurement_counts["flow"]+=1
            self.core.finalize();self.firmware_parallel=self.core.state()
            self.refresh_attitude(deck["stamp_s"])
        accel=self.control_acc/max(1,self.control_count)
        self.control_acc*=0;self.control_count=0
        R=rotation_rpy(self.roll,self.pitch,self.heading)
        world_acc=R@accel-np.array([0.,0.,9.81])
        measured_range=deck["range_m"]
        offset=np.asarray(self.physical["mounts"]["flow_position_body_m"])
        body_height_above_surface=None if measured_range is None else measured_range*R[2,2]-(R@offset)[2]
        factor=35./.71674/.1
        angular=np.asarray(deck["flow_raw_dpixel"])/(factor*deck["flow_dt_s"])-np.array([-gyro[1],gyro[0]])
        range_for_flow=measured_range if measured_range is not None else .1
        lever_v=np.cross(gyro,offset)
        body_v=np.r_[angular*range_for_flow,(R.T@self.velocity)[2]+lever_v[2]]-lever_v
        heading_v=rotation(self.heading).T@(R@body_v)
        packet={"stamp_s":deck["stamp_s"],"gyro_z_rad_s":euler_yaw_rate(gyro,self.roll,self.pitch),
                "accel_heading_m_s2":(rotation(self.heading).T@world_acc).tolist(),
                "range_m":body_height_above_surface,"range_valid":deck["range_valid"],
                "range_accurate":deck.get("range_accurate",True),
                "range_new":deck.get("range_new",True),"range_stamp_s":deck.get("range_stamp_s",deck["stamp_s"]),
                "flow_angular_rad_s":(heading_v[:2]/max(body_height_above_surface or .1,.005)).tolist(),
                "flow_valid":deck["flow_valid"],"virtual_level_probe":False,
                "mount_and_sensor_transfer_are_conditional":True}
        state=super().update(packet)
        state.update(rpy_rad=[self.roll,self.pitch,self.heading],
                     gyro_body_rad_s=gyro.tolist(),specific_force_body_m_s2=accel.tolist(),
                     range_accurate=packet["range_accurate"],flow_valid=deck["flow_valid"],
                     pc_attitude_source=("independent body-IMU PC prediction at100Hz; no deck corrections"
                                         if self.pc_core else "onboard estimator attitude"),
                     sensor_scope=("conditional body sensors + project surface observer; official full Flow/ToF EKF supplies onboard attitude only"
                                   if self.full_onboard_ekf else "conditional body IMU+finite-patch Flow/ToF with project surface observer; official Kalman IMU attitude propagation"))
        return state,packet

    def lower_state(self):
        gyro=np.asarray(self.latest_imu["gyro_body_rad_s"])
        acc=np.asarray(self.latest_imu["specific_force_body_m_s2"])
        return (self.position,self.velocity,np.rad2deg([self.roll,-self.pitch,self.heading]),np.rad2deg(gyro),acc/9.81)

    def onboard_attitude_state(self):
        """Only the onboard EKF's attitude and measured IMU for RPYT control."""
        q=np.asarray(self.firmware_parallel["quaternion_wxyz"])
        roll,pitch,yaw=quaternion_rpy(np.r_[q[1:],q[0]])
        return (np.rad2deg([roll,-pitch,yaw]),np.rad2deg(self.latest_imu["gyro_body_rad_s"]),
                np.asarray(self.latest_imu["specific_force_body_m_s2"])/9.81)
