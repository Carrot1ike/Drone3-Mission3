"""조건부 접촉 후보: IMU specific force와 명령 추력의 차이.

접촉/기체 참값을 받지 않는다. 모델 오차, 지면 효과, IMU 장착 오차는
지지력과 혼동될 수 있으므로 이 결과는 실기 접촉 보장이 아니다.
최종 성공은 별도의 물리 로그 평가와 독립 검토에서 판단한다.
"""
from collections import deque
import math
import numpy as np


class TouchdownObserver:
    def __init__(self, settings):
        self.c=dict(settings)
        self.force=np.zeros(4)
        self.samples=deque()
        self.support_since=None
        self.hold_bad_since=None
        self.latest=None
        self.force_to_accel=1/self.c["assumed_mass_kg"]
        self.calibration=deque()
        self.cal_ff=self.cal_fa=self.cal_aa=0.
        self.calibration_valid=not self.c.get("require_freeflight_calibration",False)
        self.calibration_rms=None
        self.calibration_last_valid_s=None

    def calibrate_freeflight(self, sample, permitted):
        """Identify actuator force-model/actual mass ratio from IMU in free air.

        Caller requires a qualified floor datum and height above the board.
        The ratio is held during final approach; it is not a hardware mass spec.
        """
        if not permitted:return
        now=sample["stamp_s"];force=float(self.force.sum())
        accel=float(sample["specific_force_body_m_s2"][2])
        if force<.10 or not 3.<accel<15. or np.linalg.norm(sample["gyro_body_rad_s"])>.3:return
        self.calibration.append((now,force,accel))
        self.cal_ff+=force*force;self.cal_fa+=force*accel;self.cal_aa+=accel*accel
        while self.calibration and now-self.calibration[0][0]>self.c.get("calibration_window_s",2.):
            _,f,a=self.calibration.popleft();self.cal_ff-=f*f;self.cal_fa-=f*a;self.cal_aa-=a*a
        if now-self.calibration[0][0]<self.c.get("calibration_min_span_s",1.):return
        fitted=self.cal_fa/max(self.cal_ff,1e-9)
        rms=math.sqrt(max(0.,self.cal_aa-2*fitted*self.cal_fa+fitted*fitted*self.cal_ff)/len(self.calibration))
        nominal=1/self.c["assumed_mass_kg"]
        valid=(.5*nominal<fitted<1.5*nominal and rms<self.c.get("calibration_max_residual_rms_m_s2",.3))
        if valid:
            self.force_to_accel=fitted;self.calibration_valid=True;self.calibration_rms=rms
            self.calibration_last_valid_s=now

    def calibration_is_fresh(self,now):
        limit=self.c.get("calibration_max_age_s")
        return bool(self.calibration_valid and (limit is None or
                    (self.calibration_last_valid_s is not None and 0<=now-self.calibration_last_valid_s<=limit)))

    def predict_motor(self, commanded_static_force_N, dt):
        # Commanded PWM -> official software force curve -> assumed motor lag.
        # This is an observer state, never actual plant motor force.
        target=np.asarray(commanded_static_force_N,float)
        tau=self.c["assumed_motor_response_s"]
        gain=1. if tau<=0 else -math.expm1(-dt/tau)
        self.force+=gain*(target-self.force)

    def imu(self, sample):
        now=float(sample["stamp_s"])
        accel=np.asarray(sample["specific_force_body_m_s2"],float)
        gyro=np.asarray(sample["gyro_body_rad_s"],float)
        expected=float(self.force.sum()*self.force_to_accel)
        # At the assumed CoM, rotor force acts on body Z. Gravity is absent
        # from specific force. Hover alone therefore gives residual ~0.
        residual=float(accel[2]-expected)
        self.samples.append((now,residual,float(accel[2]),float(np.linalg.norm(gyro))))
        while self.samples and now-self.samples[0][0]>self.c["filter_window_s"]:
            self.samples.popleft()
        self.latest={"t":now,"support_residual_m_s2":float(np.mean([s[1] for s in self.samples])),
                     "specific_force_z_m_s2":float(np.mean([s[2] for s in self.samples])),
                     "gyro_norm_rad_s":float(np.mean([s[3] for s in self.samples])),
                     "command_model_acceleration_m_s2":expected,
                     "freeflight_calibration_valid":self.calibration_valid,
                     "force_model_to_specific_acceleration_gain_per_kg":self.force_to_accel,
                     "freeflight_calibration_residual_rms_m_s2":self.calibration_rms,
                     "freeflight_calibration_last_valid_s":self.calibration_last_valid_s,
                     "freeflight_calibration_fresh":self.calibration_is_fresh(now),
                     "window_s":now-self.samples[0][0],
                     "scope":"conditional command-model support candidate, no contact truth"}
        return self.latest

    def contact_candidate(self, now, roll, pitch, enabled,estimated_body_height_above_pad_m=None):
        s=self.latest
        tilt=math.hypot(roll,pitch)
        expected_height=self.c.get("support_body_height_above_pad_m")
        height_ok=bool(expected_height is None or
            (estimated_body_height_above_pad_m is not None and math.isfinite(estimated_body_height_above_pad_m)
             and abs(estimated_body_height_above_pad_m-expected_height)<self.c["support_height_tolerance_m"]))
        if s:
            s["support_height_gate_valid"]=height_ok
            s["estimated_body_height_above_pad_m"]=(None if estimated_body_height_above_pad_m is None
                                                    else float(estimated_body_height_above_pad_m))
        ok=bool(enabled and height_ok and self.calibration_is_fresh(now) and s and s["window_s"]>=self.c["filter_window_s"]*.8
                and s["support_residual_m_s2"]>=self.c["support_residual_min_m_s2"]
                and s["command_model_acceleration_m_s2"]<=self.c.get("probe_motor_accel_max_m_s2",float("inf"))
                and abs(s["specific_force_z_m_s2"]-9.81)<self.c["supported_accel_tolerance_m_s2"]
                and s["gyro_norm_rad_s"]<self.c["gyro_limit_rad_s"]
                and tilt<math.radians(self.c["tilt_limit_deg"]))
        if ok:
            if self.support_since is None:self.support_since=now
        else:self.support_since=None
        return bool(self.support_since is not None and now-self.support_since>=self.c["support_confirm_s"])

    def hold_supported(self, now, roll, pitch):
        s=self.latest
        stable=bool(s and abs(s["specific_force_z_m_s2"]-9.81)<self.c["supported_accel_tolerance_m_s2"]
                    and s["gyro_norm_rad_s"]<self.c["gyro_limit_rad_s"]
                    and math.hypot(roll,pitch)<math.radians(self.c["tilt_limit_deg"]))
        if stable:self.hold_bad_since=None
        elif self.hold_bad_since is None:self.hold_bad_since=now
        failed=self.hold_bad_since is not None and now-self.hold_bad_since>self.c["hold_bad_grace_s"]
        return stable,failed
