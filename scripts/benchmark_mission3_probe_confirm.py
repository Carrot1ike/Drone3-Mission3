#!/usr/bin/env python3
"""조건부 비미션 probe: 최소 collective 확인·두 수준 IMU 응답 반례.

공유 파일은 읽기 전용이다. 동결 observer/물리 코드를 artifact에 복사한
private runtime만 사용한다. truth는 기록·종료 후 평가만 하며 후보 입력이 아니다.
"""
import argparse
from collections import deque
from copy import deepcopy
import csv
import ctypes as ct
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
from zoneinfo import ZoneInfo

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = ROOT/"artifacts/mission3/probe-confirm-negative"
FROZEN_OBSERVER = ROOT/"artifacts/mission3/calibration-change-negative/20261008-062707-833982/source/scripts/mission3/touchdown.py"
OBSERVER_SHA256 = "94e339227e1c3c553ee18e7196bd36b34d77f9c2c5015a3fd9c91334cac77a8f"
MASS = .035
CALIBRATION_END_S = 2.5
EARLY_GAIN_CHANGE_S = 2.6
PROBE_START_S = 2.9
UPPER_ACCEL = 6.5
MINIMUM_MULTIPLIER = .60
LOWER_ACCEL = 9.81*MINIMUM_MULTIPLIER
UPPER_RAMP_S = .25
UPPER_HOLD_S = .65
LOWER_START_S = PROBE_START_S+UPPER_RAMP_S+UPPER_HOLD_S
LOWER_RAMP_S = .20
MINIMUM_START_S = LOWER_START_S+LOWER_RAMP_S
MINIMUM_HOLD_S = 1.15
END_S = MINIMUM_START_S+MINIMUM_HOLD_S
SETTLE_S = .25
PREDICTION_TOLERANCE = .03
MULTIPLIER_TOLERANCE = .001
DUAL_MIN_EXCITATION = .4
DUAL_MAX_IMU_DIFFERENCE = .2


def serial(value):
    if isinstance(value, dict): return {k: serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)): return [serial(v) for v in value]
    if isinstance(value, (float, np.floating)): return float(value) if math.isfinite(value) else None
    if isinstance(value, np.generic): return value.item()
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(serial(value), ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def line(file, value):
    file.write(json.dumps(serial(value), ensure_ascii=False, allow_nan=False)+"\n")


def rows(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines()]


def scheduled_command(now):
    if now < PROBE_START_S: return 9.81, "NOMINAL"
    if now < PROBE_START_S+UPPER_RAMP_S:
        f=(now-PROBE_START_S)/UPPER_RAMP_S
        return 9.81+(UPPER_ACCEL-9.81)*f, "UPPER_RAMP"
    if now < LOWER_START_S: return UPPER_ACCEL, "UPPER_HOLD"
    if now < MINIMUM_START_S:
        f=(now-LOWER_START_S)/LOWER_RAMP_S
        return UPPER_ACCEL+(LOWER_ACCEL-UPPER_ACCEL)*f, "LOWER_RAMP"
    return LOWER_ACCEL, "MINIMUM_HOLD"


class NominalMotorCurve:
    def __init__(self, library, voltage):
        self.voltage=voltage
        self.lib=ct.CDLL(str(library))
        self.lib.flight_static_force.argtypes=[ct.c_uint, ct.c_float]
        self.lib.flight_static_force.restype=ct.c_float
        for name in ("flight_arm", "flight_thrust_to_torque"):
            getattr(self.lib, name).argtypes=[]
            getattr(self.lib, name).restype=ct.c_float
        self.arm=self.lib.flight_arm();self.torque=self.lib.flight_thrust_to_torque()

    def force(self, pwm): return float(self.lib.flight_static_force(pwm, self.voltage))

    def command(self, acceleration):
        lo,hi=0,65535;target=MASS*acceleration/4
        while lo<hi:
            mid=(lo+hi)//2
            if self.force(mid)<target: lo=mid+1
            else: hi=mid
        pwm=min((lo,max(0,lo-1)),key=lambda p:abs(self.force(p)-target))
        force=self.force(pwm)
        return {"motor_pwm":[pwm]*4,"motor_static_force_N":[force]*4,
                "nominal_requested_accel_m_s2":acceleration,"requested_multiplier":acceleration/9.81,
                "nominal_curve_accel_m_s2":4*force/MASS,"supply_voltage_V":self.voltage,
                "source":"scheduled symmetric nonmission fixture; no state/truth feedback"}


class SensorTilt:
    def __init__(self): self.rp=np.zeros(2)

    def update(self, sample):
        self.rp+=np.asarray(sample["gyro_body_rad_s"])[:2]*.01
        accel=np.asarray(sample["specific_force_body_m_s2"])
        if accel[2]>3. and np.linalg.norm(accel)>3.:
            measured=np.array([math.atan2(accel[1],accel[2]),
                math.atan2(-accel[0],math.hypot(accel[1],accel[2]))])
            self.rp+=.05*(measured-self.rp)
        return tuple(self.rp)


class StableWindow:
    """연속 센서 stamp와 commanded prediction의 .25s 안정 구간."""
    def __init__(self): self.samples=deque();self.last_stamp=None

    def update(self, now, eligible, prediction, measured):
        timing_ok=(self.last_stamp is None or 0<now-self.last_stamp<=.0201)
        self.last_stamp=now
        if not eligible or not timing_ok:
            self.samples.clear()
            return False
        self.samples.append((now,prediction,measured))
        while self.samples and now-self.samples[0][0]>SETTLE_S+.010001:
            self.samples.popleft()
        span=now-self.samples[0][0]
        spread=max(s[1] for s in self.samples)-min(s[1] for s in self.samples)
        return span>=SETTLE_S-1e-9 and spread<=PREDICTION_TOLERANCE

    def means(self):
        return {"stamp_s":self.samples[-1][0],"prediction_m_s2":float(np.mean([s[1] for s in self.samples])),
                "specific_force_z_m_s2":float(np.mean([s[2] for s in self.samples])),
                "span_s":self.samples[-1][0]-self.samples[0][0],"count":len(self.samples)}


class ProbeConfirm:
    """동결 observer를 감싸는 실험용 sensor/command-only wrapper.

    baseline/minimum_settled/dual_response는 동일한 명령 일정이다. height,
    접촉, 실제 gain/힘/속도는 받지 않는다. dual_response의 flat 응답은
    contact 보장이 아니라 후보 허가용 S heuristic이다.
    """
    def __init__(self, observer, policy):
        self.base=observer;self.policy=policy
        self.minimum=StableWindow();self.upper=StableWindow()
        self.upper_reference=None;self.first_minimum_ready=None

    def candidate(self, now, sample, held_command, phase, roll, pitch, enabled):
        evidence=self.base.latest
        predicted=evidence["command_model_acceleration_m_s2"]
        target=sum(held_command["motor_static_force_N"])*self.base.force_to_accel
        multiplier=held_command["requested_multiplier"]
        finite=(math.isfinite(predicted) and math.isfinite(target) and math.isfinite(multiplier)
                and np.isfinite(sample["specific_force_body_m_s2"]).all()
                and np.isfinite(sample["gyro_body_rad_s"]).all())
        at_minimum=bool(finite and abs(multiplier-MINIMUM_MULTIPLIER)<=MULTIPLIER_TOLERANCE
                        and abs(held_command["nominal_curve_accel_m_s2"]/9.81-MINIMUM_MULTIPLIER)<=MULTIPLIER_TOLERANCE)
        settled_instant=bool(finite and abs(predicted-target)<=PREDICTION_TOLERANCE)
        z=float(sample["specific_force_body_m_s2"][2])
        minimum_ready=self.minimum.update(now, at_minimum and settled_instant, predicted, z)
        if minimum_ready and self.first_minimum_ready is None: self.first_minimum_ready=now
        at_upper=bool(finite and phase=="UPPER_HOLD" and abs(held_command["nominal_curve_accel_m_s2"]-UPPER_ACCEL)<.02)
        upper_ready=self.upper.update(now,at_upper and settled_instant,predicted,z)
        if upper_ready: self.upper_reference=self.upper.means()
        dual={"available":False,"support_like_response":False,"air_response_veto":False,
              "upper":self.upper_reference,"lower":None,"delta_prediction_m_s2":None,
              "delta_imu_m_s2":None,"response_slope":None}
        if minimum_ready and self.upper_reference and 0<=now-self.upper_reference["stamp_s"]<=2.:
            lower=self.minimum.means();upper=self.upper_reference
            dp=upper["prediction_m_s2"]-lower["prediction_m_s2"]
            dz=upper["specific_force_z_m_s2"]-lower["specific_force_z_m_s2"]
            available=dp>=DUAL_MIN_EXCITATION
            flat=bool(available and abs(dz)<=DUAL_MAX_IMU_DIFFERENCE)
            dual.update(available=available,support_like_response=flat,
                        air_response_veto=bool(available and not flat),lower=lower,
                        delta_prediction_m_s2=dp,delta_imu_m_s2=dz,
                        response_slope=dz/dp if available else None)
        permitted=enabled and finite
        if self.policy!="baseline": permitted=permitted and minimum_ready
        if self.policy=="dual_response": permitted=permitted and dual["support_like_response"]
        confirmed=self.base.contact_candidate(now,roll,pitch,permitted)
        details={"policy":self.policy,"minimum_engaged":at_minimum,"prediction_settled_instant":settled_instant,
            "minimum_settled_window_ready":minimum_ready,"first_minimum_ready_s":self.first_minimum_ready,
            "settled_window_span_s":0. if not self.minimum.samples else now-self.minimum.samples[0][0],
            "support_enabled":permitted,"dual":dual,"height_gate_enabled":False}
        return confirmed,details


def plan(gain_change_stage):
    cases=[]
    for height in (1.,3.5):
        for gain in (1.,1.5,1.65,1.7):
            for policy in ("baseline","minimum_settled","dual_response"):
                cases.append({"kind":"constant_postcal_gain","gain":gain,"height_m":height,
                              "policy":policy,"seed":1})
        for policy in ("minimum_settled","dual_response"):
            cases.append({"kind":"gain_change_between_levels_S","gain_upper":9.81/UPPER_ACCEL,
                "gain_minimum":9.81/LOWER_ACCEL,"height_m":height,"policy":policy,"seed":1})
    for i,case in enumerate(cases):
        case["gain_change_s"]=(EARLY_GAIN_CHANGE_S if gain_change_stage=="early" else PROBE_START_S+UPPER_RAMP_S)
        case["gain_change_stage"]=gain_change_stage
        law=f"gain{case['gain']:g}" if case["kind"]=="constant_postcal_gain" else "gain-level-cancellation-S"
        case["case_id"]=f"{i+1:03d}-height{case['height_m']:g}-{law}-{case['policy']}"
    return cases


def prepare(out):
    assert hashlib.sha256(FROZEN_OBSERVER.read_bytes()).hexdigest()==OBSERVER_SHA256
    paths=[Path(__file__).resolve(),*sorted((ROOT/"scripts/mission3").glob("*.py")),
           ROOT/"scripts/yaw_core.py",ROOT/"scripts/provided_hardware.py",
           ROOT/"config/mission3.json",ROOT/"config/mission3-physical-sensitivity.json",
           ROOT/"config/mission3-contact-sensitivity.json",ROOT/"config/provided-hardware.json",
           ROOT/"build/mission3-flight-controller/libmission3_controller.so",
           ROOT/"build/mission3-flight-controller/source-manifest.json"]
    hashes={};origins={}
    for path in paths:
        relative=path.relative_to(ROOT)
        origin=FROZEN_OBSERVER if str(relative)=="scripts/mission3/touchdown.py" else path
        target=out/"source"/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(origin,target)
        hashes[str(relative)]=hashlib.sha256(target.read_bytes()).hexdigest()
        origins[str(relative)]=str(origin)
    write_json(out/"source-manifest.json",{"created_kst":datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
        "sha256":hashes,"origins":origins,"python":sys.version,"python_executable":sys.executable,
        "numpy":np.__version__,"mujoco":mujoco.__version__,"physical_runtime":"copied modules only",
        "shared_source_written":False,"frozen_observer_sha256":OBSERVER_SHA256})
    sys.path.insert(0,str(out/"source/scripts"))
    from mission3.physical_scene import PhysicalPlant
    from mission3.touchdown import TouchdownObserver
    from mission3.settings import load
    assert Path(sys.modules[PhysicalPlant.__module__].__file__).resolve().is_relative_to(out/"source")
    assert Path(sys.modules[TouchdownObserver.__module__].__file__).resolve().is_relative_to(out/"source")
    cfg=load();cfg["entry"]["inject_initial_uncertainty"]=False
    profile=json.loads((out/"source/config/mission3-physical-sensitivity.json").read_text())
    return cfg,profile,PhysicalPlant,TouchdownObserver


def observer_settings(profile):
    settings=deepcopy(profile["contact_observer"])
    settings.update(supported_accel_tolerance_m_s2=.2,probe_motor_accel_max_m_s2=6.5,probe_multiplier_min=.60)
    settings.pop("support_body_height_above_pad_m",None)
    settings.pop("calibration_max_age_s",None)
    return settings


def run_case(cfg,profile,out,case,physical_class,observer_class,library,deadline):
    out.mkdir(parents=True,exist_ok=False)
    actual=deepcopy(profile);actual["aircraft_contact_case"]["mass_kg"]=MASS
    actual["motor"]["force_gain"]=1.
    class GainPlant(physical_class):
        def step(self,output,arm,torque):
            if self.time<case["gain_change_s"]: gain=1.
            elif case["kind"]=="constant_postcal_gain": gain=case["gain"]
            else: gain=case["gain_upper"] if self.time<LOWER_START_S else case["gain_minimum"]
            self.actual_gain=gain;self.physical["motor"]["force_gain"]=gain
            super().step(output,arm,torque)
    curve=NominalMotorCurve(library,profile["motor"]["supply_voltage_V"])
    base=observer_class(observer_settings(profile));wrapper=ProbeConfirm(base,case["policy"]);tilt=SensorTilt()
    plant=GainPlant(cfg,actual,out,1,case["seed"],initial_arc_m=30.,
        initial_position=[1.,2.,case["height_m"]],initial_velocity=[0.,0.,9.81*actual["motor"]["response_time_constant_s"]])
    plant.actual_gain=1.
    assert abs(plant.model.body_mass[plant.air]-MASS)<1e-12
    dt=float(plant.model.opt.timestep);assert abs(dt-.001)<1e-12
    write_json(out/"fixture.json",{"case":case,"observer_settings":base.c,"height_gate":False,
        "initialization":"known nonmission position/velocity, no state writes after initialization",
        "command_schedule":{"probe_start_s":PROBE_START_S,"upper_accel_m_s2":UPPER_ACCEL,
            "upper_ramp_s":UPPER_RAMP_S,"upper_hold_s":UPPER_HOLD_S,"lower_start_s":LOWER_START_S,
            "minimum_accel_m_s2":LOWER_ACCEL,"lower_ramp_s":LOWER_RAMP_S,"minimum_hold_s":MINIMUM_HOLD_S},
        "gain_change_s":case["gain_change_s"],"settle_s":SETTLE_S,"prediction_tolerance_m_s2":PREDICTION_TOLERANCE,
        "minimum_multiplier_tolerance":MULTIPLIER_TOLERANCE,
        "dual_min_excitation_m_s2":DUAL_MIN_EXCITATION,"dual_max_imu_difference_m_s2":DUAL_MAX_IMU_DIFFERENCE,
        "mass_kg":MASS,"sensor_hz":100,"physics_predict_hz":1000,"raw_truth_to_observer":False,
        "nominal_profile":profile,"gain_change_only_in_plant_force_law":True})
    output=curve.command(9.81);motor_off=hold_failure=None
    sums=np.zeros(6);count=0;last_contact=None;failure=None;latch=None
    files={name:(out/name).open("w") for name in ("observer.jsonl","commands.jsonl","truth-offline-only.jsonl","imu-raw.csv")}
    csvfile=csv.writer(files["imu-raw.csv"]);csvfile.writerow(("t","gx","gy","gz","ax","ay","az"))
    start=time.monotonic()
    try:
        for tick in range(math.ceil(END_S/dt)+1):
            now=plant.time
            if tick%1000==0 and datetime.now(ZoneInfo("Asia/Seoul"))>=deadline:
                failure="session_deadline";break
            raw=plant.imu();values=np.r_[raw["gyro_body_rad_s"],raw["specific_force_body_m_s2"]]
            csvfile.writerow((now,*values));sums+=values;count+=1
            truth=plant.truth();truth["actual_gain_plant_only"]=plant.actual_gain
            signature=tuple((c["aircraft_part"],c["other"]) for c in truth["contacts"])
            delivery=(tick+1)%10==0
            if delivery or signature!=last_contact: line(files["truth-offline-only.jsonl"],truth)
            last_contact=signature
            if delivery:
                mean=sums/count;sums*=0;count=0
                sample={"stamp_s":now,"gyro_body_rad_s":mean[:3].tolist(),"specific_force_body_m_s2":mean[3:].tolist(),
                        "source":"100Hz mean of 1kHz conditional IMU"}
                roll,pitch=tilt.update(sample)
                packets=None;permitted=False
                if now<CALIBRATION_END_S:
                    tof=plant.range_packet();flow=plant.flow_packet(.01);packets={"range":tof,"flow":flow}
                    permitted=bool(tof["range_valid"] and tof["range_accurate"] and tof["range_m"]>.65
                                   and flow["flow_valid"] and math.hypot(roll,pitch)<math.radians(10))
                base.calibrate_freeflight(sample,permitted)
                if now>=CALIBRATION_END_S and latch is None:
                    latch={"valid":base.calibration_valid,"gain":base.force_to_accel,
                           "last_valid_s":base.calibration_last_valid_s,"rms":base.calibration_rms}
                evidence=base.imu(sample)
                requested,phase=scheduled_command(now)
                enabled=now>=PROBE_START_S and motor_off is None
                candidate,gate=wrapper.candidate(now,sample,output,phase,roll,pitch,enabled)
                if candidate and motor_off is None: motor_off=now
                stable=None
                if motor_off is not None:
                    stable,bad=base.hold_supported(now,roll,pitch)
                    if bad and hold_failure is None: hold_failure=now
                held=output
                output=curve.command(0. if motor_off is not None else requested)
                line(files["observer.jsonl"],{"t":now,"phase":phase,"imu":sample,"sensor_tilt_rad":[roll,pitch],
                    "calibration_permitted":permitted,"calibration_packets":packets,"observer":evidence,
                    "gate":gate,"context_enabled":enabled,"held_command":held,"confirmed_candidate":candidate,
                    "motor_off_requested":motor_off is not None,"hold_supported":stable,"hold_failure_s":hold_failure})
                line(files["commands.jsonl"],{"t":now,**output})
            base.predict_motor(output["motor_static_force_N"],dt)
            plant.step(output,curve.arm,curve.torque)
            if not np.isfinite(plant.data.qpos).all() or not np.isfinite(plant.data.qvel).all():
                failure="nonfinite_physical_state";break
        line(files["truth-offline-only.jsonl"],{**plant.truth(),"actual_gain_plant_only":plant.actual_gain})
    finally:
        for file in files.values(): file.close()
        plant.close()
    return {"latch":latch,"motor_off_s":motor_off,"first_hold_failure_s":hold_failure,
            "duration_s":plant.time,"runtime_s":time.monotonic()-start,"harness_failure":failure}


def evaluate(out,case,details):
    # 비행이 끝난 뒤에만 참값으로 공중/환경 접촉을 분리한다.
    observed=rows(out/"observer.jsonl");truth=rows(out/"truth-offline-only.jsonl")
    first_contact=next((t for t in truth if t["contacts"]),None)
    first_floor=next((t for t in truth if any(c["other"]=="floor" for c in t["contacts"])),None)
    by_time={round(t["t"],6):t for t in truth}
    candidates=[]
    for r in observed:
        if not r["confirmed_candidate"]: continue
        t=by_time[round(r["t"],6)]
        airborne=bool(not t["contacts"] and (first_contact is None or r["t"]<first_contact["t"]-1e-6)
                      and t["aircraft"]["center_world_m"][2]>.05)
        candidates.append({"t":r["t"],"airborne_before_any_contact":airborne,"contacts":t["contacts"],
            "body_height_m":t["aircraft"]["center_world_m"][2],"actual_gain_plant_only":t["actual_gain_plant_only"],
            "observer":r["observer"],"gate":r["gate"],"command_at_candidate":r["held_command"]})
    false_off=bool(candidates and candidates[0]["airborne_before_any_contact"])
    contacted=first_contact is not None
    if details["harness_failure"]: label="HARNESS_FAILURE"
    elif false_off: label="FALSE_AIRBORNE_MOTOR_OFF"
    elif contacted and details["motor_off_s"] is not None: label="ENVIRONMENT_CONTACT_THEN_MOTOR_OFF"
    elif contacted: label="ENVIRONMENT_CONTACT_CONFOUNDS_FIXTURE"
    elif not details["latch"] or not details["latch"]["valid"]: label="CALIBRATION_NOT_VALID"
    else: label="AIRBORNE_NO_FALSE_STOP"
    result={"case":case,"classification":label,"airborne_false_motor_off":false_off,
        "confirmed_candidates":candidates,"first_environment_contact_s":None if first_contact is None else first_contact["t"],
        "first_floor_contact_s":None if first_floor is None else first_floor["t"],
        "negative_control_pass":label=="AIRBORNE_NO_FALSE_STOP","fixture_contact_free":not contacted,
        "offline_safety":{"minimum_body_height_m":min(t["aircraft"]["center_world_m"][2] for t in truth),
            "maximum_body_height_m":max(t["aircraft"]["center_world_m"][2] for t in truth),
            "floor_contact":first_floor is not None,"safety_gate_implemented":False},
        "mission_success":False,"manufacturer_matched":False,"scope":"nonmission probe negative only",**details}
    write_json(out/"evaluation.json",result)
    return result


def audit_case(out,case,observer_class):
    # sensor/command-only 재생: truth/evaluation을 열지 않는다.
    observed=rows(out/"observer.jsonl");commands=rows(out/"commands.jsonl")
    fixture=json.loads((out/"fixture.json").read_text())
    base=observer_class(fixture["observer_settings"]);wrapper=ProbeConfirm(base,case["policy"]);tilt=SensorTilt()
    raw=np.loadtxt(out/"imu-raw.csv",delimiter=",",skiprows=1)
    assert len(observed)==len(commands) and 0<=len(raw)-10*len(observed)<=9
    averages=raw[:10*len(observed),1:].reshape(len(observed),10,6).mean(axis=1)
    delivered=np.array([r["imu"]["gyro_body_rad_s"]+r["imu"]["specific_force_body_m_s2"] for r in observed])
    assert np.max(np.abs(averages-delivered))<1e-10
    held=observed[0]["held_command"];tick=0;off=None;hold_failure=None
    for r,command in zip(observed,commands):
        target=round(r["t"]/.001)
        while tick<target:
            base.predict_motor(held["motor_static_force_N"],.001);tick+=1
        sample=r["imu"];rp=tilt.update(sample)
        assert np.max(np.abs(np.asarray(rp)-r["sensor_tilt_rad"]))<1e-10
        base.calibrate_freeflight(sample,r["calibration_permitted"])
        evidence=base.imu(sample)
        candidate,gate=wrapper.candidate(r["t"],sample,held,r["phase"],*rp,r["context_enabled"])
        assert serial(evidence)==r["observer"]
        assert serial(gate)==r["gate"]
        assert candidate==r["confirmed_candidate"]
        if candidate and off is None: off=r["t"]
        assert (off is not None)==r["motor_off_requested"]
        if off is not None:
            stable,bad=base.hold_supported(r["t"],*rp)
            if bad and hold_failure is None: hold_failure=r["t"]
            assert stable==r["hold_supported"] and hold_failure==r["hold_failure_s"]
            assert command["motor_pwm"]==[0]*4 and command["motor_static_force_N"]==[0.]*4
        held=command;base.predict_motor(held["motor_static_force_N"],.001);tick+=1
    return {"case_id":case["case_id"],"pass":True,"imu_packets":len(observed),
            "off_reproduced_s":off,"truth_files_read":False,"frozen_observer_sha256":OBSERVER_SHA256}


def self_checks(observer_class,settings):
    results=[]
    def check(name,condition):
        assert condition,name
        results.append({"name":name,"pass":True})
    window=StableWindow()
    check("no minimum eligibility even at low prediction",not window.update(0.,False,6.,9.81))
    for i in range(26): ready=window.update(.01*(i+1),True,5.886,9.81)
    check("continuous .25s window eligible",ready)
    check("stale gap resets settle",not window.update(.4,True,5.886,9.81))
    check("duplicate stamp resets settle",not window.update(.4,True,5.886,9.81))
    unstable=StableWindow()
    for i in range(30): ready=unstable.update(i*.01,True,5.8+.1*(i%2),9.81)
    check("prediction spread refuses settle",not ready)
    # 합성 센서 열은 wrapper 시간 계약만 검사하며 physical 증거가 아니다.
    base=observer_class(settings);base.calibration_valid=True;base.force_to_accel=1/MASS
    base.calibration_last_valid_s=0.;base.force[:]=MASS*LOWER_ACCEL/4
    wrapper=ProbeConfirm(base,"minimum_settled")
    command={"motor_static_force_N":[MASS*LOWER_ACCEL/4]*4,"requested_multiplier":.60,
             "nominal_curve_accel_m_s2":LOWER_ACCEL}
    found=None
    for i in range(70):
        t=i*.01;sample={"stamp_s":t,"gyro_body_rad_s":[0,0,0],"specific_force_body_m_s2":[0,0,9.81]}
        base.imu(sample);candidate,gate=wrapper.candidate(t,sample,command,"MINIMUM_HOLD",0.,0.,True)
        if candidate and found is None: found=t
    check("settle .25 precedes support .25",found is not None and found>=.5)
    check("confirmed support resets when command leaves minimum",
          not wrapper.candidate(.71,sample,{**command,"requested_multiplier":.7},"LOWER_RAMP",0.,0.,True)[0])
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path)
    parser.add_argument("--gain-change-stage",choices=("early","upper"),default="upper",
                        help="early=2.6s; upper=3.15s, 진실 높이와 무관한 사전 일정")
    args=parser.parse_args()
    now=datetime.now(ZoneInfo("Asia/Seoul"))
    out=(args.out or ARTIFACT_ROOT/now.strftime("%Y%m%d-%H%M%S-%f")).resolve()
    if out.exists() and any(out.iterdir()): raise RuntimeError("기존 결과 보존: 새 output directory 필요")
    out.mkdir(parents=True,exist_ok=True)
    cfg,profile,physical_class,observer_class=prepare(out)
    deadline=datetime.fromisoformat(cfg["deadline_kst"])
    if now>=deadline: raise RuntimeError("session deadline 이후 새 fixture 금지")
    cases=plan(args.gain_change_stage);write_json(out/"experiment.json",{"preregistered_cases":cases,"fixed_seed":1,
        "sensor_only_wrapper":True,"height_gate":False,"mass_kg":MASS,"planned_cases":len(cases),
        "mission_environment":cfg,"nominal_profile":profile,"observer_settings":observer_settings(profile),
        "truth_inputs_to_candidate":False,"private_frozen_runtime":True})
    checks=self_checks(observer_class,observer_settings(profile));write_json(out/"self-checks.json",checks)
    results=[];audits=[];errors=[]
    for case in cases:
        if datetime.now(ZoneInfo("Asia/Seoul"))>=deadline: break
        directory=out/case["case_id"]
        try:
            details=run_case(cfg,profile,directory,case,physical_class,observer_class,
                out/"source/build/mission3-flight-controller/libmission3_controller.so",deadline)
            result=evaluate(directory,case,details);results.append(result)
            try: audits.append(audit_case(directory,case,observer_class))
            except Exception as exc:
                audits.append({"case_id":case["case_id"],"pass":False,"error":f"{type(exc).__name__}: {exc}"})
            print(case["case_id"],result["classification"],"off",details["motor_off_s"],
                  "floor",result["first_floor_contact_s"],flush=True)
        except Exception as exc:
            error={"case_id":case["case_id"],"error":f"{type(exc).__name__}: {exc}"}
            errors.append(error);directory.mkdir(parents=True,exist_ok=True);write_json(directory/"error.json",error)
            print(error,flush=True)
        write_json(out/"matrix.json",{"planned":len(cases),"completed":len(results),"results":results,"errors":errors})
        write_json(out/"sensor-replay-audit.json",audits)
    summary={"planned":len(cases),"completed":len(results),"errors":errors,
        "false_airborne_motor_off":sum(r["airborne_false_motor_off"] for r in results),
        "floor_cases":sum(r["first_floor_contact_s"] is not None for r in results),
        "contact_free_negative_pass":sum(r["negative_control_pass"] for r in results),
        "truth_free_replay_passed":sum(a["pass"] for a in audits),"self_checks_passed":len(checks),
        "benchmark_ran_to_completion":len(results)==len(cases) and not errors,
        "mission_success_count":0,"manufacturer_validated_success_count":0}
    write_json(out/"summary.json",summary);print(json.dumps(summary,ensure_ascii=False),flush=True);print(out,flush=True)
    return 2 if summary["false_airborne_motor_off"] or summary["floor_cases"] or errors or len(results)!=len(cases) or any(not a["pass"] for a in audits) else 0


if __name__=="__main__": raise SystemExit(main())
