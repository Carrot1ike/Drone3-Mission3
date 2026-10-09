#!/usr/bin/env python3
"""보정 후 force-model 변화: 공중 접촉 후보/모터 off 음성 fixture.

비미션 명령, 조건부 35g 강체/모터/센서 S. 접촉 참값은 기록과 오프라인
판정만 수행한다. observer에는 평균 IMU, 명령 nominal curve, 센서 자세만 준다.
"""
import argparse
from copy import deepcopy
from datetime import datetime
import csv
import ctypes as ct
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import sys
import time
from zoneinfo import ZoneInfo

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from mission3.settings import ROOT, load
from mission3.physical_scene import PhysicalPlant
from mission3.touchdown import TouchdownObserver


ARTIFACT_ROOT = ROOT / "artifacts/mission3/calibration-change-negative"
LIBRARY = ROOT / "build/mission3-flight-controller/libmission3_controller.so"
MASS = .035
CALIBRATION_END_S = 2.5
PROBE_RAMP_S = .25
PROBE_HOLD_S = 1.0
PROBE_ACCEL_M_S2 = 7.0


def serial(value):
    if isinstance(value, dict): return {k: serial(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, np.ndarray)): return [serial(v) for v in value]
    if isinstance(value, (float, np.floating)): return float(value) if math.isfinite(value) else None
    if isinstance(value, np.generic): return value.item()
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(serial(value), ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def line(file, value):
    file.write(json.dumps(serial(value), ensure_ascii=False, allow_nan=False)+"\n")


class NominalMotorCurve:
    """이미 빌드된 공식 software curve만 읽는다. PID/빌드/라디오 실행 없음."""
    def __init__(self, voltage):
        self.voltage = voltage
        self.lib = ct.CDLL(str(LIBRARY))
        self.lib.flight_static_force.argtypes = [ct.c_uint, ct.c_float]
        self.lib.flight_static_force.restype = ct.c_float
        for name in ("flight_arm", "flight_thrust_to_torque"):
            getattr(self.lib, name).argtypes = []
            getattr(self.lib, name).restype = ct.c_float
        self.arm = self.lib.flight_arm()
        self.thrust_to_torque = self.lib.flight_thrust_to_torque()

    def force(self, pwm):
        return float(self.lib.flight_static_force(pwm, self.voltage))

    def command(self, nominal_accel):
        target = MASS*nominal_accel/4
        lo, hi = 0, 65535
        while lo < hi:
            mid = (lo+hi)//2
            if self.force(mid) < target: lo = mid+1
            else: hi = mid
        pwm = min((lo, max(0, lo-1)), key=lambda p: abs(self.force(p)-target))
        force = self.force(pwm)
        return {"motor_pwm": [pwm]*4, "motor_static_force_N": [force]*4,
                "nominal_requested_acceleration_m_s2": nominal_accel,
                "nominal_curve_acceleration_m_s2": force*4/MASS,
                "supply_voltage_V": self.voltage,
                "command_source": "prescribed symmetric offline fixture; no mission/PID/truth feedback"}


class SensorTilt:
    """수평 fixture의 센서 gyro/accelerometer complementary 자세. 참 RPY 없음."""
    def __init__(self): self.rp = np.zeros(2)

    def update(self, sample, dt):
        gyro = np.asarray(sample["gyro_body_rad_s"])
        accel = np.asarray(sample["specific_force_body_m_s2"])
        self.rp += gyro[:2]*dt
        if accel[2] > 3. and np.linalg.norm(accel) > 3.:
            measured = np.array([math.atan2(accel[1], accel[2]),
                                 math.atan2(-accel[0], math.hypot(accel[1], accel[2]))])
            self.rp += .05*(measured-self.rp)
        return tuple(self.rp)


class ChangePlant(PhysicalPlant):
    """가상 힘 law만 바꾸는 로컬 subclass. 공유 PhysicalPlant는 읽기 전용.

    ground-effect S의 실제 높이는 plant 내부 힘 생성에만 쓰인다. observer,
    fixture command, 보정 허가에는 이 높이/접촉/실제 힘을 전달하지 않는다.
    """
    def __init__(self, cfg, profile, out, case):
        self.case = case
        self.change_s = CALIBRATION_END_S+case["latch_age_s"]
        self.probe_s = self.change_s+( .3 if case["kind"] == "motor_step" else 0.)
        self.actual_gain = 1.
        super().__init__(cfg, profile, out, 1, case["seed"], initial_arc_m=30.,
                         initial_position=[1., 2., case["initial_height_m"]],
                         initial_velocity=[0., 0., 9.81*profile["motor"]["response_time_constant_s"]])

    def step(self, output, arm, torque):
        if self.time < self.change_s:
            gain = 1.
        elif self.case["kind"] == "motor_step":
            gain = self.case["new_motor_gain"]
        else:
            # 가정 S: 정상 ground-effect 모델이 아니라 의도적인 민감도 law.
            activation = min(1., max(0., (self.time-self.change_s)/.05))
            height = max(0., float(self.data.xpos[self.air, 2]))
            gain = 1.+activation*(self.case["ground_peak_gain"]-1.)*math.exp(-height/self.case["ground_length_m"])
        self.actual_gain = gain
        self.physical["motor"]["force_gain"] = gain
        super().step(output, arm, torque)


def plan(preset, seeds, overlay="none"):
    cases = []
    if overlay == "paired-tight":
        # 두 operating point를 사전 등록한다. gate plateau는 허용 probe 경로의
        # 반례, minimum plateau는 요청한 .60 하한의 별도 음성 대조다.
        for seed in seeds:
            for probe_mode in ("gate", "minimum"):
                for gain in (.8, 1., 1.1, 1.2, 1.3, 1.5, 1.8):
                    for name in ("original", "tight"):
                        cases.append({"kind": "motor_step", "seed": seed, "latch_age_s": .1,
                            "new_motor_gain": gain, "initial_height_m": 3.5,
                            "observer_overlay": name, "probe_mode": probe_mode})
        for i, case in enumerate(cases):
            case["case_id"] = (f"{i+1:03d}-paired-seed{case['seed']}-gain{case['new_motor_gain']:.1f}"
                               f"-{case['probe_mode']}-{case['observer_overlay']}")
        return cases
    ages = (.1,) if preset == "smoke" else (.1, 3., 30.)
    for seed in seeds:
        for age in ages:
            for gain in (.8, 1., 1.1, 1.2, 1.3):
                cases.append({"kind": "motor_step", "seed": seed, "latch_age_s": age,
                              "new_motor_gain": gain, "initial_height_m": 3.5})
        if preset == "full":
            for peak, length in ((1., 1.), (1.3, .5), (1.3, 1.), (1.5, .5), (1.5, 1.), (1.8, .5), (1.8, 1.)):
                cases.append({"kind": "ground_effect_S", "seed": seed, "latch_age_s": .1,
                              "ground_peak_gain": peak, "ground_length_m": length, "initial_height_m": 1.5})
    for i, case in enumerate(cases):
        case["observer_overlay"] = "tight" if overlay == "tight" else "original"
        case["probe_mode"] = "gate"
        case["case_id"] = f"{i+1:03d}-{case['kind']}-seed{case['seed']}-"+(f"gain{case['new_motor_gain']:.1f}-age{case['latch_age_s']:g}"
            if case["kind"] == "motor_step" else f"peak{case['ground_peak_gain']:.1f}-length{case['ground_length_m']:g}")
    return cases


def evaluate(out, case, details, failure):
    # 물리 loop가 끝난 뒤 observer와 독립으로 참값을 판정한다.
    records = [json.loads(s) for s in (out/"observer.jsonl").read_text().splitlines()]
    truths = [json.loads(s) for s in (out/"truth-offline-only.jsonl").read_text().splitlines()]
    first_contact = next((t for t in truths if t["contacts"]), None)
    floor_contacts = [t for t in truths if any(c["other"] == "floor" for c in t["contacts"])]
    confirmed = [r for r in records if r["confirmed_candidate"]]
    off = next((r for r in records if r["motor_off_requested"]), None)
    by_time = {round(t["t"], 6): t for t in truths}
    candidates = []
    for r in confirmed:
        t = by_time[round(r["t"], 6)]
        before_any_contact = first_contact is None or r["t"] < first_contact["t"]-1e-6
        clean_air = not t["contacts"] and before_any_contact and t["aircraft"]["center_world_m"][2] > .05
        candidates.append({"t": r["t"], "airborne_before_any_contact": clean_air,
            "center_height_m": t["aircraft"]["center_world_m"][2], "true_contacts": t["contacts"],
            "observer_evidence": r["observer"], "calibration_age_s": r["calibration_age_s"],
            "actual_force_gain_plant_only": t["applied_force_gain_S"]})
    false_off = bool(off and any(c["airborne_before_any_contact"] and abs(c["t"]-off["t"])<1e-6 for c in candidates))
    if false_off: classification = "FALSE_AIRBORNE_MOTOR_OFF"
    elif first_contact: classification = "ENVIRONMENT_CONTACT_CONFOUNDS_FIXTURE"
    elif not details["calibration_valid_at_latch"]: classification = "CALIBRATION_NOT_VALID"
    elif failure: classification = "HARNESS_FAILURE"
    else: classification = "AIRBORNE_NO_FALSE_STOP"
    result = {"case": case, "classification": classification, "harness_failure": failure,
        "airborne_false_motor_off": false_off, "confirmed_candidates": candidates,
        "motor_off_requested_at_s": None if off is None else off["t"],
        "first_environment_contact_s": None if first_contact is None else first_contact["t"],
        "first_floor_contact_s": None if not floor_contacts else floor_contacts[0]["t"],
        "floor_contact_samples": len(floor_contacts), "fixture_contact_free": first_contact is None,
        "min_height_m": min(t["aircraft"]["center_world_m"][2] for t in truths),
        "max_height_m": max(t["aircraft"]["center_world_m"][2] for t in truths),
        "negative_control_pass": classification == "AIRBORNE_NO_FALSE_STOP",
        "mission_success": False, "physical_success_count": 0, "manufacturer_matched": False,
        "scope": "conditional nonmission airborne negative fixture; raw truth is offline evidence only", **details}
    write_json(out/"evaluation.json", result)
    return result


def run_case(cfg, profile, out, case, deadline):
    out.mkdir(parents=True, exist_ok=False)
    actual = deepcopy(profile)
    actual["aircraft_contact_case"]["mass_kg"] = MASS
    actual["motor"]["force_gain"] = 1.
    observer_settings = deepcopy(profile["contact_observer"])
    if case.get("observer_overlay") == "tight":
        observer_settings.update(supported_accel_tolerance_m_s2=.2,
                                 probe_motor_accel_max_m_s2=6.5, probe_multiplier_min=.60)
    minimum_accel = 9.81*observer_settings["probe_multiplier_min"]
    probe_accel = (minimum_accel if case.get("probe_mode") == "minimum" else
                   min(PROBE_ACCEL_M_S2, observer_settings["probe_motor_accel_max_m_s2"]))
    assert probe_accel >= minimum_accel
    curve = NominalMotorCurve(profile["motor"]["supply_voltage_V"])
    observer = TouchdownObserver(observer_settings)
    tilt = SensorTilt()
    write_json(out/"fixture.json", {"case": case, "observer_parameters": observer.c,
        "nominal_profile": profile, "calibration_end_s": CALIBRATION_END_S,
        "probe_acceleration_m_s2": probe_accel, "probe_ramp_s": PROBE_RAMP_S,
        "probe_command_multiplier": probe_accel/9.81, "probe_multiplier_floor": observer_settings["probe_multiplier_min"],
        "probe_operating_point": case.get("probe_mode", "gate"),
        "probe_hold_s": PROBE_HOLD_S, "physics_hz": 1000, "imu_delivery_hz": 100,
        "calibration_gate": "scheduled nominal stage + sensor floor range/Flow + IMU tilt; no contact truth",
        "context_gate": "prescribed probe phase enabled; not Mission3 relative-position/height gating",
        "ground_effect_law_S": "1+(peak-1)*activation*exp(-plant_floor_height/length); plant force only; not validated aerodynamics",
        "initial_velocity_source": "known g*nominal_motor_tau spinup impulse compensation; initialization only"})
    plant = ChangePlant(cfg, actual, out, case)
    assert abs(float(plant.model.body_mass[plant.air])-MASS) < 1e-12
    dt = float(plant.model.opt.timestep)
    assert abs(dt-.001) < 1e-12
    output = curve.command(9.81)
    motor_off = None;hold_failure = None;last_cal = None;latch = None;failure = None
    sum_acc = np.zeros(3);sum_gyro = np.zeros(3);count = 0
    flow = tof = None;last_contact_signature = None
    files = {n:(out/n).open("w") for n in ("observer.jsonl", "commands.jsonl", "truth-offline-only.jsonl", "imu-raw.csv")}
    csvfile = csv.writer(files["imu-raw.csv"])
    csvfile.writerow(("t", "gyro_x", "gyro_y", "gyro_z", "specific_accel_x", "specific_accel_y", "specific_accel_z"))
    final = plant.probe_s+PROBE_RAMP_S+PROBE_HOLD_S
    start_wall = time.monotonic()
    try:
        for tick in range(int(math.ceil(final/dt))+1):
            now = plant.time
            if tick%1000 == 0 and datetime.now(ZoneInfo("Asia/Seoul")) >= deadline:
                failure = "session_deadline";break
            raw = plant.imu()
            csvfile.writerow((now, *raw["gyro_body_rad_s"], *raw["specific_force_body_m_s2"]))
            sum_acc += raw["specific_force_body_m_s2"];sum_gyro += raw["gyro_body_rad_s"];count += 1
            # 기록용 truth는 아래 제어/observer 분기에 전달하지 않는다.
            truth = plant.truth()
            truth["applied_force_gain_S"] = plant.actual_gain
            signature = tuple((c["aircraft_part"], c["other"]) for c in truth["contacts"])
            delivery = (tick+1)%10 == 0
            if delivery or signature != last_contact_signature:
                line(files["truth-offline-only.jsonl"], truth)
            last_contact_signature = signature
            if delivery:
                sample = {"stamp_s": now, "specific_force_body_m_s2": (sum_acc/count).tolist(),
                          "gyro_body_rad_s": (sum_gyro/count).tolist(), "source": "100Hz averaged conditional body IMU"}
                sum_acc *= 0;sum_gyro *= 0;count = 0
                roll, pitch = tilt.update(sample, .01)
                before_latch = now < CALIBRATION_END_S
                if before_latch:
                    tof = plant.range_packet();flow = plant.flow_packet(.01)
                permitted = bool(before_latch and tof and tof["range_valid"] and tof["range_accurate"]
                    and tof["range_m"] > .65 and flow["flow_valid"] and math.hypot(roll,pitch)<math.radians(10))
                observer.calibrate_freeflight(sample, permitted)
                if permitted and observer.calibration_valid: last_cal = now
                if not before_latch and latch is None:
                    latch = {"t": now, "valid": observer.calibration_valid, "gain_per_kg": observer.force_to_accel,
                             "rms_m_s2": observer.calibration_rms, "last_permitted_valid_sample_s": last_cal}
                evidence = observer.imu(sample)
                enabled = now >= plant.probe_s and motor_off is None
                candidate = observer.contact_candidate(now, roll, pitch, enabled)
                if candidate and motor_off is None: motor_off = now
                if motor_off is not None:
                    stable, bad = observer.hold_supported(now, roll, pitch)
                    if bad and hold_failure is None: hold_failure = now
                else: stable = None
                fraction = float(np.clip((now-plant.probe_s)/PROBE_RAMP_S, 0., 1.))
                requested = 0. if motor_off is not None else 9.81+(probe_accel-9.81)*fraction
                output = curve.command(requested)
                line(files["observer.jsonl"], {"t": now, "phase": "OFF" if motor_off is not None else "PROBE" if enabled else "CALIBRATE" if before_latch else "LATCHED_HOVER",
                    "delivered_imu": sample, "sensor_roll_pitch_rad": [roll,pitch], "calibration_permitted": permitted,
                    "calibration_sensor_packets": {"range": tof, "flow": flow} if before_latch else None,
                    "calibration_age_s": None if last_cal is None else now-last_cal, "observer": evidence,
                    "context_enabled": enabled, "confirmed_candidate": candidate, "motor_off_requested": motor_off is not None,
                    "hold_supported": stable, "hold_failed_at_s": hold_failure})
                line(files["commands.jsonl"], {"t": now, **output})
            observer.predict_motor(output["motor_static_force_N"], dt)
            plant.step(output, curve.arm, curve.thrust_to_torque)
            if not np.isfinite(plant.data.qpos).all() or not np.isfinite(plant.data.qvel).all():
                failure = "nonfinite_physical_state";break
        line(files["truth-offline-only.jsonl"], {**plant.truth(), "applied_force_gain_S": plant.actual_gain})
        details = {"latch": latch, "calibration_valid_at_latch": bool(latch and latch["valid"]),
            "final_calibration_valid": observer.calibration_valid, "final_force_to_accel_per_kg": observer.force_to_accel,
            "first_hold_failure_s": hold_failure, "duration_s": plant.time, "runtime_s": time.monotonic()-start_wall,
            "actual_mass_kg": MASS, "observer_assumed_mass_kg": observer.c["assumed_mass_kg"],
            "observer_parameters": observer.c, "probe_acceleration_m_s2": probe_accel,
            "probe_multiplier_floor": observer_settings["probe_multiplier_min"]}
    finally:
        for file in files.values(): file.close()
        plant.close()
    return evaluate(out, case, details, failure)


def freeze_inputs(out, cfg, profile, cases):
    paths = [Path(__file__).resolve(), ROOT/"scripts/benchmark_mission3_contact_observer.py",
             *sorted((ROOT/"scripts/mission3").glob("*.py")), ROOT/"scripts/yaw_core.py",
             ROOT/"scripts/provided_hardware.py", ROOT/"config/mission3.json",
             ROOT/"config/mission3-physical-sensitivity.json", ROOT/"config/mission3-contact-sensitivity.json",
             ROOT/"config/provided-hardware.json", LIBRARY,
             ROOT/"build/mission3-flight-controller/source-manifest.json"]
    hashes = {}
    for path in paths:
        target = out/"source"/path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True);shutil.copy2(path, target)
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(target.read_bytes()).hexdigest()
    manifest = {"created_kst": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
        "code_and_binary_sha256": hashes, "python": sys.version, "python_executable": sys.executable,
        "mujoco": mujoco.__version__, "numpy": np.__version__, "platform": platform.platform(),
        "previous_benchmark_actual_filename": "scripts/benchmark_mission3_contact_observer.py",
        "source_class": "conditional virtual S; no manufacturer model/contact/mission promotion"}
    write_json(out/"source-manifest.json", manifest)
    write_json(out/"experiment.json", {"mission_environment": cfg, "nominal_profile": profile, "preregistered_cases": cases,
        "mass_kg": MASS, "calibration_end_s": CALIBRATION_END_S, "probe_accel_m_s2": PROBE_ACCEL_M_S2,
        "probe_ramp_s": PROBE_RAMP_S, "probe_hold_s": PROBE_HOLD_S,
        "observer_inputs": ["nominal command curve + assumed lag", "100Hz averaged sensor IMU", "sensor-derived tilt", "prescribed context enable"],
        "truth_inputs_to_observer": False, "nonmission_fixture_commands": True})
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--preset", choices=("smoke", "full"), default="full")
    parser.add_argument("--overlay", choices=("none", "tight", "paired-tight"), default="none",
                        help="공유 설정 수정 없이 로컬 observer만 덮어쓰기; paired-tight는 7 gains × 2 overlays × 2 operating points/seed")
    parser.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    args = parser.parse_args()
    cfg = load();profile = json.loads((ROOT/"config/mission3-physical-sensitivity.json").read_text())
    assert profile["contact_observer"]["assumed_mass_kg"] == MASS
    assert profile["contact_observer"]["probe_motor_accel_max_m_s2"] == 7.3
    deadline = datetime.fromisoformat(cfg["deadline_kst"])
    now = datetime.now(ZoneInfo("Asia/Seoul"))
    if now >= deadline: raise RuntimeError("session deadline elapsed; no new cases")
    out = args.out or ARTIFACT_ROOT/now.strftime("%Y%m%d-%H%M%S-%f")
    if out.exists() and any(out.iterdir()): raise RuntimeError("preserve prior output; use a fresh directory")
    out.mkdir(parents=True, exist_ok=True)
    cases = plan(args.preset, args.seeds, args.overlay)
    hashes = freeze_inputs(out, cfg, profile, cases)
    results = [];errors = []
    for case in cases:
        if datetime.now(ZoneInfo("Asia/Seoul")) >= deadline: break
        changed = [rel for rel, digest in hashes.items() if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=digest]
        if changed:
            errors.append({"case": case, "error": "frozen source changed", "paths": changed});break
        try:
            result = run_case(cfg, profile, out/case["case_id"], case, deadline)
            results.append(result)
            print(case["case_id"], result["classification"], "off", result["motor_off_requested_at_s"],
                  "floor", result["first_floor_contact_s"], flush=True)
        except Exception as exc:
            error = {"case": case, "error": f"{type(exc).__name__}: {exc}"}
            errors.append(error);write_json(out/case["case_id"]/"error.json", error)
            print(case["case_id"], error["error"], flush=True)
        write_json(out/"matrix.json", {"planned": len(cases), "completed": len(results), "results": results, "errors": errors})
    summary = {"planned": len(cases), "completed": len(results), "errors": errors,
        "false_airborne_motor_off": sum(r["airborne_false_motor_off"] for r in results),
        "fixture_contact_free": sum(r["fixture_contact_free"] for r in results),
        "floor_contact_cases": sum(r["first_floor_contact_s"] is not None for r in results),
        "negative_controls_passed": sum(r["negative_control_pass"] for r in results),
        "mission_success_count": 0, "manufacturer_validated_success_count": 0,
        "benchmark_ran_to_completion": len(results)==len(cases) and not errors,
        "changed_sources_after_run": [rel for rel, digest in hashes.items() if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=digest]}
    write_json(out/"summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False), flush=True);print(out, flush=True)
    return 2 if summary["false_airborne_motor_off"] or summary["floor_contact_cases"] or errors or len(results)!=len(cases) else 0


if __name__ == "__main__": raise SystemExit(main())
