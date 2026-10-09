#!/usr/bin/env python3
"""After-run evaluation: simulator truth is read here, never in control node."""
import csv
import json
import math
from pathlib import Path
import numpy as np
from yaw_core import quaternion_rpy, wrap


def evaluate(out):
    out = Path(out)
    summary_path = out / "control-summary.json"
    if not summary_path.exists():
        result = {"passed": False, "failure": "control_summary_missing", "artifact_dir": str(out)}
        (out / "evaluation.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    summary = json.loads(summary_path.read_text())
    config = json.loads((out / "experiment.json").read_text())
    truth = list(csv.DictReader((out / "truth.csv").open()))
    control = list(csv.DictReader((out / "control.csv").open()))
    obs = [json.loads(line) for line in (out / "observations.jsonl").read_text().splitlines()]
    times = np.array([float(t["monotonic_s"]) for t in truth])
    angles = np.unwrap([quaternion_rpy([float(t[k]) for k in ("qx", "qy", "qz", "qw")])[2] for t in truth])
    poses = np.array([[float(t[k]) for k in ("x_m", "y_m", "z_m")] for t in truth])
    phases = [(e["phase"], e["monotonic_s"]) for e in summary["events"] if e["event"] == "phase"]
    phase_times = dict(phases)
    armed = phase_times.get("ARM", times[0])
    initial = (times >= armed-.5) & (times <= armed)
    reference = float(np.median(angles[initial])) if np.any(initial) else float(angles[0])
    xy_reference = np.median(poses[initial, :2], axis=0) if np.any(initial) else poses[0, :2]
    intervals = {}
    for name in ("RECOVER_POS", "RECOVER_NEG", "HOLD"):
        if name in phase_times:
            index = [x[0] for x in phases].index(name)
            end = phases[index+1][1] if index+1 < len(phases) else times[-1]
            selected = (times >= max(phase_times[name], end-1.5)) & (times < end)
            if np.any(selected):
                error = np.rad2deg([wrap(a-reference) for a in angles[selected]])
                intervals[name] = {"final_abs_error_deg": float(abs(np.mean(error))), "final_rmse_deg": float(np.sqrt(np.mean(error**2)))}
    responses = []
    for name, direction in (("SIGN_POS", 1), ("SIGN_NEG", -1)):
        if name in phase_times:
            start = phase_times[name]
            delta = float(np.rad2deg(np.interp(start+config["control"]["pulse_s"], times, angles)-np.interp(start, times, angles)))
            sign = summary.get("ros_to_physical_yaw_sign")
            responses.append({"phase": name, "physical_delta_deg": delta,
                              "passed": sign is not None and delta*direction*sign > 3.})
    flown = (times >= phase_times.get("ACQUIRE", times[-1])) & (times <= phase_times.get("QUIET", times[-1]))
    peak_xy = float(np.max(np.linalg.norm(poses[flown, :2]-xy_reference, axis=1))) if np.any(flown) else None
    active_obs = [r for r in obs if r["phase"] in ("ACQUIRE", "SIGN_POS", "SIGN_NEG", "RECOVER_POS", "RECOVER_NEG", "HOLD")]
    accepted = sum(bool(r.get("fused")) for r in active_obs)
    valid_fraction = accepted / len(active_obs) if active_obs else 0.
    aligned = []
    for row in control:
        if row["yaw_est_deg"] and row["phase"] in ("ACQUIRE", "SIGN_POS", "SIGN_NEG", "RECOVER_POS", "RECOVER_NEG", "HOLD"):
            stamp = float(row["monotonic_s"])
            true_angle = float(np.interp(stamp, times, angles))
            aligned.append({"monotonic_s": stamp, "phase": row["phase"], "truth_yaw_deg": math.degrees(wrap(true_angle)),
                            "yaw_est_deg": float(row["yaw_est_deg"]), "imu_only_deg": float(row["yaw_imu_only_deg"]),
                            "ros_rate_deg_s": math.degrees(float(row["ros_rate_rad_s"]))})
    errors = [math.degrees(wrap(math.radians(r["yaw_est_deg"]-r["truth_yaw_deg"]))) for r in aligned]
    imu_only_errors = [math.degrees(wrap(math.radians(r["imu_only_deg"]-r["truth_yaw_deg"]))) for r in aligned]
    held = [r for r in aligned if r["phase"] == "HOLD"]
    final_held = held[-min(75, len(held)):]
    final_est_errors = [math.degrees(wrap(math.radians(r["yaw_est_deg"]-r["truth_yaw_deg"]))) for r in final_held]
    final_imu_errors = [math.degrees(wrap(math.radians(r["imu_only_deg"]-r["truth_yaw_deg"]))) for r in final_held]
    final_ground = float(np.median(poses[-10:, 2]))
    fault_expected = config["test"]["drop_images_during_hold_s"] > config["control"]["loss_abort_s"]
    criteria = {
        "expected_finish_state": summary["failure"] == "visual_loss" if fault_expected else bool(summary["completed"]),
        "both_rotation_signs": len(responses) == 2 and all(r["passed"] for r in responses),
        "heading_recovered": len(intervals) == 3 and all(r["final_abs_error_deg"] <= config["evaluation"]["max_final_yaw_error_deg"] for r in intervals.values()),
        "horizontal_drift_bounded": peak_xy is not None and peak_xy <= config["evaluation"]["max_peak_xy_displacement_m"],
        "visual_observations": valid_fraction >= config["evaluation"]["min_visual_valid_fraction"],
        "returned_to_ground": final_ground < .12 and ("LANDED_CHECK" in phase_times or "DISARMED_WAIT" in phase_times),
        "no_truth_input": not summary["truth_used_by_controller"]
    }
    if control and "last_publish_s" in control[0]:
        pub_times = sorted(set(float(c["last_publish_s"]) for c in control if c["last_publish_s"]))
        criteria["no_hover_after_land_plan"] = bool(pub_times) and pub_times[-1] < phase_times.get("LAND_PLAN", float("-inf"))
        stale = [c for c in control if c["hover_owned"] == "True" and float(c["visual_age_s"]) > config["observer"]["max_visual_age_s"]]
        criteria["stale_visual_commands_zero"] = all(abs(float(c["ros_rate_rad_s"])) < 1e-9 for c in stale)
        if (out / "motors.csv").exists():
            motors = list(csv.DictReader((out / "motors.csv").open()))
            last_motor_values = [float(motors[-1][k]) for k in ("m1_pwm", "m2_pwm", "m3_pwm", "m4_pwm")] if motors else []
            criteria["motor_pwm_zero_on_finish"] = len(last_motor_values) == 4 and max(last_motor_values) == 0.
    else:
        pub_times = []
    report = {"passed": all(criteria.values()), "criteria": criteria, "control_failure": summary["failure"],
              "test_type": "visual_dropout_recovery" if fault_expected else "static_yaw_correction",
              "stop_condition": summary.get("stop_condition", "not individually confirmed in earlier run"),
              "experimentally_injected_gyro_bias_deg_s": config["test"]["observer_gyro_bias_deg_s"],
              "initial_truth_yaw_deg": math.degrees(reference), "target_visual_yaw_deg": summary["target_yaw_deg"],
              "rotation_responses": responses, "recovery": intervals, "peak_horizontal_displacement_m": peak_xy,
              "valid_visual_fraction": valid_fraction, "observer_rmse_deg": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
              "imu_only_observer_rmse_deg": float(np.sqrt(np.mean(np.square(imu_only_errors)))) if imu_only_errors else None,
              "final_hold_fused_estimate_error_deg": float(np.mean(final_est_errors)) if final_est_errors else None,
              "final_hold_imu_only_estimate_error_deg": float(np.mean(final_imu_errors)) if final_imu_errors else None,
              "observer_comparison": "same flight data, prediction-only vs visual fusion; not two separate flight controllers",
              "final_body_height_m": final_ground, "truth_samples": len(truth), "imu_samples": summary["imu_samples"],
              "image_frames": len(obs), "artifact_dir": str(out),
              "limits": ["static marker only", "simulator camera calibration only", "image capture delay approximated",
                         "firmware yaw estimate not corrected", "not moving-platform landing or equipment acceptance"]}
    active_times = times[flown]
    sim_times = np.array([float(t["sim_s"]) for t in truth])[flown]
    report["mean_sim_realtime_factor"] = float((sim_times[-1]-sim_times[0])/(active_times[-1]-active_times[0])) if len(active_times) > 1 else None
    report["max_hover_command_gap_s"] = float(np.max(np.diff(pub_times))) if len(pub_times) > 1 else None
    (out / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "aligned.json").write_text(json.dumps(aligned) + "\n")
    return report
