#!/usr/bin/env python3
"""대기 시선 고정·독립 차량 통과·후방 정렬 조건을 종료 후 참값으로 확인."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from provided_hardware import ROOT


def evaluate(out):
    out = out.resolve()
    config = json.loads((out / "experiment.json").read_text())
    result = json.loads((out / "evaluation.json").read_text())
    truth = list(csv.DictReader((out / "truth.csv").open()))
    commands = [json.loads(s) for s in (out / "commands.jsonl").read_text().splitlines()]
    observations = [json.loads(s) for s in (out / "observations.jsonl").read_text().splitlines()]
    if "monotonic_s" in truth[0]:
        origin = np.median([r["monotonic_s"]-r["time_s"] for r in commands])
        times = np.array([float(r["monotonic_s"])-origin for r in truth])
    else:
        times = np.array([float(r["time_s"]) for r in truth])
    first = truth[0]
    phases = []
    for r in commands:
        if not phases or phases[-1] != r["phase"]:
            phases.append(r["phase"])
    wait = [r for r in commands if r["phase"] == "WAIT"]
    accepted = [r for r in observations if r["valid"]]
    end_wait = next((r["time_s"] for r in commands if r["phase"] != "WAIT"), commands[-1]["time_s"])
    before = [r for r, t in zip(truth, times) if 0. <= t < end_wait]
    keys = ("camera_x_m", "camera_y_m", "camera_z_m")
    translation = max(math.dist([float(r[k]) for k in keys], [float(first[k]) for k in keys]) for r in before)
    turn = max(abs((float(r["heading_deg"])-float(first["heading_deg"])+180.)%360.-180.) for r in before)
    rear = [float(first["pad_x_m"])-config["vehicle"]["pad_length_m"]/2, float(first["pad_y_m"])]
    bearing = math.atan2(rear[1]-float(first["camera_y_m"]), rear[0]-float(first["camera_x_m"]))
    angle = abs(math.degrees((math.radians(float(first["heading_deg"]))-bearing+math.pi)%(2*math.pi)-math.pi))
    travel = np.array([float(r["vehicle_travel_m"]) for r in truth])
    moving = np.array([str(r["vehicle_moving"]).lower() == "true" for r in truth])
    active = (times[1:] >= 0.) & moving[1:] & moving[:-1]
    dt, dx = np.diff(times)[active], np.diff(travel)[active]
    speed_error = float(np.max(np.abs(dx-config["vehicle"]["speed_m_s"]*dt)))
    checks = {
        "recorded_optical_test_passed": bool(result["test_passed"]),
        "requested_initial_smaller_angle_70": abs(angle-70.) < 1e-8,
        "first_image_has_no_marker": not observations[0]["valid"],
        "wait_no_translation_command": bool(wait) and all(r["velocity_m_s"] == [0., 0., 0.] for r in wait),
        "wait_no_scan_command": bool(wait) and all(r["yaw_rate_rad_s"] == 0. for r in wait),
        "wait_position_held": translation < 1e-9,
        "wait_gaze_held": turn < 1e-9,
        "no_vehicle_start_request": all(not r["start_vehicle"] for r in commands),
        "car_moves_before_discovery": float(before[-1]["vehicle_travel_m"]) > .1,
        "constant_independent_vehicle_motion": speed_error < 1e-7,
    }
    report = {"artifact_dir": str(out), "phases": phases, "initial_smaller_angle_deg": angle,
        "max_wait_translation_m": translation, "max_wait_heading_change_deg": turn,
        "vehicle_travel_before_detection_or_timeout_m": float(before[-1]["vehicle_travel_m"]),
        "max_vehicle_displacement_timing_error_m": speed_error,
        "physical_touchdown_tested": False, "truth_for_evaluation_only": True, "checks": checks}
    if config["test"].get("hide_marker_all_frames", False):
        checks.update(expected_wait_abort=result["phase"] == "ABORT" and result["failure"] == "wait_timeout",
            no_approach_or_descent=phases == ["WAIT", "ABORT"], no_accepted_marker=not accepted,
            detector_received_blank_frames=all(r.get("reason") == "marker_missing" for r in observations),
            vehicle_kept_moving=float(truth[-1]["vehicle_travel_m"]) > 10.)
    else:
        acquire = next(r for r in commands if r["phase"] == "ACQUIRE")
        approach = next(r for r in commands if r["phase"] == "APPROACH")
        checks["exact_phase_order"] = phases == ["WAIT", "ACQUIRE", "LINEUP", "APPROACH", "TRACK", "DESCEND", "SETTLE", "COMPLETE"]
        checks["discovery_drives_acquire"] = abs(acquire["time_s"]-accepted[0]["time_s"]) < .021
        hold = config["search"]["lineup_hold_s"]
        window = [r for r in commands if approach["time_s"]-hold-1e-9 <= r["time_s"] <= approach["time_s"]]
        def aligned(r):
            s = r["observation"]
            limits = config["search"]
            return (s is not None and s["valid"]
                and abs(s["relative_pad_m"][0]-limits["lineup_standoff_m"]) <= limits["lineup_distance_tolerance_m"]+1e-9
                and abs(s["relative_pad_m"][1]) <= limits["lineup_tolerance_m"]+1e-9
                and abs(s["relative_velocity_m_s"][0]) <= limits["lineup_x_speed_tolerance_m_s"]+1e-9
                and abs(s["relative_velocity_m_s"][1]) <= limits["lineup_speed_tolerance_m_s"]+1e-9
                and abs(math.degrees(s["heading_error_rad"])) <= 2.+1e-9
                and r["time_s"]-s["stamp_s"] <= .15+1e-9)
        checks["full_lineup_held_before_approach"] = len(window) >= 45 and all(aligned(r) for r in window)
        row = truth[int(np.clip(np.searchsorted(times, approach["time_s"]), 0, len(truth)-1))]
        report.update(discovery_time_s=accepted[0]["time_s"], approach_time_s=approach["time_s"],
            complete_time_s=commands[-1]["time_s"], final_optical_height_m=float(truth[-1]["height_above_pad_m"]),
            truth_at_approach_evaluation_only={k: float(row[k]) for k in ("relative_x_m", "relative_y_m", "heading_deg")})
    report["passed"] = all(checks.values())
    (out / "passby-evaluation.json").write_text(json.dumps(report, indent=2)+"\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path, nargs="+")
    args = parser.parse_args()
    reports = [evaluate(out) for out in args.artifacts]
    target = ROOT / "artifacts/optical-passby/validation-summary.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    summary = {"all_passby_checks_passed": all(r["passed"] for r in reports), "hardware_landing_validated": False, "runs": reports}
    target.write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2))
    raise SystemExit(0 if summary["all_passby_checks_passed"] else 1)
