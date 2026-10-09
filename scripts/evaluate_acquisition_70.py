#!/usr/bin/env python3
"""70도 정의·영상 발견·정렬 순서의 종료 후 검사. 참값은 제어에 넣지 않는다."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from provided_hardware import ROOT


def json_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def evaluate(out):
    out = out.resolve()
    config = json.loads((out / "experiment.json").read_text())
    evaluation = json.loads((out / "evaluation.json").read_text())
    truth = list(csv.DictReader((out / "truth.csv").open()))
    commands = json_lines(out / "commands.jsonl")
    observations = json_lines(out / "observations.jsonl")
    if "monotonic_s" in truth[0]:
        origin = np.median([r["monotonic_s"]-r["time_s"] for r in commands])
        times = np.array([float(r["monotonic_s"])-origin for r in truth])
    else:
        times = np.array([float(r["time_s"]) for r in truth])
    initial = truth[0]
    phases = []
    for r in commands:
        if not phases or phases[-1] != r["phase"]:
            phases.append(r["phase"])
    scan = [r for r in commands if r["phase"] == "SEARCH"]
    checks = {
        "recorded_test_passed": bool(evaluation["test_passed"]),
        "first_image_marker_missing": not observations[0]["valid"],
        "scan_no_translation_commands": all(r["velocity_m_s"] == [0., 0., 0.] for r in scan),
        "scan_vehicle_start_false": all(not r["start_vehicle"] for r in scan),
        "scan_rate_within_limit": all(abs(r["yaw_rate_rad_s"]) <= math.radians(config["search"]["rate_deg_s"])+1e-9 for r in scan),
    }
    report = {"artifact_dir": str(out), "scenario": config["scenario"], "phases": phases,
        "evaluation_scope": "image-driven optical kinematics and acquisition contracts only",
        "physical_touchdown_tested": False, "checks": checks}
    if config["test"].get("hide_marker_all_frames", False):
        checks.update(
            no_valid_observations=not any(r["valid"] for r in observations),
            blank_images_reached_detector=all(r.get("reason") == "marker_missing" for r in observations),
            search_timeout_abort=evaluation["phase"] == "ABORT" and evaluation["failure"] == "search_timeout",
            no_approach_or_descent=phases == ["SEARCH", "ABORT"],
            no_vehicle_travel=float(truth[-1]["vehicle_travel_m"]) == 0.)
        search_end = commands[-1]["time_s"]
    else:
        expected = ["SEARCH", "ACQUIRE", "ALIGN", "LINEUP", "APPROACH", "TRACK", "DESCEND", "SETTLE", "COMPLETE"]
        checks["expected_phase_order"] = phases == expected
        acquire = next(r for r in commands if r["phase"] == "ACQUIRE")
        approach = next(r for r in commands if r["phase"] == "APPROACH")
        first_detection = next(r for r in observations if r["valid"])
        report.update(discovery_time_s=first_detection["time_s"], approach_time_s=approach["time_s"],
            completion_time_s=commands[-1]["time_s"], final_optical_height_m=float(truth[-1]["height_above_pad_m"]))
        checks["discovery_drives_acquire"] = abs(acquire["time_s"]-first_detection["time_s"]) <= .021
        checks["car_started_on_discovery"] = acquire["start_vehicle"] is True
        rear_x = float(initial["pad_x_m"])-config["vehicle"]["pad_length_m"]/2
        line = math.atan2(float(initial["pad_y_m"])-float(initial["camera_y_m"]), rear_x-float(initial["camera_x_m"]))
        delta = (math.radians(float(initial["heading_deg"]))-line+math.pi)%(2*math.pi)-math.pi
        report["initial_smaller_forward_to_rear_angle_deg"] = abs(math.degrees(delta))
        checks["requested_initial_angle_70_deg"] = abs(report["initial_smaller_forward_to_rear_angle_deg"]-70.) < 1e-8
        window = [r for r in commands if approach["time_s"]-config["search"]["lineup_hold_s"]-1e-9 <= r["time_s"] <= approach["time_s"]]
        def aligned(r):
            state = r["observation"]
            return (state is not None and state["valid"]
                and abs(state["relative_pad_m"][1]) <= config["search"]["lineup_tolerance_m"]+1e-9
                and abs(state["relative_velocity_m_s"][1]) <= config["search"]["lineup_speed_tolerance_m_s"]+1e-9
                and abs(math.degrees(state["heading_error_rad"])) <= config["control"].get("align_tolerance_deg", 2.)+1e-9
                and r["time_s"]-state["stamp_s"] <= config["control"].get("descent_max_visual_age_s", .15)+1e-9)
        checks["one_second_lineup_gate_before_approach"] = len(window) >= 45 and all(aligned(r) for r in window)
        at_approach = truth[int(np.clip(np.searchsorted(times, approach["time_s"]), 0, len(truth)-1))]
        report["truth_at_approach_evaluation_only"] = {key: float(at_approach[key]) for key in ("relative_y_m", "heading_deg", "height_above_pad_m")}
        search_end = acquire["time_s"]
    before = [r for r, t in zip(truth, times) if t < search_end]
    displacement = max(math.dist([float(r[key]) for key in ("camera_x_m", "camera_y_m", "camera_z_m")],
                                [float(initial[key]) for key in ("camera_x_m", "camera_y_m", "camera_z_m")]) for r in before)
    report["scan_max_optical_translation_m"] = displacement
    checks["scan_holds_optical_position"] = displacement < 1e-10
    report["passed"] = all(checks.values())
    (out / "acquisition-evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path, nargs="+")
    args = parser.parse_args()
    reports = [evaluate(out) for out in args.artifacts]
    target = ROOT / "artifacts/optical-acquisition/validation-summary.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    summary = {"all_contract_checks_passed": all(r["passed"] for r in reports),
               "hardware_landing_validated": False, "runs": reports}
    target.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    raise SystemExit(0 if summary["all_contract_checks_passed"] else 1)
