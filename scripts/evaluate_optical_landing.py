"""종료 후 참값과 영상 제어 로그를 대조하는 광학점 시험 평가."""
import csv
import json
from pathlib import Path
import numpy as np


def evaluate_ros(out):
    out = Path(out)
    summary_path = out / "control-summary.json"
    if not summary_path.is_file():
        report = {"test_passed": False, "failure": "no_control_summary", "hardware_landing_validated": False}
        (out / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
        return report
    report = json.loads(summary_path.read_text())
    criteria = json.loads((out / "experiment.json").read_text())["optical_evaluation"]
    rows = list(csv.DictReader((out / "truth.csv").open()))
    commands = [json.loads(line) for line in (out / "commands.jsonl").read_text().splitlines()]
    terminal = next((row for row in reversed(commands) if row["phase"] not in ("COMPLETE", "ABORT")), None)
    t = terminal["monotonic_s"] if terminal else float(rows[-1]["monotonic_s"])
    times = np.array([float(row["monotonic_s"]) for row in rows])
    # Terminal before zero-output shutdown, require a full 1s stable interval.
    selected = [row for row, stamp in zip(rows, times) if t-criteria["terminal_window_s"] <= stamp <= t]
    errors = [np.hypot(float(row["relative_x_m"]), float(row["relative_y_m"])) for row in selected]
    heights = [float(row["height_above_pad_m"]) for row in selected]
    speeds = [float(row["relative_speed_m_s"]) for row in selected]
    report.update(terminal_window_samples=len(selected),
                  terminal_max_center_error_m=float(max(errors)) if errors else None,
                  terminal_max_relative_speed_m_s=float(max(speeds)) if speeds else None,
                  terminal_height_range_m=[float(min(heights)), float(max(heights))] if heights else None,
                  final_vehicle_travel_m=float(rows[-1]["vehicle_travel_m"]),
                  evaluation_truth_not_control_input=True)
    report["test_passed"] = bool(report["phase"] == "COMPLETE" and report["inferred_completion"]
        and len(selected) >= 20 and max(errors) <= criteria["max_center_error_m"] and max(speeds) <= criteria["max_relative_speed_m_s"]
        and all(abs(h-criteria["terminal_height_m"]) <= criteria["height_tolerance_m"] for h in heights)
        and report["final_vehicle_travel_m"] >= criteria["min_vehicle_travel_m"]
        and report["accepted_observations"]/max(1, report["frames"]) >= criteria["min_valid_observation_fraction"])
    report["evaluation_criteria"] = criteria
    (out / "evaluation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report
