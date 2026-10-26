#!/usr/bin/env python3
"""Renderer -> image observer -> mission command -> optical kinematics loop."""
import argparse
from collections import deque
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo
import cv2
import numpy as np
from provided_hardware import ROOT, hardware_profile
from optical_landing_plant import OpticalPlant, annotated_image
from optical_landing_observer import OpticalLandingObserver
from landing_mission import LandingMission
from export_optical_video import export_video
from optical_landing_scenarios import SCENARIOS, configure_scenario


def run(config, out):
    out.mkdir(parents=True)
    (out / "experiment.json").write_text(json.dumps(config, indent=2) + "\n")
    (out / "provided-hardware.json").write_text(json.dumps(hardware_profile(), indent=2) + "\n")
    names = ["yaw_core.py", "provided_hardware.py", "moving_landing_core.py", "moving_landing_scene.py",
             "optical_landing_observer.py", "optical_landing_plant.py", "landing_mission.py", "run_optical_landing.py",
             "optical_landing_scenarios.py"]
    (out / "implementation-sha256.json").write_text(json.dumps({name: hashlib.sha256((ROOT / "scripts" / name).read_bytes()).hexdigest() for name in names}, indent=2) + "\n")
    plant = OpticalPlant(config, out)
    (out / "camera-profile.json").write_text(json.dumps(plant.profile, indent=2) + "\n")
    observer, mission = OpticalLandingObserver(plant.profile, config), LandingMission(config)
    pending = deque()
    truth_file = (out / "truth.csv").open("w")
    observation_file = (out / "observations.jsonl").open("w")
    command_file = (out / "commands.jsonl").open("w")
    video = cv2.VideoWriter(str(out / "mission.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 20., (320, 320), isColor=False)
    events, rendered, accepted, dropout_start = [], 0, 0, None
    command = {"phase": mission.phase, "velocity_m_s": [0., 0., 0.], "yaw_rate_rad_s": 0., "start_vehicle": False, "done": False}
    next_capture, now, dt = 0., 0., .02
    writer = None
    try:
        while now <= config["control"]["mission_timeout_s"] + 3.:
            if now + 1e-9 >= next_capture:
                gray = plant.render()
                if config["test"].get("hide_marker_all_frames", False):
                    gray[:] = 127  # 명시적인 마커 없는 입력 시험; 영상에도 동일하게 기록.
                rendered += 1
                pending.append((now + config["optical_test"]["transport_delay_s"], gray))
                next_capture += .05
                video.write(annotated_image(gray, command["phase"], observer.state(now)))
                if not (out / f"frame-{command['phase']}.png").exists():
                    cv2.imwrite(str(out / f"frame-{command['phase']}.png"), gray)
            while pending and pending[0][0] <= now + 1e-9:
                _, gray = pending.popleft()
                if command["phase"] == config["test"]["image_dropout_phase"] and dropout_start is None:
                    dropout_start = now
                drop = dropout_start is not None and now-dropout_start < config["test"]["image_dropout_s"]
                result = {"valid": False, "reason": "test_image_dropout"} if drop else observer.frame(gray, max(0., now-config["observer"]["assumed_image_delay_s"]), now)
                accepted += int(result["valid"])
                observation_file.write(json.dumps(dict(time_s=now, phase=command["phase"], **result)) + "\n")
            state = observer.state(now)
            previous = command["phase"]
            command = mission.update(now, state)
            command_file.write(json.dumps(dict(time_s=now, observation=state, **command)) + "\n")
            if command["phase"] != previous:
                events.append({"time_s": now, "phase": command["phase"]})
                print(f"{out.name}: t={now:.2f}s {previous} -> {command['phase']}", flush=True)
            truth = plant.truth(now, command)
            if writer is None:
                writer = csv.DictWriter(truth_file, fieldnames=list(truth))
                writer.writeheader()
            writer.writerow(truth)
            if command["done"] or command["phase"] == "ABORT":
                break
            plant.step(command, dt)
            now = round(now+dt, 8)
        if not command["done"] and command["phase"] != "ABORT":
            command["failure"] = "harness_timeout"
    finally:
        video.release()
        plant.close()
        truth_file.close(); command_file.close(); observation_file.close()
    summary = {"scope": "image-driven closed-loop optical-center kinematics; no aircraft dynamics/body mount/physical touchdown",
               "scenario": config.get("scenario", {}),
               "phase": command["phase"], "failure": command.get("failure"), "duration_s": now,
               "inferred_completion": bool(command.get("inferred_completion")), "events": events,
               "rendered_frames": rendered, "accepted_observations": accepted,
               "final_truth_evaluation_only": truth, "hardware_landing_validated": False,
               "controller_inputs": "mono320 image estimates, timestamps, configured vehicle speed; no simulator pose or velocity",
               "kinematic_assumptions": "perfect commanded optical-point velocity/yaw tracking; fixed virtual roll/pitch zero; ideal nominal projection"}
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    ff = config["test"]["ff_enabled"]
    dropout = config["test"]["image_dropout_s"]
    criteria = config["optical_evaluation"]
    actual = command["phase"] == "COMPLETE" and np.hypot(truth["relative_x_m"], truth["relative_y_m"]) <= criteria["max_center_error_m"] and truth["relative_speed_m_s"] <= criteria["max_relative_speed_m_s"] and abs(truth["height_above_pad_m"]-criteria["terminal_height_m"]) <= criteria["height_tolerance_m"] and truth["vehicle_travel_m"] >= criteria["min_vehicle_travel_m"] and accepted/rendered >= criteria["min_valid_observation_fraction"]
    expected_abort = dropout >= 2. and command["phase"] == "ABORT" and command.get("failure") is not None
    summary["nominal_optical_completion_passed"] = bool(actual)
    summary["expected_long_loss_abort_passed"] = bool(expected_abort)
    summary["expected_search_timeout_passed"] = bool(config["test"].get("hide_marker_all_frames", False)
        and command["phase"] == "ABORT" and command.get("failure") == "search_timeout"
        and not any(e["phase"] in ("ACQUIRE", "APPROACH", "DESCEND") for e in events)
        and truth["vehicle_travel_m"] == 0.)
    summary["expected_wait_timeout_passed"] = bool(config["test"].get("hide_marker_all_frames", False)
        and config.get("search", {}).get("passive_wait", False)
        and command["phase"] == "ABORT" and command.get("failure") == "wait_timeout"
        and not any(e["phase"] in ("ACQUIRE", "APPROACH", "DESCEND") for e in events)
        and truth["vehicle_travel_m"] > 0.)
    summary["feedback_only_expected_offset_observed"] = bool(not ff and command["phase"] != "COMPLETE" and np.hypot(truth["relative_x_m"], truth["relative_y_m"]) >= criteria["feedback_only_min_final_offset_m"] and accepted/rendered >= criteria["min_valid_observation_fraction"])
    summary["test_passed"] = bool(expected_abort if dropout >= 2. else actual) if ff else summary["feedback_only_expected_offset_observed"]
    if config["test"].get("hide_marker_all_frames", False):
        summary["test_passed"] = summary["expected_search_timeout_passed"] or summary["expected_wait_timeout_passed"]
    summary["evaluation_criteria"] = criteria
    (out / "evaluation.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    # Export a browser-compatible video when the system encoder is available.
    summary["mp4_exported"] = export_video(out)
    summary["plot_exported"] = subprocess.run([str(ROOT / "build/yaw-analysis-venv/bin/python"), str(ROOT / "scripts/plot_optical_landing.py"), str(out)]).returncode == 0
    (out / "evaluation.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-ff", action="store_true")
    parser.add_argument("--dropout", type=float, default=0.)
    parser.add_argument("--config", type=Path, default=ROOT / "config/moving-landing.json")
    parser.add_argument("--scenario", choices=SCENARIOS, default="passby70")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    configure_scenario(config, args.scenario)
    config["scope"] = "closed-loop optical-point kinematic test only"
    config["optical_test"] = {"transport_delay_s": .05, "image_fps": 20, "command_hz": 50}
    config["observer"]["assumed_image_delay_s"] = .05
    config["test"]["ff_enabled"] = not args.no_ff
    config["test"]["image_dropout_phase"] = "DESCEND" if args.dropout > 0 else ""
    config["test"]["image_dropout_s"] = args.dropout
    if args.no_ff:
        config["control"]["mission_timeout_s"] = 30.
    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out = ROOT / "artifacts/optical-landing" / run_id
    summary = run(config, out)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print(f"결과: {out}", flush=True)
    (ROOT / "artifacts/optical-landing/latest.json").write_text(json.dumps({"artifact_dir": str(out), "test_passed": summary["test_passed"]}, indent=2) + "\n")
    return 0 if summary["test_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
