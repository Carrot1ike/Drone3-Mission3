#!/usr/bin/env python3
"""동일 관측·미션 제어를 공유 CPX/ROS 영상 경로로 재현."""
from datetime import datetime
import hashlib
import json
import shutil
import subprocess
import sys
from zoneinfo import ZoneInfo
from provided_hardware import ROOT
from moving_landing_scene import build_scene, build_optical_preview_model, camera_profile
from evaluate_optical_landing import evaluate_ros
from export_optical_video import export_video
from optical_landing_scenarios import SCENARIOS, configure_scenario
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=[s for s in SCENARIOS if s not in ("blind-search", "passby-no-marker")], default="passby70")
    args = parser.parse_args()
    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out = ROOT / "artifacts/optical-landing-ros" / run_id
    out.mkdir(parents=True)
    config = json.loads((ROOT / "config/moving-landing.json").read_text())
    configure_scenario(config, args.scenario)
    config["scope"] = "ROS closed-loop optical-point kinematics only"
    config["observer"]["assumed_image_delay_s"] = .05
    (out / "experiment.json").write_text(json.dumps(config, indent=2) + "\n")
    scene = build_scene(config, out)
    model = build_optical_preview_model(config, scene)
    profile = camera_profile(model, config)
    (out / "camera-profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    files = ["landing_mission.py", "optical_landing_observer.py", "optical_landing_plant.py", "moving_landing_core.py",
             "optical_landing_ros_plant.py", "optical_landing_ros_node.py", "optical_landing.launch.py", "evaluate_optical_landing.py",
             "optical_landing_scenarios.py", "run_optical_landing_ros.py", "moving_landing_scene.py", "yaw_core.py", "provided_hardware.py"]
    (out / "implementation-sha256.json").write_text(json.dumps({name: hashlib.sha256((ROOT / "scripts" / name).read_bytes()).hexdigest() for name in files}, indent=2) + "\n")
    result = subprocess.run([sys.executable, str(ROOT / "scripts/run_shared_ros.py"), "--duration", str(config["control"]["mission_timeout_s"]+15),
        "--launch-file", str(ROOT / "scripts/optical_landing.launch.py"), "--experiment-dir", str(out)])
    report = evaluate_ros(out)
    report["launcher_exit_code"] = result.returncode
    (out / "evaluation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    if (out / "mission.avi").exists():
        report["mp4_exported"] = export_video(out)
    if (out / "truth.csv").exists() and (out / "control-summary.json").exists():
        report["plot_exported"] = subprocess.run([str(ROOT / "build/yaw-analysis-venv/bin/python"), str(ROOT / "scripts/plot_optical_landing.py"), str(out)]).returncode == 0
    (out / "evaluation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    (ROOT / "artifacts/optical-landing-ros/latest.json").write_text(json.dumps({"artifact_dir": str(out), "test_passed": report["test_passed"]}, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print(f"ROS 폐루프 결과: {out}", flush=True)
    return 0 if result.returncode == 0 and report["test_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
