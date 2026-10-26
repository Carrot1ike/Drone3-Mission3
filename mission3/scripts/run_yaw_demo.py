#!/usr/bin/env python3
"""Create reproducible experiment assets, run firmware ROS SIL, then evaluate."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from zoneinfo import ZoneInfo
import yaml
from yaw_scene import ROOT, build_scene, model_and_profile
from evaluate_yaw import evaluate


def main():
    parser = argparse.ArgumentParser(description="정지 ArUco yaw 보정 SIL 실험")
    parser.add_argument("--config", type=Path, default=ROOT / "config/yaw-static.json")
    parser.add_argument("--test-gyro-bias", type=float, default=None, help="PC 관측기 입력에만 넣는 가상 오차 deg/s")
    parser.add_argument("--test-image-dropout", type=float, default=None, help="HOLD 시작 시 영상 관측을 가리는 초")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if args.test_gyro_bias is not None:
        config["test"]["observer_gyro_bias_deg_s"] = args.test_gyro_bias
    if args.test_image_dropout is not None:
        config["test"]["drop_images_during_hold_s"] = args.test_image_dropout
    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out = ROOT / "artifacts/yaw" / run_id
    out.mkdir(parents=True)
    implementation = {}
    for name in ["yaw_core.py", "yaw_scene.py", "yaw_sim.py", "yaw_sitl.sh", "yaw_static.launch.py",
                 "yaw_control_node.py", "run_yaw_demo.py", "run_shared_ros.py", "evaluate_yaw.py"]:
        path = ROOT / "scripts" / name
        implementation[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (out / "implementation-sha256.json").write_text(json.dumps(implementation, indent=2) + "\n")
    (out / "experiment.json").write_text(json.dumps(config, indent=2) + "\n")
    path = build_scene(config, out)
    _, profile = model_and_profile(config, path)
    (out / "camera-profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    shared = ROOT / "src/crazyflie-ros2-sil"
    baseline = shared / "crazyswarm2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
    settings = yaml.safe_load(baseline.read_text())
    settings["robots"] = {"cf231": settings["robots"]["cf231"]}
    settings["robot_types"]["cf_sim"]["firmware_logging"]["custom_topics"]["yaw_imu"] = {
        "frequency": 50, "vars": ["gyro.x", "gyro.y", "gyro.z", "stabilizer.roll", "stabilizer.pitch"]}
    settings["robot_types"]["cf_sim"]["firmware_logging"]["custom_topics"]["yaw_motors"] = {
        "frequency": 10, "vars": ["motor.m1", "motor.m2", "motor.m3", "motor.m4"]}
    (out / "crazyflies.yaml").write_text(yaml.safe_dump(settings, sort_keys=False))
    print(f"정지 yaw 실험: {out}", flush=True)
    result = subprocess.run([sys.executable, str(ROOT / "scripts/run_shared_ros.py"), "--duration", "95",
                             "--launch-file", str(ROOT / "scripts/yaw_static.launch.py"), "--experiment-dir", str(out)])
    report = evaluate(out)
    report["launcher_exit_code"] = result.returncode
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    (ROOT / "artifacts/yaw/latest.json").write_text(json.dumps({"artifact_dir": str(out), "passed": report["passed"]}, indent=2) + "\n")
    return 0 if report["passed"] and result.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
