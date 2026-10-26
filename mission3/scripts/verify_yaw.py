#!/usr/bin/env python3
"""Yaw geometry, lateral motion, roll/pitch, ambiguity, delay and wrap checks."""
import argparse
import json
import math
from pathlib import Path
import cv2
import mujoco
import numpy as np
from yaw_core import MarkerYaw, YawFusion, euler_yaw_rate, rotation_rpy, wrap, rate_command
from yaw_scene import ROOT, build_scene, model_and_profile


def verify(out):
    config = json.loads((ROOT / "config/yaw-static.json").read_text())
    scene = build_scene(config, out)
    model, profile = model_and_profile(config, scene)
    data = mujoco.MjData(model)
    detector = MarkerYaw(profile, config["observer"])
    checks = []
    renderer = mujoco.Renderer(model, height=244, width=324)
    for yaw in (-20, -10, 0, 10, 20):
        for y, roll, pitch in [(0, 0, 0), (-.08, 0, 0), (.08, 0, 0), (0, -5, 3), (0, 5, -3)]:
            rpy = np.deg2rad([roll, pitch, yaw])
            data.qpos[:3] = [0., y, .30]
            mujoco.mju_mat2Quat(data.qpos[3:7], rotation_rpy(*rpy).flatten())
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="cf0_fpv_cam")
            rgb = renderer.render()
            gray = (rgb @ np.array([.2989, .5870, .1140])).astype(np.uint8)
            result = detector.detect(gray, *rpy[:2])
            error = None if not result["valid"] else math.degrees(wrap(math.radians(result["yaw_deg"]-yaw)))
            checks.append({"yaw_deg": yaw, "lateral_m": y, "roll_deg": roll, "pitch_deg": pitch,
                           "error_deg": error, "passed": result["valid"] and abs(error) <= 4., "observation": result})
            cv2.imwrite(str(out / f"yaw{yaw:+d}-y{y:+.2f}-r{roll:+d}-p{pitch:+d}.png"), gray)
    rejection_checks = []
    for name, xyz, yaw in [("distant_marker", [-1., 0., .30], 0.), ("camera_facing_away", [0., 0., .30], math.pi)]:
        data.qpos[:3] = xyz
        mujoco.mju_mat2Quat(data.qpos[3:7], rotation_rpy(0., 0., yaw).flatten())
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera="cf0_fpv_cam")
        gray = (renderer.render() @ np.array([.2989, .5870, .1140])).astype(np.uint8)
        observed = detector.detect(gray, 0., 0.)
        rejection_checks.append({"case": name, "passed": not observed["valid"], "reason": observed["reason"]})
        cv2.imwrite(str(out / f"rejection-{name}.png"), gray)
    data.qpos[:3] = [0., 0., .30]
    mujoco.mju_mat2Quat(data.qpos[3:7], rotation_rpy(0., 0., 0.).flatten())
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera="cf0_fpv_cam")
    gray = (renderer.render() @ np.array([.2989, .5870, .1140])).astype(np.uint8)
    observed = detector.detect(gray, math.radians(10.), 0.)
    rejection_checks.append({"case": "inconsistent_roll", "passed": not observed["valid"], "reason": observed["reason"]})
    renderer.close()
    # Translation with a fixed heading must not masquerade as marker-centering yaw.
    lateral_errors = [c["error_deg"] for c in checks if c["yaw_deg"] == 0 and c["roll_deg"] == 0]
    fusion = YawFusion(config["observer"])
    for i in range(101):
        fusion.imu(i*.01, [0, 0, 10], 0, 0)
    fusion.visual(0., .5, 1.)
    delayed_passed = abs(math.degrees(fusion.angle)-5.) < 1e-6
    wrap_passed = abs(math.degrees(wrap(math.radians(-179)-math.radians(179)))-2.) < 1e-9
    capped = rate_command(math.pi/2, 0, config["control"])
    cap_passed = abs(capped-math.radians(config["control"]["max_rate_deg_s"])) < 1e-9
    roll, pitch = .12, -.08
    vector = np.array([.1, .2, .3])
    dt = 1e-6
    initial = rotation_rpy(roll, pitch, 0.)
    change = cv2.Rodrigues(vector*dt)[0]
    expected = math.atan2((initial@change)[1, 0], (initial@change)[0, 0]) / dt
    gyro_passed = bool(abs(euler_yaw_rate(vector, roll, pitch)-expected) < 1e-6)
    report = {"passed": all(c["passed"] for c in checks+rejection_checks) and all((delayed_passed, wrap_passed, cap_passed, gyro_passed)),
              "rendered_cases": checks, "delay_compensation_passed": delayed_passed, "wrap_passed": wrap_passed,
              "rejection_checks": rejection_checks,
              "rate_limit_passed": cap_passed, "body_rate_conversion_passed": gyro_passed,
              "lateral_heading_errors_deg": lateral_errors, "profile": profile}
    (out / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("rendered_cases", "profile")}, indent=2))
    for c in checks:
        if not c["passed"]:
            print("FAIL", c["yaw_deg"], c["lateral_m"], c["roll_deg"], c["pitch_deg"], c["error_deg"], c["observation"]["reason"])
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/yaw-verification")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    raise SystemExit(verify(args.output))
