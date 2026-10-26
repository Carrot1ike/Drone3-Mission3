#!/usr/bin/env python3
"""Fixed mono320 input and optical-center-only marker visibility checks.

The renderer contains no aircraft. Camera poses are prescribed fixtures,
not a commanded flight, a measured mounting or a landing trajectory result.
"""
import argparse
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np

from moving_landing_scene import build_scene, build_optical_preview_model, camera_profile
from provided_hardware import (ROOT, CameraContractError, HardwareModelUnresolved,
                               decode_mono320, validate_camera_config, require_manufacturer_flight_model)
from moving_landing_core import relative_pad_position


def input_contract_checks():
    pixels = np.arange(320 * 320, dtype=np.uint8).reshape(320, 320)
    decoded = decode_mono320(320, 320, "mono8", 320, pixels)
    padded = np.full((320, 324), 253, dtype=np.uint8)
    padded[:, :320] = pixels
    padded_decoded = decode_mono320(320, 320, "mono8", 324, padded)
    checks = [
        {"case": "native_mono320_without_resampling", "passed": bool(np.array_equal(decoded, pixels) and np.shares_memory(decoded, pixels))},
        {"case": "row_padding_without_resampling", "passed": bool(np.array_equal(padded_decoded, pixels) and np.shares_memory(padded_decoded, padded))}
    ]
    for name, arguments in [
        ("reject_baseline_324x244", (324, 244, "mono8", 324, bytes(324 * 244))),
        ("reject_color", (320, 320, "rgb8", 960, bytes(320 * 320 * 3))),
        ("reject_smaller_resolution", (160, 160, "mono8", 160, bytes(160 * 160))),
        ("reject_short_frame", (320, 320, "mono8", 320, bytes(320 * 320 - 1))),
        ("reject_extra_frame_bytes", (320, 320, "mono8", 320, bytes(320 * 320 + 1))),
        ("reject_short_row", (320, 320, "mono8", 319, bytes(319 * 320)))
    ]:
        rejected = False
        try:
            decode_mono320(*arguments)
        except CameraContractError:
            rejected = True
        checks.append({"case": name, "passed": rejected})
    for name, configuration in [
        ("reject_changed_fov", {"width": 320, "height": 320, "encoding": "mono8", "vertical_fov_deg": 100.}),
        ("reject_unresolved_flight_model", None)
    ]:
        rejected = False
        try:
            validate_camera_config(configuration) if configuration is not None else require_manufacturer_flight_model()
        except (CameraContractError, HardwareModelUnresolved):
            rejected = True
        checks.append({"case": name, "passed": rejected})
    return checks


def verify(out):
    out.mkdir(parents=True, exist_ok=True)
    config = json.loads((ROOT / "config/moving-landing.json").read_text())
    scene = build_scene(config, out)
    model = build_optical_preview_model(config, scene)
    profile = camera_profile(model, config)
    (out / "camera-profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    (out / "experiment.json").write_text(json.dumps(config, indent=2) + "\n")
    checks = input_contract_checks()
    try:
        relative_pad_position({}, 0., 0., profile)
        rejected_transform = False
    except HardwareModelUnresolved:
        rejected_transform = True
    checks.append({"case": "reject_ideal_camera_as_aircraft_mounting", "passed": rejected_transform})
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["marker"]["dictionary"]))
    settings = cv2.aruco.DetectorParameters()
    settings.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, settings)
    camera = model.camera("optical_preview").id
    data = mujoco.MjData(model)
    joint = model.joint("vehicle_slide").qposadr[0]
    car_origin = np.asarray(config["vehicle"]["initial_pad_center_m"])
    Rcv = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    renderer = mujoco.Renderer(model, height=320, width=320)
    rendered = []
    overview = []
    try:
        # Camera optical-center fixtures only. Aircraft offset/attitude is absent.
        for stage, dx, height in [("approach", -.35, .30), ("over_pad", 0., .30),
                                  ("lower_view", 0., .15), ("near_pad_view", 0., .03)]:
            for lateral, yaw in [(0., 0.), (-.08, 0.), (.08, 0.), (0., -10.), (0., 10.)]:
                for car_travel in (0., 1.50):
                    data.qpos[joint] = car_travel
                    pad = car_origin + np.array([car_travel, 0., 0.])
                    optical_point = pad + [dx, lateral, height]
                    model.cam_pos[camera] = optical_point
                    angle = np.deg2rad(yaw)
                    Rz = np.array([[np.cos(angle), -np.sin(angle), 0.], [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
                    mujoco.mju_mat2Quat(model.cam_quat[camera], (Rz @ Rcv @ np.diag([1., -1., -1.])).flatten())
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera="optical_preview")
                    # Synthetic monochrome sensor output from the graphics renderer.
                    # No resize or conversion of an incoming color-camera message.
                    mono = (renderer.render() @ np.array([.2989, .5870, .1140])).astype(np.uint8)
                    raw = mono.tobytes()
                    image = decode_mono320(320, 320, "mono8", 320, raw)
                    corners, ids, _ = detector.detectMarkers(image)
                    found = ids is not None and config["marker"]["id"] in ids.flatten()
                    side, border_clear = None, False
                    if found:
                        pts = np.asarray(corners[list(ids.flatten()).index(config["marker"]["id"])], float).reshape(4, 2)
                        side = float(np.min(np.linalg.norm(pts-np.roll(pts, 1, axis=0), axis=1)))
                        border_clear = bool(np.all(pts >= 3.) and np.all(pts <= 316.))
                    marker = pad + config["vehicle"]["marker_offset_from_pad_m"]
                    optical_range = float(np.linalg.norm(marker-optical_point))
                    passed = bool(found and side >= config["observer"]["min_side_px"] and border_clear and optical_range >= .30)
                    label = f"{stage}-y{lateral:+.2f}-yaw{yaw:+.0f}-travel{car_travel:.2f}"
                    cv2.imwrite(str(out / f"{label}.png"), image)
                    rendered.append({"fixture": label, "marker_detected": bool(found), "minimum_side_px": side,
                                     "border_clear": border_clear, "specified_optical_range_m": optical_range,
                                     "width": 320, "height": 320, "encoding": "mono8", "bytes": len(raw), "passed": passed})
                    if lateral == yaw == car_travel == 0.:
                        annotated = image.copy()
                        cv2.rectangle(annotated, (0, 0), (319, 22), 0, -1)
                        cv2.putText(annotated, stage, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, .45, 255, 1)
                        overview.append(annotated)
    finally:
        renderer.close()
    cv2.imwrite(str(out / "nominal-optical-overview.png"), np.block([[overview[0], overview[1]], [overview[2], overview[3]]]))
    report = {
        "scope": "fixed camera input checks and prescribed camera optical-point geometry; no aircraft dynamics, mounting, closed-loop pursuit or touchdown",
        "camera_checks_passed": all(c["passed"] for c in checks),
        "nominal_visibility_checks_passed": all(c["passed"] for c in rendered),
        "eligible_for_supplied_aircraft_landing_validation": False,
        "input_contract_checks": checks, "rendered_fixtures": rendered,
        "calibration_and_sensor_caveats": "ideal projection, no calibrated distortion/exposure/rolling shutter/blur/noise/link delay or measured mounting"
    }
    (out / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("rendered_fixtures",)}, ensure_ascii=False, indent=2))
    print(f"지정한 광학점 조건의 검출: {sum(c['passed'] for c in rendered)}/{len(rendered)}")
    for result in rendered:
        if not result["passed"]:
            print("FAIL", result)
    return 0 if report["camera_checks_passed"] and report["nominal_visibility_checks_passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/provided-camera-check")
    args = parser.parse_args()
    raise SystemExit(verify(args.output))
