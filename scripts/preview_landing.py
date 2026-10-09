#!/usr/bin/env python3
"""Verify a scripted optical-center path, not autonomous Crazyflie flight."""
import argparse
from contextlib import ExitStack
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
from zoneinfo import ZoneInfo


def main():
    parser = argparse.ArgumentParser(description="이동 차량·수직 마커의 시야와 ArUco 검출 기하 시험")
    root = Path(__file__).resolve().parents[1]
    parser.add_argument("--config", type=Path, default=root / "config/landing.toml")
    parser.add_argument("--gui", action="store_true", help="MuJoCo 창에서 광학 중심의 지정 경로를 실시간 표시")
    args = parser.parse_args()
    if not args.gui:
        os.environ.setdefault("MUJOCO_GL", "egl")
    else:
        os.environ["MUJOCO_GL"] = "glfw"

    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    from aruco_pose import detect_pose, draw_detection, pinhole_matrix
    from landing_scene import LandingScene, read_config

    config = read_config(args.config)
    scene = LandingScene(config)
    marker_asset = scene.save_assets()
    camera, marker, trajectory = (config[key] for key in ("camera", "marker", "trajectory"))
    matrix = pinhole_matrix(camera["width"], camera["height"], camera["vertical_fov_deg"])
    run_id = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    folder = root / "artifacts/visibility" / run_id
    folder.mkdir(parents=True)
    (folder / "scene.xml").write_text(scene.xml)
    (folder / "experiment.toml").write_bytes(args.config.read_bytes())
    duration = trajectory["duration_s"]
    count = math.ceil(duration*trajectory["sample_hz"])+1
    times = np.linspace(0., duration, count)
    selected_indices = {0, int(np.argmin(abs(times-duration*0.5))), count-1}
    frames = []
    snapshots = []
    parameters = config["verification"]

    def observe(seconds, renderer, *, height_override=None):
        scene.set_time(float(seconds), height_override=height_override)
        frame = scene.render(renderer)
        estimate = detect_pose(frame, matrix, marker["length_m"], marker["dictionary"], marker["id"])
        corners_camera = scene.marker_corners_camera()
        projected = corners_camera @ matrix.T
        projected = projected[:, :2]/projected[:, 2:3]
        visible = bool(np.all(corners_camera[:, 2] > 0)
                       and np.all(projected[:, 0] >= 0) and np.all(projected[:, 0] < camera["width"])
                       and np.all(projected[:, 1] >= 0) and np.all(projected[:, 1] < camera["height"]))
        pad_camera_truth = scene.world_to_camera(scene.pad_position)
        record = {
            "time_s": float(seconds), "phase": scene.phase,
            "vehicle_position_world_m": scene.vehicle_position.tolist(),
            "camera_optical_position_world_m": scene.optical_position.tolist(),
            "camera_height_above_pad_m": float(scene.optical_position[2]-scene.pad_position[2]),
            "geometric_corners_inside_image": visible,
            "predicted_marker_shortest_edge_px": float(np.min(np.linalg.norm(projected-np.roll(projected, 1, axis=0), axis=1))),
            "detected": estimate is not None,
            "pad_translation_camera_truth_m": pad_camera_truth.tolist(),
        }
        if estimate is not None:
            pad_camera_estimate = (estimate.rotation_camera_marker @ scene.pad_offset_marker
                                   +estimate.translation_camera_marker)
            difference_rotation = estimate.rotation_camera_marker.T @ scene.marker_rotation_camera()
            rotation_error = math.degrees(math.acos(float(np.clip((np.trace(difference_rotation)-1.)/2., -1., 1.))))
            record.update(
                marker_translation_camera_estimate_m=estimate.translation_camera_marker.tolist(),
                pad_translation_camera_estimate_m=pad_camera_estimate.tolist(),
                pad_translation_error_m=float(np.linalg.norm(pad_camera_estimate-pad_camera_truth)),
                marker_rotation_error_deg=rotation_error,
                reprojection_error_px=estimate.reprojection_error_px,
            )
        return record, frame, estimate

    with ExitStack() as stack:
        front_renderer = stack.enter_context(mujoco.Renderer(scene.model, height=camera["height"], width=camera["width"]))
        side_renderer = stack.enter_context(mujoco.Renderer(scene.model, height=360, width=640))
        viewer = None
        if args.gui:
            import mujoco.viewer
            viewer = stack.enter_context(mujoco.viewer.launch_passive(scene.model, scene.data))
        started = time.monotonic()
        for index, seconds in enumerate(times):
            if viewer is not None and not viewer.is_running():
                raise KeyboardInterrupt
            record, rgb, estimate = observe(seconds, front_renderer)
            frames.append(record)
            if index in selected_indices:
                side = scene.render(side_renderer, camera=scene.overview_camera(), show_glyph=True)
                annotated = draw_detection(rgb, estimate)
                Image.fromarray(rgb).save(folder / f"camera-{index:04d}.png")
                Image.fromarray(annotated).save(folder / f"camera-detected-{index:04d}.png")
                Image.fromarray(side).save(folder / f"overview-{index:04d}.png")
                snapshots.append((record, side, annotated))
            if viewer is not None:
                viewer.cam.lookat[:] = scene.overview_camera().lookat
                viewer.cam.distance = 1.65
                viewer.cam.azimuth = -43.
                viewer.cam.elevation = -26.
                viewer.sync()
                time.sleep(max(0., started+float(seconds)-time.monotonic()))

        # Same horizontal location, with a deliberately excessive optical-center height.
        negative, negative_rgb, negative_estimate = observe(duration, front_renderer,
            height_override=parameters["negative_probe_camera_height_m"])
        Image.fromarray(draw_detection(negative_rgb, negative_estimate)).save(folder / "negative-high-camera.png")

    detected = [frame for frame in frames if frame["detected"]]
    near = [frame for frame in frames
            if np.linalg.norm(np.array(frame["camera_optical_position_world_m"][:2])
                              -np.array(frame["vehicle_position_world_m"][:2])) <= 0.08]
    near_errors = [frame["pad_translation_error_m"] for frame in near if frame["detected"]]
    detection_fraction = len(detected)/len(frames)
    reprojections = [frame["reprojection_error_px"] for frame in detected]
    checks = {
        "all_nominal_marker_corners_inside_image": all(frame["geometric_corners_inside_image"] for frame in frames),
        "detection_fraction_meets_internal_threshold": detection_fraction >= parameters["minimum_detection_fraction"],
        "all_near_pad_frames_detected": bool(near) and all(frame["detected"] for frame in near),
        "near_pad_pose_error_meets_internal_threshold": bool(near_errors) and max(near_errors) <= parameters["maximum_near_pad_error_m"],
        "reprojection_meets_internal_threshold": bool(reprojections) and max(reprojections) <= parameters["maximum_reprojection_error_px"],
        "negative_high_camera_not_detected": not negative["geometric_corners_inside_image"] and not negative["detected"],
        "vehicle_end_displacement_matches_speed": abs(frames[-1]["vehicle_position_world_m"][0]
                                                       -config["vehicle"]["speed_m_s"]*duration) < 1e-10,
    }
    summary = {
        "nominal_frames": len(frames), "detected_frames": len(detected),
        "detection_fraction": detection_fraction,
        "minimum_marker_shortest_edge_px": min(frame["predicted_marker_shortest_edge_px"] for frame in frames),
        "near_pad_frames": len(near),
        "maximum_near_pad_translation_error_m": max(near_errors) if near_errors else None,
        "maximum_translation_error_m": max((frame["pad_translation_error_m"] for frame in detected), default=None),
        "maximum_reprojection_error_px": max(reprojections, default=None),
        "checks": checks, "geometry_checks_passed": all(checks.values()),
    }
    report = {
        "generated_at_kst": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
        "stage": "scripted_camera_visibility_and_aruco_pose_only",
        "autonomous_controller_implemented": False,
        "aircraft_dynamics_validated": False, "touchdown_validated": False,
        "equipment_acceptance_passed": None,
        "camera_to_body_transform": None,
        "projection": camera["projection"],
        "vision_inputs": ["rendered_rgb_frame", "nominal_projection_matrix",
                          "marker_dictionary_and_id", "marker_length_m", "fixed_marker_to_pad_transform"],
        "simulator_truth_usage": "geometry evaluation only; not a vision input",
        "marker_asset": str(marker_asset), "config": config,
        "projection_matrix": matrix.tolist(), "summary": summary,
        "negative_high_camera_probe": negative, "frames": frames,
    }
    report_text = json.dumps(report, ensure_ascii=False, indent=2)
    (folder / "report.json").write_text(report_text)
    (root / "artifacts/landing-visibility.json").write_text(report_text)

    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    def font(size):
        return ImageFont.truetype(font_path, size, index=1)
    preview = Image.new("RGB", (1060, 1400), "#f7f9fc")
    draw = ImageDraw.Draw(preview)
    draw.text((24, 19), "수직 ArUco · 이동 착륙장 시야 시험", font=font(32), fill="#15354d")
    draw.text((24, 72), "카메라 경로 직접 지정 / 비행 제어·접촉 검증 전", font=font(23), fill="#586879")
    draw.text((24, 107), f"마커 {marker['length_m']*100:g} cm · 착륙장 {config['pad']['length_m']*100:g}×{config['pad']['width_m']*100:g} cm · 차량 {config['vehicle']['speed_m_s']:.1f} m/s", font=font(22), fill="#586879")
    phase_names = {"align": "정렬 전", "approach": "접근", "match": "착륙점 위 유지", "descend": "광학 중심 하강"}
    for n, (record, side, annotated) in enumerate(snapshots):
        top = 161+n*395
        error_cm = record.get("pad_translation_error_m", float("nan"))*100
        caption = f"t={record['time_s']:.1f}s · {phase_names[record['phase']]} · 검출 {'성공' if record['detected'] else '실패'}"
        draw.text((24, top), caption, font=font(22), fill="#15354d")
        preview.paste(Image.fromarray(side), (24, top+36))
        preview.paste(Image.fromarray(annotated).resize((360, 360), Image.Resampling.NEAREST), (680, top+36))
        draw.text((687, top+46), f"착륙점 추정 오차 {error_cm:.1f} cm", font=font(16), fill="#c2ffcf", stroke_width=1, stroke_fill="#102a22")
    draw.text((24, 1356), f"검출 {len(detected)}/{len(frames)} 프레임 · 장착 변환·기체 동역학·자동 착륙은 미검증", font=font(20), fill="#586879")
    preview.save(folder / "preview.png")
    preview.save(root / "artifacts/landing-visibility.png")
    print(json.dumps({"run_folder": str(folder), "summary": summary}, ensure_ascii=False, indent=2))
    return 0 if summary["geometry_checks_passed"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("시야 시험을 종료했습니다.", file=sys.stderr)
        raise SystemExit(130)
