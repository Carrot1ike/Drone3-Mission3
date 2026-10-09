#!/usr/bin/env python3
"""합성 카메라 영상과 종료 후 평가 위치를 함께 보여 주는 검토용 영상.

오른쪽 도식의 참값은 평가 자료이며 제어 입력이 아니다. 왼쪽 영상은
저장된 320×320 그대로 표시한다. 도식의 점은 드론이 아닌 광학점이다.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from export_optical_video import export_video
from camera_visibility_panel import panel, SCREEN_SIZE
from optical_visibility import horizontal_fov_deg


def review(out):
    config = json.loads((out / "experiment.json").read_text())
    profile = json.loads((out / "camera-profile.json").read_text())
    observations = [json.loads(line) for line in (out / "observations.jsonl").read_text().splitlines()]
    truth = list(csv.DictReader((out / "truth.csv").open()))
    surface_path = out / "surface-replay/trace.csv"
    surface = list(csv.DictReader(surface_path.open())) if surface_path.exists() else None
    if surface is not None and len(surface) != len(truth):
        raise ValueError("surface replay does not match truth row count")
    commands = [json.loads(line) for line in (out / "commands.jsonl").read_text().splitlines()]
    command_times = np.array([row["time_s"] for row in commands])
    ros = "monotonic_s" in truth[0]
    if ros:
        origin = np.median([r["monotonic_s"]-r["time_s"] for r in commands])
        stamps = np.array([float(row["monotonic_s"])-origin for row in truth])
    else:
        stamps = np.array([float(row["time_s"]) for row in truth])
    capture = cv2.VideoCapture(str(out / "mission.avi"))
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or fps <= 0:
        raise ValueError("recorded video is required")
    frames = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    capture.release()
    if not frames:
        raise ValueError("recorded video contains no frames")
    if ros:
        if len(observations) != len(frames):
            raise ValueError("ROS frame timestamps do not match recorded video")
        frame_times = np.array([r["time_s"] for r in observations])
    else:
        frame_times = np.arange(len(frames))/fps
    rear_x = config["vehicle"]["pad_length_m"]/2
    initial = truth[0]
    initial_line = np.arctan2(float(initial["relative_y_m"]), float(initial["relative_x_m"])-rear_x)
    initial_delta = (np.deg2rad(float(initial["heading_deg"]))-initial_line+np.pi)%(2*np.pi)-np.pi
    initial_angle = float(abs(np.rad2deg(initial_delta)))
    writer = cv2.VideoWriter(str(out / "review.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, SCREEN_SIZE, isColor=False)
    if not writer.isOpened():
        raise ValueError("review video writer failed")
    saved_phases = set()
    saved_low_descent = False
    frame_count = int(np.ceil(frame_times[-1]*fps))+1
    for frame_index in range(frame_count):
        t = frame_index/fps
        truth_index = int(np.clip(np.searchsorted(stamps, t), 0, len(truth)-1))
        row = truth[truth_index]
        phase = commands[int(np.clip(np.searchsorted(command_times, t, side="right")-1, 0, len(commands)-1))]["phase"]
        index = int(np.clip(np.searchsorted(frame_times, t, side="right")-1, 0, len(frames)-1))
        frame_observation = observations[index] if index < len(observations) else None
        screen = panel(frames[index], row, phase, t, config, profile, initial_angle, frame_observation,
                       surface[truth_index] if surface is not None else None)
        writer.write(screen)
        if frame_index == 0:
            cv2.imwrite(str(out / "review-preview.png"), screen)
        if phase not in saved_phases:
            cv2.imwrite(str(out / f"review-{phase}.png"), screen)
            saved_phases.add(phase)
        if not saved_low_descent and phase == "DESCEND" and float(row["height_above_pad_m"]) <= .12:
            cv2.imwrite(str(out / "review-DESCEND-low.png"), screen)
            saved_low_descent = True
    writer.release()
    cv2.imwrite(str(out / "review-LAST.png"), screen)
    # Export uses conventional mission filename in a temporary local subfolder.
    review_dir = out / "review-export"
    review_dir.mkdir(exist_ok=True)
    (review_dir / "mission.avi").symlink_to(out / "review.avi")
    exported = export_video(review_dir)
    if exported:
        (review_dir / "mission.mp4").replace(out / "review.mp4")
    (review_dir / "mission.avi").unlink()
    review_dir.rmdir()
    metadata = {"source_artifact_dir": str(out), "source_frames": len(frames), "review_frames": frame_count,
        "fps": fps, "duration_s": frame_count/fps, "initial_smaller_forward_to_rear_angle_deg": initial_angle,
        "frame_timing": "ROS observation receipt times, previous frame held between arrivals" if ros else "offline camera capture times",
        "input_resolution": [320, 320], "truth_used_for_evaluation_overlay_only": True,
        "review_resolution": list(SCREEN_SIZE), "horizontal_fov_deg": horizontal_fov_deg(profile),
        "fixed_triangle_range_cutoff_m": None,
        "candidate_region": "ideal marker projection: front facing, all corners within 3px edge margin, min side >= configured detector pixels; image verdict recorded separately",
        "recognition_range_is_manufacturer_specification": False,
        "side_view_height_added": True, "surface_replay_overlay": surface is not None,
        "surface_probe_is_real_flow_deck_mounting": False, "flow_sensor_connected_to_control": False,
        "physical_touchdown_tested": False, "mp4_exported": exported,
        "review_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "visibility_source_sha256": {name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
            for name in ("camera_visibility_panel.py", "optical_visibility.py")}}
    (out / "review-metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    review(args.output.resolve())
