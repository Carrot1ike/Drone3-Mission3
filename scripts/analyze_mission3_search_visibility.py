#!/usr/bin/env python3
"""Offline counterfactual projection at saved pre-discovery search positions.

This changes no input pixels or flight. A different gaze/altitude would change
the closed loop; these projections cannot be counted as detections or flights.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import multiprocessing
from pathlib import Path
import shutil

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from analyze_mission3_native_geometry import rows, sha
from aruco_pose import pinhole_matrix
from mission3.scene import R_HEADING_CAMERA, R_VEHICLE_MARKER
from mission3.track import rotation
from yaw_core import rotation_rpy


VARIANTS = [("saved_pose", None, False),
            ("entry_gaze_saved_height", None, True),
            ("entry_gaze_height085", .85, True),
            ("entry_gaze_height070", .70, True),
            ("entry_gaze_height060", .60, True),
            ("saved_gaze_height070", .70, False)]


def analyze(task):
    run_name, geometry_name = task
    run = Path(run_name)
    experiment = json.loads((run / "experiment.json").read_text())
    cfg, physical = experiment["mission"], experiment["physical"]
    entry = next(x for x in cfg["entry"]["positions"] if x["id"] == experiment["entry_id"])
    truth, frames, commands = (rows(run / p) for p in ("truth.jsonl", "camera_frames.jsonl", "commands.jsonl"))
    original = json.loads(Path(geometry_name).read_text())
    first_discovery = min((x["t"] for x in commands if x["phase"] in ("COARSE_ACQUIRE", "ACQUIRE")), default=float("inf"))
    command_times = np.array([x["t"] for x in commands])
    selected = []
    for i, frame in enumerate(frames):
        t = frame["t"]
        j = max(0, int(np.searchsorted(command_times, t, side="right")) - 1)
        if commands[j]["phase"] == "ENTRY_SEARCH" and t < first_discovery:
            selected.append(i)
    ts = np.array([x["t"] for x in truth])
    times = np.array([frames[i]["t"] for i in selected])
    if len(times) == 0 or np.any(times < ts[0]) or np.any(times > ts[-1] + 1e-8):
        raise ValueError("empty/outside pre-discovery search capture times")
    times = np.clip(times, ts[0], ts[-1])
    q = np.array([x["aircraft"]["quaternion_wxyz"] for x in truth])[:, [1, 2, 3, 0]]
    Rwb = Slerp(ts, Rotation.from_quat(q))(times).as_matrix()
    interp = lambda array: np.column_stack([np.interp(times, ts, array[:, j]) for j in range(array.shape[1])])
    positions = interp(np.array([x["aircraft"]["center_world_m"] for x in truth]))
    pads = interp(np.array([x["pad"]["center_world_m"] for x in truth]))
    headings = np.interp(times, ts, np.unwrap([x["vehicle"]["heading_rad"] for x in truth]))
    rpy = Rotation.from_matrix(Rwb).as_euler("xyz")
    entry_rpy = rpy.copy()
    entry_rpy[:, 2] = np.deg2rad(entry["heading_deg"])
    entry_Rwb = Rotation.from_euler("xyz", entry_rpy).as_matrix()
    mounts = physical["mounts"]
    Rbc = rotation_rpy(*np.deg2rad(mounts["camera_rpy_offset_deg"])) @ R_HEADING_CAMERA
    optics = cfg["camera"].get("optics_sensitivity", {})
    K = np.asarray(optics.get("K", pinhole_matrix(320, 320, cfg["camera"]["vertical_fov_deg"])), float)
    D = np.asarray(optics.get("distortion", [0.] * 5), float)
    half = cfg["marker"]["code_side_m"] / 2
    local = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    offset = np.array(cfg["marker"]["known_marker_to_pad_vehicle_m"])
    gate = cfg["observer"]["min_side_px"]
    reports = []
    for name, height, entry_gaze in VARIANTS:
        samples = []
        for index, t, p, pad, actual_R, fixed_R, heading in zip(selected, times, positions, pads, Rwb, entry_Rwb, headings):
            p = p.copy()
            if height is not None:
                p[2] = height
            R = fixed_R if entry_gaze else actual_R
            Rwv = rotation(heading)
            center = pad + Rwv @ offset
            camera = p + R @ mounts["camera_position_body_m"]
            Rwm = Rwv @ R_VEHICLE_MARKER
            camera_corners = (local @ Rwm.T + center - camera) @ (R @ Rbc)
            front = float(Rwm[:, 2] @ (camera - center)) > 0
            positive = bool(np.all(camera_corners[:, 2] > .001))
            uv = cv2.projectPoints(camera_corners, np.zeros(3), np.zeros(3), K, D)[0].reshape(4, 2) if positive else None
            side = float(np.linalg.norm(uv - np.roll(uv, 1, axis=0), axis=1).min()) if positive else None
            full = bool(positive and np.all(uv >= 3) and np.all(uv <= 316))
            depth = float(((center - camera) @ (R @ Rbc))[2])
            focus = depth >= cfg["camera"]["published_depth_of_field_near_m"]
            eligible = front and full and focus
            samples.append({"source_frame_index": index, "capture_s": float(t), "front_full_focus": eligible,
                            "min_projected_side_px": side, "passes_size_gate": bool(eligible and side >= gate),
                            "center_depth_m": depth})
            if name == "saved_pose":
                old = original["samples"][index]
                assert full == old["whole_code_in_frame_3px_margin"] and front == old["front_face"]
                assert (side is None and old["min_projected_side_px"] is None) or np.isclose(side, old["min_projected_side_px"], atol=1e-8, rtol=1e-9)
        candidates = [s for s in samples if s["front_full_focus"]]
        eligible_size = [s for s in candidates if s["passes_size_gate"]]
        reports.append({"variant": name, "height_m": height, "fixed_entry_gaze": entry_gaze,
                        "pre_discovery_search_frames": len(samples), "front_full_focus_frames": len(candidates),
                        "projected_size_gate_frames": len(eligible_size),
                        "first_size_candidate_s": eligible_size[0]["capture_s"] if eligible_size else None,
                        "max_min_side_px": max((s["min_projected_side_px"] for s in candidates), default=None),
                        "largest_candidates": sorted(candidates, key=lambda s: s["min_projected_side_px"], reverse=True)[:6],
                        "samples": samples})
    return {"run": str(run), "entry": experiment["entry_id"], "seed": experiment["seed"],
            "scope": "offline S counterfactual at unchanged saved XY/time and roll/pitch, before first discovery only",
            "limitations": ["No camera pixels rendered or decoded; no occlusion, PSF, exposure, network or hardware calibration verified.",
                            "Different gaze/height changes closed-loop dynamics, estimator covariance and surface measurements; not a feasible trajectory or detection guarantee.",
                            "Entry-relative translation was not changed and no track waiting station/vehicle phase policy was introduced.",
                            "Lower height changes vertical framing, not optical magnification at unchanged horizontal depth."],
            "live_input_connection": False, "size_gate_px": gate, "baseline_projection_crosscheck": "pass",
            "source_sha256": {p: sha(run / p) for p in ("experiment.json", "truth.jsonl", "commands.jsonl", "camera_frames.jsonl")},
            "reference_geometry_sha256": sha(geometry_name), "analysis_sha256": sha(__file__), "variants": reports}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True)
    parser.add_argument("--geometry-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    # Preserve both new analysis and imported original helper bytes before work.
    for script in (__file__, Path(__file__).with_name("analyze_mission3_native_geometry.py")):
        shutil.copyfile(script, args.out / Path(script).name)
    tasks = [(r, str(args.geometry_dir / (Path(r).name + ".json"))) for r in args.run]
    (args.out / "plan.json").write_text(json.dumps({"tasks": tasks, "variants": VARIANTS, "max_workers": 4,
         "scope": "read-only geometry diagnosis; no new flight/vision input/entry progression"}, indent=2) + "\n")
    with ProcessPoolExecutor(max_workers=min(4, len(tasks)), mp_context=multiprocessing.get_context("spawn")) as pool:
        results = list(pool.map(analyze, tasks))
    summaries = []
    for result in results:
        (args.out / (Path(result["run"]).name + ".json")).write_text(json.dumps(result, indent=2) + "\n")
        summaries.append({**{k: v for k, v in result.items() if k != "variants"},
            "variants": [{k: v for k, v in x.items() if k not in ("samples", "largest_candidates")} for x in result["variants"]]})
    (args.out / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    print(json.dumps([{k: x[k] for k in ("seed", "variants")} for x in summaries], indent=2))


if __name__ == "__main__":
    main()
