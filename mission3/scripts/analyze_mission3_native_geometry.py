#!/usr/bin/env python3
"""Post-run projection diagnostics only: truth is never a live input."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import multiprocessing
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from aruco_pose import pinhole_matrix
from mission3.scene import R_HEADING_CAMERA, R_VEHICLE_MARKER
from mission3.track import rotation
from yaw_core import rotation_rpy


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def analyze(run_name):
    run = Path(run_name)
    experiment = json.loads((run / 'experiment.json').read_text())
    cfg, physical = experiment['mission'], experiment['physical']
    truth = rows(run / 'truth.jsonl')
    frames = rows(run / 'camera_frames.jsonl')
    observations = rows(run / 'observations.jsonl')
    ts = np.array([x['t'] for x in truth])
    times = np.array([x['t'] for x in frames])
    if np.any(times < ts[0]) or np.any(times > ts[-1] + 1e-8):
        raise ValueError('capture time outside saved pose history')
    times = np.clip(times, ts[0], ts[-1])
    body_q = np.array([x['aircraft']['quaternion_wxyz'] for x in truth])[:, [1, 2, 3, 0]]
    rotations = Slerp(ts, Rotation.from_quat(body_q))(times).as_matrix()
    interpolate = lambda array: np.column_stack([np.interp(times, ts, array[:, i]) for i in range(array.shape[1])])
    positions = interpolate(np.array([x['aircraft']['center_world_m'] for x in truth]))
    pads = interpolate(np.array([x['pad']['center_world_m'] for x in truth]))
    headings = np.interp(times, ts, np.unwrap([x['vehicle']['heading_rad'] for x in truth]))
    mounts = physical['mounts']
    Rbc = rotation_rpy(*np.deg2rad(mounts['camera_rpy_offset_deg'])) @ R_HEADING_CAMERA
    optics = cfg['camera'].get('optics_sensitivity', {})
    K = np.asarray(optics.get('K', pinhole_matrix(320, 320, cfg['camera']['vertical_fov_deg'])), float)
    D = np.asarray(optics.get('distortion', [0.] * 5), float)
    half = cfg['marker']['code_side_m'] / 2
    local = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    offset = np.array(cfg['marker']['known_marker_to_pad_vehicle_m'])
    decoded = {round(x['capture_stamp_s'], 7): x for x in observations if x.get('corners') is not None}
    report = []
    for index, (time, position, pad, Rwb, heading) in enumerate(zip(times, positions, pads, rotations, headings)):
        Rwv = rotation(heading)
        center = pad + Rwv @ offset
        camera = position + Rwb @ mounts['camera_position_body_m']
        Rwm = Rwv @ R_VEHICLE_MARKER
        corners = local @ Rwm.T + center
        camera_corners = (corners - camera) @ (Rwb @ Rbc)
        normal = Rwm[:, 2]
        front = float(normal @ (camera - center)) > 0
        positive = bool(np.all(camera_corners[:, 2] > .001))
        uv = cv2.projectPoints(camera_corners, np.zeros(3), np.zeros(3), K, D)[0].reshape(4, 2) if positive else None
        side = float(np.linalg.norm(uv - np.roll(uv, 1, axis=0), axis=1).min()) if positive else None
        full = bool(positive and np.all(uv >= 3) and np.all(uv <= 316))
        depth = float(((center - camera) @ (Rwb @ Rbc))[2])
        observation = decoded.get(round(float(time), 7))
        report.append({'index': index, 'capture_s': float(time), 'front_face': front,
                       'positive_depth': positive, 'whole_code_in_frame_3px_margin': full,
                       'min_projected_side_px': side, 'center_depth_m': depth,
                       'published_focus_envelope': depth >= cfg['camera']['published_depth_of_field_near_m'],
                       'decoded_target': observation is not None,
                       'observed_min_side_px': observation.get('min_side_px') if observation else None,
                       'projection_corners_px': uv.tolist() if uv is not None else None})
    candidates = [x for x in report if x['front_face'] and x['whole_code_in_frame_3px_margin'] and x['published_focus_envelope']]
    min_side = cfg['observer']['min_side_px']
    usable_geometry = [x for x in candidates if x['min_projected_side_px'] >= min_side]
    result = {
        'run': str(run), 'scope': 'offline truth projection diagnostic only; SO3/linear interpolated capture poses; no live connection',
        'not_detection_guarantee': 'front-facing whole projected code excludes no occlusion, blur, exposure, reflectance or decoder effects',
        'source_sha256': {p: sha(run / p) for p in ('experiment.json', 'truth.jsonl', 'camera_frames.jsonl',
                                                   'observations.jsonl', 'camera-serialization-validation.json')},
        'analysis_source_sha256': sha(__file__), 'frames': len(frames),
        'front_fully_projected_focus_frames': len(candidates),
        'same_geometry_min_side_acquisition_gate_frames': len(usable_geometry),
        'live_min_side_gate_px': min_side,
        'max_min_side_in_candidate_px': max((x['min_projected_side_px'] for x in candidates), default=None),
        'target_decoded_frames': sum(x['decoded_target'] for x in report),
        'largest_candidates': sorted(candidates, key=lambda x: x['min_projected_side_px'], reverse=True)[:8],
        'samples': report}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='append', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    with ProcessPoolExecutor(max_workers=min(4, len(args.run)), mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(analyze, args.run))
    summaries = []
    for result in results:
        name = Path(result['run']).name
        (args.out / (name + '.json')).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        summaries.append({k: v for k, v in result.items() if k not in ('samples', 'largest_candidates')})
    (args.out / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps([{k: r[k] for k in ('run', 'frames', 'same_geometry_min_side_acquisition_gate_frames',
                                      'max_min_side_in_candidate_px', 'target_decoded_frames')} for r in results], indent=2))


if __name__ == '__main__':
    main()
