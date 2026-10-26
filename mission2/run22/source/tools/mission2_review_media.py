"""Offline MuJoCo replay worker. Never constructs CrazySimMuJoCo/DroneAgent or steps physics.

Run by render_mission2_review.py using the launcher's simulation interpreter.
stdin: GT row indices; stdout: one readiness JSON line, then packed RGB images.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_dynamic_model(model):
    """The legacy pose-only log is sufficient ONLY for this static M2 topology."""
    body = int(model.body('cf0_drone').id)
    if (model.nq != 7 or model.nv != 6 or model.njnt != 1 or model.nmocap != 0
            or int(model.jnt_type[0]) != 0 or int(model.jnt_bodyid[0]) != body):
        raise ValueError('pose-only replay refused: unrecorded joint/mocap state')
    if getattr(model, 'nflex', 0) or getattr(model, 'nskin', 0):
        raise ValueError('pose-only replay refused: deformable scene')
    # There must be no dynamic body outside the one recorded drone tree.
    for index in range(1, model.nbody):
        ancestor = index
        while ancestor not in (0, body):
            ancestor = int(model.body_parentid[ancestor])
        if ancestor != body:
            raise ValueError('pose-only replay refused: non-drone body not independently recorded')
    camera = model.camera('cf0_fpv_cam')
    if int(model.cam_mode[camera.id]) != 0 or int(model.cam_bodyid[camera.id]) != body:
        raise ValueError('FPV camera is not rigidly attached to recorded drone')
    return body, int(camera.id)


def rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project_marker(camera, position, width, height):
    """Project GT origin using the renderer's actual averaged OpenGL camera."""
    delta = np.asarray(position)-camera.pos
    depth = float(delta@camera.forward)
    if depth <= 0:
        return None
    vertical = camera.frustum_top-camera.frustum_bottom
    horizontal = vertical*width/height
    right = np.cross(camera.forward, camera.up)
    x = camera.frustum_near*float(delta@right)/depth
    y = camera.frustum_near*float(delta@camera.up)/depth
    return [float(width*(.5+(x-camera.frustum_center)/horizontal)),
            float(height*(camera.frustum_top-y)/vertical)]


def asset_inventory(xml):
    """Hash resolved assets; optional preflight evidence is checked separately."""
    xml = Path(xml).resolve()
    root = ET.parse(xml).getroot()
    compiler = root.find('compiler')
    dirs = {} if compiler is None else compiler.attrib
    files = [xml]
    for item in root.iter():
        name = item.get('file')
        if not name:
            continue
        folder = dirs.get('meshdir' if item.tag == 'mesh' else 'texturedir', '')
        target = xml.parent / folder / name
        files.append(target.resolve())
        if item.tag == 'include' and target.exists():
            files.extend(Path(p) for p in asset_inventory(target))
    return {str(p): digest(p) if p.is_file() else None for p in set(files)}


def verify_preflight_assets(manifest, baseline, actual_assets, trial_started_monotonic):
    """Fail closed unless ALL used baseline assets match a declared pretrial manifest."""
    if manifest is None:
        return dict(status='not_provided', verified=False)
    baseline = Path(baseline).resolve()
    if not isinstance(manifest, dict) or not isinstance(manifest.get('baseline'), str):
        raise ValueError('preflight assets: manifest requires baseline, captured_monotonic and sha256')
    if Path(manifest['baseline']).resolve() != baseline:
        raise ValueError('preflight assets: baseline mismatch')
    captured = manifest.get('captured_monotonic')
    if (isinstance(captured, bool) or not isinstance(captured, (int, float))
            or not math.isfinite(captured) or isinstance(trial_started_monotonic, bool)
            or not isinstance(trial_started_monotonic, (int, float))
            or not math.isfinite(trial_started_monotonic)
            or not 0 <= captured < trial_started_monotonic):
        raise ValueError('preflight assets: captured_monotonic must precede trial.started_monotonic')
    hashes = manifest.get('sha256')
    if not isinstance(hashes, dict):
        raise ValueError('preflight assets: sha256 must map absolute baseline asset paths to hashes')
    normalized = {}
    for path, expected in hashes.items():
        target = Path(path).resolve()
        if (not Path(path).is_absolute() or not target.is_relative_to(baseline)
                or not isinstance(expected, str) or len(expected) != 64
                or any(c not in '0123456789abcdefABCDEF' for c in expected)):
            raise ValueError(f'preflight assets: invalid baseline path or SHA256: {path}')
        key, value = str(target), expected.lower()
        if key in normalized and normalized[key] != value:
            raise ValueError(f'preflight assets: conflicting aliases: {path}')
        normalized[key] = value
    required = {str(Path(p).resolve()): value for p, value in actual_assets.items()
                if Path(p).resolve().is_relative_to(baseline)}
    if not required:
        raise ValueError('preflight assets: no used baseline assets found')
    missing = sorted(p for p in required if p not in normalized or required[p] is None)
    mismatches = sorted(p for p in required if p in normalized and required[p] is not None
                        and required[p] != normalized[p])
    if missing or mismatches:
        raise ValueError(f'preflight assets: missing={missing}; mismatch={mismatches}')
    return dict(status='verified_pretrial_manifest', verified=True, baseline=str(baseline),
        captured_monotonic=captured, trial_started_monotonic=trial_started_monotonic,
        verified_asset_count=len(required), verified_assets_sha256=required,
        unused_manifest_paths=sorted(set(normalized)-set(required)),
        timing_evidence='declared same-host monotonic capture before recorded trial start; not a signed capture-time attestation')


def build_replay(job):
    import mujoco
    baseline = Path(job['baseline']).resolve()
    runtime = Path(job['runtime_log_dir']).resolve()
    provenance = json.loads((runtime/'evaluation/provenance.json').read_text())
    checked = {}
    for relative, expected in provenance['sha256'].items():
        source = baseline/relative
        actual = digest(source)
        checked[str(source)] = actual
        if expected != actual:
            raise ValueError(f'baseline provenance mismatch: {relative}')
    if provenance.get('evidence') != 'native_instrumented_source':
        raise ValueError('external replay requires instrumented native source provenance')
    scene = runtime/'arena/scene.xml'
    manifest = json.loads((runtime/'arena/arena_manifest.json').read_text())
    if manifest['config']['scenario'] != 'mission2':
        raise ValueError('pose-only replay supports static mission2 only')
    if digest(scene) != manifest['scene_sha256']:
        raise ValueError('recorded scene hash differs from arena manifest')
    layout=manifest.get('evaluation_layout')
    if job.get('evaluation_setup_present') and layout not in ('mirror_y','reshuffled_722026','reshuffled_722026_mirror','reshuffled_822026'):
        raise ValueError('custom evaluation_setup is not covered by static M2 reconstruction')
    if layout:
        variant=json.loads((runtime/'arena/evaluation_variant.json').read_text())
        if variant['setup'].get('mission2_layout')!=layout or variant['removed_geoms'] or variant['scene_sha256']!=digest(scene):
            raise ValueError('declared static layout does not match the recorded scene')
    sim_path = baseline/'crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco'
    sys.path.insert(0, str(sim_path))
    # Only import and call the pure model builder, never simulator constructors.
    import crazysim
    model_path = sim_path/crazysim.MODEL_PATHS[crazysim.DEFAULT_MODEL_TYPE]
    model = crazysim._build_spec_multi_safe(str(model_path), [(0, 0)],
        crazysim.MOTOR_PARAMS[crazysim.DEFAULT_MODEL_TYPE], str(scene))
    body, camera = check_dynamic_model(model)
    data = mujoco.MjData(model)
    # Match native scene initialization; no arena build or physics execution.
    spawn = manifest['config']
    yaw = spawn['spawn_yaw_rad']
    model.qpos0[:] = [*spawn['spawn_xyz'], np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    rows = job['truth']
    maxima = dict(drone_position_m=0., drone_rotation=0., all_body_position_m=0.,
                  all_body_rotation=0., fpv_position_m=0., fpv_rotation=0.)
    fpv_samples = []

    def apply(row):
        p = np.asarray(row['drone']['position_world_m'], float)
        q = np.asarray(row['drone']['quaternion_wxyz'], float)
        if (p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all()
                or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-7):
            raise ValueError('invalid recorded drone pose; normalization is not invented')
        data.time = row['sim_time_s']
        data.qpos[:] = np.r_[p, q]
        # No mj_step and no integration/interpolation. qvel not needed for transforms.
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_camlight(model, data)
        return p, rotation(q)

    # Independently compose every rigid body and FPV transform at EVERY GT row.
    # This validates kinematic reconstruction, not pixel equivalence to noisy Mono8.
    for index, row in enumerate(rows):
        p, r = apply(row)
        maxima['drone_position_m'] = max(maxima['drone_position_m'], float(np.max(abs(data.xpos[body]-p))))
        maxima['drone_rotation'] = max(maxima['drone_rotation'], float(np.max(abs(data.xmat[body].reshape(3, 3)-r))))
        expected = {0: (np.zeros(3), np.eye(3)), body: (p, r)}
        for b in range(1, model.nbody):
            if b == body:
                continue
            pp, rr = expected[int(model.body_parentid[b])]
            ep, er = pp+rr@model.body_pos[b], rr@rotation(model.body_quat[b])
            expected[b] = ep, er
            maxima['all_body_position_m'] = max(maxima['all_body_position_m'], float(np.max(abs(data.xpos[b]-ep))))
            maxima['all_body_rotation'] = max(maxima['all_body_rotation'], float(np.max(abs(data.xmat[b].reshape(3, 3)-er))))
        cp, cr = p+r@model.cam_pos[camera], r@rotation(model.cam_quat[camera])
        maxima['fpv_position_m'] = max(maxima['fpv_position_m'], float(np.max(abs(data.cam_xpos[camera]-cp))))
        maxima['fpv_rotation'] = max(maxima['fpv_rotation'], float(np.max(abs(data.cam_xmat[camera].reshape(3, 3)-cr))))
        if index in (0, len(rows)-1):
            fpv_samples.append(dict(gt_index=index, position_world_m=cp.tolist(), rotation_world=cr.tolist()))
    if max(maxima.values()) > 1e-8:
        raise ValueError(f'body/camera kinematic reconstruction failed: {maxima}')
    assets = dict(asset_inventory(scene), **asset_inventory(model_path))
    params = sim_path/'drone-models/drone_models/data/params.toml'
    assets[str(params)] = digest(params)
    if 'preflight_assets' in job and not isinstance(job['preflight_assets'], dict):
        raise ValueError('preflight assets: manifest must be a JSON object')
    preflight = verify_preflight_assets(job.get('preflight_assets'), baseline, assets,
                                       job.get('trial_started_monotonic'))
    if job.get('preflight_assets') is not None:
        preflight.update(manifest_path=job['preflight_assets_path'],
                         manifest_sha256=job['preflight_assets_sha256'])
    report = dict(schema=1, status='verified_static_mission2_pose_reconstruction',evaluation_layout=layout,
        evidence='REPLAY - independently rendered evaluation; never a native external observation',
        mujoco_version=mujoco.__version__, interpreter=sys.executable,
        nq=model.nq, nv=model.nv, njnt=model.njnt, nmocap=model.nmocap,
        bodies=[model.body(i).name for i in range(model.nbody)],
        dynamic_body='cf0_drone', fixed_children=4, fpv_camera='cf0_fpv_cam',
        validated_gt_rows=len(rows), max_absolute_transform_errors=maxima,
        fpv_transform_examples=fpv_samples, recorded_scene_sha256=digest(scene),
        baseline_provenance_verified=checked, current_assets_sha256=assets,
        preflight_assets_verification=preflight,
        limitations=['qpos reconstructed only from recorded body origin position and WXYZ attitude',
            'static M2 only; additional joints, mocap, bodies or deformables are rejected',
            'no interpolation or physics stepping; preceding GT sample held on host timeline',
            'camera/body transforms verified algebraically, not against separately logged camera poses',
            ('used baseline assets match the supplied pretrial manifest; temporary changes during flight cannot be excluded'
             if preflight['verified'] else 'drone XML/mesh/params hashes were not retained at flight time; current assets hashed at export'),
            'unrecorded LED/material/light changes use model defaults; native pixel equivalence not claimed',
            'renderer version at capture not recorded; replay version shown here'],
        free_camera=dict(azimuth=135., elevation=-24., distance=1.7,
                         lookat_offset_world_m=[0., 0., .06]),
        screen_marker='GT drone origin projected through actual averaged renderer camera; external overlay only, may indicate occluded body')
    return mujoco, model, data, apply, report


def serve(job, report_path):
    with contextlib.redirect_stdout(sys.stderr):
        mujoco, model, data, apply, report = build_replay(job)
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, job['width'])
        model.vis.global_.offheight = max(model.vis.global_.offheight, job['height'])
        renderer = mujoco.Renderer(model, height=job['height'], width=job['width'])
    Path(report_path).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(ready=True)), flush=True)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    for key in ('azimuth', 'elevation', 'distance'):
        setattr(camera, key, report['free_camera'][key])
    try:
        for line in sys.stdin:
            index = int(line)
            p, _ = apply(job['truth'][index])
            camera.lookat[:] = p+np.array(report['free_camera']['lookat_offset_world_m'])
            renderer.update_scene(data, camera=camera)
            average = mujoco.mjv_averageCamera(renderer.scene.camera[0], renderer.scene.camera[1])
            marker = project_marker(average, p, job['width'], job['height'])
            sys.stdout.buffer.write((json.dumps(dict(gt_index=index, marker_px=marker))+'\n').encode())
            sys.stdout.buffer.write(renderer.render().tobytes())
            sys.stdout.buffer.flush()
    finally:
        renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    os.environ.setdefault('MUJOCO_GL', 'egl')
    serve(json.loads(args.job.read_text()), args.report)


if __name__ == '__main__':
    main()
