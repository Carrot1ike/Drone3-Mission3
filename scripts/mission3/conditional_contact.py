#!/usr/bin/env python3
"""Mission3 conditional contact/support fixture (MuJoCo, sensitivity only).

What it does
    A passive rigid body sized from the Crazyflie 2.1 datasheet (92 x 92 x 29 mm)
    with the derived conditional total mass 35 g (29 g + AI-deck 1.1 4.4 g +
    Flow deck v2 1.6 g) is lowered by an ideal vertical support force onto a
    0.20 m pad that keeps moving at 0.25 m/s.  At a pre-scheduled instant the
    support force is removed (motor force 0).  An evaluator classifies the next
    3 s as hold / hold_after_slip / slip / partial_support / no_pad_support /
    obstacle_contact / hull_contact / tipover / floor_contact.

What it does not do
    No flight control, propellers, motor dynamics, aerodynamics, ground effect,
    onboard IMU/contact sensing or deck geometry.  Contact truth is read only by
    the evaluator.  Uniform-box inertia, foot families, friction and contact
    softness are sensitivity settings.  No weld/equality, mocap, aircraft
    actuator or state write after t=0 is used.  Every result is a conditional
    sensitivity check, never manufacturer-matched dynamics, mission landing or
    entry advancement.

Public API
    build_model(case=None, out_dir=None, config=None) -> dict
        keys: xml, xml_path, params, derived, source
    run_fixture(case=None, out_dir=None, config=None, record_trace=False,
                record_video=False) -> dict
        keys: case_id, classification, success_like, flags, metrics, params,
              derived, artifacts, source, claims, runtime_s
    run_matrix(config=None, out_root=None, matrices=("core", "extended"),
               representative=True) -> dict
        keys: out_dir, artifacts, counts, n_cases, source, claims

CLI (project root)
    PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python \
        scripts/mission3/conditional_contact.py --matrix core extended
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
import os
import platform
import sys
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = Path(__file__).resolve()
DEFAULT_CONFIG = ROOT / "config" / "mission3-contact-sensitivity.json"
ARTIFACT_ROOT = ROOT / "artifacts" / "mission3" / "contact-fixtures"
CASE_KEYS = ("placement", "init_world_vx_m_s", "foot_family", "inertia_scale",
             "friction_sliding", "solref_timeconst_s", "solref_dampratio",
             "touchdown_vz_m_s", "cut_delay_s", "com_height_m", "mass_kg", "drop_gap_m")
FOOT_NAMES = ("foot_fl", "foot_fr", "foot_rl", "foot_rr")
FOOT_SIGNS = ((1, 1), (1, -1), (-1, 1), (-1, -1))   # body x forward, y left
ROLE_OTHER, ROLE_FLOOR, ROLE_PAD, ROLE_COLUMN, ROLE_BOARD, ROLE_HULL = 0, 1, 2, 3, 4, 5
ROLE_FOOT0 = 10


def _mujoco():
    """Import MuJoCo lazily so the CLI can select MUJOCO_GL before import."""
    import mujoco
    return mujoco


def _rel(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _py(obj):
    """Convert numpy scalars/arrays for JSON."""
    if isinstance(obj, dict):
        return {k: _py(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_py(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _py(obj.tolist())
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


def load_config(config=None):
    """Return (config_dict, config_path_or_None)."""
    if isinstance(config, dict):
        return copy.deepcopy(config), None
    path = Path(config) if config else DEFAULT_CONFIG
    return json.loads(path.read_text()), path


def source_info(cfg, cfg_path=None):
    mujoco = _mujoco()
    info = {"module": _rel(MODULE_PATH), "module_sha256": sha256_file(MODULE_PATH),
            "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
            "python": platform.python_version()}
    if cfg_path is not None:
        info["config"] = _rel(cfg_path)
        info["config_sha256"] = sha256_file(cfg_path)
    else:
        info["config_inline_sha256"] = hashlib.sha256(
            json.dumps(cfg, sort_keys=True).encode()).hexdigest()
    return info


def claims(cfg):
    out = dict(cfg["claims"])
    out.update({"manufacturer_matched": False, "mission_landing": False,
                "entry_advancement": False, "physical_flight": False,
                "contact_truth_used_by_controller": False})
    return out


def _case_id(p):
    return ("{placement}_vx{init_world_vx_m_s:g}_{foot_family}_I{inertia_scale:g}_mu{friction_sliding:g}"
            "_tc{solref_timeconst_s:g}_dr{solref_dampratio:g}_vz{touchdown_vz_m_s:g}"
            "_cd{cut_delay_s:g}_com{com_height_m:g}_m{mass_kg:g}_gap{drop_gap_m:g}").format(**p)


def resolve_case(case=None, cfg=None):
    """Merge a partial case into config nominal_case and validate it."""
    if cfg is None:
        cfg, _ = load_config()
    case = dict(case or {})
    params = copy.deepcopy(cfg["nominal_case"])
    label = case.pop("id", None)
    case.pop("case_id", None)
    case.pop("matrix", None)
    for key, value in case.items():
        if key not in CASE_KEYS:
            raise KeyError(f"unknown case key: {key}")
        params[key] = value
    if params["placement"] not in cfg["fixture"]["placements"]:
        raise ValueError(f"unknown placement {params['placement']}")
    if params["foot_family"] not in cfg["aircraft"]["foot_families"]:
        raise ValueError(f"unknown foot family {params['foot_family']}")
    if params["touchdown_vz_m_s"] >= 0:
        raise ValueError("touchdown_vz_m_s must be negative (descending)")
    if params["inertia_scale"] <= 0 or params["mass_kg"] <= 0:
        raise ValueError("mass and inertia scale must be positive")
    if params["cut_delay_s"] < 0:
        raise ValueError("cut_delay_s must be >= 0 (support cut is never before scheduled contact)")
    params = {k: (float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else v)
              for k, v in params.items()}
    params["case_id"] = label or _case_id(params)
    return params


def derive(params, cfg):
    """All derived quantities, computed before simulation from params/config only."""
    a, s, f = cfg["aircraft"], cfg["scene"], cfg["fixture"]
    m = params["mass_kg"]
    lx, ly, lz = a["envelope_m"]["value"]
    base = np.array([m * (ly ** 2 + lz ** 2) / 12.0, m * (lx ** 2 + lz ** 2) / 12.0,
                     m * (lx ** 2 + ly ** 2) / 12.0])
    inertia = base * params["inertia_scale"]
    r_foot = a["foot_radius_m"]["value"]
    fxy = a["foot_families"][params["foot_family"]]["xy_m"]
    outer = fxy + r_foot
    pad = s["pad"]
    hl, hw = pad["length_m"] / 2.0, pad["width_m"] / 2.0
    margin, clearance = f["near_edge_margin_m"], f["off_pad_clearance_m"]
    rel_xy = {"center": (0.0, 0.0),
              "near_edge_lateral": (0.0, hw - outer - margin),
              "near_edge_rear": (-(hl - outer - margin), 0.0),
              "edge_straddle": (0.0, hw),
              "off_pad": (0.0, hw + outer + clearance)}[params["placement"]]
    vz, gap = params["touchdown_vz_m_s"], params["drop_gap_m"]
    t_contact = gap / abs(vz)
    t_cut = t_contact + params["cut_delay_s"]
    v_pad = s["cart"]["speed_m_s"]
    vx0 = params["init_world_vx_m_s"]
    com_h = params["com_height_m"]
    x0 = rel_xy[0] + (v_pad - vx0) * t_contact
    z0 = pad["top_height_m"] + com_h + gap
    return {"inertia_uniform_box_kg_m2": base.tolist(), "inertia_kg_m2": inertia.tolist(),
            "foot_xy_m": fxy, "foot_radius_m": r_foot, "foot_outer_extent_m": outer,
            "com_height_m": com_h,
            "feet_body_xy_m": [[sx * fxy, sy * fxy] for sx, sy in FOOT_SIGNS],
            "placement_rel_xy_at_contact_m": list(rel_xy),
            "t_contact_scheduled_s": t_contact, "t_cut_s": t_cut,
            "duration_s": t_cut + f["hold_window_s"],
            "initial_world_com_m": [x0, rel_xy[1], z0],
            "initial_world_velocity_m_s": [vx0, 0.0, vz],
            "pad_speed_m_s": v_pad, "support_force_N": m * cfg["simulation"]["gravity_m_s2"],
            "pad_half_m": [hl, hw], "pad_top_m": pad["top_height_m"],
            "cut_uses_contact_truth": False}


def _xml(params, d, cfg):
    a, s, c, sim = cfg["aircraft"], cfg["scene"], cfg["contact"], cfg["simulation"]
    hull = a["hull"]
    h = params["com_height_m"]
    bz = lambda height: height - h   # body frame origin is the CoM
    mu = params["friction_sliding"]
    fr = f"{mu:g} {c['friction_torsional']:g} {c['friction_rolling']:g}"
    solref = f"{params['solref_timeconst_s']:g} {params['solref_dampratio']:g}"
    solimp = " ".join(f"{v:g}" for v in c["solimp"])
    air = f'contype="2" conaffinity="5" condim="{c["condim"]}" friction="{fr}" solref="{solref}" solimp="{solimp}"'
    env = f'contype="4" conaffinity="2" condim="{c["condim"]}" friction="{fr}" solref="{solref}" solimp="{solimp}"'
    r = d["foot_radius_m"]
    geoms = []
    for name, (x, y) in zip(FOOT_NAMES, d["feet_body_xy_m"]):
        geoms.append(f'<geom name="{name}" type="sphere" size="{r:g}" pos="{x:g} {y:g} {bz(r):g}" {air} rgba="0.9 0.2 0.1 1"/>')
    pcb, bat, col, arm = hull["pcb_box"], hull["battery_block"], hull["motor_columns"], hull["arms"]
    z0, z1 = pcb["z_range_m"]
    geoms.append(f'<geom name="hull_pcb" type="box" size="{pcb["half_xy_m"][0]:g} {pcb["half_xy_m"][1]:g} {(z1 - z0) / 2:g}" pos="0 0 {bz((z0 + z1) / 2):g}" {air} rgba="0.2 0.5 0.2 1"/>')
    z0, z1 = bat["z_range_m"]
    geoms.append(f'<geom name="hull_battery" type="box" size="{bat["half_xy_m"][0]:g} {bat["half_xy_m"][1]:g} {(z1 - z0) / 2:g}" pos="0 0 {bz((z0 + z1) / 2):g}" {air} rgba="0.3 0.3 0.8 1"/>')
    rc = col["radius_m"]
    for k, (x, y) in enumerate(d["feet_body_xy_m"]):
        geoms.append(f'<geom name="hull_motor_{k}" type="capsule" size="{rc:g}" fromto="{x:g} {y:g} {bz(col["z_bottom_surface_m"] + rc):g} {x:g} {y:g} {bz(col["z_top_m"]):g}" {air} rgba="0.4 0.4 0.4 1"/>')
        geoms.append(f'<geom name="hull_arm_{k}" type="capsule" size="{arm["radius_m"]:g}" fromto="0 0 {bz(arm["z_m"]):g} {x:g} {y:g} {bz(arm["z_m"]):g}" {air} rgba="0.2 0.5 0.2 1"/>')
    lx, ly, lz = a["envelope_m"]["value"]
    geoms.append(f'<geom name="visual_envelope" type="box" size="{lx / 2:g} {ly / 2:g} {lz / 2:g}" pos="0 0 {bz(lz / 2):g}" contype="0" conaffinity="0" group="3" rgba="0.9 0.9 0.2 0.15"/>')
    ixx, iyy, izz = d["inertia_kg_m2"]
    x0, y0, zc = d["initial_world_com_m"]
    pad, cart, board = s["pad"], s["cart"], s["marker_board"]
    hl, hw = d["pad_half_m"]
    th = pad["thickness_m"]
    top = pad["top_height_m"]
    cz0, cz1 = cart["column_z_range_m"]
    bth = board["thickness_m"]
    bx = hl + board["gap_from_pad_front_edge_m"] + bth / 2
    bzc = top + board["height_m"] / 2
    floor_fr = f"{s['floor']['friction_sliding']:g} {c['friction_torsional']:g} {c['friction_rolling']:g}"
    return f"""<mujoco model="mission3_conditional_contact">
  <compiler angle="radian" inertiafromgeom="auto"/>
  <option timestep="{sim['timestep_s']:g}" integrator="{sim['integrator']}" cone="{c['cone']}" gravity="0 0 {-sim['gravity_m_s2']:g}"/>
  <visual><global offwidth="960" offheight="720"/><headlight ambient="0.3 0.3 0.3" diffuse="0.5 0.5 0.5"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256" rgb1="0.75 0.75 0.75" rgb2="0.6 0.6 0.6"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
  </asset>
  <worldbody>
    <light pos="0 -1 3" dir="0 0.3 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="grid" contype="1" conaffinity="2" condim="{c['condim']}" friction="{floor_fr}" solref="{solref}" solimp="{solimp}"/>
    <body name="cart" pos="0 0 0">
      <joint name="cart_x" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0.2" mass="{cart['mass_kg']:g}" diaginertia="1 1 1"/>
      <geom name="pad" type="box" size="{hl:g} {hw:g} {th / 2:g}" pos="0 0 {top - th / 2:g}" {env} rgba="0.2 0.2 0.25 1"/>
      <geom name="cart_column" type="box" size="{cart['column_half_xy_m']:g} {cart['column_half_xy_m']:g} {(cz1 - cz0) / 2:g}" pos="0 0 {(cz0 + cz1) / 2:g}" {env} rgba="0.5 0.5 0.55 1"/>
      <geom name="marker_board" type="box" size="{bth / 2:g} {board['width_m'] / 2:g} {board['height_m'] / 2:g}" pos="{bx:g} 0 {bzc:g}" {env} rgba="0.95 0.95 0.95 1"/>
      <camera name="side" pos="0 -0.40 0.36" xyaxes="1 0 0 0 0.15 1"/>
      <camera name="rear" pos="-0.40 0 0.36" xyaxes="0 -1 0 0.15 0 1"/>
      <camera name="top" pos="0 0 0.75" xyaxes="1 0 0 0 1 0"/>
    </body>
    <body name="aircraft" pos="{x0:g} {y0:g} {zc:g}">
      <freejoint name="aircraft_free"/>
      <inertial pos="0 0 0" mass="{params['mass_kg']:g}" diaginertia="{ixx:.6e} {iyy:.6e} {izz:.6e}"/>
      {chr(10).join('      ' + g for g in geoms).strip()}
    </body>
  </worldbody>
  <actuator>
    <velocity name="cart_speed" joint="cart_x" kv="{cart['velocity_servo_kv']:g}" ctrlrange="-1 1" forcerange="{-cart['force_limit_N']:g} {cart['force_limit_N']:g}"/>
  </actuator>
</mujoco>
"""


def build_model(case=None, out_dir=None, config=None):
    """Build the MJCF for one case.

    Returns dict(xml, xml_path, params, derived, source).  Writes model.xml when
    out_dir is given.  Only params/config are used; no contact information.
    """
    cfg, cfg_path = load_config(config)
    params = resolve_case(case, cfg)
    derived = derive(params, cfg)
    xml = _xml(params, derived, cfg)
    xml_path = None
    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        xml_path = out_dir / "model.xml"
        xml_path.write_text(xml)
    return {"xml": xml, "xml_path": _rel(xml_path) if xml_path else None,
            "params": params, "derived": _py(derived), "source": source_info(cfg, cfg_path)}


def _ids(model):
    mujoco = _mujoco()
    gid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
    bid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
    jid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    air_body = bid("aircraft")
    air_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == air_body]
    role = np.zeros(model.ngeom, dtype=int)
    role[gid("floor")] = ROLE_FLOOR
    role[gid("pad")] = ROLE_PAD
    role[gid("cart_column")] = ROLE_COLUMN
    role[gid("marker_board")] = ROLE_BOARD
    for g in air_geoms:
        role[g] = ROLE_HULL
    feet = [gid(n) for n in FOOT_NAMES]
    for k, g in enumerate(feet):
        role[g] = ROLE_FOOT0 + k
    role[gid("visual_envelope")] = ROLE_OTHER
    return {"air_body": air_body, "cart_body": bid("cart"), "air_joint": jid("aircraft_free"),
            "cart_joint": jid("cart_x"), "air_geoms": set(air_geoms), "feet": feet, "role": role}


def model_audit(model):
    """Structural checks used by tests and stored with each result."""
    ids = _ids(model)
    air = ids["air_body"]
    return {"n_equality": int(model.neq), "n_mocap": int(model.nmocap), "n_actuators": int(model.nu),
            "aircraft_actuators": int(sum(1 for a in range(model.nu)
                                           if model.actuator_trntype[a] == 0 and model.jnt_bodyid[model.actuator_trnid[a, 0]] == air)),
            "aircraft_mass_kg": float(model.body_mass[air]),
            "aircraft_inertia_kg_m2": model.body_inertia[air].tolist(),
            "n_aircraft_collision_geoms": int(sum(1 for g in ids["air_geoms"] if model.geom_contype[g] or model.geom_conaffinity[g]))}


def _simulate(model, data, params, derived, cfg, record_trace=False, frame_cb=None, frame_dt=0.02):
    mujoco = _mujoco()
    ids = _ids(model)
    air, cart = ids["air_body"], ids["cart_body"]
    va = model.jnt_dofadr[ids["air_joint"]]
    vc = model.jnt_dofadr[ids["cart_joint"]]
    role, air_geoms = ids["role"], ids["air_geoms"]
    dt = model.opt.timestep
    ev, sim = cfg["evaluation"], cfg["simulation"]
    v_pad = derived["pad_speed_m_s"]
    data.qvel[vc] = v_pad                                   # initial condition only
    data.qvel[va:va + 3] = derived["initial_world_velocity_m_s"]
    data.ctrl[0] = v_pad
    mujoco.mj_forward(model, data)
    support = float(model.body_mass[air]) * sim["gravity_m_s2"]
    t_cut = derived["t_cut_s"]
    pad_top = derived["pad_top_m"]
    n_steps = int(round(derived["duration_s"] / dt))
    rec_every = max(1, int(round(sim["record_dt_s"] / dt)))
    frame_every = max(1, int(round(frame_dt / dt)))
    trace = {k: [] for k in ("t", "rel_pos", "rel_vel", "tilt_deg", "feet", "floor", "hull",
                             "normal_force_N", "cart_vx", "support_N", "air_pos", "air_quat")}
    ev_t = {"floor": None, "hull": None, "board": None, "column": None, "first_pad": None}
    first_state = None
    max_pen, max_tilt = 0.0, 0.0
    floor_since, hull_since = False, False
    f6 = np.zeros(6)

    def rel_state():
        p = data.xipos[air] - data.xpos[cart] - np.array([0.0, 0.0, pad_top])
        v = data.qvel[va:va + 3] - np.array([data.qvel[vc], 0.0, 0.0])
        return p.copy(), v.copy()

    for step in range(n_steps):
        t = data.time
        sup = support if t < t_cut - 1e-12 else 0.0
        data.xfrc_applied[air, :] = 0.0
        data.xfrc_applied[air, 2] = sup
        mujoco.mj_step(model, data)
        tn = data.time
        feet_now = [False, False, False, False]
        floor_now = hull_now = False
        ncon = data.ncon
        if ncon:
            try:
                pairs = data.contact.geom.tolist()
            except AttributeError:
                pairs = list(zip(data.contact.geom1.tolist(), data.contact.geom2.tolist()))
            dists = data.contact.dist.tolist()
            for i, (g1, g2) in enumerate(pairs):
                a1, a2 = g1 in air_geoms, g2 in air_geoms
                if not (a1 or a2):
                    continue
                ga, go = (g1, g2) if a1 else (g2, g1)
                ra, ro = role[ga], role[go]
                if -dists[i] > max_pen:
                    max_pen = -dists[i]
                if ro == ROLE_FLOOR:
                    floor_now = True
                    if ev_t["floor"] is None:
                        ev_t["floor"] = tn
                elif ro == ROLE_BOARD:
                    if ev_t["board"] is None:
                        ev_t["board"] = tn
                    if ra == ROLE_HULL:
                        hull_now = True
                elif ro == ROLE_COLUMN:
                    if ev_t["column"] is None:
                        ev_t["column"] = tn
                    if ra == ROLE_HULL:
                        hull_now = True
                elif ro == ROLE_PAD:
                    if ROLE_FOOT0 <= ra < ROLE_FOOT0 + 4:
                        feet_now[ra - ROLE_FOOT0] = True
                    elif ra == ROLE_HULL:
                        hull_now = True
        if hull_now and ev_t["hull"] is None:
            ev_t["hull"] = tn
        floor_since |= floor_now
        hull_since |= hull_now
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(data.xmat[air][8])))))
        max_tilt = max(max_tilt, tilt)
        if ev_t["first_pad"] is None and any(feet_now):
            ev_t["first_pad"] = tn
            p, v = rel_state()
            first_state = {"t_s": tn, "rel_pos_m": p.tolist(), "rel_vel_m_s": v.tolist(),
                           "rel_horizontal_speed_m_s": float(np.hypot(v[0], v[1]))}
        last = step == n_steps - 1
        if step % rec_every == 0 or last:
            p, v = rel_state()
            fsum = 0.0
            for i in range(ncon):
                con = data.contact[i]
                if con.geom1 in air_geoms or con.geom2 in air_geoms:
                    mujoco.mj_contactForce(model, data, i, f6)
                    fsum += f6[0]
            trace["t"].append(tn)
            trace["rel_pos"].append(p.tolist())
            trace["rel_vel"].append(v.tolist())
            trace["tilt_deg"].append(tilt)
            trace["feet"].append(feet_now)
            trace["floor"].append(floor_since)
            trace["hull"].append(hull_since)
            trace["normal_force_N"].append(fsum)
            trace["cart_vx"].append(float(data.qvel[vc]))
            trace["support_N"].append(sup)
            trace["air_pos"].append(data.xipos[air].tolist())
            trace["air_quat"].append(data.xquat[air].tolist())
            floor_since = hull_since = False
        if frame_cb is not None and (step % frame_every == 0 or last):
            frame_cb(data, tn)
    return trace, ev_t, first_state, max_pen, max_tilt


def _metrics(trace, ev_t, first_state, max_pen, max_tilt, derived, cfg):
    ev = cfg["evaluation"]
    t = np.asarray(trace["t"])
    rel = np.asarray(trace["rel_pos"])
    relv = np.asarray(trace["rel_vel"])
    feet = np.asarray(trace["feet"], dtype=bool)
    cart_v = np.asarray(trace["cart_vx"])
    hl, hw = derived["pad_half_m"]
    idx = lambda when: int(np.argmin(np.abs(t - when)))
    t_end = t[-1]
    end_mask = t >= t_end - ev["end_window_s"] - 1e-9
    feet_frac = feet[end_mask].mean(axis=0)
    feet_end = int((feet_frac > 0.5).sum())
    settle_ref = max(derived["t_cut_s"], ev_t["first_pad"] or derived["t_cut_s"]) + ev["settle_s"]
    i_set, i_end = idx(settle_ref), len(t) - 1
    mujoco = _mujoco()
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, np.asarray(trace["air_quat"][-1], dtype=float))
    rot = rot.reshape(3, 3)
    z_foot = derived["foot_radius_m"] - derived["com_height_m"]
    feet_b = np.array([[x, y, z_foot] for x, y in derived["feet_body_xy_m"]])
    feet_rel = rel[i_end] + feet_b @ rot.T
    margins = [min(hl - abs(fx), hw - abs(fy)) for fx, fy, _ in feet_rel]
    creep_mask = t >= t_end - ev["tilt_creep_window_s"] - 1e-9
    tilt = np.asarray(trace["tilt_deg"])
    creep = float(np.polyfit(t[creep_mask], tilt[creep_mask], 1)[0]) if creep_mask.sum() > 2 else 0.0
    touchdown_slip = None
    if first_state is not None:
        touchdown_slip = float(np.hypot(*(rel[i_set, :2] - np.asarray(first_state["rel_pos_m"][:2]))))
    post_slip = float(np.hypot(*(rel[i_end, :2] - rel[i_set, :2])))
    final_speed = float(np.mean(np.hypot(relv[end_mask, 0], relv[end_mask, 1])))
    return {"first_pad_contact": first_state,
            "floor_contact_t_s": ev_t["floor"], "hull_contact_t_s": ev_t["hull"],
            "board_contact_t_s": ev_t["board"], "column_contact_t_s": ev_t["column"],
            "settle_reference_t_s": float(t[i_set]),
            "touchdown_slip_m": touchdown_slip, "post_settle_slip_m": post_slip,
            "final_rel_horizontal_speed_m_s": final_speed,
            "final_rel_pos_m": rel[i_end].tolist(),
            "com_inside_pad_end": bool(abs(rel[i_end, 0]) <= hl and abs(rel[i_end, 1]) <= hw),
            "feet_in_contact_end": feet_end, "feet_contact_fraction_end": feet_frac.tolist(),
            "foot_center_margin_end_m": [float(x) for x in margins],
            "min_foot_center_margin_end_m": float(min(margins)),
            "tilt_rate_end_deg_s": creep,
            "max_tilt_deg": float(max_tilt), "final_tilt_deg": float(trace["tilt_deg"][-1]),
            "max_penetration_m": float(max_pen), "peak_normal_force_N": float(np.max(trace["normal_force_N"])),
            "pad_speed_min_m_s": float(cart_v.min()), "pad_speed_max_m_s": float(cart_v.max()),
            "duration_s": float(t_end)}


def classify(metrics, cfg):
    """Evaluator-only classification from contact truth. Returns (class, flags)."""
    ev = cfg["evaluation"]
    flags = {
        "floor_contact": metrics["floor_contact_t_s"] is not None,
        "tipover": metrics["max_tilt_deg"] > ev["tilt_tipover_deg"],
        "hull_contact": metrics["hull_contact_t_s"] is not None,
        "obstacle_contact": metrics["board_contact_t_s"] is not None or metrics["column_contact_t_s"] is not None,
        "never_contacted_pad": metrics["first_pad_contact"] is None,
        "no_pad_support": metrics["feet_in_contact_end"] == 0,
        "partial_support": (0 < metrics["feet_in_contact_end"] < ev["min_feet_in_contact_end"]
                            or metrics["final_tilt_deg"] > ev["tilt_hold_max_deg"]
                            or not metrics["com_inside_pad_end"]
                            or metrics["min_foot_center_margin_end_m"] < ev["foot_center_margin_min_m"]
                            or abs(metrics["tilt_rate_end_deg_s"]) > ev["tilt_creep_max_deg_s"]),
        "foot_over_edge": metrics["min_foot_center_margin_end_m"] < ev["foot_center_margin_min_m"],
        "tilt_creep": abs(metrics["tilt_rate_end_deg_s"]) > ev["tilt_creep_max_deg_s"],
        "slip": (metrics["post_settle_slip_m"] > ev["post_settle_slip_max_m"]
                 or metrics["final_rel_horizontal_speed_m_s"] > ev["final_rel_speed_max_m_s"]),
        "touchdown_slip": (metrics["touchdown_slip_m"] is not None
                           and metrics["touchdown_slip_m"] > ev["touchdown_slip_hold_max_m"]),
    }
    order = [("floor_contact", "floor_contact"), ("tipover", "tipover"), ("hull_contact", "hull_contact"),
             ("obstacle_contact", "obstacle_contact"), ("no_pad_support", "no_pad_support"),
             ("partial_support", "partial_support"), ("slip", "slip"), ("touchdown_slip", "hold_after_slip")]
    for flag, cls in order:
        if flags[flag]:
            return cls, flags
    return "hold", flags


def _plot(trace, result, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.asarray(trace["t"])
    rel = np.asarray(trace["rel_pos"]) * 1000
    relv = np.asarray(trace["rel_vel"])
    feet = np.asarray(trace["feet"]).sum(axis=1)
    fig, ax = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
    ax[0].plot(t, rel[:, 0], label="rel x"); ax[0].plot(t, rel[:, 1], label="rel y"); ax[0].plot(t, rel[:, 2], label="rel z")
    ax[0].set_ylabel("pad-relative CoM [mm]"); ax[0].legend(loc="best", fontsize=8)
    ax[1].plot(t, np.hypot(relv[:, 0], relv[:, 1]), label="|rel v_xy|"); ax[1].plot(t, relv[:, 2], label="rel v_z")
    ax[1].set_ylabel("[m/s]"); ax[1].legend(loc="best", fontsize=8)
    ax[2].plot(t, trace["tilt_deg"]); ax[2].set_ylabel("tilt [deg]")
    ax[3].step(t, feet, where="post", label="feet on pad"); ax[3].plot(t, np.asarray(trace["normal_force_N"]) / max(1e-9, result["derived"]["support_force_N"]), label="normal force / mg")
    ax[3].plot(t, np.asarray(trace["floor"], dtype=float) * 4.2, "r.", ms=2, label="floor contact")
    ax[3].set_ylabel("count / ratio"); ax[3].set_xlabel("time [s]"); ax[3].legend(loc="best", fontsize=8)
    for a in ax:
        a.axvline(result["derived"]["t_contact_scheduled_s"], color="k", ls=":", lw=0.8)
        a.axvline(result["derived"]["t_cut_s"], color="m", ls="--", lw=0.8)
        a.grid(alpha=0.3)
    fig.suptitle(f"{result['case_id']}\nclass={result['classification']} (conditional sensitivity, not flight/equipment validation)", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def run_fixture(case=None, out_dir=None, config=None, record_trace=False, record_video=False):
    """Run one conditional contact case.

    Returns dict with classification, flags, metrics, params, derived, model audit,
    artifact paths (when out_dir), source (module/config sha256, versions) and
    claims.  The support-force cut time is fixed before simulation; contact truth
    is used only after stepping, by the evaluator.
    """
    mujoco = _mujoco()
    cfg, cfg_path = load_config(config)
    built = build_model(case, out_dir, cfg if cfg_path is None else cfg_path)
    params, derived = built["params"], built["derived"]
    t0 = time.perf_counter()
    model = mujoco.MjModel.from_xml_string(built["xml"])
    data = mujoco.MjData(model)
    audit = model_audit(model)
    frames, frame_cb, video_note = [], None, None
    renderer = None
    if record_video and out_dir is not None:
        try:
            renderer = mujoco.Renderer(model, 360, 480)
            cams = ("side", "rear", "top")

            def frame_cb(dat, tn):
                imgs = []
                for cam in cams:
                    renderer.update_scene(dat, camera=cam)
                    imgs.append(renderer.render().copy())
                frames.append((tn, np.hstack(imgs)))
        except Exception as exc:   # rendering backend unavailable
            video_note = f"video skipped: {type(exc).__name__}: {exc}"
            frame_cb = None
    trace, ev_t, first_state, max_pen, max_tilt = _simulate(model, data, params, derived, cfg,
                                                             record_trace, frame_cb)
    metrics = _metrics(trace, ev_t, first_state, max_pen, max_tilt, derived, cfg)
    cls, flags = classify(metrics, cfg)
    success_like = cls in cfg["evaluation"]["success_like_classes"]
    if flags["floor_contact"] and success_like:
        raise AssertionError("floor contact must never be classified as success")
    result = {"case_id": params["case_id"], "classification": cls, "success_like": success_like,
              "flags": flags, "metrics": metrics, "params": params, "derived": derived,
              "model_audit": audit, "artifacts": {}, "source": built["source"], "claims": claims(cfg),
              "runtime_s": time.perf_counter() - t0}
    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        result["artifacts"]["model_xml"] = built["xml_path"]
        if record_trace:
            tp = out_dir / "trace.json"
            tp.write_text(json.dumps(_py(trace)) + "\n")
            result["artifacts"]["trace_json"] = _rel(tp)
            pp = out_dir / "trace.png"
            _plot(trace, result, pp)
            result["artifacts"]["trace_png"] = _rel(pp)
        if record_video:
            if frames:
                import cv2
                vp = out_dir / "video.mp4"
                h, w = frames[0][1].shape[:2]
                writer = cv2.VideoWriter(str(vp), cv2.VideoWriter_fourcc(*"mp4v"), 25, (w, h))
                for tn, img in frames:
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    phase = "support force on" if tn < derived["t_cut_s"] else "motor force 0"
                    for text, org in ((f"t={tn:5.2f}s {phase} | views: side(x-z) rear(y-z) top(x-y), pad frame", (8, 20)),
                                      (f"{cls} | conditional sensitivity, not flight/equipment validation", (8, h - 10))):
                        cv2.putText(bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                        cv2.putText(bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    writer.write(bgr)
                writer.release()
                result["artifacts"]["video_mp4"] = _rel(vp)
            else:
                result["artifacts"]["video_note"] = video_note or "no frames rendered"
        rp = out_dir / "result.json"
        result["artifacts"]["result_json"] = _rel(rp)
        rp.write_text(json.dumps(_py(result), indent=2) + "\n")
    if renderer is not None:
        renderer.close()
    return _py(result)


def expand_matrix(cfg, names=("core", "extended")):
    """Expand configured matrices into partial case dicts (tagged with matrix name)."""
    cases, seen = [], set()

    def add(case, tag):
        p = resolve_case(case, cfg)
        if p["case_id"] in seen:
            return
        seen.add(p["case_id"])
        cases.append(dict(case, matrix=tag))

    for name in names:
        spec = cfg["matrices"][name]
        if spec["type"] == "product":
            keys = list(spec["factors"])
            for values in itertools.product(*(spec["factors"][k] for k in keys)):
                add(dict(zip(keys, values)), name)
        elif spec["type"] == "one_at_a_time":
            base_keys = list(spec["base_factors"])
            bases = [dict(zip(base_keys, v)) for v in itertools.product(*(spec["base_factors"][k] for k in base_keys))]
            for base in bases:
                for key, values in spec["variations"].items():
                    for value in values:
                        add(dict(base, **{key: value}), name)
            for key, values in spec.get("extra_family", {}).items():
                for base in bases:
                    for value in values:
                        add(dict(base, **{key: value}), name)
        else:
            raise ValueError(f"unknown matrix type {spec['type']}")
    return cases


def _summarize(rows, cfg):
    classes = cfg["evaluation"]["class_priority"]
    count = lambda rs: {c: sum(1 for r in rs if r["classification"] == c) for c in classes}
    out = {"n_cases": len(rows), "overall": count(rows), "by_matrix": {}, "by_placement_vx": {},
           "by_factor": {}, "floor_contact_success_violations": sum(1 for r in rows if r["flags"]["floor_contact"] and r["success_like"])}
    for m in sorted({r["matrix"] for r in rows}):
        out["by_matrix"][m] = count([r for r in rows if r["matrix"] == m])
    for pl in cfg["fixture"]["placements"]:
        for vx in cfg["fixture"]["init_world_vx_m_s"]:
            rs = [r for r in rows if r["params"]["placement"] == pl and r["params"]["init_world_vx_m_s"] == vx]
            if rs:
                slips = [r["metrics"]["touchdown_slip_m"] for r in rs if r["metrics"]["touchdown_slip_m"] is not None]
                out["by_placement_vx"][f"{pl}|vx{vx:g}"] = {
                    "n": len(rs), "classes": count(rs),
                    "success_like_fraction": sum(r["success_like"] for r in rs) / len(rs),
                    "touchdown_slip_m_min_max": [min(slips), max(slips)] if slips else None}
    for key in ("foot_family", "inertia_scale", "friction_sliding", "solref_dampratio", "solref_timeconst_s",
                "touchdown_vz_m_s", "cut_delay_s", "com_height_m"):
        vals = sorted({r["params"][key] for r in rows}, key=lambda v: str(v))
        out["by_factor"][key] = {str(v): count([r for r in rows if r["params"][key] == v]) for v in vals}
    return out


def _summary_plot(summary, cfg, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    classes = cfg["evaluation"]["class_priority"]
    colors = {"floor_contact": "#b2182b", "tipover": "#ef8a62", "hull_contact": "#f7b6a0", "obstacle_contact": "#fddbc7",
              "no_pad_support": "#d6604d", "partial_support": "#f4a582", "slip": "#92c5de",
              "hold_after_slip": "#4393c3", "hold": "#2166ac"}
    keys = list(summary["by_placement_vx"])
    fig, ax = plt.subplots(figsize=(10, 4.5))
    bottom = np.zeros(len(keys))
    for c in classes:
        vals = np.array([summary["by_placement_vx"][k]["classes"][c] for k in keys], dtype=float)
        ax.bar(range(len(keys)), vals, bottom=bottom, color=colors.get(c), label=c)
        bottom += vals
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels([k.replace("|", "\n") for k in keys], fontsize=7)
    ax.set_ylabel("cases (all S/E settings)")
    ax.set_title("Conditional contact sensitivity classes by placement and initial world vx\n(not manufacturer-matched, not mission landing or entry advancement)", fontsize=9)
    ax.legend(fontsize=7, ncol=4, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def run_matrix(config=None, out_root=None, matrices=("core", "extended"), representative=True):
    """Run configured matrices and representative cases; write artifacts."""
    cfg, cfg_path = load_config(config)
    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S")
    out = Path(out_root or ARTIFACT_ROOT) / f"{stamp}-matrix"
    out.mkdir(parents=True, exist_ok=True)
    src = source_info(cfg, cfg_path)
    cases = expand_matrix(cfg, matrices)
    rows, t0 = [], time.perf_counter()
    with open(out / "results.jsonl", "w") as fh:
        for case in cases:
            res = run_fixture(case, None, cfg)
            row = {"matrix": case["matrix"], "case_id": res["case_id"], "classification": res["classification"],
                   "success_like": res["success_like"], "flags": res["flags"], "params": res["params"],
                   "metrics": res["metrics"], "derived": {k: res["derived"][k] for k in (
                       "inertia_kg_m2", "foot_xy_m", "placement_rel_xy_at_contact_m", "t_contact_scheduled_s", "t_cut_s")},
                   "model_audit": res["model_audit"], "runtime_s": res["runtime_s"]}
            rows.append(row)
            fh.write(json.dumps(_py(row)) + "\n")
    summary = _summarize(rows, cfg)
    summary.update({"wall_time_s": time.perf_counter() - t0, "source": src, "claims": claims(cfg),
                    "matrices": list(matrices)})
    (out / "summary.json").write_text(json.dumps(_py(summary), indent=2) + "\n")
    _summary_plot(summary, cfg, out / "summary.png")
    (out / "config-snapshot.json").write_text(json.dumps(cfg, indent=2) + "\n")
    reps = {}
    if representative:
        for rc in cfg["representative_cases"]:
            res = run_fixture(rc, out / "representative" / rc["id"], cfg, record_trace=True, record_video=True)
            reps[rc["id"]] = {"classification": res["classification"], "artifacts": res["artifacts"],
                              "metrics": {k: res["metrics"][k] for k in ("touchdown_slip_m", "post_settle_slip_m",
                                                                         "max_tilt_deg", "feet_in_contact_end",
                                                                         "floor_contact_t_s", "first_pad_contact")}}
    artifacts = {"results_jsonl": _rel(out / "results.jsonl"), "summary_json": _rel(out / "summary.json"),
                 "summary_png": _rel(out / "summary.png"), "config_snapshot": _rel(out / "config-snapshot.json")}
    index = {"out_dir": _rel(out), "artifacts": artifacts, "representative": reps, "n_cases": len(rows),
             "counts": summary["overall"], "source": src, "claims": claims(cfg)}
    (out / "index.json").write_text(json.dumps(_py(index), indent=2) + "\n")
    artifacts["index_json"] = _rel(out / "index.json")
    return _py(index)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--out-root", type=Path, default=None)
    p.add_argument("--matrix", nargs="*", default=["core", "extended"], choices=["core", "extended"])
    p.add_argument("--case", type=str, default=None, help="JSON object for a single case")
    p.add_argument("--out", type=Path, default=None, help="output dir for --case")
    p.add_argument("--no-representative", action="store_true")
    p.add_argument("--gl", choices=["default", "egl", "osmesa", "glfw"], default=None)
    a = p.parse_args(argv)
    if a.gl and a.gl != "default":
        os.environ["MUJOCO_GL"] = a.gl
    if a.case:
        out = a.out or (Path(a.out_root or ARTIFACT_ROOT) / (datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S") + "-case"))
        res = run_fixture(json.loads(a.case), out, a.config, record_trace=True, record_video=True)
        print(json.dumps({k: res[k] for k in ("case_id", "classification", "success_like", "artifacts", "claims")}, indent=2))
        return 0
    res = run_matrix(a.config, a.out_root, tuple(a.matrix), not a.no_representative)
    print(json.dumps({k: res[k] for k in ("out_dir", "n_cases", "counts", "artifacts", "claims")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
