#!/usr/bin/env python3
"""Checks for scripts/mission3/conditional_contact.py (conditional contact sensitivity).

These tests verify the fixture's construction rules (no weld/mocap/teleport, passive
aircraft after the scheduled cut, evaluator-only contact truth, floor contact never
counted as success) and its declared parameters.  Passing them does not mean
manufacturer-matched dynamics, mission landing, entry advancement or equipment
validation.

Run from the project root:
    PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/test_mission3_contact.py
The file is also pytest-compatible.  The runner writes
artifacts/mission3/contact-fixtures/<timestamp>-tests/report.json.
"""
import hashlib
import inspect
import json
import math
import re
import struct
import sys
import time
import traceback
import unittest
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
from mission3 import conditional_contact as cc  # noqa: E402

CFG, CFG_PATH = cc.load_config()
MESH_DIR = Path("/tmp/drone3-evidence/crazyflie-simulation/meshes/stl_files")
_OUT = {}


def _out_dir():
    if "dir" not in _OUT:
        stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S")
        _OUT["dir"] = cc.ARTIFACT_ROOT / f"{stamp}-tests"
        _OUT["dir"].mkdir(parents=True, exist_ok=True)
    return _OUT["dir"]


def _run(case=None, **kw):
    return cc.run_fixture(case, config=CFG_PATH, **kw)


def _sim_trace(case=None):
    mujoco = cc._mujoco()
    params = cc.resolve_case(case, CFG)
    derived = cc.derive(params, CFG)
    model = mujoco.MjModel.from_xml_string(cc._xml(params, derived, CFG))
    data = mujoco.MjData(model)
    trace, ev_t, first, _, _ = cc._simulate(model, data, params, derived, CFG)
    return params, derived, trace, ev_t, first


def test_structure_no_weld_mocap_teleport_or_aircraft_actuator():
    mujoco = cc._mujoco()
    built = cc.build_model(config=CFG_PATH)
    xml = built["xml"]
    for token in ("<weld", "<equality", "mocap=", "<connect"):
        assert token not in xml, token
    audit = cc.model_audit(mujoco.MjModel.from_xml_string(xml))
    assert audit["n_equality"] == 0 and audit["n_mocap"] == 0
    assert audit["n_actuators"] == 1 and audit["aircraft_actuators"] == 0
    src = inspect.getsource(cc._simulate)
    loop = src.index("for step in range(n_steps)")
    writes = [m.start() for m in re.finditer(r"\.(qpos|qvel)\[[^\]]*\]\s*=", src)]
    assert writes and all(w < loop for w in writes), "state writes must happen only before the step loop"
    assert not re.search(r"\.qpos\[[^\]]*\]\s*=", inspect.getsource(cc)), "no qpos writes anywhere"


def test_mass_and_uniform_box_inertia_with_scale():
    mujoco = cc._mujoco()
    src = CFG["sources"]["datasheet_decks"]
    assert abs(0.029 + (src["ai_deck_1_1_g"] + src["flow_deck_v2_g"]) / 1000 - CFG["aircraft"]["mass_kg"]["value"]) < 1e-12
    lx, ly, lz = CFG["aircraft"]["envelope_m"]["value"]
    m = CFG["nominal_case"]["mass_kg"]
    base = np.array([m * (ly**2 + lz**2), m * (lx**2 + lz**2), m * (lx**2 + ly**2)]) / 12
    for scale in CFG["matrices"]["core"]["factors"]["inertia_scale"]:
        b = cc.build_model({"inertia_scale": scale}, config=CFG_PATH)
        model = mujoco.MjModel.from_xml_string(b["xml"])
        audit = cc.model_audit(model)
        assert abs(audit["aircraft_mass_kg"] - m) < 1e-12
        assert np.allclose(b["derived"]["inertia_kg_m2"], base * scale, rtol=1e-12)
        assert np.allclose(audit["aircraft_inertia_kg_m2"], base * scale, rtol=1e-5)


def _lowest_point(model, g):
    """Lowest body-frame z of a sphere/capsule/box geom (exact for these primitives)."""
    mujoco = cc._mujoco()
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.geom_quat[g])
    rz = rot.reshape(3, 3)[2]
    pos, size, gtype = model.geom_pos[g], model.geom_size[g], model.geom_type[g]
    if gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
        return pos[2] - size[0]
    if gtype == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return pos[2] - abs(rz[2]) * size[1] - size[0]
    if gtype == mujoco.mjtGeom.mjGEOM_BOX:
        return pos[2] - float(np.dot(np.abs(rz), size[:3]))
    raise AssertionError(f"unexpected geom type {gtype}")


def test_foot_family_positions_and_foot_plane():
    mujoco = cc._mujoco()
    r = CFG["aircraft"]["foot_radius_m"]["value"]
    for fam, spec in CFG["aircraft"]["foot_families"].items():
        for com in (0.0145, 0.020):
            b = cc.build_model({"foot_family": fam, "com_height_m": com}, config=CFG_PATH)
            model = mujoco.MjModel.from_xml_string(b["xml"])
            for name, (sx, sy) in zip(cc.FOOT_NAMES, cc.FOOT_SIGNS):
                g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                pos = model.geom_pos[g]
                assert np.allclose(pos[:2], [sx * spec["xy_m"], sy * spec["xy_m"]], atol=1e-9)
                assert abs((pos[2] - r) + com) < 1e-9, "foot bottom must lie on the foot plane"
            ids = cc._ids(model)
            lowest_hull = min(_lowest_point(model, g) for g in ids["air_geoms"] if ids["role"][g] == cc.ROLE_HULL)
            assert lowest_hull > -com + 1e-3, "hull must stay above the foot plane"


def test_pad_moves_at_commanded_speed_including_failures():
    for case in ({}, {"placement": "off_pad"}, {"init_world_vx_m_s": 0.0, "cut_delay_s": 0.1}):
        m = _run(case)["metrics"]
        assert abs(m["pad_speed_min_m_s"] - 0.25) < 1e-3 and abs(m["pad_speed_max_m_s"] - 0.25) < 1e-3, m


def test_floor_contact_never_success():
    for case in ({"placement": "off_pad"}, {"placement": "off_pad", "init_world_vx_m_s": 0.0},
                 {"placement": "edge_straddle"}, {"placement": "edge_straddle", "init_world_vx_m_s": 0.0}):
        res = _run(case)
        assert res["flags"]["floor_contact"], case
        assert res["classification"] == "floor_contact" and not res["success_like"], case
    fake = {"floor_contact_t_s": 1.0, "max_tilt_deg": 0.0, "hull_contact_t_s": None, "board_contact_t_s": None,
            "column_contact_t_s": None, "first_pad_contact": None, "feet_in_contact_end": 4, "final_tilt_deg": 0.0,
            "com_inside_pad_end": True, "post_settle_slip_m": 0.0, "final_rel_horizontal_speed_m_s": 0.0,
            "touchdown_slip_m": 0.0, "min_foot_center_margin_end_m": 0.05, "tilt_rate_end_deg_s": 0.0}
    assert cc.classify(fake, CFG)[0] == "floor_contact"


def test_classification_priority_and_hull_contact_semantics():
    base = {"floor_contact_t_s": None, "max_tilt_deg": 0.0, "hull_contact_t_s": None, "board_contact_t_s": None,
            "column_contact_t_s": None, "first_pad_contact": {"t_s": 0.2}, "feet_in_contact_end": 4,
            "final_tilt_deg": 0.0, "com_inside_pad_end": True, "post_settle_slip_m": 0.0,
            "final_rel_horizontal_speed_m_s": 0.0, "touchdown_slip_m": 0.0,
            "min_foot_center_margin_end_m": 0.05, "tilt_rate_end_deg_s": 0.0}
    cases = [({}, "hold"), ({"touchdown_slip_m": 0.02}, "hold_after_slip"), ({"post_settle_slip_m": 0.02}, "slip"),
             ({"feet_in_contact_end": 2}, "partial_support"), ({"min_foot_center_margin_end_m": -0.0004}, "partial_support"),
             ({"tilt_rate_end_deg_s": 0.08}, "partial_support"), ({"feet_in_contact_end": 0}, "no_pad_support"),
             ({"board_contact_t_s": 1.0}, "obstacle_contact"), ({"hull_contact_t_s": 1.0, "max_tilt_deg": 14.0}, "hull_contact"),
             ({"hull_contact_t_s": 1.0, "max_tilt_deg": 60.0}, "tipover"),
             ({"floor_contact_t_s": 2.0, "max_tilt_deg": 60.0, "hull_contact_t_s": 1.0}, "floor_contact")]
    for change, expected in cases:
        assert cc.classify(dict(base, **change), CFG)[0] == expected, (change, expected)
    assert set(CFG["evaluation"]["class_priority"]) == set(CFG["evaluation"]["class_definitions"])


def test_center_matched_velocity_holds_nominal():
    res = _run({})
    assert res["classification"] == "hold", res["classification"]
    assert res["metrics"]["first_pad_contact"]["rel_horizontal_speed_m_s"] < 1e-3


def test_unmatched_velocity_produces_touchdown_slip():
    matched = _run({})["metrics"]["touchdown_slip_m"]
    unmatched = _run({"init_world_vx_m_s": 0.0})
    assert unmatched["metrics"]["first_pad_contact"]["rel_horizontal_speed_m_s"] > 0.2
    assert unmatched["metrics"]["touchdown_slip_m"] > matched + 1e-3
    low = _run({"init_world_vx_m_s": 0.0, "friction_sliding": 0.3})["metrics"]["touchdown_slip_m"]
    high = _run({"init_world_vx_m_s": 0.0, "friction_sliding": 1.0})["metrics"]["touchdown_slip_m"]
    assert low > high


def test_no_teleport_trajectory_continuity():
    for case in ({}, {"init_world_vx_m_s": 0.0}, {"placement": "edge_straddle"}, {"placement": "off_pad"}):
        params, derived, trace, _, _ = _sim_trace(case)
        t = np.asarray(trace["t"])
        p = np.asarray(trace["air_pos"])
        v = np.asarray(trace["rel_vel"]) + np.c_[np.asarray(trace["cart_vx"]), np.zeros((len(t), 2))]
        dt = np.diff(t)
        step = np.linalg.norm(np.diff(p, axis=0), axis=1)
        bound = (np.maximum(np.linalg.norm(v[:-1], axis=1), np.linalg.norm(v[1:], axis=1)) + 9.81 * dt) * dt + 1e-4
        assert np.all(step <= bound), (case, float(np.max(step - bound)))


def test_cut_schedule_is_fixed_before_simulation_and_ignores_contact():
    base = cc.derive(cc.resolve_case({}, CFG), CFG)["t_cut_s"]
    for case in ({"friction_sliding": 0.3}, {"solref_dampratio": 0.5}, {"inertia_scale": 2.0},
                 {"placement": "off_pad"}, {"foot_family": "envelope46"}):
        d = cc.derive(cc.resolve_case(case, CFG), CFG)
        assert abs(d["t_cut_s"] - base) < 1e-15 and d["cut_uses_contact_truth"] is False
    d = cc.derive(cc.resolve_case({"cut_delay_s": 0.05, "touchdown_vz_m_s": -0.2}, CFG), CFG)
    assert abs(d["t_cut_s"] - (0.02 / 0.2 + 0.05)) < 1e-12
    for case in ({}, {"placement": "off_pad"}):
        _, derived, trace, ev_t, _ = _sim_trace(case)
        t = np.asarray(trace["t"])
        sup = np.asarray(trace["support_N"])
        assert np.all(sup[t < derived["t_cut_s"] - 1e-3] > 0) and np.all(sup[t > derived["t_cut_s"] + 1e-3] == 0)


def test_determinism():
    a = _run({"init_world_vx_m_s": 0.0, "friction_sliding": 0.3})
    b = _run({"init_world_vx_m_s": 0.0, "friction_sliding": 0.3})
    assert json.dumps(a["metrics"], sort_keys=True) == json.dumps(b["metrics"], sort_keys=True)


def test_claim_labels_on_every_result():
    for case in ({}, {"placement": "off_pad"}):
        c = _run(case)["claims"]
        assert c["class"] == "conditional_contact_sensitivity"
        for key in ("manufacturer_matched", "mission_landing", "entry_advancement", "physical_flight",
                    "contact_truth_used_by_controller", "onboard_sensing_modeled", "flight_control_modeled"):
            assert c[key] is False, key


def test_artifacts_and_source_sha():
    out = _out_dir() / "artifact_check"
    res = _run({"init_world_vx_m_s": 0.0}, out_dir=out, record_trace=True)
    for key in ("model_xml", "trace_json", "trace_png", "result_json"):
        assert (cc.ROOT / res["artifacts"][key]).exists(), key
    assert res["source"]["module_sha256"] == hashlib.sha256(Path(cc.__file__).read_bytes()).hexdigest()
    assert res["source"]["config_sha256"] == hashlib.sha256(CFG_PATH.read_bytes()).hexdigest()
    b = cc.build_model({}, out_dir=out / "build_only", config=CFG_PATH)
    assert (cc.ROOT / b["xml_path"]).exists() and b["params"]["case_id"]


def test_config_mirrors_mission3_geometry():
    mission = json.loads((ROOT / "config" / "mission3.json").read_text())
    v, s = mission["vehicle_plant_only"], CFG["scene"]
    pairs = [(v["pad_length_m"], s["pad"]["length_m"]), (v["pad_width_m"], s["pad"]["width_m"]),
             (v["pad_height_m"], s["pad"]["top_height_m"]), (v["pad_thickness_m"], s["pad"]["thickness_m"]),
             (v["marker_plane_gap_from_pad_front_edge_m"], s["marker_board"]["gap_from_pad_front_edge_m"]),
             (v["marker_board_width_m"], s["marker_board"]["width_m"]),
             (v["marker_board_height_m"], s["marker_board"]["height_m"]), (v["speed_m_s"], s["cart"]["speed_m_s"])]
    assert all(abs(a - b) < 1e-12 for a, b in pairs), pairs


def test_matrix_expansion():
    core = cc.expand_matrix(CFG, ("core",))
    f = CFG["matrices"]["core"]["factors"]
    assert len(core) == math.prod(len(v) for v in f.values())
    both = cc.expand_matrix(CFG, ("core", "extended"))
    ids = [cc.resolve_case(c, CFG)["case_id"] for c in both]
    assert len(ids) == len(set(ids)) and len(both) > len(core)


def test_official_mesh_evidence_if_available():
    path = MESH_DIR / "4_motormounts.stl"
    if not path.exists():
        raise unittest.SkipTest("official mesh clone not present; evidence values remain from config")
    meta = CFG["sources"]["sim_mesh_stl"]["files"]["meshes/stl_files/4_motormounts.stl"]
    blob = path.read_bytes()
    assert hashlib.sha256(blob).hexdigest() == meta["sha256"]
    n = struct.unpack("<I", blob[80:84])[0]
    rec = np.frombuffer(blob[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    v = rec["v"].reshape(-1, 3).astype(float)
    low = v[v[:, 2] < v[:, 2].min() + 1.5e-3]
    q = low[(low[:, 0] > 0) & (low[:, 1] > 0)]
    assert abs(q[:, 0].mean() - meta["lowest_vertex_cluster_centroid_xy"]) < 2e-4
    assert abs(v[:, 2].min() - meta["z_min"]) < 1e-5


def _run_all():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    tests.sort(key=lambda nf: inspect.getsourcelines(nf[1])[1])
    results, t_all = [], time.perf_counter()
    for name, fn in tests:
        t0 = time.perf_counter()
        try:
            fn()
            status, detail = "pass", ""
        except unittest.SkipTest as exc:
            status, detail = "skip", str(exc)
        except Exception as exc:  # noqa: BLE001
            status, detail = "fail", f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=3)}"
        results.append({"test": name, "status": status, "seconds": round(time.perf_counter() - t0, 3), "detail": detail})
        print(f"{status.upper():4s} {name} ({results[-1]['seconds']:.2f}s)" + (f" -- {detail.splitlines()[0]}" if detail else ""))
    out = _out_dir()
    report = {"scope": "construction/parameter checks of the conditional contact fixture; not manufacturer-matched, "
                       "not mission landing, not entry advancement, not equipment validation",
              "results": results, "counts": {s: sum(r["status"] == s for r in results) for s in ("pass", "fail", "skip")},
              "wall_time_s": round(time.perf_counter() - t_all, 2),
              "source": dict(cc.source_info(CFG, CFG_PATH),
                             test_file=cc._rel(Path(__file__)),
                             test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
              "claims": cc.claims(CFG)}
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"report": cc._rel(out / "report.json"), "counts": report["counts"]}))
    return 1 if report["counts"]["fail"] else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
