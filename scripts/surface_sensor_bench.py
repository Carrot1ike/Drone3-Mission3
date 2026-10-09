#!/usr/bin/env python3
"""비접촉 이동 표면 벤치: 가상 광학점의 이상적 ray distance와 각도 flow.

실제 Flow deck 장착/PMW pixel 보정/기체 동역학을 정의하지 않는다.
실행: PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/surface_sensor_bench.py
"""
from __future__ import annotations

import argparse
import ast
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

# Imported project modules must not write __pycache__ outside the allowed scope.
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = ROOT / "artifacts/surface-sensor-bench"
SIL_PYTHON = ROOT / "src/crazyflie-ros2-sil/.venv/bin/python"
ATOL = 1e-9


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def yaw_matrix(yaw_deg):
    angle = np.deg2rad(yaw_deg)
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class SurfaceBench:
    def __init__(self, config, scene_path, rotating=False):
        tree = ET.parse(scene_path)
        if rotating:
            body = tree.getroot().find("worldbody/body[@name='landing_vehicle']")
            ET.SubElement(body, "joint", name="virtual_surface_yaw", type="hinge",
                          axis="0 0 1", pos="0 0 0", damping="0")
        # Geometry-only copy: neither contact forces nor time integration are used.
        for geom in tree.getroot().iter("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
        name = "scene-kinematic-yaw.xml" if rotating else "scene-kinematic.xml"
        self.path = scene_path.parent.parent / name
        tree.write(self.path, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.data = mujoco.MjData(self.model)
        self.config = config
        self.cart_id = self.model.body("landing_vehicle").id
        self.slide_id = self.model.joint("vehicle_slide").id
        self.yaw_id = self.model.joint("virtual_surface_yaw").id if rotating else None
        self.ray_groups = np.array([1, 1, 0, 1, 1, 1], dtype=np.uint8)
        self.set_state()

    def set_state(self, time_s=0., cart_speed=None, surface_yaw_deg=0., surface_yaw_rate=0.):
        speed = self.config["vehicle"]["speed_m_s"] if cart_speed is None else cart_speed
        self.data.qpos[:] = 0.
        self.data.qvel[:] = 0.
        self.data.qpos[self.model.jnt_qposadr[self.slide_id]] = speed * time_s
        self.data.qvel[self.model.jnt_dofadr[self.slide_id]] = speed
        if self.yaw_id is not None:
            self.data.qpos[self.model.jnt_qposadr[self.yaw_id]] = np.deg2rad(surface_yaw_deg)
            self.data.qvel[self.model.jnt_dofadr[self.yaw_id]] = surface_yaw_rate
        elif surface_yaw_deg or surface_yaw_rate:
            raise ValueError("표면 회전은 별도 virtual_surface_yaw 장면에서만 허용")
        self.data.time = time_s
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def ray(self, origin, direction=(0., 0., -1.), include_group2=False):
        origin = np.asarray(origin, dtype=float)
        direction = np.asarray(direction, dtype=float)
        if not np.isclose(np.linalg.norm(direction), 1., atol=ATOL, rtol=0):
            raise ValueError("ray distance(m)를 위해 방향은 단위벡터여야 함")
        geom_id = np.array([-1], dtype=np.int32)
        normal = np.zeros(3)
        groups = np.ones(6, dtype=np.uint8) if include_group2 else self.ray_groups
        distance = float(mujoco.mj_ray(self.model, self.data, origin, direction,
                                      groups, 1, -1, geom_id, normal))
        if geom_id[0] < 0 or distance < 0:
            return {"valid": False, "ray_distance_m": None, "geom_id": -1,
                    "geom_name": None, "body_id": None, "hit_world_m": None}
        gid = int(geom_id[0])
        bid = int(self.model.geom_bodyid[gid])
        point = origin + distance * direction
        # A point attached to the hit body includes both translation and omega x r.
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, self.data, jacp, jacr, point, bid)
        return {"valid": True, "ray_distance_m": distance, "geom_id": gid,
                "geom_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid),
                "geom_group": int(self.model.geom_group[gid]), "body_id": bid,
                "body_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid),
                "hit_world_m": point.tolist(), "surface_normal_world": normal.tolist(),
                "surface_velocity_world_m_s": (jacp @ self.data.qvel).tolist(),
                "surface_omega_world_rad_s": (jacr @ self.data.qvel).tolist()}

    def measure(self, name, origin, velocity=(0., 0., 0.), yaw_deg=0.):
        R = yaw_matrix(yaw_deg)
        ray = self.ray(origin, R @ np.array([0., 0., -1.]))
        row = {"case": name, "time_s": float(self.data.time),
               "sensor_origin_world_m": list(origin), "sensor_velocity_world_m_s": list(velocity),
               "R_world_virtual_sensor": R.tolist(), "sensor_yaw_deg": yaw_deg,
               "sensor_omega_world_rad_s": [0., 0., 0.],
               "ray_direction_world": [0., 0., -1.], **ray}
        if not ray["valid"]:
            row.update(angular_flow_rad_s=None, upstream_translation_proxy_rad_s=None)
            return row
        height = ray["ray_distance_m"]
        if height <= 0 or not np.allclose(ray["surface_normal_world"], [0., 0., 1.],
                                          atol=ATOL, rtol=0):
            raise ValueError("각도 flow 시험식은 양의 거리/수평 표면/수직 ray에 한정")
        relative = R.T @ (np.asarray(velocity) - np.asarray(ray["surface_velocity_world_m_s"]))
        row.update(relative_velocity_virtual_sensor_m_s=relative.tolist(),
                   angular_flow_rad_s=(relative[:2] / height).tolist(),
                   # Only the upstream deterministic translation term, without pixel scale/noise.
                   upstream_translation_proxy_rad_s=((R.T @ np.asarray(velocity))[:2]
                                                       / max(height, 0.1)).tolist())
        return row


class Checks:
    def __init__(self):
        self.results = []

    def record(self, name, actual, expected):
        errors = []
        for key, value in expected.items():
            observed = actual.get(key)
            numeric = (isinstance(value, (int, float)) and not isinstance(value, bool)) or (
                isinstance(value, list) and np.asarray(value).dtype.kind in "iuf")
            if numeric:
                ok = (observed is not None and np.shape(observed) == np.shape(value)
                      and np.allclose(observed, value, atol=ATOL, rtol=0))
            else:
                ok = observed == value
            if not ok:
                errors.append(f"{key}: expected={value!r}, actual={observed!r}")
        self.results.append({"name": name, "passed": not errors,
                             "expected": expected, "actual": actual, "errors": errors})


def source_evidence(path):
    source = path.read_text()
    lines = source.splitlines()
    funcs = {}
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.FunctionDef) and node.name in ("read_tof_down", "compute_flow"):
            funcs[node.name] = {"start_line": node.lineno, "end_line": node.end_lineno,
                                "source": "\n".join(lines[node.lineno - 1:node.end_lineno])}
    masks = [{"line": i + 1, "source": line.strip()} for i, line in enumerate(lines)
             if "self._ray_geomgroup =" in line]
    return {"path": str(path), "sha256": digest(path), "functions": funcs,
            "ray_group_initialization": masks,
            "comparison": "rotation0 수평 시험의 결정론적 translation 항만 rad/s로 비교; "
                          "upstream 호출/PMW pixel scale/노이즈 재사용 없음"}


def run_cases(bench, rotating):
    checks, samples = Checks(), []
    speed = float(bench.config["vehicle"]["speed_m_s"])
    bench.set_state(time_s=.75)
    x, y, _ = bench.data.xpos[bench.cart_id]
    checks.record("scene_contract", {
        "speed_m_s": speed, "pad_top_m": float(bench.data.geom_xpos[bench.model.geom("landing_pad").id, 2]
                                                   + bench.model.geom_size[bench.model.geom("landing_pad").id, 2]),
        "slide_position_m": float(bench.data.qpos[bench.model.jnt_qposadr[bench.slide_id]]),
        "contact_masks_disabled": bool(np.all(bench.model.geom_contype == 0)
                                       and np.all(bench.model.geom_conaffinity == 0)),
        "body_names": [mujoco.mj_id2name(bench.model, mujoco.mjtObj.mjOBJ_BODY, i)
                       for i in range(bench.model.nbody)]},
        {"speed_m_s": .3, "pad_top_m": .2, "slide_position_m": .225,
         "contact_masks_disabled": True, "body_names": ["world", "landing_vehicle"]})

    def scenario(name, point, vel, flow, geom="landing_pad", yaw=0., model=bench,
                 distance=.3, surface=(.3, 0., 0.), proxy=None):
        row = model.measure(name, point, vel, yaw)
        samples.append(row)
        expected = {"geom_name": geom, "body_id": model.cart_id if geom == "landing_pad" else 0,
                    "ray_distance_m": distance, "surface_velocity_world_m_s": list(surface),
                    "angular_flow_rad_s": list(flow)}
        if proxy is not None:
            expected["upstream_translation_proxy_rad_s"] = list(proxy)
        checks.record(name, row, expected)
        return row

    pad = (float(x), float(y), .5)
    ground = (float(x - .45), float(y), .5)
    co = scenario("sensor_cart_co_moving", pad, (.3, 0., 0.), (0., 0.), proxy=(1., 0.))
    scenario("sensor_moving_over_ground", ground, (.3, 0., 0.), (.6, 0.), "floor",
             distance=.5, surface=(0., 0., 0.), proxy=(.6, 0.))
    stationary = scenario("sensor_stationary_over_moving_pad", pad, (0., 0., 0.), (-1., 0.), proxy=(0., 0.))
    scenario("sensor_faster_than_cart", pad, (.6, 0., 0.), (1., 0.))
    scenario("ground_lateral_y", ground, (0., .3, 0.), (0., .6), "floor",
             distance=.5, surface=(0., 0., 0.))
    scenario("yaw90_reference_axes", ground, (.3, 0., 0.), (0., -.6), "floor", yaw=90.,
             distance=.5, surface=(0., 0., 0.))
    scenario("yaw180_reference_axes", ground, (.3, 0., 0.), (-.6, 0.), "floor", yaw=180.,
             distance=.5, surface=(0., 0., 0.))
    scenario("descent_no_height_clamp", (float(x), float(y), .25), (0., 0., 0.), (-6., 0.),
             distance=.05, proxy=(0., 0.))
    scenario("vertical_descent_at_optical_axis", pad, (.3, 0., -.035), (0., 0.))
    bench.set_state(cart_speed=0.)
    x0, y0, _ = bench.data.xpos[bench.cart_id]
    scenario("stationary_over_static_pad", (float(x0), float(y0), .5), (0., 0., 0.), (0., 0.),
             surface=(0., 0., 0.))
    scenario("stationary_over_ground", ground, (0., 0., 0.), (0., 0.), "floor",
             distance=.5, surface=(0., 0., 0.))
    floor = bench.measure("before_floor_to_pad", (float(x0 - .151), float(y0), .5))
    top = bench.measure("after_floor_to_pad", (float(x0 - .149), float(y0), .5))
    samples.extend([floor, top])
    checks.record("floor_to_pad_height_jump", {
        "before_geom": floor["geom_name"], "after_geom": top["geom_name"],
        "before_distance_m": floor["ray_distance_m"], "after_distance_m": top["ray_distance_m"],
        "distance_drop_m": floor["ray_distance_m"] - top["ray_distance_m"]},
        {"before_geom": "floor", "after_geom": "landing_pad", "before_distance_m": .5,
         "after_distance_m": .3, "distance_drop_m": .2})

    # At the original pad location after translation, the ray must see the floor.
    bench.set_state(time_s=.75)
    scenario("cart_moved_old_position_is_ground", (float(x0), float(y0), .5), (0., 0., 0.),
             (0., 0.), "floor", distance=.5, surface=(0., 0., 0.))

    # Independent central finite difference of a material point on the rotating pad.
    rotating.set_state(time_s=.75, surface_yaw_deg=30., surface_yaw_rate=1.)
    center = rotating.data.xpos[rotating.cart_id].copy()
    local = np.array([.1, .04, 0.])
    lever = yaw_matrix(30.) @ local
    point = center + lever
    expected_velocity = np.array([.3, 0., 0.]) + np.cross([0., 0., 1.], lever)
    row = scenario("rotating_pad_hit_point_velocity", (float(point[0]), float(point[1]), .5),
                   (0., 0., 0.), -expected_velocity[:2] / .3, model=rotating,
                   surface=expected_velocity)
    eps = 1e-6
    finite_positions = []
    for sign in (-1, 1):
        rotating.set_state(time_s=.75 + sign * eps,
                           surface_yaw_deg=30. + np.rad2deg(sign * eps), surface_yaw_rate=1.)
        finite_positions.append(rotating.data.xpos[rotating.cart_id].copy()
                                + rotating.data.xmat[rotating.cart_id].reshape(3, 3) @ local)
    finite_v = (finite_positions[1] - finite_positions[0]) / (2 * eps)
    checks.record("rotating_pad_material_point_finite_difference", {
        "finite_difference_velocity_m_s": finite_v.tolist(),
        "surface_omega_world_rad_s": row["surface_omega_world_rad_s"],
        "jacobian_minus_finite_difference_m_s": (np.asarray(row["surface_velocity_world_m_s"]) - finite_v).tolist()},
        {"finite_difference_velocity_m_s": expected_velocity.tolist(),
         "surface_omega_world_rad_s": [0., 0., 1.], "jacobian_minus_finite_difference_m_s": [0., 0., 0.]})

    board_id = bench.model.geom("marker_board").id
    board = bench.data.geom_xpos[board_id].copy()
    board[1] += .04  # Away from the post and coplanar marker cells.
    board[2] += bench.model.geom_size[board_id, 2] + .1
    flag_unfiltered = bench.ray(board, include_group2=True)
    flag_filtered = bench.ray(board)
    checks.record("foreground_flag_group2_excluded", {
        "unfiltered_geom": flag_unfiltered["geom_name"], "unfiltered_group": flag_unfiltered["geom_group"],
        "unfiltered_distance_m": flag_unfiltered["ray_distance_m"],
        "filtered_geom": flag_filtered["geom_name"], "filtered_group": flag_filtered["geom_group"],
        "filtered_distance_m": flag_filtered["ray_distance_m"]},
        {"unfiltered_geom": "marker_board", "unfiltered_group": 2, "unfiltered_distance_m": .1,
         "filtered_geom": "floor", "filtered_group": 0, "filtered_distance_m": float(board[2])})
    ray_tests = {"flag_origin_world_m": board.tolist(), "flag_unfiltered": flag_unfiltered,
                 "flag_filtered": flag_filtered}
    miss = bench.ray(pad, (0., 0., 1.))
    checks.record("upward_ray_no_hit", miss, {"valid": False, "ray_distance_m": None, "geom_id": -1})
    oblique = bench.ray(ground, (-1. / np.sqrt(2.), 0., -1. / np.sqrt(2.)))
    checks.record("oblique_tof_is_ray_distance", oblique,
                  {"geom_name": "floor", "ray_distance_m": .5 * np.sqrt(2.)})
    ray_tests.update(upward_no_hit=miss, oblique_tof=oblique)
    checks.record("upstream_missing_surface_velocity_counterexamples", {
        "co_moving_error_rad_s": (np.asarray(co["upstream_translation_proxy_rad_s"])
                                  - co["angular_flow_rad_s"]).tolist(),
        "stationary_pad_error_rad_s": (np.asarray(stationary["upstream_translation_proxy_rad_s"])
                                       - stationary["angular_flow_rad_s"]).tolist()},
        {"co_moving_error_rad_s": [1., 0.], "stationary_pad_error_rad_s": [1., 0.]})

    trace = []
    for t in np.linspace(0., 2., 101):
        bench.set_state(time_s=float(t))
        trace.append(bench.measure("stationary_sensor_cart_pass", (.75, 0., .5)))
    trace_floor = [r for r in trace if r["geom_name"] == "floor"]
    trace_pad = [r for r in trace if r["geom_name"] == "landing_pad"]
    checks.record("cart_pass_trace_surface_selection", {
        "samples": len(trace), "both_surfaces_hit": bool(trace_floor and trace_pad),
        "floor_flow_zero": all(np.allclose(r["angular_flow_rad_s"], [0., 0.], atol=ATOL) for r in trace_floor),
        "pad_flow_negative": all(np.allclose(r["angular_flow_rad_s"], [-1., 0.], atol=ATOL) for r in trace_pad),
        "first_geom": trace[0]["geom_name"], "last_geom": trace[-1]["geom_name"]},
        {"samples": 101, "both_surfaces_hit": True, "floor_flow_zero": True, "pad_flow_negative": True,
         "first_geom": "floor", "last_geom": "floor"})
    return checks.results, samples, trace, ray_tests


def write_csv(path, rows):
    fields = ["case", "time_s", "sensor_yaw_deg", "geom_name", "geom_id", "body_id", "ray_distance_m",
              "sensor_x_m", "sensor_y_m", "sensor_z_m", "sensor_vx_m_s", "sensor_vy_m_s", "sensor_vz_m_s",
              "surface_vx_m_s", "surface_vy_m_s", "surface_vz_m_s", "surface_wx_rad_s", "surface_wy_rad_s",
              "surface_wz_rad_s", "angular_flow_x_rad_s", "angular_flow_y_rad_s",
              "upstream_translation_proxy_x_rad_s", "upstream_translation_proxy_y_rad_s"]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in fields[:7]}
            for vector, columns in (
                ("sensor_origin_world_m", fields[7:10]), ("sensor_velocity_world_m_s", fields[10:13]),
                ("surface_velocity_world_m_s", fields[13:16]), ("surface_omega_world_rad_s", fields[16:19]),
                ("angular_flow_rad_s", fields[19:21]), ("upstream_translation_proxy_rad_s", fields[21:23])):
                flat.update(zip(columns, row[vector]))
            writer.writerow(flat)


PLOT_CODE = r'''
import json, pathlib, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
out = pathlib.Path(sys.argv[1])
data = json.loads((out / "results.json").read_text())
rows = {r["case"]: r for r in data["samples"]}
names = ["sensor_cart_co_moving", "sensor_moving_over_ground", "sensor_stationary_over_moving_pad"]
labels = ["Sensor + cart\n0.3 m/s on pad", "Sensor 0.3 m/s\nover ground", "Sensor stationary\ncart 0.3 m/s"]
fig, axes = plt.subplots(1, 3, figsize=(14, 4.4))
xs = list(range(3))
axes[0].bar([x-.18 for x in xs], [rows[n]["angular_flow_rad_s"][0] for n in names],
            width=.36, label="Surface-relative ideal", color="#157d8e")
axes[0].bar([x+.18 for x in xs], [rows[n]["upstream_translation_proxy_rad_s"][0] for n in names],
            width=.36, label="Upstream translation proxy", color="#d18228")
axes[0].set_xticks(xs, labels)
axes[0].set_ylabel("Signed angular flow X [rad/s]")
axes[0].set_title("Moving surface changes flow")
axes[0].axhline(0, color="black", linewidth=.6)
axes[0].legend(fontsize=8)
trace = data["trace"]
time = [r["time_s"] for r in trace]
axes[1].step(time, [r["ray_distance_m"] for r in trace], where="mid", color="#157d8e")
axes[1].set_ylim(.25, .55)
axes[1].set_ylabel("Ideal ToF ray distance [m]")
axes[1].set_xlabel("Prescribed cart time [s]")
axes[1].set_title("Stationary virtual optical point")
axes[1].text(1.25, .42, "Pad top +0.20 m", ha="center", fontsize=9)
axes[2].step(time, [r["angular_flow_rad_s"][0] for r in trace], where="mid", color="#157d8e",
             label="Surface-relative ideal")
axes[2].plot(time, [r["upstream_translation_proxy_rad_s"][0] for r in trace], color="#d18228",
             linestyle="--", label="Upstream translation proxy")
axes[2].set_ylim(-1.2, .2)
axes[2].set_ylabel("Signed angular flow X [rad/s]")
axes[2].set_xlabel("Prescribed cart time [s]")
axes[2].set_title("Floor / moving-pad ray selection")
axes[2].legend(fontsize=8)
for ax in axes:
    ax.grid(axis="y", alpha=.2)
fig.suptitle("Geometry-only virtual sensor bench | no hardware calibration / noise / latency", fontsize=12)
fig.tight_layout()
fig.savefig(out / "surface-sensor-bench.png", dpi=170)
fig.savefig(out / "surface-sensor-bench.svg")
plt.close(fig)
print(json.dumps({"python": sys.executable, "matplotlib": matplotlib.__version__,
                  "outputs": ["surface-sensor-bench.png", "surface-sensor-bench.svg"]}))
'''


def make_plot(out, python):
    env = {**os.environ, "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
           "MPLCONFIGDIR": str(out / "plot-cache"), "XDG_CACHE_HOME": str(out / "plot-cache")}
    result = subprocess.run([str(python), "-c", PLOT_CODE, str(out)], env=env,
                            capture_output=True, text=True, timeout=60)
    (out / "plot.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        return {"status": "failed", "python": str(python), "log": str(out / "plot.log")}
    return {"status": "passed", **json.loads(result.stdout)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--plot-python", type=Path, default=ROOT / "build/yaw-analysis-venv/bin/python")
    args = parser.parse_args()
    if Path(sys.executable).absolute() != SIL_PYTHON or os.environ.get("PYTHONNOUSERSITE") != "1":
        parser.error("PYTHONNOUSERSITE=1과 src/crazyflie-ros2-sil/.venv/bin/python으로 실행해야 합니다")
    out = args.output.resolve()
    if out != OUTPUT_ROOT.resolve() and OUTPUT_ROOT.resolve() not in out.parents:
        parser.error("쓰기 범위는 artifacts/surface-sensor-bench/ 아래로 제한됩니다")
    out.mkdir(parents=True, exist_ok=True)
    global mujoco, np
    import mujoco
    import numpy as np
    import cv2
    from moving_landing_scene import build_scene
    from yaw_scene import SIM_DIR

    config_path = ROOT / "config/moving-landing.json"
    hardware_path = ROOT / "config/provided-hardware.json"
    config = json.loads(config_path.read_text())
    if (config["camera"]["width"], config["camera"]["height"], config["camera"]["encoding"]) != (320, 320, "mono8"):
        raise ValueError("사용자 고정 camera 입력은 mono8 320x320")
    watched = [config_path, hardware_path, ROOT / "scripts/moving_landing_scene.py",
               ROOT / "scripts/yaw_scene.py", ROOT / "scripts/provided_hardware.py",
               SIM_DIR / "scene.xml", SIM_DIR / "crazysim.py"]
    before = {str(p): digest(p) for p in watched}
    scene = build_scene(config, out / "scene-original")
    bench, rotating = SurfaceBench(config, scene), SurfaceBench(config, scene, rotating=True)
    checks, samples, trace, ray_tests = run_cases(bench, rotating)
    after = {str(p): digest(p) for p in watched}
    input_unchanged = before == after
    checks.append({"name": "input_sources_unchanged", "passed": input_unchanged,
                   "expected": before, "actual": after, "errors": [] if input_unchanged else ["입력 파일 hash 변경"]})
    passed = all(c["passed"] for c in checks)
    hardware = json.loads(hardware_path.read_text())
    results = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "비접촉 geometry-only 가상센서 광학점 시험; 실제 Flow deck 수치/비행/착륙 검증 아님",
        "status": "passed" if passed else "failed", "assert_case_count": len(checks),
        "passed_case_count": sum(c["passed"] for c in checks), "absolute_tolerance": ATOL,
        "producer": {"path": str(Path(__file__).resolve()), "sha256": digest(__file__)},
        "runtime": {"python": sys.executable, "python_version": sys.version,
                    "mujoco": mujoco.__version__, "numpy": np.__version__, "cv2": cv2.__version__,
                    "python_no_user_site": os.environ.get("PYTHONNOUSERSITE"),
                    "mujoco_module": mujoco.__file__, "numpy_module": np.__file__, "cv2_module": cv2.__file__},
        "virtual_sensor_condition": {"refFrame": np.eye(3).tolist(),
            "refFrame_status": "사용자가 정한 가상 기준축 identity; 실제 장착 변환 아님",
            "R_world_virtual_sensor": "Rz(yaw); yaw는 고정 자세, sensor angular velocity=0",
            "ray": "각 optical point에서 world -Z 단위 ray; oblique/no-hit는 별도 ToF 시험",
            "height_definition": "수평 표면/수직 ray 시험에서 ideal ray distance와 동일",
            "flow_definition": "[R_world_virtual_sensor.T @ (v_sensor - v_surface_at_hit)][:2] / height",
            "sign_convention": "sensor relative to surface의 signed angular flow; PMW raw pixel 부호 아님",
            "surface_velocity": "mj_jac(hit_world_point, geom_bodyid[hit_geom]).jacp @ qvel",
            "real_sensor_origin_body_m": None, "real_sensor_R_body": None,
            "pmw_pixel_calibration": None, "noise": "not modeled / not verified",
            "latency": "not modeled / not verified", "maximum_performance": "not verified"},
        "hardware_input_constraint": {"camera": "manufacturer supplied mono8 320x320 fixed",
            "hardware_modified": False, "camera_mount_R": hardware["camera"]["R_body_camera_cv"],
            "camera_mount_position_m": hardware["camera"]["camera_position_body_m"],
            "assembled_inertia_kg_m2": hardware["aircraft"]["assembled_inertia_kg_m2"]},
        "scene": {"source": str(scene), "kinematic_scene": str(bench.path),
            "virtual_surface_rotation_scene": str(rotating.path), "aircraft_body_present": False,
            "contact_masks": "all zero in bench copies", "integration_steps": 0,
            "pad_top_m": .2, "cart_speed_m_s": .3, "pad_size_m": [.3, .3],
            "vehicle_inertia_status": "기존 장면의 실험 소품 관성; 기체 수치로 사용하지 않고 역학 적분도 하지 않음"},
        "inputs_sha256": before, "upstream_evidence": source_evidence(SIM_DIR / "crazysim.py"),
        "checks": checks, "samples": samples, "trace": trace, "ray_tests": ray_tests}
    (out / "results.json").write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    write_csv(out / "cases.csv", samples)
    write_csv(out / "cart-pass-trace.csv", trace)
    try:
        results["plot"] = make_plot(out, args.plot_python.absolute())
    except (OSError, subprocess.TimeoutExpired) as exc:
        results["plot"] = {"status": "failed", "error": str(exc)}
    (out / "results.json").write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    failed = [c["name"] for c in checks if not c["passed"]]
    summary = {"status": results["status"], "passed_cases": results["passed_case_count"],
               "total_cases": len(checks), "failed_cases": failed, "plot": results["plot"]["status"],
               "output": str(out)}
    summary_json = json.dumps(summary, ensure_ascii=False)
    (out / "run.log").write_text(summary_json + "\n")
    print(summary_json)
    return 0 if passed and results["plot"]["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
