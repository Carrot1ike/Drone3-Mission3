#!/usr/bin/env python3
"""저장한 경로에 이상적 하향 probe를 재생. 실제 Flow deck/Kalman 연결 아님."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
import mujoco
import surface_sensor_bench as bench_module
from moving_landing_scene import build_scene
from provided_hardware import ROOT


def replay(out):
    config = json.loads((out / "experiment.json").read_text())
    truth = list(csv.DictReader((out / "truth.csv").open()))
    commands = [json.loads(s) for s in (out / "commands.jsonl").read_text().splitlines()]
    cmd_times = np.array([r["time_s"] for r in commands])
    origin = np.median([r["monotonic_s"]-r["time_s"] for r in commands]) if "monotonic_s" in truth[0] else None
    times = np.array([float(r["monotonic_s"])-origin if origin is not None else float(r["time_s"]) for r in truth])
    positions = np.array([[float(r[k]) for k in ("camera_x_m", "camera_y_m", "camera_z_m")] for r in truth])
    velocity = np.zeros_like(positions)
    velocity[1:] = np.diff(positions, axis=0)/np.diff(times)[:, None]
    target = out / "surface-replay"
    target.mkdir(exist_ok=True)
    # 22조건 검증된 기존 벤치의 의존성을 동일 공유 환경에서 주입해 재사용한다.
    bench_module.np, bench_module.mujoco = np, mujoco
    bench = bench_module.SurfaceBench(config, build_scene(config, target / "scene-original"))
    rows, transitions, last_surface = [], [], None
    speed = config["vehicle"]["speed_m_s"]
    for i, r in enumerate(truth):
        moving = str(r["vehicle_moving"]).lower() == "true"
        travel = float(r["vehicle_travel_m"])
        bench.set_state(time_s=travel/speed if speed else 0., cart_speed=speed if moving else 0.)
        sample = bench.measure("mission_replay", positions[i].tolist(), velocity[i].tolist(), float(r["heading_deg"]))
        if not sample["valid"]:
            raise ValueError("virtual downward probe did not hit a surface")
        surface_v = np.asarray(sample["surface_velocity_world_m_s"])
        relative_world = velocity[i]-surface_v
        phase = commands[int(np.clip(np.searchsorted(cmd_times, times[i], side="right")-1, 0, len(commands)-1))]["phase"]
        row = {"time_s": float(times[i]), "phase": phase, "surface": sample["geom_name"],
            "world_z_m": float(positions[i, 2]), "height_above_pad_m": float(r["height_above_pad_m"]),
            "virtual_range_m": sample["ray_distance_m"], "surface_z_m": sample["hit_world_m"][2],
            "camera_vx_m_s": float(velocity[i, 0]), "surface_vx_m_s": float(surface_v[0]),
            "relative_vx_world_m_s": float(relative_world[0]),
            "translation_flow_x_rad_s": sample["angular_flow_rad_s"][0],
            "translation_flow_y_rad_s": sample["angular_flow_rad_s"][1],
            "naive_static_ground_vx_m_s": float(relative_world[0]),
            "naive_static_ground_vx_error_m_s": float(-surface_v[0]),
            "naive_ground_z_error_m": float(sample["ray_distance_m"]-positions[i, 2])}
        if row["surface"] != last_surface and times[i] >= 0.:
            transitions.append({"time_s": row["time_s"], "phase": phase, "from_surface": last_surface,
                "to_surface": row["surface"], "virtual_range_m": row["virtual_range_m"], "world_z_m": row["world_z_m"]})
        last_surface = row["surface"]
        rows.append(row)
    with (target / "trace.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    checks = {
        "ground_and_vehicle_both_observed": {r["surface"] for r in rows} == {"floor", "landing_pad"},
        "world_z_minus_range_matches_hit_surface": all(abs(r["world_z_m"]-r["virtual_range_m"]-r["surface_z_m"]) < 1e-8 for r in rows),
        "vehicle_static_ground_velocity_bias_is_minus_speed": all(abs(r["naive_static_ground_vx_error_m_s"]+speed) < 1e-8 for r in rows if r["surface"] == "landing_pad"),
        "floor_static_ground_velocity_bias_zero": all(abs(r["naive_static_ground_vx_error_m_s"]) < 1e-8 for r in rows if r["surface"] == "floor"),
    }
    report = {"scope": "post-run ideal downward colocated mathematical probe; no real Flow deck, PMW pixels, Kalman fusion or gyro compensation",
        "sensor_connected_to_controller": False, "real_flow_sensor_origin_body_m": None, "real_flow_sensor_mounting_R": None,
        "pmw_pixel_calibration": None, "noise_occlusion_reflectance_mixed_returns_modeled": False,
        "flow_is_translation_component_only": True, "virtual_probe_origin": "camera optical position for diagnostic replay only; not physical deck location",
        "surface_groups": "floor and pad; group2 artwork/post/wheels excluded as in existing surface bench",
        "input_artifact_dir": str(out), "transitions": transitions, "checks": checks, "checks_passed": all(checks.values()),
        "producer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "hardware_landing_validated": False}
    (target / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
    plot = subprocess.run([str(ROOT / "build/yaw-analysis-venv/bin/python"), str(ROOT / "scripts/plot_surface_replay.py"), str(target)])
    report["plot_exported"] = plot.returncode == 0
    (target / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact", type=Path)
    args = parser.parse_args()
    result = replay(args.artifact.resolve())
    raise SystemExit(0 if result["checks_passed"] and result["plot_exported"] else 1)
