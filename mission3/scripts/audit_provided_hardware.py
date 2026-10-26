#!/usr/bin/env python3
"""Read the pinned baseline and report conflicts with supplied hardware."""
import argparse
import ast
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from provided_hardware import ROOT, hardware_profile, require_manufacturer_flight_model, HardwareModelUnresolved
from yaw_scene import SIM_DIR, load_upstream


def audit():
    expected = hardware_profile()
    sim = load_upstream()
    params = sim.MOTOR_PARAMS["cf2x_P250"]
    tree = ET.parse(SIM_DIR / sim.MODEL_PATHS["cf2x_P250"])
    camera = tree.find(".//camera[@name='fpv_cam']")
    source_ast = ast.parse((SIM_DIR / "crazysim.py").read_text())
    constructor = next(node for cls in source_ast.body if isinstance(cls, ast.ClassDef) and cls.name == "CrazySimMuJoCo"
                       for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
    default_nodes = dict(zip((arg.arg for arg in constructor.args.args[-len(constructor.args.defaults):]), constructor.args.defaults))
    default_nodes.update(zip((arg.arg for arg in constructor.args.kwonlyargs), constructor.args.kw_defaults))
    defaults = {key: ast.literal_eval(default_nodes[key]) for key in ("cam_width", "cam_height", "cam_fps")}
    baseline = {
        "model": "cf2x_P250", "manufacturer_validated": False,
        "runtime_mass_kg": params.mass, "runtime_inertia_kg_m2": list(params.diaginertia),
        "rpm_to_thrust_coefficients": list(params.rpm2thrust), "motor_response_s": params.tau_up,
        "camera_defaults": defaults, "camera_fovy_deg": float(camera.get("fovy")),
        "camera_position_body_m": [float(v) for v in camera.get("pos").split()],
        "camera_euler_degrees": [float(v) for v in camera.get("euler").split()],
        "parameter_source": str(SIM_DIR / "drone-models/drone_models/data/params.toml"),
        "camera_source": str(SIM_DIR / sim.MODEL_PATHS["cf2x_P250"])
    }
    try:
        require_manufacturer_flight_model()
        reason = None
    except HardwareModelUnresolved as exc:
        reason = str(exc)
    return {
        "date": "2026-10-04", "supplied_camera_contract": expected["camera"],
        "stock_aircraft_mass_kg": expected["aircraft"]["stock_takeoff_mass_kg"],
        "conditional_component_mass_sum_kg": expected["derived"]["conditional_component_mass_sum_kg"],
        "assembled_mass_kg": expected["aircraft"]["assembled_mass_kg"],
        "shared_integration_baseline": baseline,
        "native_camera_matches_baseline": (defaults["cam_width"], defaults["cam_height"]) == (320, 320),
        "nominal_fov_matches_baseline": float(camera.get("fovy")) == 87.0,
        "eligible_for_supplied_aircraft_landing_validation": reason is None,
        "blocking_reason": reason,
        "permitted_current_result_scope": "camera contract, nominal optical geometry and software integration; no supplied-aircraft dynamics/contact claim"
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/hardware-audit.json")
    args = parser.parse_args()
    report = audit()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
