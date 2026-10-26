#!/usr/bin/env python3
"""Project static marker scene and *simulation-defined* calibration export."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "src/crazyflie-ros2-sil/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco"
# Marker x points right on its printed face, y up, z towards the approaching drone.
R_WORLD_MARKER = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def load_upstream():
    sys.path.insert(0, str(SIM_DIR))
    spec = importlib.util.spec_from_file_location("drone3_pinned_crazysim", SIM_DIR / "crazysim.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def build_scene(config, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(SIM_DIR / "scene.xml")
    root = tree.getroot()
    root.set("model", "drone3_static_yaw_marker")
    root.find("visual/global").set("offwidth", "960")
    root.find("visual/global").set("offheight", "640")
    root.find("visual/map").set("znear", "0.001")
    root.find("statistic").set("center", "0.3 0 0.2")
    root.find("statistic").set("extent", "1.0")
    world = root.find("worldbody")
    marker = config["marker"]
    x, y, z = marker["center_m"]
    length, margin = marker["length_m"], marker["quiet_margin_m"]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker["dictionary"]))
    modules = dictionary.markerSize + 2
    bits = cv2.aruco.generateImageMarker(dictionary, marker["id"], modules)

    def geom(**attrs):
        return ET.SubElement(world, "geom", contype="0", conaffinity="0", **attrs)
    # Rendering props are experimental scene geometry, never aircraft specifications.
    geom(name="marker_board", type="box", pos=f"{x + .003} {y} {z}",
         size=f".002 {length/2+margin} {length/2+margin}", rgba="1 1 1 1")
    geom(name="marker_post", type="cylinder", fromto=f"{x+.008} {y} 0 {x+.008} {y} {z}",
         size=".003", rgba=".4 .4 .4 1")
    cell = length / modules
    for row, col in np.argwhere(bits == 0):
        geom(type="box", pos=f"{x+.0005} {y+length/2-(col+.5)*cell} {z+length/2-(row+.5)*cell}",
             size=f".0005 {cell/2} {cell/2}", rgba="0 0 0 1")
    path = out / "scene.xml"
    tree.write(path, encoding="unicode")
    marker_image = cv2.aruco.generateImageMarker(dictionary, marker["id"], 600)
    cv2.imwrite(str(out / "aruco.png"), cv2.copyMakeBorder(marker_image, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))
    return path


def model_and_profile(config, scene_path):
    sim = load_upstream()
    sim._SCENE_XML = str(scene_path)
    model_path = SIM_DIR / sim.MODEL_PATHS["cf2x_P250"]
    model = sim._build_spec_multi_safe(str(model_path), [(0., 0.)], sim.MOTOR_PARAMS["cf2x_P250"])
    cam = model.camera("cf0_fpv_cam").id
    rotation_gl = np.empty(9)
    mujoco.mju_quat2Mat(rotation_gl, model.cam_quat[cam])
    width, height = config["camera"]["width"], config["camera"]["height"]
    focal = height / (2 * np.tan(np.deg2rad(model.cam_fovy[cam]) / 2))
    profile = {
        "source": "compiled pinned cf2x_P250 simulation camera, not real AI-deck calibration",
        "width": width, "height": height,
        "K": [[float(focal), 0., width / 2.], [0., float(focal), height / 2.], [0., 0., 1.]],
        "distortion": [0., 0., 0., 0., 0.],
        "R_body_camera_cv": (rotation_gl.reshape(3, 3) @ np.diag([1., -1., -1.])).tolist(),
        "camera_position_body_m": model.cam_pos[cam].tolist(),
        "fovy_deg": float(model.cam_fovy[cam]),
        "R_world_marker": R_WORLD_MARKER.tolist(),
        "marker_length_m": config["marker"]["length_m"],
        "marker_id": config["marker"]["id"], "dictionary": config["marker"]["dictionary"]
    }
    return model, profile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "config/yaw-static.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    path = build_scene(config, args.output)
    _, profile = model_and_profile(config, path)
    (args.output / "camera-profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    print(path)


if __name__ == "__main__":
    main()
