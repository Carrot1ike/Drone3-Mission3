#!/usr/bin/env python3
"""Moving pad geometry and nominal manufacturer mono-camera projection."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import mujoco
import numpy as np
from yaw_scene import ROOT, SIM_DIR, R_WORLD_MARKER
from provided_hardware import validate_camera_config


def build_scene(config, out):
    validate_camera_config(config["camera"])
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(SIM_DIR / "scene.xml")
    root = tree.getroot()
    root.set("model", "drone3_moving_landing")
    root.find("compiler").set("fusestatic", "false")
    root.find("visual/global").set("offwidth", "960")
    root.find("visual/global").set("offheight", "640")
    root.find("visual/map").set("znear", "0.001")
    world = root.find("worldbody")
    car, marker = config["vehicle"], config["marker"]
    x, y, z = car["initial_pad_center_m"]
    body = ET.SubElement(world, "body", name="landing_vehicle", pos=f"{x} {y} {z}")
    ET.SubElement(body, "joint", name="vehicle_slide", type="slide", axis="1 0 0", damping="0")
    # Prescribed test vehicle mass/inertia are experimental props, not aircraft data.
    ET.SubElement(body, "inertial", pos="0 0 -.05", mass="10", diaginertia="1 1 1")
    thickness = car["pad_thickness_m"]
    ET.SubElement(body, "geom", name="landing_pad", type="box", pos=f"0 0 {-thickness/2}",
                  size=f"{car['pad_length_m']/2} {car['pad_width_m']/2} {thickness/2}",
                  rgba=".82 .88 .83 1", friction="1 .005 .0001", condim="3", group="0")
    ET.SubElement(body, "geom", name="vehicle_artwork", type="box", pos=".15 0 -.08",
                  size=".36 .18 .05", rgba=".25 .3 .4 1", group="2", contype="0", conaffinity="0")
    for wheel_x in (-.08, .42):
        for wheel_y in (-.17, .17):
            ET.SubElement(body, "geom", type="cylinder", pos=f"{wheel_x} {wheel_y} -.13", size=".055 .02",
                          quat=".70710678 .70710678 0 0", rgba=".08 .08 .08 1", group="2", contype="0", conaffinity="0")
    mx, my, mz = car["marker_offset_from_pad_m"]
    length, margin = marker["length_m"], marker["quiet_margin_m"]
    ET.SubElement(body, "geom", name="marker_post", type="cylinder", fromto=f"{mx+.008} {my} -.03 {mx+.008} {my} {mz}",
                  size=".003", rgba=".4 .4 .4 1", group="2", contype="0", conaffinity="0")
    ET.SubElement(body, "geom", name="marker_board", type="box", pos=f"{mx+.003} {my} {mz}",
                  size=f".002 {length/2+margin} {length/2+margin}", rgba="1 1 1 1", group="2", contype="0", conaffinity="0")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker["dictionary"]))
    modules = dictionary.markerSize+2
    bits = cv2.aruco.generateImageMarker(dictionary, marker["id"], modules)
    cell = length/modules
    for row, col in np.argwhere(bits == 0):
        ET.SubElement(body, "geom", type="box", pos=f"{mx+.0005} {my+length/2-(col+.5)*cell} {mz+length/2-(row+.5)*cell}",
                      size=f".0005 {cell/2} {cell/2}", rgba="0 0 0 1", group="2", contype="0", conaffinity="0")
    path = out / "scene.xml"; tree.write(path, encoding="unicode")
    cv2.imwrite(str(out / "aruco.png"), cv2.copyMakeBorder(cv2.aruco.generateImageMarker(dictionary, marker["id"], 600), 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))
    return path


def build_optical_preview_model(config, scene):
    """A camera optical point only: no aircraft model or body mounting transform.

    The ideal preview looks along +X. This is a specified optical trajectory,
    not a numerical assumption about the standard AI-deck mounting.
    """
    validate_camera_config(config["camera"])
    tree = ET.parse(scene)
    world = tree.getroot().find("worldbody")
    R_world_camera_cv = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, (R_world_camera_cv @ np.diag([1., -1., -1.])).flatten())
    ET.SubElement(world, "camera", name="optical_preview", pos="0 0 .5",
                  quat=" ".join(str(v) for v in quat), fovy=str(config["camera"]["vertical_fov_deg"]))
    return mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding="unicode"))


def camera_profile(model, config):
    camera = model.camera("optical_preview").id
    width, height = config["camera"]["width"], config["camera"]["height"]
    focal = height/(2*np.tan(np.deg2rad(model.cam_fovy[camera])/2))
    return {"source": "geometry-only ideal projection from manufacturer nominal 87deg; not calibrated hardware",
            "width": width, "height": height, "encoding": "mono8", "K": [[float(focal), 0, width/2], [0, float(focal), height/2], [0, 0, 1]],
            "ideal_distortion": [0, 0, 0, 0, 0], "calibrated_K": None, "calibrated_distortion": None,
            "R_body_camera_cv": None, "camera_position_body_m": None, "body_transform_source": None,
            "fovy_deg": float(model.cam_fovy[camera]),
            "R_world_marker": R_WORLD_MARKER.tolist(), "marker_length_m": config["marker"]["length_m"],
            "marker_id": config["marker"]["id"], "dictionary": config["marker"]["dictionary"],
            "marker_offset_from_pad_m": config["vehicle"]["marker_offset_from_pad_m"]}
