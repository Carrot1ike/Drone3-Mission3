#!/usr/bin/env python3
"""Moving-pad camera geometry, with no aircraft dynamics or invented extrinsics."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np
import tomli

ROOT = Path(__file__).resolve().parents[1]

# Columns are marker +x (viewer right), +y (up), +z (towards rear).
ROTATION_WORLD_MARKER = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
ROTATION_WORLD_CAMERA_GL = ROTATION_WORLD_MARKER.copy()
GL_TO_CV = np.diag([1., -1., -1.])


def read_config(path=ROOT / "config/landing.toml"):
    with Path(path).open("rb") as stream:
        config = tomli.load(stream)
    for section, keys in {
        "camera": ("width", "height", "vertical_fov_deg"),
        "pad": ("width_m", "length_m", "top_height_m"),
        "marker": ("length_m", "quiet_margin_m"),
        "trajectory": ("duration_s", "sample_hz", "camera_height_above_pad_m",
                       "end_camera_height_above_pad_m"),
        "vehicle": ("render_length_m", "render_width_m"),
    }.items():
        for key in keys:
            value = config[section][key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{section}.{key} must be positive and finite")
    if not 0 < config["camera"]["vertical_fov_deg"] < 180:
        raise ValueError("camera.vertical_fov_deg must be between 0 and 180")
    if config["camera"]["width"] != config["camera"]["height"]:
        raise ValueError("The selected full-image nominal FOV profile requires a square image")
    offset = np.asarray(config["marker"]["offset_from_pad_m"], dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all() or offset[0] <= 0:
        raise ValueError("marker offset must be a finite vector with positive forward distance")
    if not math.isfinite(config["vehicle"]["speed_m_s"]):
        raise ValueError("vehicle speed must be finite")
    return config


def _numbers(values):
    return " ".join(f"{float(value):.10g}" for value in values)


def _geom(parent, **attributes):
    attributes.update(contype="0", conaffinity="0", density="0")
    return ET.SubElement(parent, "geom", {key: str(value) for key, value in attributes.items()})


def _smooth(value):
    value = min(1., max(0., value))
    return value*value*(3.-2.*value)


class LandingScene:
    def __init__(self, config):
        self.config = config
        pad, marker, camera = (config[key] for key in ("pad", "marker", "camera"))
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker["dictionary"]))
        count = len(dictionary.bytesList)
        if not 0 <= marker["id"] < count:
            raise ValueError("marker.id is outside the selected dictionary")
        modules = dictionary.markerSize+2
        self.marker_bits = cv2.aruco.generateImageMarker(dictionary, marker["id"], modules)
        self.pad_offset_marker = ROTATION_WORLD_MARKER.T @ -np.asarray(marker["offset_from_pad_m"])

        xml = ET.Element("mujoco", model="drone3_camera_geometry_only")
        ET.SubElement(xml, "compiler", angle="radian", fusestatic="false")
        ET.SubElement(xml, "option", gravity="0 0 0")
        visual = ET.SubElement(xml, "visual")
        ET.SubElement(visual, "global", offwidth="960", offheight="640")
        ET.SubElement(visual, "map", znear="0.001")
        ET.SubElement(visual, "headlight", ambient="0.7 0.7 0.7", diffuse="0.3 0.3 0.3", specular="0 0 0")
        world = ET.SubElement(xml, "worldbody")
        ET.SubElement(world, "light", pos="0 0 3", dir="0 0 -1", directional="true",
                      castshadow="false", diffuse="0.5 0.5 0.5", specular="0 0 0")
        _geom(world, type="plane", size="15 15 0.1", rgba="0.82 0.85 0.89 1")
        for y in (-0.50, 0.50):
            _geom(world, type="box", size="15 0.006 0.001", pos=f"0 {y} 0.002", rgba="0.45 0.5 0.57 1")

        car = ET.SubElement(world, "body", name="vehicle", mocap="true")
        top = pad["top_height_m"]
        _geom(car, type="box", pos=_numbers([0.2, 0, top-0.051]),
              size=_numbers([config["vehicle"]["render_length_m"]/2,
                             config["vehicle"]["render_width_m"]/2, 0.045]),
              rgba="0.22 0.31 0.44 1")
        _geom(car, type="box", name="landing_pad", pos=_numbers([0, 0, top-0.002]),
              size=_numbers([pad["length_m"]/2, pad["width_m"]/2, 0.002]), rgba="0.85 0.92 0.86 1")
        # Center cross and edge stripes are vehicle artwork, not hardware dimensions.
        for size in ((0.065, 0.002, 0.001), (0.002, 0.065, 0.001)):
            _geom(car, type="box", size=_numbers(size), pos=_numbers([0, 0, top+0.001]), rgba="0.25 0.5 0.38 1")
        for x in (-0.09, 0.47):
            for y in (-0.18, 0.18):
                _geom(car, type="cylinder", pos=_numbers([x, y, top-0.10]),
                      size="0.055 0.018", quat="0.7071067812 0.7071067812 0 0", rgba="0.10 0.12 0.15 1")

        self.marker_position_vehicle = np.asarray(marker["offset_from_pad_m"], dtype=float)+[0, 0, top]
        mx, my, mz = self.marker_position_vehicle
        _geom(car, type="cylinder", fromto=_numbers([mx+0.008, my, top, mx+0.008, my, mz]),
              size="0.003", rgba="0.35 0.39 0.43 1")
        half_board = marker["length_m"]/2+marker["quiet_margin_m"]
        _geom(car, type="box", pos=_numbers([mx+0.003, my, mz]),
              size=_numbers([0.002, half_board, half_board]), rgba="1 1 1 1")
        module_size = marker["length_m"]/modules
        for row in range(modules):
            for col in range(modules):
                if self.marker_bits[row, col] == 0:
                    y = my+marker["length_m"]/2-(col+0.5)*module_size
                    z = mz+marker["length_m"]/2-(row+0.5)*module_size
                    _geom(car, type="box", pos=_numbers([mx+0.0005, y, z]),
                          size=_numbers([0.0005, module_size/2, module_size/2]), rgba="0 0 0 1")

        optical = ET.SubElement(world, "body", name="camera_rig", mocap="true")
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, ROTATION_WORLD_CAMERA_GL.reshape(9))
        ET.SubElement(optical, "camera", name="front_camera", quat=_numbers(quat), fovy=str(camera["vertical_fov_deg"]))
        ET.SubElement(optical, "site", name="optical_center_glyph", size="0.008",
                      rgba="0.2 0.45 0.95 1", group="3")
        ET.SubElement(optical, "site", name="optical_axis_glyph", type="capsule",
                      fromto="0 0 0 0.08 0 0", size="0.0015", rgba="0.2 0.45 0.95 1", group="3")
        self.xml = ET.tostring(xml, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.vehicle_mocap = self.model.body("vehicle").mocapid[0]
        self.camera_mocap = self.model.body("camera_rig").mocapid[0]
        self.camera_id = self.model.camera("front_camera").id
        self.set_time(0.)

    def set_time(self, seconds, height_override=None):
        trajectory = self.config["trajectory"]
        scaled = seconds/trajectory["duration_s"]*10.
        align = _smooth(scaled/2.)
        approach = _smooth((scaled-2.)/3.)
        descend = _smooth((scaled-7.)/3.)
        relative = np.array([
            trajectory["start_x_relative_pad_m"]*(1.-approach),
            trajectory["start_y_relative_pad_m"]*(1.-align),
            trajectory["camera_height_above_pad_m"]*(1.-descend)
            +trajectory["end_camera_height_above_pad_m"]*descend,
        ])
        if height_override is not None:
            relative[2] = height_override
        self.vehicle_position = np.array([self.config["vehicle"]["speed_m_s"]*seconds, 0., 0.])
        self.pad_position = self.vehicle_position+[0, 0, self.config["pad"]["top_height_m"]]
        self.optical_position = self.pad_position+relative
        yaw = math.radians(trajectory["initial_yaw_deg"])*(1.-align)
        self.data.mocap_pos[self.vehicle_mocap] = self.vehicle_position
        self.data.mocap_quat[self.vehicle_mocap] = [1, 0, 0, 0]
        self.data.mocap_pos[self.camera_mocap] = self.optical_position
        self.data.mocap_quat[self.camera_mocap] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        self.data.time = seconds
        mujoco.mj_forward(self.model, self.data)
        self.phase = ("align" if scaled < 2. else "approach" if scaled < 5.
                      else "match" if scaled < 7. else "descend")

    def world_to_camera(self, points):
        rotation_world_camera = self.data.cam_xmat[self.camera_id].reshape(3, 3) @ GL_TO_CV
        return (np.asarray(points)-self.data.cam_xpos[self.camera_id]) @ rotation_world_camera

    def marker_corners_camera(self):
        half = self.config["marker"]["length_m"]/2
        corners_marker = np.array([[-half, half, 0], [half, half, 0],
                                   [half, -half, 0], [-half, -half, 0]])
        corners_world = corners_marker @ ROTATION_WORLD_MARKER.T+self.vehicle_position+self.marker_position_vehicle
        return self.world_to_camera(corners_world)

    def marker_rotation_camera(self):
        rotation_world_camera = self.data.cam_xmat[self.camera_id].reshape(3, 3) @ GL_TO_CV
        return rotation_world_camera.T @ ROTATION_WORLD_MARKER

    def overview_camera(self):
        camera = mujoco.MjvCamera()
        camera.lookat[:] = self.pad_position+[0.16, 0, 0.10]
        camera.distance = 1.65
        camera.azimuth = -43.
        camera.elevation = -26.
        return camera

    def render(self, renderer, camera="front_camera", show_glyph=False):
        options = mujoco.MjvOption()
        options.sitegroup[3] = int(show_glyph)
        renderer.update_scene(self.data, camera=camera, scene_option=options)
        return renderer.render().copy()

    def save_assets(self):
        folder = ROOT / "assets/markers"
        folder.mkdir(parents=True, exist_ok=True)
        marker = self.config["marker"]
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, marker["dictionary"]))
        pixels = 600
        raw = cv2.aruco.generateImageMarker(dictionary, marker["id"], pixels)
        margin = round(pixels*marker["quiet_margin_m"]/marker["length_m"])
        path = folder / f'{marker["dictionary"].lower()}-id{marker["id"]}.png'
        canvas = cv2.copyMakeBorder(raw, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
        if not cv2.imwrite(str(path), canvas):
            raise OSError(f"Cannot save marker asset: {path}")
        return path
