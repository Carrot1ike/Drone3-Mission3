"""Draft relative-pose/FF calculations; aircraft conversion requires verified mounting.

No flight or physical-landing validation has been performed for this module.
"""
import math
import numpy as np
from yaw_core import rotation_rpy, rate_command
from provided_hardware import require_camera_body_transform


def relative_pad_position(observation, roll, pitch, profile):
    require_camera_body_transform(profile)
    Rwb = rotation_rpy(roll, pitch, math.radians(observation["yaw_deg"]))
    Rbc = np.array(profile["R_body_camera_cv"])
    p_body_marker = Rbc @ np.array(observation["t_camera_marker_m"]) + np.array(profile["camera_position_body_m"])
    return Rwb @ p_body_marker - np.array(profile["marker_offset_from_pad_m"])


class RelativeFilter:
    def __init__(self, settings):
        self.settings = settings
        self.position = None
        self.velocity = np.zeros(3)
        self.stamp = None
        self.last_capture = None
        self.last_visual = None

    def predict(self, stamp):
        if self.position is not None and self.stamp is not None:
            self.position += self.velocity * max(0., min(stamp-self.stamp, .20))
        self.stamp = stamp

    def observe(self, raw_position, capture_stamp, now):
        raw_position = np.asarray(raw_position, dtype=float)
        if (raw_position.shape != (3,) or not np.all(np.isfinite(raw_position))
                or capture_stamp > now or (self.last_capture is not None and capture_stamp <= self.last_capture)
                or (self.stamp is not None and now < self.stamp)):
            return False
        if self.position is None:
            self.position = raw_position.copy()
            self.stamp, self.last_capture, self.last_visual = now, capture_stamp, now
            return True
        self.predict(now)
        aligned = np.asarray(raw_position) + self.velocity * min(.25, max(0., now-capture_stamp))
        residual = aligned-self.position
        if np.linalg.norm(residual) > self.settings["max_pose_innovation_m"]:
            return False
        dt = max(.025, min(.15, capture_stamp-self.last_capture))
        self.position += self.settings["relative_alpha"]*residual
        self.velocity += self.settings["relative_beta"]*residual/dt
        self.velocity = np.clip(self.velocity, -.8, .8)
        self.last_capture, self.last_visual = capture_stamp, now
        return True


def surface_is_pad(relative_position, roll, pitch, yaw, tof_m, config):
    if tof_m is None or relative_position is None:
        return False
    h = -relative_position[2]
    R = rotation_rpy(roll, pitch, yaw)
    down = R @ np.array([0., 0., -1.])
    if h <= 0 or abs(down[2]) < .7:
        return False
    foot_xy = down[:2] * (-h/down[2])
    # Conservative interior estimate; actual model ray and camera/ToF extrinsics
    # remain a simplified simulation sensor model.
    within = np.all(np.abs(relative_position[:2]-foot_xy) < np.array([config["vehicle"]["pad_length_m"], config["vehicle"]["pad_width_m"]])/2-.005)
    return bool(within and abs(tof_m*(-down[2])-h) < .06)


def horizontal_command(relative_position, relative_velocity, surface_fraction, config):
    c = config["control"]
    ff = np.array([config["vehicle"]["speed_m_s"], 0.]) if config["test"]["ff_enabled"] else np.zeros(2)
    if config["test"]["surface_compensation_enabled"]:
        ff *= (1.-surface_fraction)
    feedback = c["xy_kp_per_s"]*relative_position[:2] + c["xy_kd"]*relative_velocity[:2]
    command = ff + feedback
    magnitude = np.linalg.norm(command)
    if magnitude > c["max_velocity_m_s"]:
        command *= c["max_velocity_m_s"]/magnitude
    return command, ff, feedback
