"""속도 명령을 적분하는 카메라 광학점 운동학 시험대. 기체 모델 없음."""
import math
import cv2
import mujoco
import numpy as np
from moving_landing_scene import build_scene, build_optical_preview_model, camera_profile
from optical_landing_observer import R_REFERENCE_CAMERA_CV
from provided_hardware import decode_mono320
from yaw_core import rotation_rpy, wrap


class OpticalPlant:
    def __init__(self, config, out):
        self.config = config
        self.model = build_optical_preview_model(config, build_scene(config, out))
        self.data = mujoco.MjData(self.model)
        self.profile = camera_profile(self.model, config)
        self.camera = self.model.camera("optical_preview").id
        self.slide = self.model.joint("vehicle_slide").qposadr[0]
        self.position = np.array([config["spawn"].get("x_m", 0.), config["spawn"]["y_m"], config["control"]["height_m"]])
        self.heading = math.radians(config["spawn"]["yaw_deg"])
        self.travel = 0.
        self.vehicle_started = config["vehicle"].get("motion_mode") == "continuous"
        self.last_velocity = np.zeros(3)
        self.pad_origin = np.asarray(config["vehicle"]["initial_pad_center_m"])
        self.renderer = mujoco.Renderer(self.model, height=320, width=320)

    def step(self, command, dt):
        self.vehicle_started |= bool(command["start_vehicle"])
        if self.vehicle_started:
            self.travel += self.config["vehicle"]["speed_m_s"] * dt
        # The only camera-state update is integration of controller output.
        # No target-relative camera path, pose correction, weld or teleport.
        self.last_velocity = np.asarray(command["velocity_m_s"], dtype=float)
        self.position += self.last_velocity * dt
        self.heading = wrap(self.heading + command["yaw_rate_rad_s"] * dt)

    def render(self):
        self.data.qpos[self.slide] = self.travel
        self.model.cam_pos[self.camera] = self.position
        R = rotation_rpy(0., 0., self.heading) @ R_REFERENCE_CAMERA_CV @ np.diag([1., -1., -1.])
        mujoco.mju_mat2Quat(self.model.cam_quat[self.camera], R.flatten())
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="optical_preview")
        gray = (self.renderer.render() @ np.array([.2989, .5870, .1140])).astype(np.uint8)
        return decode_mono320(320, 320, "mono8", 320, gray)

    def truth(self, now, command):
        pad = self.pad_origin + [self.travel, 0., 0.]
        relative = pad - self.position
        velocity = np.array([self.config["vehicle"]["speed_m_s"] if self.vehicle_started else 0., 0., 0.]) - self.last_velocity
        return {"time_s": now, "camera_x_m": float(self.position[0]), "camera_y_m": float(self.position[1]),
                "camera_z_m": float(self.position[2]), "heading_deg": math.degrees(self.heading),
                "pad_x_m": float(pad[0]), "pad_y_m": float(pad[1]), "pad_z_m": float(pad[2]),
                "relative_x_m": float(relative[0]), "relative_y_m": float(relative[1]),
                "height_above_pad_m": float(-relative[2]), "relative_speed_m_s": float(np.linalg.norm(velocity[:2])),
                "vehicle_travel_m": self.travel, "phase": command["phase"],
                "vehicle_moving": self.vehicle_started,
                "vx_command_m_s": command["velocity_m_s"][0], "vy_command_m_s": command["velocity_m_s"][1],
                "vz_command_m_s": command["velocity_m_s"][2], "yaw_rate_command_rad_s": command["yaw_rate_rad_s"]}

    def close(self):
        self.renderer.close()


def annotated_image(gray, phase, observation):
    image = gray.copy()
    cv2.rectangle(image, (0, 0), (319, 36), 0, -1)
    cv2.putText(image, f"OPTICAL TEST | {phase}", (4, 13), cv2.FONT_HERSHEY_SIMPLEX, .36, 255, 1)
    if observation is not None:
        e = observation["relative_pad_m"]
        cv2.putText(image, f"eXY {np.linalg.norm(e[:2]):.3f}m | h {-e[2]:.3f}m", (4, 29), cv2.FONT_HERSHEY_SIMPLEX, .36, 255, 1)
    return image
