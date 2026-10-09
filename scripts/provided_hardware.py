"""Fixed supplied-camera contract and explicit hardware-model eligibility.

This module does not fill unpublished parameters or resample incoming images.
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HARDWARE_PATH = ROOT / "config/provided-hardware.json"


class CameraContractError(ValueError):
    pass


class HardwareModelUnresolved(ValueError):
    pass


def hardware_profile():
    return json.loads(HARDWARE_PATH.read_text())


def validate_camera_config(camera):
    expected = hardware_profile()["camera"]
    for key, required in (("width", expected["width"]), ("height", expected["height"]),
                          ("encoding", expected["ros_encoding"])):
        if camera.get(key) != required:
            raise CameraContractError(f"제공 카메라 고정 조건: {key}={required}, 입력={camera.get(key)}")
    if "vertical_fov_deg" in camera and camera["vertical_fov_deg"] != expected["nominal_vertical_fov_deg"]:
        raise CameraContractError("제조사 공칭 수직 FOV 87도를 임의로 변경할 수 없습니다")


def decode_mono320(width, height, encoding, step, data):
    """Validate a ROS Image and expose pixels without resize/color conversion.

    Row padding is allowed by ROS; it is excluded without resampling the image.
    """
    validate_camera_config({"width": width, "height": height, "encoding": encoding})
    if not isinstance(step, (int, np.integer)) or isinstance(step, bool) or step < width:
        raise CameraContractError("잘못된 mono8 행 바이트 수")
    raw = memoryview(data)
    if raw.nbytes != height * step:
        raise CameraContractError(f"잘못된 영상 길이: {raw.nbytes}, 기대값={height * step}")
    return np.frombuffer(raw, dtype=np.uint8).reshape(height, step)[:, :width]


def require_manufacturer_flight_model():
    profile = hardware_profile()
    aircraft, camera = profile["aircraft"], profile["camera"]
    missing = [f"aircraft.{key}" for key in (
        "assembled_mass_kg", "assembled_center_of_mass_m", "assembled_inertia_kg_m2",
        "applicable_pwm_to_thrust_model", "motor_response_model", "aerodynamic_model",
        "manufacturer_validated_simulation_model") if aircraft[key] is None]
    missing += [f"camera.{key}" for key in ("calibrated_K", "calibrated_distortion",
                                           "R_body_camera_cv", "camera_position_body_m")
                if camera[key] is None]
    if missing:
        raise HardwareModelUnresolved("제공 기체 동역학·착륙 검증 모델 미확정: " + ", ".join(missing))
    return profile


def require_camera_body_transform(profile):
    """Refuse use of a shared-model/ideal transform as supplied-aircraft data."""
    if (profile.get("body_transform_source") != "manufacturer_verified"
            or profile.get("R_body_camera_cv") is None
            or profile.get("camera_position_body_m") is None):
        raise HardwareModelUnresolved("제조사 근거가 확인된 카메라→기체 변환이 필요합니다")
