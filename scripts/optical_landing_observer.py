"""영상만 사용하는 이상적 카메라 광학점 상대 관측기.

가상 광학점의 roll/pitch는 시험 계약상 0이다. 실제 AI-deck 장착이나
실기체 IMU와 동일하다고 가정하지 않는다. 참값 위치/yaw 입력은 없다.
"""
import math
import numpy as np
from yaw_core import MarkerYaw, rotation_rpy, wrap
from moving_landing_core import RelativeFilter
from provided_hardware import validate_camera_config, CameraContractError

# Virtual optical-heading axes: forward +X, left +Y, up +Z.
# This is not R_body_camera and is never stored in the hardware profile.
R_REFERENCE_CAMERA_CV = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


class OpticalLandingObserver:
    def __init__(self, profile, config):
        if not profile["source"].startswith("geometry-only"):
            raise ValueError("광학점 시험 전용 프로파일이 필요합니다")
        ideal = dict(profile, distortion=profile["ideal_distortion"])
        self.detector = MarkerYaw(ideal, config["observer"], reference_rotation_cv=R_REFERENCE_CAMERA_CV)
        self.filter = RelativeFilter(config["observer"])
        self.offset = np.asarray(profile["marker_offset_from_pad_m"])
        self.last_heading = None
        self.last_measurement = None

    def frame(self, gray, capture_estimate_s, receipt_s):
        if gray.ndim != 2 or gray.dtype != np.uint8:
            raise CameraContractError("관측기는 uint8 단색 영상을 받아야 합니다")
        validate_camera_config({"width": gray.shape[1], "height": gray.shape[0], "encoding": "mono8"})
        image = gray
        observed = self.detector.detect(image, 0., 0.)
        observed.update(capture_estimate_s=float(capture_estimate_s), receipt_s=float(receipt_s))
        if not observed["valid"]:
            return observed
        yaw = math.radians(observed["yaw_deg"])
        if self.last_heading is not None and abs(wrap(yaw-self.last_heading)) > math.radians(20.):
            observed.update(valid=False, reason="heading_innovation_rejected")
            return observed
        R_reference_camera = rotation_rpy(0., 0., yaw) @ R_REFERENCE_CAMERA_CV
        relative = R_reference_camera @ np.asarray(observed["t_camera_marker_m"]) - self.offset
        if not self.filter.observe(relative, capture_estimate_s, receipt_s):
            observed.update(valid=False, reason="relative_innovation_or_timestamp_rejected")
            return observed
        self.last_heading = yaw
        self.last_measurement = float(receipt_s)
        observed["raw_relative_pad_m"] = relative.tolist()
        observed["relative_pad_m"] = self.filter.position.tolist()
        observed["relative_velocity_m_s"] = self.filter.velocity.tolist()
        return observed

    def state(self, now_s):
        self.filter.predict(now_s)
        if self.last_measurement is None:
            return None
        return {"valid": True, "stamp_s": self.last_measurement,
                "prediction_stamp_s": now_s,
                "relative_pad_m": self.filter.position.tolist(),
                "relative_velocity_m_s": self.filter.velocity.tolist(),
                "heading_error_rad": float(wrap(-self.last_heading)),
                "reference_frame": "marker-defined optical-heading axes, not aircraft body"}
