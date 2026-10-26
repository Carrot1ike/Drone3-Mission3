"""평가용 화각·마커 투영. 이상적인 가상 카메라 좌표이며 실기 보정 아님.

삼각형의 각도는 카메라 투영에서, 인식 후보 영역은 가상 마커 전체의
화면 포함·정면 여부·픽셀 크기에서 계산한다. 실제 인식은 영상 관측 로그다.
이 모듈의 참값·후보 영역은 제어기 입력에 연결하지 않는다.
"""
import math
import numpy as np
from optical_landing_observer import R_REFERENCE_CAMERA_CV
from yaw_core import rotation_rpy


def horizontal_fov_deg(profile):
    K = np.asarray(profile["K"], dtype=float)
    # 주점이 중앙이라는 현재 이상적 프로파일에 한정한다.
    return math.degrees(2*math.atan(profile["width"]/(2*K[0, 0])))


def project_marker(centers_relative_world_m, heading_rad, profile, min_side_px):
    """마커 중심 벡터 (...,3)를 현재 가상 카메라에 투영한다."""
    centers = np.asarray(centers_relative_world_m, dtype=float)
    half = profile["marker_length_m"]/2
    corners = np.array([[-half, half, 0.], [half, half, 0.], [half, -half, 0.], [-half, -half, 0.]])
    Rwm = np.asarray(profile["R_world_marker"])
    Rcw = R_REFERENCE_CAMERA_CV.T @ rotation_rpy(0., 0., heading_rad).T
    xyz = (centers[..., None, :]+corners @ Rwm.T) @ Rcw.T
    projected = xyz @ np.asarray(profile["K"]).T
    safe_depth = np.where(np.abs(projected[..., 2:]) > 1e-9, projected[..., 2:], 1e-9)
    uv = projected[..., :2]/safe_depth
    positive = np.all(xyz[..., 2] > .02, axis=-1)
    inside = (positive & np.all(uv[..., 0] >= 3., axis=-1)
        & np.all(uv[..., 0] <= profile["width"]-4., axis=-1)
        & np.all(uv[..., 1] >= 3., axis=-1)
        & np.all(uv[..., 1] <= profile["height"]-4., axis=-1))
    side = np.min(np.linalg.norm(uv-np.roll(uv, 1, axis=-2), axis=-1), axis=-1)
    front = (-centers @ Rwm[:, 2]) > 0.
    center_camera = centers @ Rcw.T
    half_angle = math.radians(horizontal_fov_deg(profile)/2)
    center_in_horizontal_fov = (center_camera[..., 2] > 0.) & (np.abs(center_camera[..., 0]) <= center_camera[..., 2]*math.tan(half_angle))
    return {"pixel_corners": uv, "min_side_px": side, "front_facing": front,
        "all_corners_in_frame": inside, "center_in_horizontal_fov": center_in_horizontal_fov,
        "geometric_candidate": inside & front & (side >= min_side_px)}


def display_masks(shape, camera_xy, pixels_per_m, heading_rad, camera_z_m, marker_z_m, profile, min_side_px, step=4):
    """도식 픽셀이 뜻하는 가상 마커 중심의 후보 영역. 카메라 입력은 그대로다."""
    height, width = shape
    yy, xx = np.mgrid[0:height:step, 0:width:step]
    dx, dy = (xx-camera_xy[0])/pixels_per_m, -(yy-camera_xy[1])/pixels_per_m
    forward = dx*np.cos(heading_rad)+dy*np.sin(heading_rad)
    lateral = -dx*np.sin(heading_rad)+dy*np.cos(heading_rad)
    fov = (forward > 0.) & (np.abs(lateral) <= forward*np.tan(math.radians(horizontal_fov_deg(profile)/2)))
    centers = np.stack([dx, dy, np.full_like(dx, marker_z_m-camera_z_m)], axis=-1)
    projected = project_marker(centers, heading_rad, profile, min_side_px)
    return fov, projected["geometric_candidate"]
