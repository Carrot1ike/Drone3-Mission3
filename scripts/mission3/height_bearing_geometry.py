"""고정 마커 중심 높이와 decoded quad로 얻는 조건부 거리 연구 전용.

차량 yaw/pad 위치/차속을 만들지 않는다. native 좌표를 undistort하며
영상을 확대/변환하지 않는다. K/D/mount/높이 datum의 실기 보정은U다.
"""
import math
import cv2
import numpy as np
from yaw_core import rotation_rpy


def diagonal_center(points):
    p = np.asarray(points, float)
    if p.shape != (4, 2) or not np.all(np.isfinite(p)):
        raise ValueError('finite four ordered corners required')
    a = np.cross(np.r_[p[0], 1.], np.r_[p[2], 1.])
    b = np.cross(np.r_[p[1], 1.], np.r_[p[3], 1.])
    x = np.cross(a, b)
    if abs(x[2]) < 1e-10:
        raise ValueError('degenerate quad diagonals')
    center = x[:2]/x[2]
    if not np.all(np.isfinite(center)) or cv2.pointPolygonTest(p.astype(np.float32), tuple(center), False) < 0:
        raise ValueError('diagonal intersection outside decoded quad')
    return center


def relative_marker(corners, own_height, roll, pitch, marker_height, K, D, Rbc, camera_offset,
                    minimum_elevation_deg=6.):
    corners = np.asarray(corners, float)
    if corners.shape != (4, 2) or not np.all(np.isfinite(corners)) or np.any(corners < 0) or np.any(corners > 319):
        raise ValueError('decoded corners must be in native320 image; no resize')
    if not np.all(np.isfinite([own_height, roll, pitch, marker_height])):
        raise ValueError('finite estimated height/attitude and known marker datum required')
    if not math.isfinite(minimum_elevation_deg) or not 0 < minimum_elevation_deg < 45:
        raise ValueError('positive elevation conditioning margin required')
    normalized = cv2.undistortPoints(corners[:, None, :], np.asarray(K, float), np.asarray(D, float)).reshape(4, 2)
    center = diagonal_center(normalized)
    R = rotation_rpy(roll, pitch, 0.)
    ray = R @ np.asarray(Rbc) @ np.r_[center, 1.]
    offset = R @ np.asarray(camera_offset)
    if abs(ray[2])/np.linalg.norm(ray) < math.sin(math.radians(minimum_elevation_deg)):
        raise ValueError('near-horizontal bearing has ill-conditioned height range')
    factor = (marker_height-own_height-offset[2])/ray[2]
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError('height ray intersects marker plane behind camera')
    point = offset + factor*ray
    return point, {'camera_ray_scale_m':float(factor),
        'slant_distance_from_camera_m':float(factor*np.linalg.norm(ray)),
        'elevation_deg':math.degrees(math.asin(ray[2]/np.linalg.norm(ray))),
        'undistorted_diagonal_center':center.tolist(),
        'scope':'relative MARKER center, not pad pose or vehicle heading/speed; known vertical datum and calibration assumptions required'}


def conditional_covariance(corners, own_height, roll, pitch, marker_height, K, D, Rbc, camera_offset,
                           pixel_std, height_std_m, tilt_std_deg):
    """First-order S sensitivity only; shared systematic calibration excluded.

    No correlated world pose posterior or calibrated probability is asserted.
    """
    if not np.all(np.isfinite([pixel_std,height_std_m,tilt_std_deg])) or min(pixel_std,height_std_m,tilt_std_deg) <= 0:
        raise ValueError('positive finite sensitivity settings required')
    state = np.r_[np.asarray(corners, float).ravel(), own_height, roll, pitch]
    sigmas = np.r_[[pixel_std]*8, height_std_m, [math.radians(tilt_std_deg)]*2]
    J = np.zeros((2, len(state)))
    for i in range(len(state)):
        delta = np.eye(len(state))[i]*(1e-4 if i < 8 else 1e-6)
        def fit(x):
            return relative_marker(x[:8].reshape(4, 2), *x[8:], marker_height, K, D, Rbc, camera_offset)[0][:2]
        J[:, i] = (fit(state+delta)-fit(state-delta))/(2*np.linalg.norm(delta))
    covariance = (J*sigmas) @ (J*sigmas).T
    return covariance, J
