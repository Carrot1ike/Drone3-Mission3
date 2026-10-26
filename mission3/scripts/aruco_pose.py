"""이상적 RGB/회색 렌더 영상에서 ArUco 마커의 카메라 좌표 자세를 구한다.

카메라 좌표는 +X 오른쪽, +Y 아래쪽, +Z 전방이다. 마커 좌표는 +X
오른쪽, +Y 위쪽이며, 원점은 마커 중심이다. 변환은 p_camera = R @
p_marker + t이고 t와 marker_length_m의 단위는 미터다. 모서리는 디코딩된
마커의 좌상, 우상, 우하, 좌하 순서다.

왜곡 0은 렌더링의 이상적 핀홀 투영 규약이다. 실제 카메라의 보정값이나
제조사 사양을 주장하지 않는다. ``python -B scripts/aruco_pose.py``로 실행하는
자체 검증도 수치/좌표/단위 검증이며 실제 장비 검증이 아니다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

import cv2
import numpy as np


__all__ = ["PoseEstimate", "detect_pose", "draw_detection", "pinhole_matrix"]


@dataclass
class PoseEstimate:
    """마커→카메라 변환과 관측 모서리; RMSE는 모서리별 2D 거리의 RMS다."""

    marker_id: int
    corners: np.ndarray  # (4, 2), pixels
    rotation_camera_marker: np.ndarray  # (3, 3)
    translation_camera_marker: np.ndarray  # (3,), meters
    reprojection_error_px: float


def _image_array(image: np.ndarray, *, rgb_only: bool = False) -> np.ndarray:
    if not isinstance(image, np.ndarray):
        raise TypeError("image must be a numpy.ndarray")
    if image.dtype != np.uint8:
        raise ValueError("image must have dtype uint8")
    is_rgb = image.ndim == 3 and image.shape[2] == 3
    is_gray = image.ndim == 2
    if not is_rgb and (rgb_only or not is_gray):
        raise ValueError("image must have shape (H, W, 3) RGB" if rgb_only else
                         "image must have shape (H, W) gray or (H, W, 3) RGB")
    if min(image.shape[:2]) < 2:
        raise ValueError("image width and height must each be at least 2 pixels")
    return np.ascontiguousarray(image)


def _finite_array(value: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
    if not isinstance(value, np.ndarray):
        raise TypeError(f"{name} must be a numpy.ndarray")
    if value.shape != shape or value.dtype.kind not in "fiu":
        raise ValueError(f"{name} must be a real numeric array with shape {shape}")
    converted = np.ascontiguousarray(value, dtype=np.float64)
    if not np.isfinite(converted).all():
        raise ValueError(f"{name} must contain only finite values")
    return converted


def _positive_real(value: float, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return number


def _integer(value: int, name: str) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    return int(value)


def pinhole_matrix(width: int, height: int, vertical_fov_deg: float) -> np.ndarray:
    """정사각 픽셀의 이상적 렌더 투영 행렬을 만든다(제조사 프로필 아님).

    fy = height / (2 tan(vertical_fov / 2)), fx = fy이며 주점은
    (width / 2, height / 2)이다. 크기는 양의 정수, 수직 FOV는 (0, 180)도다.
    """
    width = _integer(width, "width")
    height = _integer(height, "height")
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    fov = _positive_real(vertical_fov_deg, "vertical_fov_deg")
    if fov >= 180:
        raise ValueError("vertical_fov_deg must be less than 180 degrees")
    tangent = math.tan(math.radians(fov) / 2)
    if tangent <= 0:
        raise ValueError("vertical_fov_deg is too small for a finite focal length")
    focal = height / (2 * tangent)
    if not math.isfinite(focal) or focal <= 0:
        raise ValueError("image size and FOV must produce a finite positive focal length")
    return np.array([[focal, 0.0, width / 2],
                     [0.0, focal, height / 2],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def detect_pose(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    marker_length_m: float,
    dictionary_name: str = "DICT_4X4_50",
    expected_id: int = 0,
) -> PoseEstimate | None:
    """지정 ID의 자세를 추정한다. 부재/수치적 PnP 실패 시 None을 반환한다.

    입력은 uint8 RGB 또는 회색 영상, 유한한 (3, 3) 핀홀 행렬, 양의 미터
    길이다. 행렬은 양의 fx/fy, 왜도 0, 마지막 행 [0, 0, 1]을 요구한다.
    잘못된 입력/사전 이름/ID 범위는 TypeError 또는 ValueError로 알린다.

    ArucoDetector와 IPPE_SQUARE의 복수 해를 LM으로 정밀화한다. 네 모서리가 모두
    양의 카메라 깊이인 해 중 sqrt(mean(sum((projected - observed)**2)))가
    최소인 해를 선택한다. 같은 ID가 여러 번 보이면 전체 후보 중 선택한다.
    평면 자세의 모호성은 남을 수 있으므로 RMSE는 자세 정확도의 보증이 아니다.
    광축 중심의 정확한 정면 대칭 관측에서는 OpenCV IPPE의 수치적 퇴화로
    큰 RMSE의 해가 나올 수 있다. 호출자는 반환된 RMSE의 적합성을 판단한다.
    왜곡 계수 0은 이상적 렌더 규약이며 실제 카메라 보정값이 아니다.
    """
    image = _image_array(image)
    matrix = _finite_array(camera_matrix, (3, 3), "camera_matrix")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera_matrix fx and fy must be positive")
    if (matrix[0, 1] != 0 or matrix[1, 0] != 0
            or not np.array_equal(matrix[2], [0.0, 0.0, 1.0])):
        raise ValueError("camera_matrix must be a zero-skew pinhole matrix")
    length = _positive_real(marker_length_m, "marker_length_m")
    if not isinstance(dictionary_name, str):
        raise TypeError("dictionary_name must be a string")
    if not dictionary_name.startswith("DICT_") or not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"unknown ArUco dictionary: {dictionary_name!r}")
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    except (cv2.error, TypeError, ValueError) as exc:
        raise ValueError(f"unknown ArUco dictionary: {dictionary_name!r}") from exc
    marker_id = _integer(expected_id, "expected_id")
    if not 0 <= marker_id < dictionary.bytesList.shape[0]:
        raise ValueError(f"expected_id must be in [0, {dictionary.bytesList.shape[0] - 1}]")

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    detected_corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    matches = np.flatnonzero(ids.reshape(-1) == marker_id)
    if not matches.size:
        return None

    half = length / 2
    object_points = np.array([[-half, half, 0.0],
                              [half, half, 0.0],
                              [half, -half, 0.0],
                              [-half, -half, 0.0]], dtype=np.float64)
    distortion = np.zeros((5, 1), dtype=np.float64)
    best: PoseEstimate | None = None
    for index in matches:
        corners = np.ascontiguousarray(detected_corners[index], dtype=np.float64).reshape(4, 2)
        if not np.isfinite(corners).all():
            continue
        try:
            success, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                object_points, corners, matrix, distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error:
            continue
        if not success:
            continue
        for rvec, tvec in zip(rvecs, tvecs):
            rvec = np.ascontiguousarray(rvec, dtype=np.float64).reshape(3, 1)
            translation = np.asarray(tvec, dtype=np.float64).reshape(3).copy()
            if not (np.isfinite(rvec).all() and np.isfinite(translation).all()):
                continue
            try:
                rotation, _ = cv2.Rodrigues(rvec)
                camera_points = object_points @ rotation.T + translation
                if not np.isfinite(camera_points).all() or not (camera_points[:, 2] > 0).all():
                    continue
                # IPPE is an initial planar solution. Refine both branches against
                # the observed corners; simulator truth is never used here.
                rvec, refined_tvec = cv2.solvePnPRefineLM(
                    object_points, corners, matrix, distortion, rvec,
                    translation.reshape(3, 1),
                    criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_COUNT, 30, 1e-8),
                )
                rotation, _ = cv2.Rodrigues(rvec)
                translation = np.asarray(refined_tvec, dtype=np.float64).reshape(3).copy()
                camera_points = object_points @ rotation.T+translation
                if not np.isfinite(camera_points).all() or not (camera_points[:, 2] > 0).all():
                    continue
                projected, _ = cv2.projectPoints(object_points, rvec, translation, matrix, distortion)
            except cv2.error:
                continue
            residuals = projected.reshape(4, 2) - corners
            rmse = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))
            if math.isfinite(rmse) and (best is None or rmse < best.reprojection_error_px):
                best = PoseEstimate(marker_id, corners.copy(), rotation.copy(), translation, rmse)
    return best


def draw_detection(image_rgb: np.ndarray, estimate: PoseEstimate | None) -> np.ndarray:
    """RGB 복사본에 모서리와 ID/RMSE를 그린다. 원본 변경이나 GUI 호출은 없다."""
    result = _image_array(image_rgb, rgb_only=True).copy()
    if estimate is None:
        return result
    if not isinstance(estimate, PoseEstimate):
        raise TypeError("estimate must be PoseEstimate or None")
    corners = _finite_array(estimate.corners, (4, 2), "estimate.corners")
    height, width = result.shape[:2]
    pixels = np.rint(np.clip(corners, [0, 0], [width - 1, height - 1])).astype(np.int32)
    cv2.polylines(result, [pixels.reshape(4, 1, 2)], True, (60, 220, 80), 1, cv2.LINE_AA)
    for pixel in pixels:
        cv2.circle(result, tuple(int(v) for v in pixel), 2, (255, 80, 60), -1, cv2.LINE_AA)
    label = f"ID {estimate.marker_id} | RMSE {estimate.reprojection_error_px:.2f} px"
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1
    (text_width, text_height), baseline = cv2.getTextSize(label, font, scale, thickness)
    x = max(0, min(int(pixels[:, 0].min()), width - text_width - 4))
    y = int(pixels[:, 1].min()) - 7
    if y < text_height + 3:
        y = int(pixels[:, 1].max()) + text_height + 7
    y = max(text_height + 3, min(y, height - baseline - 3))
    cv2.rectangle(result, (x, y - text_height - 3), (x + text_width + 4, y + baseline + 3),
                  (0, 0, 0), -1)
    cv2.putText(result, label, (x + 2, y), font, scale, (255, 230, 80), thickness, cv2.LINE_AA)
    return result


def _self_check() -> None:
    """10 cm 마커, 폭 320/FOV 87도는 임시 실험 설정이며 장비 사양이 아니다."""
    width, height, length = 320, 240, 0.10
    matrix = pinhole_matrix(width, height, 87.0)
    half = length / 2
    object_points = np.array([[-half, half, 0.0], [half, half, 0.0],
                              [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)
    # 정면, 광축에서 약간 벗어난 위치. 마커 +Y를 영상 위쪽에 맞춘다.
    rvec = np.array([math.pi, 0.0, 0.0], dtype=np.float64)
    translation = np.array([0.012, -0.016, 0.50], dtype=np.float64)
    projected, _ = cv2.projectPoints(object_points, rvec, translation, matrix, None)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 0, 256)
    source = np.array([[0, 0], [255, 0], [255, 255], [0, 255]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(np.float32))
    gray = cv2.warpPerspective(marker, homography, (width, height),
                               flags=cv2.INTER_NEAREST, borderValue=255)
    rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    estimate = detect_pose(rgb, matrix, length)
    assert estimate is not None, "generated ideal marker was not detected"
    assert estimate.marker_id == 0 and estimate.corners.shape == (4, 2)
    assert estimate.rotation_camera_marker.shape == (3, 3)
    assert estimate.translation_camera_marker.shape == (3,)
    assert estimate.translation_camera_marker[2] > 0.30
    assert np.linalg.norm(estimate.translation_camera_marker - translation) < 0.03
    camera_points = object_points @ estimate.rotation_camera_marker.T + estimate.translation_camera_marker
    assert (camera_points[:, 2] > 0).all()
    assert np.allclose(estimate.rotation_camera_marker.T @ estimate.rotation_camera_marker, np.eye(3))
    assert np.isclose(np.linalg.det(estimate.rotation_camera_marker), 1.0)
    assert estimate.rotation_camera_marker[2, 2] < -0.9
    assert estimate.reprojection_error_px < 1.0
    gray_estimate = detect_pose(gray, matrix, length)
    assert gray_estimate is not None
    assert np.allclose(gray_estimate.translation_camera_marker, estimate.translation_camera_marker)
    assert detect_pose(rgb, matrix, length, expected_id=1) is None
    original = rgb.copy()
    annotated = draw_detection(rgb, estimate)
    assert annotated.shape == rgb.shape and annotated.dtype == np.uint8
    assert np.array_equal(rgb, original) and not np.array_equal(annotated, original)
    blank_annotation = draw_detection(rgb, None)
    assert np.array_equal(blank_annotation, rgb) and not np.shares_memory(blank_annotation, rgb)
    print(f"PASS ideal numerical/unit check (OpenCV {cv2.__version__}, NumPy {np.__version__}): "
          f"known z=0.500 m, estimated z={estimate.translation_camera_marker[2]:.4f} m, "
          f"RMSE={estimate.reprojection_error_px:.4f} px. 실제 장비 검증 아님.")


if __name__ == "__main__":
    _self_check()
