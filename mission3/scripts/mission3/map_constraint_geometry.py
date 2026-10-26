"""정적 지도 + 단일 raw 상대 마커 자세의 관측 가능성 연구 전용.

실행 중 추정기/제어기에 연결하지 않는다. 이동 차량의 world pose를
별도 독립 측정으로 취급하지 않고 own pose와 raw 상대 측정을 결합한다.
지도 경로/차량 접선 일치, 보정/오차 상관성은 아직 조건부 가정이다.
"""
import math
import numpy as np
from .track import wrap


def _inputs(pose, relative_xy, beta):
    x = np.asarray(pose, float)
    q = np.asarray(relative_xy, float)
    if x.shape != (3,) or q.shape != (2,) or not np.all(np.isfinite(np.r_[x, q, beta])):
        raise ValueError('finite pose(px,py,yaw), relative horizontal xy and relative heading required')
    c, s = math.cos(x[2]), math.sin(x[2])
    rotated = np.array([[c, -s], [s, c]]) @ q
    derivative = np.array([-rotated[1], rotated[0]])
    return x, x[:2] + rotated, derivative


def circle_constraint(pose, relative_xy, beta, center_xy, radius, direction):
    """Residual[m,rad], Jacobian wrt(px,py,yaw), exact rotational null vector.

    direction is an image-derived candidate(+1/-1), never plant direction.
    A circle supplies two constraints for three own-pose variables; rotation
    of own pose and the observed cart together around its center is free.
    """
    x, marker, derivative = _inputs(pose, relative_xy, beta)
    center = np.asarray(center_xy, float)
    if center.shape != (2,) or not np.all(np.isfinite(center)) or not math.isfinite(radius) or radius <= 0 or direction not in (-1, 1):
        raise ValueError('finite circle with positive radius and candidate direction +/-1 required')
    d = marker - center
    distance = float(np.linalg.norm(d))
    if distance < 1e-6:
        raise ValueError('circle tangent undefined at center')
    normal = d / distance
    angle_gradient = np.array([-d[1], d[0]]) / distance**2
    tangent = math.atan2(d[1], d[0]) + direction*math.pi/2
    residual = np.array([distance-radius, wrap(x[2]+beta-tangent)])
    H = np.array([[*normal, normal @ derivative],
                  [*(-angle_gradient), 1-angle_gradient @ derivative]])
    own_radius = x[:2]-center
    null = np.array([-own_radius[1], own_radius[0], 1.])
    return residual, H, null


def straight_constraint(pose, relative_xy, beta, line_y, tangent):
    """Known horizontal line: along-line position remains unobservable."""
    x, marker, derivative = _inputs(pose, relative_xy, beta)
    if not np.all(np.isfinite([line_y, tangent])):
        raise ValueError('finite line and candidate tangent required')
    residual = np.array([marker[1]-line_y, wrap(x[2]+beta-tangent)])
    H = np.array([[0., 1., derivative[1]], [0., 0., 1.]])
    return residual, H, np.array([1., 0., 0.])
