"""스타디움 트랙 기하. Live 궤적 생성은 plant에서만 사용한다."""
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


@dataclass
class Stadium:
    center: tuple[float, float]
    straight: float
    radius: float

    @classmethod
    def from_config(cls, cfg):
        return cls(tuple(cfg["center_world_xy_m"]), cfg["straight_length_m"], cfg["turn_diameter_m"] / 2)

    @property
    def length(self):
        return 2 * self.straight + 2 * math.pi * self.radius

    def pose(self, arc_m):
        """아래 직선 +X→우반원→위 직선 -X→좌반원. 접선 연속 CCW."""
        s = arc_m % self.length
        cx, cy = self.center
        L, r = self.straight, self.radius
        if s < L:
            return np.array([cx - L / 2 + s, cy - r]), 0., False
        s -= L
        if s < math.pi * r:
            a = -math.pi / 2 + s / r
            return np.array([cx + L / 2 + r * math.cos(a), cy + r * math.sin(a)]), wrap(a + math.pi / 2), True
        s -= math.pi * r
        if s < L:
            return np.array([cx + L / 2 - s, cy + r]), math.pi, False
        a = math.pi / 2 + (s - L) / r
        return np.array([cx - L / 2 + r * math.cos(a), cy + r * math.sin(a)]), wrap(a + math.pi / 2), True

    def nearest(self, xy):
        """사전 지도 위치로만 쓰는 최근접 기하. 차량 phase 참값 없음."""
        x, y = map(float, xy)
        cx, cy = self.center
        lo, hi = cx - self.straight / 2, cx + self.straight / 2
        candidates = [(np.array([np.clip(x, lo, hi), cy - self.radius]), False),
                      (np.array([np.clip(x, lo, hi), cy + self.radius]), False)]
        for center, right in ((np.array([hi, cy]), True), (np.array([lo, cy]), False)):
            delta = np.array([x, y]) - center
            a = math.atan2(delta[1], delta[0])
            if right:
                a = np.clip(a, -math.pi / 2, math.pi / 2)
            elif abs(a) < math.pi / 2:
                a = math.copysign(math.pi / 2, a or 1.)
            candidates.append((center + self.radius * np.array([math.cos(a), math.sin(a)]), True))
        point, turn = min(candidates, key=lambda row: np.linalg.norm(np.array([x, y]) - row[0]))
        return point, turn

    def safe_straight_margin(self, estimated_pad_xy, estimated_heading, predicted_travel):
        """추정 궤적의 직선 끝까지 여유. true track phase는 받지 않는다."""
        cx, cy = self.center
        x, y = estimated_pad_xy
        lower = abs(y - (cy - self.radius)) < .6 and abs(wrap(estimated_heading)) < math.radians(10)
        upper = abs(y - (cy + self.radius)) < .6 and abs(wrap(estimated_heading - math.pi)) < math.radians(10)
        if lower:
            remaining = cx + self.straight / 2 - x
        elif upper:
            remaining = x - (cx - self.straight / 2)
        else:
            return False
        return remaining > predicted_travel + .5


@dataclass
class Straight:
    """끝에서 멈추는 유한 직선. 좌표/방향은 정적 지도 사전정보다."""
    start: tuple[float, float]
    end: tuple[float, float]

    @property
    def length(self):
        return float(np.linalg.norm(np.asarray(self.end)-self.start))

    @property
    def straight(self):
        return self.length

    @property
    def center(self):
        return tuple((np.asarray(self.start)+self.end)/2)

    radius = 0.

    @property
    def heading(self):
        delta = np.asarray(self.end)-self.start
        return math.atan2(delta[1],delta[0])

    def pose(self, arc_m):
        direction=(np.asarray(self.end)-self.start)/self.length
        return np.asarray(self.start)+np.clip(arc_m,0.,self.length)*direction,self.heading,False

    def nearest(self, xy):
        direction=(np.asarray(self.end)-self.start)/self.length
        return self.pose(float((np.asarray(xy)-self.start)@direction))[0],False

    def safe_straight_margin(self, estimated_pad_xy, estimated_heading, predicted_travel):
        direction=(np.asarray(self.end)-self.start)/self.length
        delta=np.asarray(estimated_pad_xy)-self.start
        along=float(delta@direction)
        cross=abs(float(delta@np.array([-direction[1],direction[0]])))
        return (0. <= along <= self.length and cross < .6
                and abs(wrap(estimated_heading-self.heading)) < math.radians(10)
                and self.length-along > predicted_travel+.5)


def make_track(cfg):
    if cfg.get("kind","stadium") == "stadium":
        return Stadium.from_config(cfg)
    if cfg.get("kind") != "straight":
        raise ValueError("unknown track kind")
    points=np.asarray([cfg["start_world_xy_m"],cfg["end_world_xy_m"]],float)
    if points.shape != (2,2) or not np.isfinite(points).all() or np.linalg.norm(points[1]-points[0]) <= 0:
        raise ValueError("finite distinct straight endpoints required")
    return Straight(tuple(points[0]),tuple(points[1]))
