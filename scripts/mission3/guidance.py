"""추정값과 정적 치수에서 유도된 단계 제어 입력. Plant 참값 없음."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class GuidanceFrame:
    now: float
    own: dict
    vehicle: dict
    dt: float
    pos: np.ndarray
    yaw: float
    terminal: bool
    rel: np.ndarray
    ff: np.ndarray
    rel_v: np.ndarray
    heading: float
    forward: np.ndarray
    lateral: np.ndarray
    bearing: float
    yaw_command: float
    cross: float
    along: float
    straight: bool
    stable_straight: bool


def tracking_demand(mission, frame):
    """접근·마지막 예측 제어가 공유하는 원래 FF+feedback 계산."""
    pos = frame.pos
    terminal = frame.terminal
    rel = frame.rel
    ff = frame.ff
    rel_v = frame.rel_v
    forward = frame.forward
    bridge=mission.c["terminal_policy"]=="last_fix_bridge"
    offset=mission.c["last_fix_standoff_m"] if bridge and mission.phase in ("APPROACH","TRACK") else 0.
    target_rel=rel-forward*offset
    gain=mission.c["terminal_xy_kp_per_s"] if terminal else mission.c["xy_kp_per_s"]
    v=ff+gain*target_rel+mission.c["xy_kd"]*rel_v
    v[2]=.8*(mission.c.get("lower_align_world_height_m", .50)-pos[2])
    horizontal_ready=(np.linalg.norm(target_rel[:2])<mission.c["track_xy_gate_m"]
                      and np.linalg.norm(rel_v[:2])<mission.c["track_speed_gate_m_s"])
    return v, horizontal_ready, bridge
