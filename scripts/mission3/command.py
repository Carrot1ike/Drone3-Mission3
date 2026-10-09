"""단계 제어기의 공통 명령 형식. 접촉 성공 판정과 분리한다."""
from dataclasses import dataclass


@dataclass
class Command:
    phase: str
    velocity_world_m_s: list[float]
    yaw_rate_rad_s: float
    done: bool = False
    failure: str | None = None
    motors_off: bool = False
    physical_success: bool = False
    coarse_guard: dict | None = None
    search_guard: dict | None = None
    approach_reference: dict | None = None

