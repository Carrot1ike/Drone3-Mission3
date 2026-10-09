"""한 비행에서 공유하는 속도·가속도·yaw 포화 이력.

단계 전환 시 새 제한기를 만들지 않는다. urgent는 기존 안전 중단 정책을
그대로 유지하며, 설정된 명령 상한은 실제 기체 속력의 상한 보장이 아니다.
"""
import math
import numpy as np


class CommandLimiter:
    def __init__(self, control):
        self.c = control
        self.last_v = np.zeros(3)
        self.last_yaw_rate = 0.

    def limit(self, velocity, yaw_rate, dt, urgent=False):
        velocity = np.asarray(velocity, dtype=float).copy()
        velocity[2] = np.clip(velocity[2], -self.c.get("max_descent_speed_m_s", self.c["max_speed_m_s"]),
                              self.c.get("max_climb_speed_m_s", self.c["max_speed_m_s"]))
        norm = np.linalg.norm(velocity)
        if norm > self.c["max_speed_m_s"]:
            velocity *= self.c["max_speed_m_s"] / norm
        dv = velocity-self.last_v
        max_dv = self.c["max_accel_m_s2"] * dt
        if not urgent and np.linalg.norm(dv) > max_dv and max_dv > 0:
            velocity = self.last_v + dv * max_dv / np.linalg.norm(dv)
        self.last_v = velocity
        cap = math.radians(self.c["max_yaw_rate_deg_s"])
        yaw_rate = float(np.clip(yaw_rate,-cap,cap))
        if not urgent and "max_yaw_accel_deg_s2" in self.c:
            step = math.radians(self.c["max_yaw_accel_deg_s2"]) * dt
            yaw_rate = float(np.clip(yaw_rate,self.last_yaw_rate-step,self.last_yaw_rate+step))
        self.last_yaw_rate = yaw_rate
        return velocity.tolist(), yaw_rate
