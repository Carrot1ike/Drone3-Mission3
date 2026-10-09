"""관측 만료·표면 전환 실패의 제한된 후퇴·상승과 말기 안전 검사.

전역 판단의 실패를 받아 supervisor의 단계·명령 이력을 함께 사용한다.
"""
import math
import numpy as np


class RecoveryControl:
    def start_blind_recovery(self, mission, reason, now, own, dt):
        """Latch failure and the last image estimate, never a live plant state."""
        old = mission.last_vehicle
        ff = np.asarray(old["vehicle_velocity_world_m_s"] if old is not None
                        else own["velocity_world_m_s"], dtype=float).copy()
        ff[2] = 0.
        heading = old["vehicle_heading_rad"] if old is not None else own["heading_rad"]
        board_top = (mission.pad_height+mission.cfg["marker"]["known_marker_to_pad_vehicle_m"][2]
                     +mission.cfg["marker"]["board_side_m"]/2)
        safe_height = max(board_top+.10, mission.c.get("terminal_recovery_world_height_m", board_top+.10))
        mission.blind_recovery = {"started_s": now, "failure": reason, "ff": ff,
                               "heading": heading, "safe_height_m": safe_height,
                               "brake_started_s": None,
                               "ff_budget_s": max(0., mission.c.get("terminal_recovery_ff_s", 2.))}
        mission.enter("BLIND_RECOVER", now, reason)
        mission.events[-1].update(flow_fusion_reason=own.get("flow_fusion_reason", "surface_unconfirmed"),
                               safe_world_height_m=safe_height,
                               stale_ff_budget_s=mission.blind_recovery["ff_budget_s"])
        return mission.recover_blind(now, own, dt)


    def recover_blind(self, mission, now, own, dt):
        """Bounded cart FF/retreat, then sensor-based braking and vertical rise.

        A stalled climb stays active; altitude evidence must precede world-zero
        termination. It cannot extend the cart FF budget or restart descent.
        """
        recovery = mission.blind_recovery
        if own["position_world_m"][2] >= recovery["safe_height_m"]:
            return mission.abort(recovery["failure"], now, dt)
        if now-recovery["started_s"] < recovery["ff_budget_s"]-1e-8:
            heading = recovery["heading"]
            v = recovery["ff"]-mission.c["terminal_recovery_back_speed_m_s"]*np.array(
                [math.cos(heading), math.sin(heading), 0.])
        else:
            if recovery["brake_started_s"] is None:
                recovery["brake_started_s"] = now
                recovery["brake_velocity"] = np.asarray(own["velocity_world_m_s"][:2]).copy()
                speed = np.linalg.norm(recovery["brake_velocity"])
                if speed > mission.c["max_speed_m_s"]:
                    recovery["brake_velocity"] *= mission.c["max_speed_m_s"]/speed
                recovery["brake_duration_s"] = max(.5, np.linalg.norm(recovery["brake_velocity"])
                                                    /max(mission.c["max_accel_m_s2"], 1e-6))
                # Do not retain the old cart estimate beyond its finite budget.
                recovery.pop("ff")
                mission.enter("BLIND_RISE", now, "stale_cart_ff_budget_exhausted")
            fraction = max(0., 1.-(now-recovery["brake_started_s"])/recovery["brake_duration_s"])
            v = np.r_[recovery["brake_velocity"]*fraction, 0.]
        v[2] = mission.c["terminal_recovery_climb_speed_m_s"]
        mission.last_v[2] = max(0., mission.last_v[2])
        command = mission.output(v, 0., dt)
        # Stop descent immediately when its evidence is lost, while retaining
        # the ordinary horizontal speed/acceleration limits during recovery.
        command.velocity_world_m_s[2] = max(0., command.velocity_world_m_s[2])
        mission.last_v[2] = command.velocity_world_m_s[2]
        return command


    def terminal_safety_reason(self, mission, now, own, vehicle, prediction_started=None):
        """Sensor/estimate-only gate shared by the final physical adapter.

        Flow/accurate ToF are intentionally not required here: their published
        component ranges end before support. The bounded IMU support path must
        still retain prediction, map, odometry and straight-segment limits.
        Motor-off support monitoring is a separate non-restart state.
        """
        if now-mission.start_time_s >= mission.c["mission_timeout_s"]:
            return "mission_timeout"
        if vehicle is None or vehicle.get("kind"):
            return "terminal_state_unavailable"
        start = mission.terminal_started if prediction_started is None else prediction_started
        age = float(vehicle.get("age_s",float("inf")))
        if (start is None or not math.isfinite(age) or age < 0.
                or age >= mission.c["terminal_max_prediction_s"]-1e-8
                or now-start >= mission.c["terminal_max_prediction_s"]-1e-8):
            return "terminal_prediction_expired_before_touchdown"
        pos=np.asarray(own["position_world_m"],float)
        uncertainty=float(own["position_std_m"])
        if not math.isfinite(uncertainty) or uncertainty > mission.c["odometry_uncertainty_limit_m"]:
            return "odometry_uncertainty"
        margin=mission.c["net_clearance_m"]/2
        if not (np.isfinite(pos).all() and margin <= pos[0] <= mission.cfg["arena"]["length_m"]-margin
                and margin <= pos[1] <= mission.cfg["arena"]["width_m"]-margin and .05 < pos[2] < 4.5):
            return "estimated_arena_boundary"
        rate=float(vehicle["vehicle_heading_rate_rad_s"])
        heading=float(vehicle["vehicle_heading_rad"])
        speed=np.asarray(vehicle["vehicle_velocity_world_m_s"],float)
        pad=np.asarray(vehicle["pad_world_m"],float)
        rel=np.asarray(vehicle["relative_pad_world_m"],float)
        relative_speed=np.asarray(vehicle["relative_velocity_world_m_s"],float)
        if not (math.isfinite(rate) and math.isfinite(heading) and np.isfinite(speed).all()
                and np.isfinite(pad).all() and np.isfinite(rel).all() and np.isfinite(relative_speed).all()):
            return "terminal_state_nonfinite"
        if abs(rate) >= math.radians(mission.c["straight_max_yaw_rate_deg_s"]):
            return "terminal_straight_unconfirmed"
        remaining=max(0.,mission.c["terminal_max_prediction_s"]-(now-start))
        travel=np.linalg.norm(speed[:2])*(remaining+mission.c["post_motor_off_hold_s"])
        if not mission.track.safe_straight_margin(pad[:2],heading,travel):
            return "terminal_straight_margin"
        return None


