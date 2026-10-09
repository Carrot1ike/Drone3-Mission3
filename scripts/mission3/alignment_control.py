"""관측 안정화와 차량 뒤 일직선 정렬. 차량 속도는 추정값 FF만 사용."""
import math
import numpy as np
from .track import wrap


class AlignmentControl:
    def update(self, mission, frame):
        now = frame.now
        vehicle = frame.vehicle
        dt = frame.dt
        pos = frame.pos
        yaw = frame.yaw
        rel = frame.rel
        ff = frame.ff
        rel_v = frame.rel_v
        heading = frame.heading
        forward = frame.forward
        bearing = frame.bearing
        yaw_command = frame.yaw_command
        cross = frame.cross
        along = frame.along
        stable_straight = frame.stable_straight
        if mission.phase == "ACQUIRE":
            e = rel.copy();e[2]=0
            v = ff.copy()
            v[2] = .8*(.70-pos[2])
            if mission.held(now,vehicle["samples"]>=6 and abs(wrap(bearing-yaw))<math.radians(8),.35):
                mission.enter("LINEUP",now,"relative_pose_and_estimated_motion_acquired")
            return mission.output(v,yaw_command,dt)
        if mission.phase == "LINEUP":
            target_e = rel-forward*mission.c["lineup_standoff_m"]
            target_e[2] = .70-pos[2]
            v = ff+mission.c["xy_kp_per_s"]*target_e+mission.c["xy_kd"]*rel_v
            v[2] = mission.c["height_kp_per_s"]*target_e[2]
            ready = (abs(cross)<mission.c["lineup_cross_tolerance_m"]
                     and abs(along-mission.c["lineup_standoff_m"])<mission.c["lineup_along_tolerance_m"]
                     and np.linalg.norm(rel_v[:2])<mission.c["lineup_relative_speed_m_s"]
                     and abs(wrap(heading-yaw))<math.radians(mission.c["lineup_heading_tolerance_deg"]))
            if mission.c.get("lineup_lowering_requires_straight",False):
                preparation_s = mission.c.get("lineup_preparation_budget_s",12.)
                travel = np.linalg.norm(ff[:2])*(preparation_s
                    +mission.c["terminal_max_prediction_s"]+mission.c["post_motor_off_hold_s"])
                ready = (ready and stable_straight
                    and mission.track.safe_straight_margin(np.asarray(vehicle["pad_world_m"])[:2],heading,travel))
            if mission.held(now,ready,mission.c["lineup_hold_s"]):
                mission.enter("LOWER_ALIGN",now,"behind_vehicle_lineup_stable")
            return mission.output(v,yaw_command,dt)
        return mission.abort("unhandled_phase", now, dt)
