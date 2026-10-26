"""마지막 영상 기반 예측·상대 Flow 확인·하강 제어.

GEOMETRY_READY는 물리 접촉 성공이 아니다. 접촉 관측과 모터 종료는
물리 실행기의 별도 센서 지원 경로가 검증한다.
"""
import numpy as np
from .guidance import tracking_demand


class LandingControl:
    def update(self, mission, frame):
        v, horizontal_ready, _ = tracking_demand(mission, frame)
        return self.finish(mission, frame, v, horizontal_ready)

    def finish(self, mission, frame, v, horizontal_ready):
        now = frame.now
        own = frame.own
        vehicle = frame.vehicle
        dt = frame.dt
        pos = frame.pos
        rel = frame.rel
        ff = frame.ff
        rel_v = frame.rel_v
        heading = frame.heading
        yaw_command = frame.yaw_command
        stable_straight = frame.stable_straight
        if mission.phase=="BLIND_APPROACH":
            if mission.held(now,np.linalg.norm(rel[:2])<mission.c["terminal_xy_gate_m"] and stable_straight,.2):
                mission.enter("BLIND_DESCEND",now,"bounded_prediction_pad_center")
        if mission.phase=="BLIND_DESCEND":
            relative_flow_ok=(own.get("flow_fusion_usable",False)
                              and own.get("flow_fusion_reason")=="pad_patch_contained")
            if relative_flow_ok:
                mission.relative_flow_unusable_since=None
            elif mission.relative_flow_unusable_since is None:
                mission.relative_flow_unusable_since=now
            # Optional physical-adapter handoff before texture/range loss.
            # The capability is command-model/IMU calibration evidence,
            # never a contact flag. Geometry runners and ROS transport
            # do not supply it. The default still requires relative Flow.
            inertial_ready=(mission.c.get("terminal_inertial_handoff_enabled",False)
                and own.get("inertial_support_handoff_ready",False)
                and not relative_flow_ok and stable_straight
                and np.linalg.norm(rel[:2])<mission.c["track_xy_gate_m"]
                and np.linalg.norm(rel_v[:2])<mission.c["track_speed_gate_m_s"]
                and pos[2]-mission.pad_height<mission.c["geometry_terminal_optical_height_m"]+.01
                and vehicle["age_s"]<mission.c.get("terminal_inertial_handoff_max_age_s",3.)
                and vehicle["last_image_quality"].get("min_side_px",0.)>=mission.c["last_fix_min_side_px"]
                and vehicle["last_image_quality"].get("marker_optical_depth_m",0.)>=mission.c["last_fix_min_depth_m"])
            if inertial_ready:
                if mission.inertial_handoff_since is None:mission.inertial_handoff_since=now
            else:mission.inertial_handoff_since=None
            if (mission.inertial_handoff_since is not None
                    and now-mission.inertial_handoff_since>=.20-1e-8):
                mission.geometry_ready=True;mission.support_handoff_mode="conditional_inertial_texture_bridge"
                mission.enter("GEOMETRY_READY",now,"near_pad_geometry_to_calibrated_IMU_support_adapter_without_relative_Flow")
                return mission.output(np.zeros(3),0.,dt,urgent=True)
            if (mission.relative_flow_unusable_since is not None
                    and now-mission.relative_flow_unusable_since >= mission.c.get("terminal_flow_grace_s",.25)-1e-8):
                return mission.start_blind_recovery("terminal_relative_flow_unusable",now,own,dt)
            remaining_s=max(0.,(pos[2]-mission.pad_height-mission.c["geometry_terminal_optical_height_m"])/mission.c["descent_speed_m_s"])
            margin_ok=mission.track.safe_straight_margin(np.asarray(vehicle["pad_world_m"])[:2],heading,
                np.linalg.norm(ff[:2])*(remaining_s+mission.c["post_motor_off_hold_s"]))
            ready=(relative_flow_ok and stable_straight and np.linalg.norm(rel[:2])<mission.c["track_xy_gate_m"]
                   and np.linalg.norm(rel_v[:2])<mission.c["track_speed_gate_m_s"] and margin_ok)
            v[2]=-mission.c["descent_speed_m_s"] if ready else 0.
            if not relative_flow_ok and not mission.c.get("terminal_flow_loss_slew_brake",False):
                mission.last_v[2]=max(0.,mission.last_v[2])
            # Optional bounded brake keeps the emitted command history.
            # The target above is already zero on Flow loss; ordinary 3D
            # slew must account for its remaining downward velocity.
            # Recovery/ABORT overrides remain separately logged.
            if mission.held(now,pos[2]-mission.pad_height<mission.c["geometry_terminal_optical_height_m"]+.01 and ready,.5):
                mission.geometry_ready=True
                mission.enter("GEOMETRY_READY",now,"bounded_terminal_optical_point_not_contact")
                return mission.output(np.zeros(3),0.,dt,urgent=True)
        if mission.phase=="DESCEND":
            descent_s=max(0.,(pos[2]-mission.pad_height-mission.c["geometry_terminal_optical_height_m"])/mission.c["descent_speed_m_s"])
            travel=np.linalg.norm(ff[:2])*(descent_s+mission.c["post_motor_off_hold_s"])
            margin_ok=mission.track.safe_straight_margin(np.asarray(vehicle["pad_world_m"])[:2],heading,travel)
            ready=horizontal_ready and stable_straight and margin_ok and vehicle["age_s"]<mission.cfg["observer"]["descent_max_age_s"]
            v[2]=-mission.c["descent_speed_m_s"] if ready else 0.
            if mission.held(now,pos[2]-mission.pad_height<mission.c["geometry_terminal_optical_height_m"]+.01 and ready,.5):
                mission.geometry_ready=True
                mission.enter("GEOMETRY_READY",now,"optical_center_near_pad_not_contact")
                return mission.output(np.zeros(3),0.,dt,urgent=True)
        return mission.output(v,yaw_command,dt)
