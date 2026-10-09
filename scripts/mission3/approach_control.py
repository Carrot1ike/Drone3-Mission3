"""카메라 가시성을 위한 높이 준비, 접근, 상대속도 동기화.

기존 staged/옵트인 coordinated 정책과 확인 시간을 유지한다.
전환한 바로 그 step의 계산 순서를 유지하도록 landing.finish로 이어준다.
"""
import numpy as np
from .approach_reference import CoordinatedApproach
from .guidance import tracking_demand


class ApproachControl:
    def update(self, mission, frame):
        now = frame.now
        vehicle = frame.vehicle
        dt = frame.dt
        pos = frame.pos
        rel = frame.rel
        ff = frame.ff
        rel_v = frame.rel_v
        heading = frame.heading
        forward = frame.forward
        yaw_command = frame.yaw_command
        cross = frame.cross
        along = frame.along
        stable_straight = frame.stable_straight
        if mission.phase == "LOWER_ALIGN":
            lower_height = mission.c.get("lower_align_world_height_m", .50)
            if mission.approach_motion_policy=='coordinated_quintic':
                if mission.coordinated_approach is None:
                    mission.coordinated_approach=CoordinatedApproach(now,
                        mission.c['coordinated_approach_duration_s'],
                        max(mission.c['last_fix_standoff_m'],along),mission.c['last_fix_standoff_m'],
                        max(lower_height,pos[2]),lower_height)
                ref=mission.coordinated_approach.at(now)
                target_e=rel-forward*ref['standoff_m']
                target_e[2]=ref['height_m']-pos[2]
                velocity_error=rel_v
                if mission.coordinated_velocity_feedback_policy=='reference_relative':
                    # d(pad-own-forward*standoff)/dt for constant-heading
                    # straight motion. Estimated heading variation is not a
                    # globally smooth trajectory or a turn-tracking guarantee.
                    # Perfect tracking has rel_v == forward*standoff_rate and
                    # must not damp the intended catch-up feedforward speed.
                    velocity_error=rel_v-forward*ref['standoff_rate_m_s']
                v=ff-forward*ref['standoff_rate_m_s']+.65*target_e+.35*velocity_error
                v[2]=ref['height_rate_m_s']+.8*target_e[2]
                # Keep the original height/cross confirmation. The path moves
                # after LINEUP clearance; terminal TRACK/Flow/image gates stay.
                height_cross_ready=abs(pos[2]-lower_height)<.025 and abs(cross)<.035
                # The same one-second observation dwell can accumulate during
                # the end of the path. Completion is still mandatory: early
                # near-height estimates never authorize the next phase.
                confirm_condition=height_cross_ready and (
                    ref['complete'] or mission.coordinated_confirm_policy=='concurrent')
                confirmed=mission.held(now,confirm_condition,1.)
                if ref['complete'] and confirmed:
                    mission.enter('APPROACH',now,'coordinated_lower_and_approach_reference_complete')
                command=mission.output(v,yaw_command,dt)
                command.approach_reference={'policy':mission.approach_motion_policy,**ref,
                    'scope':'S quintic position reference and derivative FF; vehicle speed still estimated'}
                if mission.coordinated_velocity_feedback_policy=='reference_relative':
                    command.approach_reference['velocity_feedback_policy']='reference_relative'
                return command
            target_e=rel-forward*mission.c["lineup_standoff_m"]
            target_e[2]=lower_height-pos[2]
            v=ff+.65*target_e+.35*rel_v
            v[2]=.8*target_e[2]
            if mission.held(now,abs(pos[2]-lower_height)<.025 and abs(cross)<.035,1.):
                mission.enter("APPROACH",now,"camera_height_adjusted_for_visibility")
            return mission.output(v,yaw_command,dt)
        if mission.phase in ("APPROACH", "TRACK"):
            v, horizontal_ready, bridge = tracking_demand(mission, frame)
            if mission.phase=="APPROACH" and horizontal_ready:
                mission.enter("TRACK",now,"pad_center_and_relative_velocity_ready")
            if mission.phase=="TRACK":
                estimated_pad=np.asarray(vehicle["pad_world_m"])
                descent_s=max(0.,(pos[2]-mission.pad_height-mission.c["geometry_terminal_optical_height_m"])/mission.c["descent_speed_m_s"])
                prediction_s=max(descent_s,mission.c["terminal_max_prediction_s"]) if bridge else descent_s
                travel=np.linalg.norm(ff[:2])*(prediction_s+mission.c["post_motor_off_hold_s"])
                margin_ok=mission.track.safe_straight_margin(estimated_pad[:2],heading,travel)
                ready=horizontal_ready and stable_straight and margin_ok and vehicle["age_s"]<mission.cfg["observer"]["descent_max_age_s"]
                if bridge:
                    q=vehicle["last_image_quality"]
                    ready=ready and q.get("min_side_px",0.)>=mission.c["last_fix_min_side_px"] and q.get("marker_optical_depth_m",0.)>=mission.c["last_fix_min_depth_m"]
                if mission.held(now,ready,mission.c["track_hold_s"]):
                    mission.enter("BLIND_APPROACH" if bridge else "DESCEND",now,"last_reliable_visual_fix" if bridge else "estimated_straight_and_fresh_relative_state")
            return mission.landing.finish(mission, frame, v, horizontal_ready)
        return mission.abort("unhandled_phase", now, dt)
