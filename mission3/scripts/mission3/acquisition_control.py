"""마커 관측 전 대기와 영상 방위만 있는 단계의 제어.

현 직선 프로필은 고정 자리·시선 대기이며, 과거 탐색 정책은 재현용이다.
단계 변경은 supervisor.enter에 요청하고 모든 출력은 공통 제한기를 쓴다.
"""
import math
import numpy as np
from .track import wrap


class AcquisitionControl:
    def entry_search_yaw_rate(self, mission, now, yaw):
        if mission.phase == "ENTRY_SEARCH" and mission.search_translation_gaze_policy == "entry_forward":
            # Known initial direction only. Retain WAIT/ENTRY_SCAN acquisition
            # and the ordinary yaw/slew limiter; no vehicle phase or map target.
            return 1.2*wrap(mission.entry_heading-yaw)
        if mission.search_gaze_policy == "continuous_yaw":
            return math.radians(mission.c["search_yaw_rate_deg_s"])
        elapsed = max(0., now-mission.entry_scan_started)
        amplitude = math.radians(mission.c["search_sector_amplitude_deg"])
        omega = 2*math.pi/mission.c["search_sector_period_s"]
        target = mission.search_gaze+amplitude*math.sin(omega*elapsed)
        # Analytic scan-rate feedforward plus measured-heading feedback.
        # Phase derives from this flight's search clock, never vehicle phase.
        return amplitude*omega*math.cos(omega*elapsed)+1.2*wrap(target-yaw)


    def entry_search_horizontal(self, mission, horizontal, yaw):
        """Scale raster speed opposing estimated body heading; never cart state.

        Body heading is a horizontal proxy for the stock forward camera. This
        S schedule is applied before the ordinary command slew limiter; it is
        not a hard limit on actual camera-relative or aircraft velocity.
        """
        horizontal = np.asarray(horizontal, float).copy()
        if mission.search_reverse_heading_speed is None:
            return horizontal, None
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        projected = float(horizontal @ forward)
        factor = min(1., mission.search_reverse_heading_speed/-projected) if projected < 0 else 1.
        horizontal *= factor
        return horizontal, {"policy":"entry_raster_reverse_heading_speed_schedule",
            "estimated_body_heading_rad":yaw,"original_desired_heading_projection_m_s":projected,
            "scheduled_desired_heading_projection_m_s":float(horizontal@forward),
            "reverse_heading_schedule_m_s":mission.search_reverse_heading_speed,"scale":factor,
            "scope":"S desired-speed schedule before slew; stock front-camera proxy, actual mount U; no target direction or speed"}


    def coarse_command(self, mission, now, pos, yaw, dt):
        """Image-only approach: finite base budget, fresh-pixel guarded extension."""
        if mission.coarse_previous_position is not None:
            distance = np.linalg.norm(pos[:2]-mission.coarse_previous_position[:2])
            mission.coarse_distance_m += distance
            mission.coarse_total_distance_m += distance
        mission.coarse_previous_position = pos.copy()
        yaw_error = wrap(mission.coarse_bearing-yaw)
        base_limit = mission.c.get("coarse_approach_distance_m",1.50)
        path_limit = mission.c.get("coarse_live_path_limit_m",4.)
        image_age = now-mission.coarse_last
        small,large = mission.coarse_min_side_px,mission.coarse_max_side_px
        maximum = mission.c.get("coarse_max_side_stop_px",36.)
        quality_known = (small is not None and large is not None
                         and math.isfinite(small) and math.isfinite(large) and 0. < small <= large)
        fresh_quality = quality_known and 0. <= image_age <= mission.c.get("coarse_motion_image_max_age_s",.15)
        extend = (fresh_quality and mission.cfg["observer"]["coarse_bearing_min_side_px"] <= small
                  < mission.cfg["observer"]["min_side_px"] and large < maximum)
        remaining = max(0.,path_limit-mission.coarse_total_distance_m)
        policy = "fresh_pixel_extension" if extend else "base_distance_only"
        if not extend:
            remaining = min(remaining,max(0.,base_limit-mission.coarse_distance_m))
        if quality_known and large >= maximum:
            remaining = 0.
            policy = "pixel_proximity_stop"
        if now-mission.coarse_started >= mission.c.get("coarse_motion_duration_s",20.):
            remaining = 0.
            policy = "episode_motion_timeout"
        if mission.coarse_total_distance_m >= path_limit:
            remaining = 0.
            policy = "total_path_exhausted"
        if not extend and mission.coarse_distance_m >= base_limit and remaining == 0.:
            policy = policy if policy != "base_distance_only" else "extension_requires_fresh_pixels"
        aligned=abs(yaw_error)<math.radians(15)
        turning_visible=(fresh_quality and extend and abs(yaw_error)<math.radians(
            mission.c.get("coarse_translate_turn_error_deg",15.)))
        limit=mission.c["coarse_approach_speed_m_s"] if aligned else mission.c.get("coarse_turn_translation_speed_m_s",.15)
        speed=min(limit,.8*remaining) if aligned or turning_visible else 0.
        translation_yaw=yaw
        if mission.coarse_translation_axis=="fresh_decoded_bearing" and fresh_quality and extend:
            # Camera gaze and horizontal velocity need not have the same
            # direction on a quadrotor. Use only current decoded image bearing;
            # stale/nonmetric observations retain the original body-forward
            # finite motion budget. No cart speed or metric pose is invented.
            translation_yaw=mission.coarse_bearing
        v = np.array([speed*math.cos(translation_yaw),speed*math.sin(translation_yaw),.8*(mission.search_target[2]-pos[2])])
        command = mission.output(v,1.4*yaw_error,dt)
        command.coarse_guard = {"policy":policy,"image_age_s":image_age,
                               "min_side_px":small if small is not None and math.isfinite(small) else None,
                               "max_side_px":large if large is not None and math.isfinite(large) else None,
                               "episode_path_m":mission.coarse_distance_m,"total_path_m":mission.coarse_total_distance_m,
                               "desired_forward_m_s":speed,"max_side_stop_px":maximum,"path_limit_m":path_limit}
        if mission.coarse_translation_axis!="body_forward":
            command.coarse_guard.update(translation_axis=mission.coarse_translation_axis,
                                        translation_heading_rad=translation_yaw,
                                        fresh_decoded_translation_used=translation_yaw==mission.coarse_bearing and fresh_quality and extend)
        return command


    def update(self, mission, now, pos, yaw, dt):
        if mission.phase=="COARSE_ACQUIRE":
            if now-mission.coarse_last>mission.c["coarse_loss_hold_s"]:
                if mission.search_station_policy == "entry_hold":
                    # Stop at the current estimated pose after losing a decoded marker.
                    mission.search_target = pos.copy()
                    mission.search_gaze = yaw
                    mission.enter("ENTRY_SCAN",now,"coarse_bearing_lost_hold_current_pose")
                else:
                    mission.enter("WATCH",now,"coarse_bearing_lost")
            elif mission.search_variant == "fixed_point":
                return mission.coarse_command(now,pos,yaw,dt)
            return mission.output(np.zeros(3),1.4*wrap(mission.coarse_bearing-yaw),dt)
        if mission.phase == "WAIT":
            if (mission.c.get("undetected_policy","scan") != "fixed_gaze"
                    and now-mission.start_time_s >= mission.c["wait_fixed_gaze_s"]):
                if mission.search_station_policy == "entry_hold":
                    mission.enter("ENTRY_SCAN",now,"15s_without_marker_rotate_at_entry")
                else:
                    mission.enter("EXPLORE",now,"15s_without_marker")
            if mission.search_station_policy == "entry_hold":
                return mission.output(.6*(mission.search_target-pos),1.2*wrap(mission.entry_heading-yaw),dt)
            return mission.output(np.zeros(3),0.,dt,urgent=True)
        if mission.phase == "ENTRY_SCAN":
            if (mission.search_motion_policy == "bounded_entry_raster" and mission.coarse_last is None
                    and now-mission.entry_scan_started >= mission.c["search_pretranslate_scan_s"]):
                mission.enter("ENTRY_SEARCH",now,"entry_relative_continuous_coverage_no_track_station")
            else:
                return mission.output(.6*(mission.search_target-pos),mission.entry_search_yaw_rate(now,yaw),dt)
        if mission.phase == "ENTRY_SEARCH":
            target=mission.entry_search_targets[mission.entry_search_index]
            if np.linalg.norm((target-pos)[:2])<mission.c.get("search_corner_radius_m",.12):
                mission.entry_search_index=(mission.entry_search_index+1)%len(mission.entry_search_targets)
                target=mission.entry_search_targets[mission.entry_search_index]
            error=target-pos
            horizontal=.6*error[:2]
            speed=np.linalg.norm(horizontal)
            limit=mission.c["search_translation_speed_m_s"]
            if speed>limit:horizontal*=limit/speed
            horizontal,guard=mission.entry_search_horizontal(horizontal,yaw)
            command=mission.output(np.r_[horizontal,.6*error[2]],mission.entry_search_yaw_rate(now,yaw),dt)
            if guard is not None:
                guard["output_heading_projection_m_s"]=float(np.array(command.velocity_world_m_s[:2])@np.array([math.cos(yaw),math.sin(yaw)]))
                command.search_guard=guard
            return command
        if mission.phase == "EXPLORE":
            e = mission.search_target-pos
            bearing = math.atan2(e[1],e[0])
            if mission.search_variant == "fixed_point" and np.linalg.norm(e[:2]) < 1.:
                bearing = mission.search_gaze
            if np.linalg.norm(e[:2]) < .15:
                mission.enter("SCAN" if mission.search_scan_enabled else "WATCH",now,
                           "map_prior_search_area_reached" if mission.search_scan_enabled else "static_observation_point_reached")
            v = .55 * e
            return mission.output(v,1.2*wrap(bearing-yaw),dt)
        if mission.phase == "SCAN":
            if now-mission.scan_started > 360 / mission.c["search_yaw_rate_deg_s"]:
                mission.enter("WATCH",now,"full_scan_without_detection")
            return mission.output([0.,0.,.8*(mission.search_target[2]-pos[2])],math.radians(mission.c["search_yaw_rate_deg_s"]),dt)
        if mission.phase == "WATCH":
            elapsed=now-mission.watch_started
            target=mission.search_target.copy()
            if mission.search_variant == "legacy_sweep":
                target_yaw = 0. if mission.lower else math.pi
                target[1]+=mission.c["search_lateral_sweep_m"]*math.sin(2*math.pi*elapsed/mission.c["search_sweep_period_s"])
                target_yaw+=math.radians(mission.c["search_gaze_sweep_deg"])*math.sin(2*math.pi*elapsed/mission.c["search_gaze_period_s"])
            else:
                # Only gaze moves. Clock phase is not a vehicle arrival estimate.
                target_yaw=mission.search_gaze+math.radians(mission.c.get("search_small_gaze_sweep_deg",5.))*math.sin(
                    2*math.pi*elapsed/mission.c.get("search_small_gaze_period_s",14.))
            e=target-pos
            return mission.output(.6*e,1.2*wrap(target_yaw-yaw),dt)
        if mission.phase == "PATROL":
            # Move parallel to known straight, looking in its travel direction.
            # This is a search pattern, not live vehicle guidance.
            direction = 1 if mission.lower else -1
            lo = mission.track.center[0]-mission.track.straight/2+1
            hi = mission.track.center[0]+mission.track.straight/2-1
            if (direction>0 and pos[0]>hi) or (direction<0 and pos[0]<lo):
                mission.lower = not mission.lower
                cx,cy=mission.track.center
                mission.search_target = np.array([pos[0],cy-mission.track.radius-1 if mission.lower else cy+mission.track.radius+1,1.])
                mission.enter("EXPLORE",now,"search_straight_endpoint")
            lateral_target = mission.track.center[1]-mission.track.radius-1 if mission.lower else mission.track.center[1]+mission.track.radius+1
            target_yaw = 0. if direction>0 else math.pi
            return mission.output([.55*direction,.6*(lateral_target-pos[1]),.8*(1.-pos[2])],
                               1.2*wrap(target_yaw-yaw),dt)
        return mission.output(np.zeros(3),0.,dt,urgent=True)
