"""미션3 판단 관리자: 공통 입력 검사·관측 전환·단계 제어 선택.

각 하위 제어기는 이 관리자의 enter/held/output을 통해 전환·확인·출력을
요청한다. 공유 이력은 한 비행의 상태이므로 단계별로 초기화하지 않는다.
물리 실행기/센서 추정기/공식 PID와는 별도 책임이며 Plant 참값을 받지 않는다.
"""
from copy import deepcopy
import math
import numpy as np
from .track import make_track, rotation, wrap
from .settings import decoded_candidate_gaze_options
from .command import Command
from .command_limiter import CommandLimiter
from .guidance import GuidanceFrame
from .acquisition_control import AcquisitionControl
from .alignment_control import AlignmentControl
from .approach_control import ApproachControl
from .landing_control import LandingControl
from .recovery_control import RecoveryControl


class MissionSupervisor:
    def __init__(self, cfg, entry, start_time_s=0.):
        if "vehicle_plant_only" in cfg:
            raise ValueError("controller config must exclude plant vehicle speed/state")
        self.cfg, self.c = cfg, cfg["control"]
        self.command_limiter = CommandLimiter(self.c)
        self.acquisition = AcquisitionControl()
        self.alignment = AlignmentControl()
        self.approach = ApproachControl()
        self.landing = LandingControl()
        self.recovery = RecoveryControl()
        self.start_time_s=float(start_time_s)
        if not math.isfinite(self.start_time_s) or self.start_time_s<0:
            raise ValueError("mission start time must be finite and nonnegative")
        self.track = make_track(cfg["track"])
        self.phase = "WAIT"
        self.last_time = None
        self.last_v = np.zeros(3)
        self.last_yaw_rate = 0.
        self.failure = None
        self.hold_start = None
        self.last_good = None
        self.discovery_time = None
        self.geometry_ready = False
        self.events = []
        self.entry_heading = math.radians(entry["heading_deg"])
        # Static-map observation station, never timed against a live vehicle.
        # The tangent is a search prior; tracking direction still comes from images.
        x, y = entry["xy_m"]
        cx, cy = self.track.center
        self.lower = y < cy
        r, L = self.track.radius, self.track.straight
        self.search_variant = self.c.get("search_variant", "fixed_point")
        if self.search_variant not in ("fixed_point", "legacy_sweep"):
            raise ValueError("search_variant must be fixed_point or legacy_sweep")
        self.search_scan_enabled = self.c.get("search_scan_enabled", self.search_variant == "legacy_sweep")
        self.search_station_policy = self.c.get("search_station_policy", "upstream")
        if self.search_station_policy not in ("upstream", "nearest_static", "entry_hold"):
            raise ValueError("search_station_policy must be upstream, nearest_static or entry_hold")
        self.candidate_gaze=decoded_candidate_gaze_options(cfg["observer"])
        if self.candidate_gaze.get("enabled",False) and self.search_station_policy != "entry_hold":
            raise ValueError("candidate gaze is an entry-hold research option")
        self.candidate_interrupt=None
        self.candidate_hold_position=None
        target_x = (np.clip(x, cx-L/2+2., cx+L/2-2.) if self.search_variant == "legacy_sweep"
                    else cx-L/2+2. if self.lower else cx+L/2-2.)
        if self.search_station_policy == "nearest_static":
            # Reduce unobserved travel using the allowed entry/map prior only.
            # This can acquire near the downstream end: TRACK still requires
            # an observed straight and sufficient remaining landing distance.
            target_x = np.clip(x, cx-L/2+2., cx+L/2-2.)
        self.search_target = np.array([target_x, cy-r-.35 if self.lower else cy+r+.35,
                                       self.c.get("search_observation_world_height_m", .70)])
        self.search_gaze = wrap((0. if self.lower else math.pi)
                               +math.radians(self.c.get("search_inward_gaze_deg",20.)))
        if self.search_station_policy == "entry_hold":
            # User entry pose only. No unobserved trip to a map/track station.
            self.search_target = np.array([x, y, cfg["entry"]["height_m"]], dtype=float)
            self.search_gaze = self.entry_heading
        self.entry_scan_started = None
        self.search_gaze_policy = self.c.get("search_gaze_policy", "continuous_yaw")
        if self.search_gaze_policy not in ("continuous_yaw", "entry_sector"):
            raise ValueError("unknown entry search gaze policy")
        if self.search_gaze_policy == "entry_sector":
            amplitude = self.c["search_sector_amplitude_deg"]
            period = self.c["search_sector_period_s"]
            if (self.search_station_policy != "entry_hold" or not math.isfinite(amplitude)
                    or not 0 < amplitude < 180 or not math.isfinite(period) or period <= 0):
                raise ValueError("entry sector requires entry_hold and finite amplitude/period")
        self.entry_search_index = 0
        self.entry_search_targets = []
        self.search_motion_policy = self.c.get("search_motion_policy", "stationary")
        if self.search_motion_policy not in ("stationary", "bounded_entry_raster"):
            raise ValueError("unknown entry search motion policy")
        self.search_translation_gaze_policy = self.c.get("search_translation_gaze_policy", "continue_scan")
        if self.search_translation_gaze_policy not in ("continue_scan", "entry_forward"):
            raise ValueError("unknown translation gaze policy")
        if (self.search_translation_gaze_policy == "entry_forward"
                and (self.search_station_policy != "entry_hold"
                     or self.search_motion_policy != "bounded_entry_raster")):
            raise ValueError("entry-forward gaze requires entry-relative bounded raster")
        self.search_reverse_heading_speed = self.c.get("search_reverse_heading_speed_m_s")
        if self.search_reverse_heading_speed is not None:
            if (self.search_motion_policy != "bounded_entry_raster"
                    or not math.isfinite(self.search_reverse_heading_speed)
                    or not 0 < self.search_reverse_heading_speed <= self.c["search_translation_speed_m_s"]):
                raise ValueError("reverse-heading search speed requires positive bounded raster setting")
        if self.search_motion_policy == "bounded_entry_raster":
            if self.search_station_policy != "entry_hold":
                raise ValueError("entry raster requires entry_hold; no track rendezvous")
            forward = self.c["search_forward_extent_m"]
            lateral = self.c["search_lateral_extent_m"]
            margin = self.c["net_clearance_m"]
            basis = rotation(self.entry_heading)[:2,:2]
            # Entry-relative coverage, independent of track position and cart phase.
            # Transit continuously between corners; no map observation station.
            for uv in ((forward,0.),(forward,lateral),(0.,lateral),
                       (0.,-lateral),(forward,-lateral),(forward,0.)):
                xy = np.array([x,y])+basis@uv
                xy = np.clip(xy,[margin,margin],
                             [cfg["arena"]["length_m"]-margin,cfg["arena"]["width_m"]-margin])
                self.entry_search_targets.append(np.r_[xy,cfg["entry"]["height_m"]])
        self.search_deadline_s = self.start_time_s+min(self.c["mission_timeout_s"], self.c.get("search_timeout_s",300.))
        self.scan_started = None
        self.straight_start = None
        self.watch_started = None
        self.coarse_started = None
        self.coarse_last=None
        self.coarse_bearing=None
        self.coarse_previous_position=None
        self.coarse_distance_m=0.
        self.coarse_total_distance_m=0.
        self.coarse_min_side_px=None
        self.coarse_max_side_px=None
        self.coarse_translation_axis=self.c.get("coarse_translation_axis","body_forward")
        if self.coarse_translation_axis not in ("body_forward","fresh_decoded_bearing"):
            raise ValueError("coarse translation must use body-forward or fresh decoded bearing")
        self.terminal_started=None
        self.last_vehicle=None
        self.pad_height = cfg["landing_geometry"]["pad_height_m"]
        self.relative_flow_unusable_since = None
        self.inertial_handoff_since = None
        self.support_handoff_mode = None
        self.coordinated_approach=None
        self.approach_motion_policy=self.c.get('approach_motion_policy','staged')
        self.coordinated_confirm_policy=self.c.get('coordinated_confirm_policy','after_reference')
        if self.coordinated_confirm_policy not in ('after_reference','concurrent'):
            raise ValueError('unsupported coordinated confirmation policy')
        self.coordinated_velocity_feedback_policy=self.c.get(
            'coordinated_velocity_feedback_policy','vehicle_relative')
        if self.coordinated_velocity_feedback_policy not in ('vehicle_relative','reference_relative'):
            raise ValueError('unsupported coordinated velocity feedback policy')
        if self.approach_motion_policy not in ('staged','coordinated_quintic'):
            raise ValueError('unknown approach motion policy')
        if self.approach_motion_policy=='coordinated_quintic':
            duration=self.c.get('coordinated_approach_duration_s')
            if duration is None or not math.isfinite(duration) or duration<=0:
                raise ValueError('coordinated approach duration must be finite and positive')
            if (self.c['terminal_policy']!='last_fix_bridge'
                    or not self.c.get('lineup_lowering_requires_straight',False)):
                raise ValueError('coordinated approach requires confirmed straight and last-fix standoff')
            # 1.875 is max derivative of the quintic time scaling. It is
            # a reference budget, not a guarantee on noisy feedback or plant.
            dz=abs(.70-self.c.get('lower_align_world_height_m',.50))
            if 1.875*dz/duration>self.c['max_descent_speed_m_s']:
                raise ValueError('coordinated height reference exceeds descent budget')
        self.blind_recovery = None
        self.uncertainty_observation = self.c.get("uncertainty_observation", {})
        self.uncertainty_hold_started = None
        self.uncertainty_hold_height = None
        self.uncertainty_heading_updates_start = 0
        if self.uncertainty_observation.get("enabled", False):
            u=self.uncertainty_observation
            values=[u[k] for k in ("soft_position_std_m","resume_position_std_m","resume_heading_std_deg",
                                   "max_observation_s","vehicle_ff_fraction")]
            if (not all(math.isfinite(v) for v in values)
                    or not 0 < u["soft_position_std_m"] < u["resume_position_std_m"] < self.c["odometry_uncertainty_limit_m"]
                    or not 0 < u["resume_heading_std_deg"] < 45 or u["max_observation_s"] <= 0
                    or not 0 <= u["vehicle_ff_fraction"] <= 1
                    or u["minimum_straight_heading_updates"] < 1):
                raise ValueError("invalid uncertainty observation budget; hard gate cannot change")


    def enter(self, phase, now, reason):
        if phase != self.phase:
            self.events.append({"time_s": now, "from": self.phase, "to": phase, "reason": reason})
            self.phase = phase
            if phase == "ENTRY_SCAN":self.entry_scan_started=now
            self.hold_start = None
            if phase=='LOWER_ALIGN':self.coordinated_approach=None
            if phase == "BLIND_APPROACH":
                self.terminal_started = now
                self.relative_flow_unusable_since = None
        self.scan_started = now if phase == "SCAN" else None
        if phase=="WATCH":self.watch_started=now
        if phase=="COARSE_ACQUIRE":self.coarse_started=now


    def held(self, now, condition, duration):
        if not condition:
            self.hold_start = None
            return False
        if self.hold_start is None:
            self.hold_start = now
        return now-self.hold_start >= duration-1e-8


    @property
    def last_v(self):
        return self.command_limiter.last_v

    @last_v.setter
    def last_v(self, value):
        self.command_limiter.last_v = value

    @property
    def last_yaw_rate(self):
        return self.command_limiter.last_yaw_rate

    @last_yaw_rate.setter
    def last_yaw_rate(self, value):
        self.command_limiter.last_yaw_rate = value

    @property
    def active_control(self):
        """계층 표시용 이름. 기록을 바꾸거나 제어를 추가 실행하지 않는다."""
        if self.phase in ("ABORT", "GEOMETRY_READY"):
            return "terminated"
        if self.blind_recovery is not None or self.phase == "RECOVER":
            return "recovery"
        if self.phase in ("ACQUIRE", "LINEUP"):
            return "alignment"
        if self.phase in ("LOWER_ALIGN", "APPROACH", "TRACK"):
            return "approach"
        if self.phase in ("DESCEND", "BLIND_APPROACH", "BLIND_DESCEND"):
            return "landing"
        if self.phase == "UNCERTAINTY_OBSERVE":
            return "uncertainty_observation"
        return "acquisition"

    def output(self, velocity, yaw_rate, dt, urgent=False):
        velocity, yaw_rate = self.command_limiter.limit(velocity, yaw_rate, dt, urgent)
        return Command(self.phase, velocity, yaw_rate,
                       done=self.phase in ("GEOMETRY_READY", "ABORT"), failure=self.failure)

    def entry_search_yaw_rate(self, now, yaw):
        return self.acquisition.entry_search_yaw_rate(self, now, yaw)

    def entry_search_horizontal(self, horizontal, yaw):
        return self.acquisition.entry_search_horizontal(self, horizontal, yaw)

    def coarse_command(self, now, pos, yaw, dt):
        return self.acquisition.coarse_command(self, now, pos, yaw, dt)

    def start_blind_recovery(self, reason, now, own, dt):
        return self.recovery.start_blind_recovery(self, reason, now, own, dt)

    def recover_blind(self, now, own, dt):
        return self.recovery.recover_blind(self, now, own, dt)

    def terminal_safety_reason(self, now, own, vehicle, prediction_started=None):
        return self.recovery.terminal_safety_reason(self, now, own, vehicle, prediction_started)

    def uncertainty_observation_command(self, now, own, vehicle, straight, yaw_command, dt):
        u=self.uncertainty_observation
        if not u.get("enabled",False) or self.phase not in (
                "ACQUIRE","LINEUP","LOWER_ALIGN","APPROACH","TRACK","UNCERTAINTY_OBSERVE"):
            return None
        heading_std=own.get("heading_std_deg")
        updates=own.get("qualified_straight_heading_updates")
        if heading_std is None or updates is None or not math.isfinite(heading_std):
            return self.abort("uncertainty_observation_evidence_unavailable",now,dt)
        if (self.phase != "UNCERTAINTY_OBSERVE"
                and own["position_std_m"] >= u["soft_position_std_m"]
                and heading_std > u["resume_heading_std_deg"]):
            if self.uncertainty_hold_started is None:
                self.uncertainty_hold_started=now
                self.uncertainty_hold_height=own["position_world_m"][2]
                self.uncertainty_heading_updates_start=updates
            self.enter("UNCERTAINTY_OBSERVE",now,"position_budget_brake_before_hard_uncertainty_gate")
        if self.phase != "UNCERTAINTY_OBSERVE":return None
        recovered=(straight and heading_std <= u["resume_heading_std_deg"]
                   and own["position_std_m"] <= u["resume_position_std_m"]
                   and updates-self.uncertainty_heading_updates_start >= u["minimum_straight_heading_updates"])
        if recovered:
            self.enter("LINEUP",now,"fresh_straight_heading_evidence_within_original_budget")
            self.uncertainty_hold_started=None
            self.uncertainty_hold_height=None
            return None
        if now-self.uncertainty_hold_started >= u["max_observation_s"]:
            return self.abort("uncertainty_observation_timeout",now,dt)
        # No approach-position feedback. Compare stopping against following only
        # the visually estimated cart velocity. Neither variant uses plant speed.
        v=u["vehicle_ff_fraction"]*np.asarray(vehicle["vehicle_velocity_world_m_s"])
        v[2]=self.c["height_kp_per_s"]*(self.uncertainty_hold_height-own["position_world_m"][2])
        return self.output(v,yaw_command,dt)


    def abort(self, reason, now, dt):
        self.failure = reason
        self.enter("ABORT",now,reason)
        return self.output(np.zeros(3),0.,dt,urgent=True)


    def update(self, now, own, vehicle):
        dt = .02 if self.last_time is None else now-self.last_time
        if self.last_time is not None and dt <= 0:
            raise ValueError("control time must increase")
        self.last_time = now
        if self.phase in ("ABORT","GEOMETRY_READY"):
            return self.output(np.zeros(3),0.,dt,urgent=True)
        if self.blind_recovery is not None:
            return self.recover_blind(now, own, dt)
        terminal=self.phase in ("BLIND_APPROACH","BLIND_DESCEND")
        if now-self.start_time_s >= self.c["mission_timeout_s"]:
            if terminal:
                return self.start_blind_recovery("mission_timeout",now,own,dt)
            return self.abort("mission_timeout",now,dt)
        if self.discovery_time is None and now >= self.search_deadline_s:
            return self.abort("search_timeout_without_metric_pose",now,dt)
        if own["position_std_m"] > self.c["odometry_uncertainty_limit_m"]:
            if terminal:
                return self.start_blind_recovery("odometry_uncertainty",now,own,dt)
            return self.abort("odometry_uncertainty",now,dt)
        pos = np.asarray(own["position_world_m"])
        yaw = own["heading_rad"]
        own_velocity = np.asarray(own["velocity_world_m_s"])
        margin = self.c["net_clearance_m"] / 2
        if not (margin <= pos[0] <= self.cfg["arena"]["length_m"]-margin
                and margin <= pos[1] <= self.cfg["arena"]["width_m"]-margin and .05 < pos[2] < 4.5):
            if terminal:
                return self.start_blind_recovery("estimated_arena_boundary",now,own,dt)
            return self.abort("estimated_arena_boundary",now,dt)
        coarse=vehicle is not None and vehicle.get("kind")=="coarse_image_bearing_only"
        tentative=vehicle is not None and vehicle.get("kind")=="unconfirmed_decoded_bearing"
        if (self.candidate_gaze.get("enabled",False) and self.discovery_time is None
                and self.phase in ("WAIT","ENTRY_SCAN","ENTRY_SEARCH","VERIFY_BEARING")):
            candidate_fresh=(tentative and vehicle.get("bearing_valid",False)
                             and 0 <= vehicle["age_s"] <= self.candidate_gaze["max_age_s"]
                             and math.isfinite(vehicle["bearing_world_rad"]))
            if candidate_fresh:
                if self.candidate_interrupt is None:
                    self.candidate_interrupt=(self.phase,self.entry_scan_started)
                    self.candidate_hold_position=pos.copy()
                    self.enter("VERIFY_BEARING",now,"single_decoded_target_gaze_only_no_approach")
                # Keep slew history: stopping takes time, not an instantaneous
                # zero velocity. No vehicle velocity/range/heading is inferred.
                return self.output(.6*(self.candidate_hold_position-pos),
                                   1.4*wrap(vehicle["bearing_world_rad"]-yaw),dt)
            if self.phase=="VERIFY_BEARING" and not coarse and not (
                    vehicle is not None and not tentative and vehicle.get("valid")
                    and vehicle["age_s"] <= self.cfg["observer"]["image_max_age_s"]):
                interrupted,scan_clock=self.candidate_interrupt
                self.enter(interrupted,now,"unconfirmed_bearing_expired_resume_search_clock")
                # enter(ENTRY_SCAN) normally restarts the sector clock; a
                # transient single-ID interruption must not do so.
                self.entry_scan_started=scan_clock
                self.candidate_interrupt=None
                self.candidate_hold_position=None
        if coarse and self.discovery_time is None:
            self.candidate_interrupt=None
            self.candidate_hold_position=None
            self.coarse_last=now if self.search_variant == "legacy_sweep" else now-max(0.,vehicle["age_s"])
            self.coarse_bearing=vehicle["bearing_world_rad"]
            self.coarse_min_side_px=vehicle.get("min_side_px")
            self.coarse_max_side_px=vehicle.get("max_side_px")
            if self.phase!="COARSE_ACQUIRE":
                self.coarse_previous_position=pos.copy()
                self.coarse_distance_m=0.
                self.enter("COARSE_ACQUIRE",now,"decoded_marker_bearing_without_metric_pose")
            if self.search_variant == "fixed_point":
                return self.coarse_command(now,pos,yaw,dt)
            yaw_error=wrap(vehicle["bearing_world_rad"]-yaw)
            forward_speed=self.c["coarse_approach_speed_m_s"] if abs(yaw_error)<math.radians(15) else 0.
            v=np.array([forward_speed*math.cos(yaw),forward_speed*math.sin(yaw),.8*(self.search_target[2]-pos[2])])
            return self.output(v,1.4*yaw_error,dt)
        fresh = vehicle is not None and not coarse and not tentative and vehicle.get("valid") and vehicle["age_s"] <= self.cfg["observer"]["image_max_age_s"]
        if terminal and (self.terminal_started is None
                         or now-self.terminal_started >= self.c["terminal_max_prediction_s"]-1e-8
                         or vehicle is None or coarse or tentative
                         or vehicle["age_s"] >= self.c["terminal_max_prediction_s"]-1e-8):
            reason = "terminal_state_unavailable" if vehicle is None or coarse or tentative else "terminal_prediction_expired"
            return self.start_blind_recovery(reason,now,own,dt)
        predicted=(terminal and vehicle is not None and not coarse and not tentative and self.terminal_started is not None
                   and now-self.terminal_started<self.c["terminal_max_prediction_s"]
                   and vehicle["age_s"]<self.c["terminal_max_prediction_s"])
        if fresh:
            self.candidate_interrupt=None
            self.candidate_hold_position=None
            self.last_good = now
            self.last_vehicle=deepcopy(vehicle)
            if self.discovery_time is None:
                self.discovery_time = now
            if self.phase in ("WAIT","ENTRY_SCAN","ENTRY_SEARCH","VERIFY_BEARING","EXPLORE","SCAN","WATCH","PATROL","RECOVER","COARSE_ACQUIRE"):
                self.enter("ACQUIRE",now,"fresh_image_marker")
        elif self.discovery_time is not None and not predicted:
            lost = now-self.last_good
            if lost > self.c["loss_reacquire_s"]:
                return self.abort("visual_loss_after_discovery",now,dt)
            self.enter("RECOVER",now,"visual_age")
            if self.last_vehicle is not None:
                old=self.last_vehicle;h=old["vehicle_heading_rad"]
                v=np.asarray(old["vehicle_velocity_world_m_s"])-self.c["terminal_recovery_back_speed_m_s"]*np.array([math.cos(h),math.sin(h),0.])
                v[2]=self.c["terminal_recovery_climb_speed_m_s"]
                return self.output(v,0.,dt)
            return self.output(np.zeros(3),0.,dt,urgent=True)
        if not fresh and not predicted:
            return self.acquisition.update(self, now, pos, yaw, dt)
        rel = np.asarray(vehicle["relative_pad_world_m"])
        ff = np.asarray(vehicle["vehicle_velocity_world_m_s"])
        rel_v = np.asarray(vehicle["relative_velocity_world_m_s"])
        heading = vehicle["vehicle_heading_rad"]
        forward = np.array([math.cos(heading),math.sin(heading),0.])
        lateral = np.array([-forward[1],forward[0],0.])
        marker_delta = rel + rotation(heading) @ np.asarray(self.cfg["marker"]["known_marker_to_pad_vehicle_m"])
        bearing = math.atan2(marker_delta[1],marker_delta[0])
        yaw_command = 1.3*wrap(bearing-yaw)
        cross = float(np.dot(rel,lateral))
        along = float(np.dot(rel,forward))
        straight = abs(vehicle["vehicle_heading_rate_rad_s"]) < math.radians(self.c["straight_max_yaw_rate_deg_s"])
        if straight:
            self.straight_start = now if self.straight_start is None else self.straight_start
        else:
            self.straight_start = None
        stable_straight = self.straight_start is not None and now-self.straight_start >= self.c["straight_confirm_s"]
        observation_command=self.uncertainty_observation_command(now,own,vehicle,stable_straight,yaw_command,dt)
        if observation_command is not None:return observation_command
        if terminal:
            if not straight:
                return self.start_blind_recovery("terminal_straight_unconfirmed",now,own,dt)
            remaining_budget = max(0.,self.c["terminal_max_prediction_s"]-(now-self.terminal_started))
            travel = np.linalg.norm(ff[:2])*(remaining_budget+self.c["post_motor_off_hold_s"])
            if not self.track.safe_straight_margin(np.asarray(vehicle["pad_world_m"])[:2],heading,travel):
                return self.start_blind_recovery("terminal_straight_margin",now,own,dt)
        frame = GuidanceFrame(now, own, vehicle, dt, pos, yaw, terminal, rel, ff, rel_v, heading, forward, lateral, bearing, yaw_command, cross, along, straight, stable_straight)
        if self.phase in ("ACQUIRE", "LINEUP"):
            return self.alignment.update(self, frame)
        if self.phase in ("LOWER_ALIGN", "APPROACH", "TRACK"):
            return self.approach.update(self, frame)
        if self.phase in ("DESCEND", "BLIND_APPROACH", "BLIND_DESCEND"):
            return self.landing.update(self, frame)
        return self.abort("unhandled_phase", now, dt)
