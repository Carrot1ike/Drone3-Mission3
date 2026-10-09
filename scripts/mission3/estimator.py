"""가상 probe·영상 상태 추정. 참값/차량 설정속도 입력 없음.

Level optical test 전용. 실제 기체 body 변환·펌웨어 EKF는 별도 검증한다.
"""
from collections import deque
import math
import numpy as np
from .track import make_track, rotation, wrap
from .uncertainty import MotionUncertainty
from .vertical_fit import instantaneous_range_velocity
from .sensor_time import packet_time,fit_history_time,history_time


class SelfMotion:
    def __init__(self, cfg, entry):
        self.cfg = cfg
        self.track = make_track(cfg["track"])
        self.position = np.r_[entry["xy_m"], cfg["entry"]["height_m"]].astype(float)
        self.velocity = np.zeros(3)
        self.heading = math.radians(entry["heading_deg"])
        self.position_std = cfg["entry"]["initial_position_std_m"]
        self.last_stamp = None
        self._last_range_input_stamp = None
        self.last_range = None
        self.surface = "unconfirmed"
        self.history = deque(maxlen=500)
        self.velocity_history=deque(maxlen=500)
        self.last_visual = None
        self.visual_heights = deque()
        self.range_heights = deque()
        self.range_velocity_window=float(cfg["observer"].get("vertical_range_velocity_window_s",.4))
        if not math.isfinite(self.range_velocity_window) or self.range_velocity_window<=0:
            raise ValueError("vertical range velocity window must be finite and positive")
        self.range_velocity_model=cfg["observer"].get("vertical_range_velocity_model","linear_window")
        if self.range_velocity_model not in ("linear_window","imu_compensated"):
            raise ValueError("unsupported vertical range velocity model")
        self.vertical_imu=deque(maxlen=500)
        self.vertical_fit_fallbacks=0
        self.events = []
        self.terminal_hint=None
        self.pad_height=cfg["landing_geometry"]["pad_height_m"]
        settings=cfg["observer"].get("motion_uncertainty")
        self.uncertainty=MotionUncertainty(cfg["entry"],settings) if settings else None
        self.gyro_z_bias=0.
        self.last_track_anchor_stamp=None
        self.straight_bias_started=None
        self.accel_bias_heading=np.zeros(3)
        self.accel_bias_floor_updates=0
        self.accel_bias_settings=cfg["observer"].get("floor_accel_bias_observer")
        if self.accel_bias_settings:
            values=[self.accel_bias_settings[k] for k in ("integral_gain_per_s2","bias_limit_m_s2")]
            if not all(math.isfinite(v) and v>0. for v in values):raise ValueError("invalid floor bias observer settings")

    def track_anchor(self, vehicle, now):
        """Known static track + observed cart pose, not visual lane detection.

        Only a straight has an unambiguous tangent line. Longitudinal position
        is not corrected. Direction is chosen from measured heading, never from
        the plant's direction/phase or configured speed.
        """
        if not vehicle or not vehicle.get("valid") or vehicle["samples"]<12:
            self.straight_bias_started=None
            return
        if abs(vehicle["heading_rate"])>math.radians(1.5):
            self.straight_bias_started=None
            return
        pad=np.asarray(vehicle["pad"])
        point,turn=self.track.nearest(pad[:2])
        if turn or np.linalg.norm(point-pad[:2])>self.cfg["observer"]["max_track_cross_innovation_m"]:
            self.straight_bias_started=None
            return
        heading=vehicle["heading"]
        tangent=min((0.,math.pi),key=lambda a:abs(wrap(a-heading)))
        gap=wrap(tangent-heading)
        if abs(gap)>math.radians(20):
            self.straight_bias_started=None
            return
        if self.straight_bias_started is None:self.straight_bias_started=now
        dt=0. if self.last_track_anchor_stamp is None else min(.1,max(0.,now-self.last_track_anchor_stamp))
        self.last_track_anchor_stamp=now
        c=self.cfg["observer"].get("straight_gyro_bias_observer")
        bias_gain=0.
        if c and now-self.straight_bias_started>=c["confirm_s"]:
            bias_gain=-c["integral_gain_per_s2"]*dt
            self.gyro_z_bias=float(np.clip(self.gyro_z_bias+bias_gain*gap,
                -math.radians(c["bias_limit_deg_s"]),math.radians(c["bias_limit_deg_s"])))
        delta=np.array([0.,.10*(point[1]-pad[1]),0.])
        yaw_delta=.02*gap
        self.position+=delta
        self.heading=wrap(self.heading+yaw_delta)
        self.velocity=rotation(yaw_delta)@self.velocity
        if self.uncertainty:self.uncertainty.change_heading_gauge(yaw_delta)
        if self.uncertainty and c:self.uncertainty.straight_heading_update(.02,bias_gain)
        self.history=deque(((t,p+delta,wrap(h+yaw_delta)) for t,p,h in self.history),maxlen=500)
        self.velocity_history=deque(((t,rotation(yaw_delta)@v,valid) for t,v,valid in self.velocity_history),maxlen=500)
        vehicle["pad"]=pad+delta
        vehicle["heading"]=wrap(heading+yaw_delta)
        self.events.append({"time_s":now,"type":"known_track_straight_constraint",
                            "cross_correction_m":delta[1],"yaw_correction_rad":yaw_delta,
                            "source":"image pose plus static track prior, no lane pixels or live truth"})
        return delta,yaw_delta

    def at(self, stamp):
        if not self.history or stamp < self.history[0][0] - 1e-6 or stamp > self.history[-1][0] + .025:
            return None
        times = np.array([r[0] for r in self.history])
        pos = np.array([np.interp(stamp, times, [r[1][i] for r in self.history]) for i in range(3)])
        yaw = np.interp(stamp, times, np.unwrap([r[2] for r in self.history]))
        return pos, wrap(float(yaw))

    def velocity_at(self, stamp):
        if not self.velocity_history or stamp<self.velocity_history[0][0]-1e-6 or stamp>self.velocity_history[-1][0]+.025:
            return None
        times=[r[0] for r in self.velocity_history]
        velocity=np.array([np.interp(stamp,times,[r[1][i] for r in self.velocity_history]) for i in range(3)])
        sample=min(self.velocity_history,key=lambda r:abs(r[0]-stamp))
        return velocity,sample[2]

    def visual_anchor(self, obs, now, raw_relative=None, capture_stamp=None):
        if not obs or not obs.get("valid"):
            return
        rel = np.asarray(obs["relative_pad_world_m"])
        # Height datum is the known pad elevation. Use the raw capture-relative
        # measurement, never an absolute pad state formed using drifting odometry.
        if raw_relative is not None:
            capture = float(capture_stamp)
            measured_height = self.pad_height-float(raw_relative[2])
            self.visual_heights.append((capture, measured_height))
            while len(self.visual_heights)>2 and capture-self.visual_heights[0][0]>.6:
                self.visual_heights.popleft()
            if len(self.visual_heights)>=4 and capture-self.visual_heights[0][0]>.15:
                ts=np.array([r[0] for r in self.visual_heights]);ts-=ts.mean()
                heights=np.array([r[1] for r in self.visual_heights])
                vz=float(np.dot(ts,heights-heights.mean())/np.dot(ts,ts))
                self.velocity[2]+=.15*(vz-self.velocity[2])
            height_now=measured_height+self.velocity[2]*(now-capture)
            self.position[2]+=.3*(height_now-self.position[2])
        self.last_visual = {"stamp_s": now, "relative_pad_world_m": rel,
                            "vehicle_velocity_world_m_s": np.asarray(obs["vehicle_velocity_world_m_s"]),
                            "vehicle_heading_rad":obs["vehicle_heading_rad"]}

    def validate_sample_time(self,sample):
        stamp,producer=packet_time(sample,self.last_stamp,self._last_range_input_stamp)
        # The packet can be valid while retained history is malformed. Check
        # before PhysicalMotion consumes the accumulator or calls the C core.
        history_time(self.range_heights,stamp,'range')
        history_time(self.vertical_imu,stamp,'IMU')
        if self.range_velocity_model=='imu_compensated' and self.vertical_imu:
            if stamp<=self.vertical_imu[-1][0]:
                raise ValueError('prospective IMU history ordering invalid')
        distance=sample.get('range_m')
        usable=(distance is not None and math.isfinite(float(distance)) and float(distance)>0
                and sample.get('range_valid',True) and sample.get('range_accurate',True))
        if usable and sample.get('range_new',True) and self.range_heights:
            if producer<=self.range_heights[-1][0]:
                raise ValueError('prospective range history ordering invalid')
        return stamp,producer

    def update(self, sample):
        # Validate before heading, height, covariance or histories can change.
        stamp,range_stamp=self.validate_sample_time(sample)
        dt = 0. if self.last_stamp is None else stamp - self.last_stamp
        self.last_stamp = stamp
        self.heading = wrap(self.heading + (float(sample["gyro_z_rad_s"])-self.gyro_z_bias) * dt)
        # The fitted horizontal residual includes tilt/gravity/model errors;
        # it is not claimed to be a calibrated accelerometer chip bias.
        corrected_acc=np.asarray(sample["accel_heading_m_s2"])-self.accel_bias_heading
        if self.range_velocity_model=="imu_compensated":
            self.vertical_imu.append((stamp,float(corrected_acc[2])))
            while len(self.vertical_imu)>2 and self.vertical_imu[1][0]<stamp-self.range_velocity_window-.05:
                self.vertical_imu.popleft()
        self.velocity += rotation(self.heading) @ corrected_acc * dt
        if self.uncertainty:self.uncertainty.predict(dt,self.heading,rotation(self.heading)@corrected_acc)
        range_m = float(sample["range_m"]) if sample.get("range_m") is not None else float("nan")
        range_valid=(bool(sample.get("range_valid",True)) and bool(sample.get("range_accurate",True))
                     and math.isfinite(range_m) and range_m>0.)
        range_new=bool(sample.get("range_new",True))
        flow_floor_gain=1-(1-.25)**((dt or .02)/.02)
        flow_pad_gain=1-(1-.08)**((dt or .02)/.02)
        fresh_visual = self.last_visual is not None and stamp-self.last_visual["stamp_s"] < .3
        hint=self.terminal_hint
        fresh_hint=hint is not None and stamp-hint["capture_stamp_s"]<self.cfg["control"]["terminal_max_prediction_s"]
        evidence=self.last_visual if fresh_visual else hint if fresh_hint else None
        rel = np.asarray(evidence["relative_pad_world_m"]) if evidence is not None else None
        near_pad = rel is not None and np.linalg.norm(rel[:2]) < .13
        ground_residual = abs(range_m - self.position[2])
        pad_residual = abs(range_m - (self.position[2] - self.pad_height))
        jump = self.last_range is not None and abs(range_m-self.last_range) > .10
        old = self.surface
        if not range_valid:
            self.surface="ambiguous"
        elif near_pad and pad_residual < .07:
            self.surface = "moving_surface_candidate"
        elif ground_residual < .07:
            self.surface = "static_floor_candidate"
        else:
            self.surface = "ambiguous"
        if self.surface != old or jump:
            self.events.append({"time_s": stamp, "from": old, "to": self.surface,
                                "range_jump": bool(jump), "uses_truth_surface_id": False})
        flow_world = rotation(self.heading) @ np.r_[np.asarray(sample["flow_angular_rad_s"]) * range_m, 0.]
        flow_usable=False;flow_reason="surface_unconfirmed"
        # A footprint overlapping an observed pad is not guaranteed to be
        # floor flow even when its central ToF return is still the floor.
        floor_pure=True
        if evidence is not None and range_valid:
            heading=evidence.get("vehicle_heading_rad",self.heading)
            local=rotation(heading).T@rel
            radius=range_m*math.tan(math.radians(self.cfg["observer"]["flow_footprint_half_angle_hypothesis_deg"]))
            g=self.cfg["landing_geometry"]
            floor_pure=(abs(local[0])>g["pad_length_m"]/2+radius or abs(local[1])>g["pad_width_m"]/2+radius)
        if self.surface == "static_floor_candidate" and not jump:
            flow_usable=bool(floor_pure and sample.get("flow_valid",True))
            flow_reason="floor_patch_clear" if flow_usable else "floor_patch_overlap_or_invalid"
            if flow_usable:
                innovation=flow_world-self.velocity
                bias_gain=0.
                if self.accel_bias_settings:
                    bias_gain=self.accel_bias_settings["integral_gain_per_s2"]*dt
                    self.accel_bias_heading[:2]-=bias_gain*(rotation(self.heading).T@innovation)[:2]
                    limit=self.accel_bias_settings["bias_limit_m_s2"]
                    self.accel_bias_heading[:2]=np.clip(self.accel_bias_heading[:2],-limit,limit)
                    self.accel_bias_floor_updates+=1
                self.velocity[:2] += flow_floor_gain * innovation[:2]
                if self.uncertainty:self.uncertainty.qualified_floor_flow(flow_world,flow_floor_gain,bias_gain,self.heading)
            if range_new:
                self.position[2] += .12 * (range_m - self.position[2])
                self.range_heights.append((sample.get("range_stamp_s",stamp),range_m))
            while len(self.range_heights)>2 and stamp-self.range_heights[0][0]>self.range_velocity_window:
                self.range_heights.popleft()
            if len(self.range_heights)>5 and range_new:
                vz=self.fitted_range_velocity(stamp)
                self.velocity[2]+=.08*(vz-self.velocity[2])
        elif self.surface == "moving_surface_candidate" and evidence is not None and not jump:
            # Finite-patch purity gate is a sensitivity model (actual FoV U).
            heading=evidence.get("vehicle_heading_rad",self.heading)
            rel_local=rotation(heading).T@rel
            radius=range_m*math.tan(math.radians(self.cfg["observer"]["flow_footprint_half_angle_hypothesis_deg"]))
            geometry=self.cfg["landing_geometry"]
            pure=(abs(rel_local[0])+radius<geometry["pad_length_m"]/2
                  and abs(rel_local[1])+radius<geometry["pad_width_m"]/2)
            if pure and sample.get("flow_valid",True):
                measured = flow_world[:2] + np.asarray(evidence["vehicle_velocity_world_m_s"])[:2]
                self.velocity[:2] += flow_pad_gain * (measured - self.velocity[:2])
                flow_usable=True;flow_reason="pad_patch_contained"
            else:flow_reason="pad_patch_mixed_or_invalid"
            if range_new:
                self.position[2]+=.12*(range_m+self.pad_height-self.position[2])
                self.range_heights.append((sample.get("range_stamp_s",stamp),range_m+self.pad_height))
            while len(self.range_heights)>2 and stamp-self.range_heights[0][0]>self.range_velocity_window:
                self.range_heights.popleft()
            if len(self.range_heights)>5 and range_new:
                vz=self.fitted_range_velocity(stamp)
                self.velocity[2]+=.08*(vz-self.velocity[2])
        else:
            self.range_heights.clear()
        self.position += self.velocity * dt
        if self.uncertainty:self.position_std=self.uncertainty.state()["position_std_m"]
        else:self.position_std += .003 * dt
        self.last_range = range_m
        self.history.append((stamp, self.position.copy(), self.heading))
        self.velocity_history.append((stamp,self.velocity.copy(),flow_usable and flow_reason=="floor_patch_clear"))
        if range_new:self._last_range_input_stamp=range_stamp
        return {"position_world_m": self.position.tolist(), "velocity_world_m_s": self.velocity.tolist(),
                "heading_rad": self.heading, "position_std_m": self.position_std,
                "gyro_z_bias_estimate_deg_s":math.degrees(self.gyro_z_bias),
                "horizontal_accel_residual_heading_m_s2":self.accel_bias_heading[:2].tolist(),
                "horizontal_accel_residual_floor_updates":self.accel_bias_floor_updates,
                "horizontal_accel_residual_scope":"optional qualified-floor velocity innovation fit; includes IMU tilt/model error, not measured chip calibration",
                "surface_hypothesis": self.surface, "range_m": range_m,
                "flow_fusion_usable":flow_usable,"flow_fusion_reason":flow_reason,
                "sensor_scope": "level optical-probe estimation, no aircraft EKF",
                **({"vertical_range_velocity_model":self.range_velocity_model,
                    "vertical_range_velocity_window_s":self.range_velocity_window,
                    "vertical_fit_linear_fallbacks":self.vertical_fit_fallbacks,
                    "vertical_fit_scope":"causal measured range/IMU scalar fit; interval timing, tilt, bias and quality S/U"}
                   if self.range_velocity_model=="imu_compensated" else {}),
                **(self.uncertainty.state() if self.uncertainty else {"uncertainty_method":"legacy linear timer heuristic","uncertainty_is_calibrated_probability":False})}

    def fitted_range_velocity(self, stamp):
        fit_history_time(self.range_heights,
                         self.vertical_imu if self.range_velocity_model=="imu_compensated" else (),stamp)
        if self.range_velocity_model=="imu_compensated":
            try:
                return instantaneous_range_velocity(self.range_heights,self.vertical_imu,stamp)
            except ValueError:
                self.vertical_fit_fallbacks+=1
        ts=np.array([r[0] for r in self.range_heights]);ts-=ts.mean()
        heights=np.array([r[1] for r in self.range_heights])
        return float(np.dot(ts,heights-heights.mean())/np.dot(ts,ts))


class VehicleState:
    """관측 pad 위치 이력 회귀로 속도 추정. Odom 오차도 포함된다."""
    def __init__(self, cfg):
        self.cfg = cfg["observer"]
        self.positions = deque()
        self.headings = deque()
        self.pad = None
        self.velocity = np.zeros(3)
        self.heading = None
        self.heading_rate = 0.
        self.stamp = None
        self.receipt = None
        self.samples = 0
        self.quality={}
        self.pad_height=cfg["landing_geometry"]["pad_height_m"]
        self.velocity_method=self.cfg.get("vehicle_velocity_method","world_position_regression")
        self.kinematics=deque()
        self.local_velocity=None
        self.velocity_source="image pad positions plus sensor odometry regression"

    def change_anchor(self, delta, yaw_delta):
        # All regression samples must move with an odometry gauge correction.
        # Otherwise its coordinate jump is misread as actual vehicle velocity.
        self.pad=self.pad+delta
        self.heading=wrap(self.heading+yaw_delta)
        self.positions=deque((t,p+delta) for t,p in self.positions)
        self.headings=deque((t,wrap(h+yaw_delta)) for t,h in self.headings)
        self.velocity=rotation(yaw_delta)@self.velocity

    def observe(self, pad, heading, capture_stamp, receipt, relative_pad_vehicle=None,
                self_velocity_vehicle=None, floor_velocity_usable=False):
        pad = np.asarray(pad)
        if self.stamp is not None:
            dt = capture_stamp - self.stamp
            if dt <= 0:
                return False, "capture_timestamp_order"
            pred = self.pad + self.velocity * dt
            if np.linalg.norm(pad-pred) > self.cfg["max_position_innovation_m"]:
                return False, "pad_position_innovation"
            gap = wrap(heading - self.heading)
            if abs(gap) > math.radians(self.cfg["max_heading_innovation_deg"]):
                return False, "vehicle_heading_innovation"
            self.pad = pred + self.cfg["position_filter_gain"] * (pad-pred)
            self.heading = wrap(self.heading + self.cfg["heading_filter_gain"] * gap)
        else:
            self.pad, self.heading = pad.copy(), heading
        self.stamp, self.receipt = capture_stamp, receipt
        self.positions.append((capture_stamp, pad.copy()))
        self.headings.append((capture_stamp, float(heading)))
        while len(self.positions) > 2 and capture_stamp-self.positions[0][0] > self.cfg["velocity_fit_window_s"]:
            self.positions.popleft()
        while len(self.headings)>2 and capture_stamp-self.headings[0][0]>self.cfg["heading_fit_window_s"]:
            self.headings.popleft()
        relative_method=(self.velocity_method=="relative_kinematics" and relative_pad_vehicle is not None
                         and self_velocity_vehicle is not None)
        if not relative_method and len(self.positions) >= 4 and self.positions[-1][0]-self.positions[0][0] >= .15:
            ts = np.array([r[0] for r in self.positions])
            points = np.array([r[1] for r in self.positions])
            ts -= np.mean(ts)
            fitted = np.sum(ts[:, None] * (points-np.mean(points, axis=0)), axis=0) / np.sum(ts**2)
            self.velocity += self.cfg["velocity_filter_gain"] * (fitted - self.velocity)
        if len(self.headings)>=4 and self.headings[-1][0]-self.headings[0][0]>=.15:
            ts=np.array([r[0] for r in self.headings]);ts-=ts.mean()
            angles=np.unwrap([r[1] for r in self.headings])
            fitted=float(np.dot(ts,angles-angles.mean())/np.dot(ts,ts))
            self.heading_rate+=.10*(fitted-self.heading_rate)
        if relative_method:
            if floor_velocity_usable:
                self.kinematics.append((capture_stamp,np.asarray(relative_pad_vehicle).copy(),np.asarray(self_velocity_vehicle).copy()))
            while len(self.kinematics)>2 and capture_stamp-self.kinematics[0][0]>self.cfg["velocity_fit_window_s"]:
                self.kinematics.popleft()
            if floor_velocity_usable and len(self.kinematics)>=4 and self.kinematics[-1][0]-self.kinematics[0][0]>=.15:
                ts=np.array([r[0] for r in self.kinematics]);ts-=ts.mean()
                rels=np.array([r[1] for r in self.kinematics])
                derivative=np.sum(ts[:,None]*(rels-rels.mean(axis=0)),axis=0)/np.dot(ts,ts)
                own_v=np.mean([r[2] for r in self.kinematics],axis=0)
                rotational=np.cross([0.,0.,self.heading_rate],rels.mean(axis=0))
                fitted=own_v+derivative+rotational;fitted[2]=0.
                if self.local_velocity is None:self.local_velocity=fitted
                else:self.local_velocity+=self.cfg["velocity_filter_gain"]*(fitted-self.local_velocity)
                self.velocity_source="capture-relative vehicle-frame kinematics plus qualified floor Flow/IMU velocity"
            elif self.local_velocity is not None:
                self.velocity_source="last floor-referenced image speed held; heading still observed; no pad-relative Flow fed back into vehicle speed"
            if self.local_velocity is not None:self.velocity=rotation(self.heading)@self.local_velocity
            else:self.velocity_source="relative kinematics awaiting qualified floor velocity baseline"
        # The pad is known to stay at .30m. Vertical image residual corrects
        # own height, rather than inventing a vehicle vertical motion signal.
        self.pad[2]=self.pad_height
        self.velocity[2]=0.
        self.samples += 1
        return True, "accepted"

    def state(self, now, self_motion):
        if self.stamp is None:
            return None
        age = now-self.stamp
        pad = self.pad + self.velocity * age
        return {"valid": age <= self.cfg["image_max_age_s"], "capture_stamp_s": self.stamp,
                "age_s": age, "pad_world_m": pad.tolist(),
                "relative_pad_world_m": (pad-self_motion.position).tolist(),
                "relative_velocity_world_m_s": (self.velocity-self_motion.velocity).tolist(),
                "vehicle_velocity_world_m_s": self.velocity.tolist(),
                "vehicle_heading_rad": wrap(self.heading+self.heading_rate*age),
                "vehicle_heading_rate_rad_s": self.heading_rate, "samples": self.samples,
                "last_image_quality":self.quality,
                "image_currently_fresh":age<=self.cfg["image_max_age_s"],
                "propagation_since_image_s":age,
                "velocity_source":self.velocity_source}
