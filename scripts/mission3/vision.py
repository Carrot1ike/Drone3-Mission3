"""차량 방향을 미리 모르는 ArUco 관측. Level 광학점 시험용."""
import math
import numpy as np
from yaw_core import MarkerYaw,rotation_rpy
from aruco_pose import pinhole_matrix
from .scene import R_VEHICLE_MARKER, R_HEADING_CAMERA
from .track import rotation, wrap
from .estimator import VehicleState
from .settings import decoded_candidate_gaze_options


class Observer:
    def __init__(self, cfg, self_motion):
        observer_optics=cfg["camera"].get("observer_optics_sensitivity",{})
        profile = {"K": observer_optics.get("K",pinhole_matrix(320,320,cfg["camera"]["vertical_fov_deg"]).tolist()),
                   "distortion":observer_optics.get("distortion",[0.,0.,0.,0.,0.]),
                   "R_world_marker": R_VEHICLE_MARKER.tolist(),
                   "marker_length_m": cfg["marker"]["code_side_m"],
                   "marker_id": cfg["marker"]["id"], "dictionary": cfg["marker"]["dictionary"]}
        settings = cfg["observer"]
        det_settings = {"min_side_px":settings["min_side_px"], "max_rmse_px":settings["max_rmse_px"],
                        "max_candidate_yaw_gap_deg": settings["max_candidate_heading_gap_deg"],
                        "min_cost_separation_px": settings["min_cost_separation_px"]}
        if "detector_profile" in settings:
            det_settings["detector_profile"]=settings["detector_profile"]
        # R_world_marker parameter is a VEHICLE reference here; no known world yaw.
        self.detector = MarkerYaw(profile, det_settings, reference_rotation_cv=R_HEADING_CAMERA)
        self.offset = np.asarray(cfg["marker"]["known_marker_to_pad_vehicle_m"])
        self.self_motion = self_motion
        self.vehicle = VehicleState(cfg)
        self.coarse=None
        self.tentative=None
        self.candidate_gaze=decoded_candidate_gaze_options(settings)
        self.coarse_history=[]
        self.coarse_min_side=cfg["observer"]["coarse_bearing_min_side_px"]
        self.acquire_min_side=settings["min_side_px"]
        self.tracking_min_side=settings["tracking_min_side_px"]
        self.metric_min_depth=float(settings.get("metric_pose_min_depth_m",0.))
        if not math.isfinite(self.metric_min_depth) or self.metric_min_depth<0:
            raise ValueError("metric pose depth guard must be finite and nonnegative")

    def frame(self, gray, capture_stamp, receipt):
        if gray.shape != (320,320) or gray.dtype != np.uint8 or gray.ndim != 2:
            raise ValueError("native mono8 320×320 required; no resize/color conversion")
        own = self.self_motion.at(capture_stamp)
        if own is None:
            return {"valid":False,"reason":"odometry_capture_history_missing"}
        # Acquisition and an already identified target have separate software
        # gates; all pose ambiguity/reprojection/innovation gates still apply.
        tracking=self.vehicle.stamp is not None and capture_stamp-self.vehicle.stamp<.35
        self.detector.settings["min_side_px"]=self.tracking_min_side if tracking else self.acquire_min_side
        # The geometry adapter has level optics. A physical adapter may expose
        # capture-time IMU attitude and its explicitly conditional mount.
        attitude=(0.,0.) if not hasattr(self.self_motion,"attitude_at") else self.self_motion.attitude_at(capture_stamp)
        if attitude is None:return {"valid":False,"reason":"attitude_capture_history_missing"}
        roll,pitch=attitude
        Rbc=getattr(self.self_motion,"camera_rotation",R_HEADING_CAMERA)
        self.detector.Rbc=Rbc
        observed = self.detector.detect(gray, roll, pitch)
        observed["capture_stamp_s"], observed["receipt_s"] = capture_stamp, receipt
        if not observed["valid"]:
            if (observed.get("reason")=="marker_too_small" and observed.get("min_side_px",0.)>=self.coarse_min_side):
                uv=np.mean(observed["corners"],axis=0)
                import cv2
                normalized=cv2.undistortPoints(np.asarray(uv,float).reshape(1,1,2),self.detector.K,self.detector.dist).reshape(2)
                ray=rotation_rpy(roll,pitch,own[1])@Rbc@np.r_[normalized,1.]
                bearing=wrap(math.atan2(ray[1],ray[0]))
                if self.candidate_gaze.get("enabled",False):
                    # A decoded ID may interrupt search gaze, but is not a
                    # temporally confirmed bearing or a metric vehicle state.
                    self.tentative={"kind":"unconfirmed_decoded_bearing","valid":False,
                                    "bearing_valid":True,"capture_stamp_s":capture_stamp,
                                    "age_s":receipt-capture_stamp,"bearing_world_rad":bearing,
                                    "min_side_px":observed["min_side_px"],
                                    "source":"one decoded target ID; gaze/braking only; no pose or FF"}
                previous=self.coarse_history[-1] if self.coarse_history else None
                continuous=previous is not None and capture_stamp-previous[0]<.7 and abs(wrap(bearing-previous[1]))<math.radians(12)
                self.coarse_history.append((capture_stamp,bearing))
                self.coarse_history=self.coarse_history[-5:]
                if continuous:
                    self.coarse={"kind":"coarse_image_bearing_only","valid":True,"age_s":receipt-capture_stamp,
                                 "capture_stamp_s":capture_stamp,"bearing_world_rad":bearing,
                                 "source":"two decoded target ids with temporal bearing agreement; no usable metric pose",
                                 "min_side_px":observed["min_side_px"],
                                 "max_side_px":float(np.linalg.norm(np.roll(np.asarray(observed["corners"]),-1,axis=0)-np.asarray(observed["corners"]),axis=1).max())}
                    observed["coarse_guidance_available"]=True
            return observed
        if observed["t_camera_marker_m"][2]<self.metric_min_depth:
            # A later marginal close-range fit must not replace the last
            # reliable pose used by the bounded terminal prediction.
            observed.update(valid=False,reason="metric_pose_depth_guard",
                            minimum_estimated_depth_m=self.metric_min_depth)
            return observed
        own_position, own_yaw = own
        vehicle_yaw = wrap(own_yaw-math.radians(observed["yaw_deg"]))
        Rwb=rotation_rpy(roll,pitch,own_yaw)
        camera_offset=np.asarray(getattr(self.self_motion,"camera_position_body",[0.,0.,0.]))
        marker_delta = Rwb @ (Rbc @ np.asarray(observed["t_camera_marker_m"])+camera_offset)
        relative = marker_delta - rotation(vehicle_yaw) @ self.offset
        pad = own_position + relative
        capture_velocity=self.self_motion.velocity_at(capture_stamp)
        kwargs={}
        if capture_velocity is not None:
            velocity,floor_usable=capture_velocity
            kwargs={"relative_pad_vehicle":rotation(vehicle_yaw).T@relative,
                    "self_velocity_vehicle":rotation(vehicle_yaw).T@velocity,
                    "floor_velocity_usable":floor_usable}
        ok, reason = self.vehicle.observe(pad,vehicle_yaw,capture_stamp,receipt,**kwargs)
        observed.update(valid=ok,reason=reason, raw_relative_pad_world_m=relative.tolist(),
                        raw_pad_world_m=pad.tolist(), raw_vehicle_heading_rad=vehicle_yaw,
                        reference_scope="relative marker fit plus capture-time estimated IMU attitude/odometry; mount source declared by adapter")
        if ok:
            self.vehicle.quality={"min_side_px":observed["min_side_px"],"rmse_px":observed["rmse_px"],
                                  "marker_optical_depth_m":observed["t_camera_marker_m"][2]}
            self.self_motion.visual_anchor(self.vehicle.state(receipt,self.self_motion),receipt,
                                           raw_relative=relative,capture_stamp=capture_stamp)
            anchor={"valid":True,"samples":self.vehicle.samples,"pad":self.vehicle.pad,
                    "heading":self.vehicle.heading,"heading_rate":self.vehicle.heading_rate}
            correction=self.self_motion.track_anchor(anchor,receipt)
            if correction is not None:
                self.vehicle.change_anchor(*correction)
        return observed

    def state(self, now):
        full=self.vehicle.state(now,self.self_motion)
        if full and full["valid"]:return full
        if self.coarse is not None and now-self.coarse["capture_stamp_s"]<.7:
            return {**self.coarse,"age_s":now-self.coarse["capture_stamp_s"]}
        if (self.vehicle.stamp is None and self.tentative is not None
                and 0 <= now-self.tentative["capture_stamp_s"] <= self.candidate_gaze["max_age_s"]):
            return {**self.tentative,"age_s":now-self.tentative["capture_stamp_s"]}
        return full
