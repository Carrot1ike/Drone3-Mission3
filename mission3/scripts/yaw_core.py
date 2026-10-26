"""Image/IMU-only static-marker observer; no simulator truth dependency."""
from collections import deque
import math

import cv2
import numpy as np
from scipy.optimize import least_squares


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotation_rpy(roll, pitch, yaw):
    sr, cr, sp, cp, sy, cy = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def quaternion_rpy(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(np.clip(2*(w*y-z*x), -1, 1)),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def euler_yaw_rate(gyro_rad_s, roll, pitch):
    if abs(math.cos(pitch)) < 0.5:
        raise ValueError("pitch too steep for this yaw experiment")
    return (gyro_rad_s[1]*math.sin(roll) + gyro_rad_s[2]*math.cos(roll)) / math.cos(pitch)


def marker_detector_parameters(name="default_subpix"):
    """고정된 소프트웨어 감도 후보. 사전/오류교정/pose gate 변경 없음."""
    if name not in ("default_subpix", "small_code_cell8_subpix"):
        raise ValueError("unknown fixed marker detector profile")
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if name == "small_code_cell8_subpix":
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 13
        params.adaptiveThreshWinSizeStep = 2
        params.adaptiveThreshConstant = 7.
        params.minMarkerPerimeterRate = .02
        params.perspectiveRemovePixelPerCell = 8
    return params


class MarkerYaw:
    def __init__(self, profile, settings, reference_rotation_cv=None):
        self.profile, self.settings = profile, settings
        self.K = np.array(profile["K"])
        self.dist = np.array(profile["distortion"])
        # Optical-point testbeds can supply their explicitly virtual heading
        # reference without creating a camera-to-aircraft mounting parameter.
        self.Rbc = np.array(profile["R_body_camera_cv"] if reference_rotation_cv is None else reference_rotation_cv)
        self.Rwm = np.array(profile["R_world_marker"])
        half = profile["marker_length_m"] / 2
        self.points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
        self.detector_profile=settings.get("detector_profile","default_subpix")
        params = marker_detector_parameters(self.detector_profile)
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, profile["dictionary"])), params)

    def detect(self, gray, roll, pitch):
        result = {"valid": False, "reason": "marker_missing"}
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or self.profile["marker_id"] not in ids.flatten():
            return result
        image_points = np.asarray(corners[list(ids.flatten()).index(self.profile["marker_id"])], dtype=float).reshape(4, 2)
        result["corners"] = image_points.tolist()
        side = float(np.min(np.linalg.norm(image_points - np.roll(image_points, 1, axis=0), axis=1)))
        result["min_side_px"] = side
        if side < self.settings["min_side_px"]:
            result["reason"] = "marker_too_small"
            return result
        if np.any(image_points < 3) or np.any(image_points[:, 0] > gray.shape[1]-4) or np.any(image_points[:, 1] > gray.shape[0]-4):
            result["reason"] = "marker_near_edge"
            return result
        ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(self.points, image_points, self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            result["reason"] = "pnp_failed"
            return result
        raw, constrained = [], []
        for rv, tv in zip(rvecs, tvecs):
            if not np.all(np.isfinite(rv)) or not np.all(np.isfinite(tv)):
                raw.append({"valid": False, "reason": "nonfinite_ippe_seed"})
                continue
            Rcm = cv2.Rodrigues(rv)[0]
            Rwb = self.Rwm @ Rcm.T @ self.Rbc.T
            yaw = math.atan2(Rwb[1, 0], Rwb[0, 0])
            projected = cv2.projectPoints(self.points, rv, tv, self.K, self.dist)[0].reshape(4, 2)
            raw.append({"yaw_deg": math.degrees(yaw), "rmse_px": float(np.sqrt(np.mean(np.sum((projected-image_points)**2, axis=1))))})

            # Fit yaw + translation with measured roll/pitch. Both IPPE seeds are kept;
            # confidence is evaluated after the independent attitude constraint.
            def residual(state):
                R = self.Rbc.T @ rotation_rpy(roll, pitch, state[0]).T @ self.Rwm
                xyz = self.points @ R.T + state[1:]
                # The attitude-constrained refinement must use the same lens
                # model as IPPE. A pinhole residual silently ignores supplied
                # distortion and biases the supposedly calibrated pose.
                uv=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),self.K,self.dist)[0].reshape(-1,2)
                return (uv-image_points).flatten()
            initial = np.r_[wrap(yaw), tv.flatten()]
            # Near a frontal planar view, an IPPE seed can become very large.
            # Keep the fit within its domain; a bad candidate is an invalid
            # observation, never an exception that stops the command loop.
            initial = np.clip(initial, [-math.pi+.000001, -9.99, -9.99, .03], [math.pi-.000001, 9.99, 9.99, 9.99])
            try:
                fit = least_squares(residual, initial, bounds=([-math.pi, -10., -10., .02], [math.pi, 10., 10., 10.]), max_nfev=35)
            except (ValueError, FloatingPointError):
                continue
            R = self.Rbc.T @ rotation_rpy(roll, pitch, fit.x[0]).T @ self.Rwm
            depth = (self.points @ R.T + fit.x[1:])[:, 2]
            constrained.append({"yaw_deg": math.degrees(wrap(fit.x[0])), "rmse_px": float(np.sqrt(np.mean(fit.fun.reshape(4, 2)**2)*2)),
                                "positive_depth": bool(np.all(depth > .02)), "converged": bool(fit.success),
                                "t_camera_marker_m": fit.x[1:].tolist()})
        result["ippe_candidates"], result["constrained_candidates"] = raw, constrained
        if not constrained:
            result["reason"] = "invalid_pnp_candidates"
            return result
        constrained.sort(key=lambda x: x["rmse_px"])
        best = constrained[0]
        if not best["positive_depth"] or not best["converged"] or best["rmse_px"] > self.settings["max_rmse_px"]:
            result["reason"] = "constrained_fit_rejected"
            return result
        if len(constrained) > 1:
            gap = abs(math.degrees(wrap(math.radians(best["yaw_deg"]-constrained[1]["yaw_deg"]))))
            result["candidate_yaw_gap_deg"] = gap
            if gap > self.settings["max_candidate_yaw_gap_deg"] and constrained[1]["rmse_px"]-best["rmse_px"] < self.settings["min_cost_separation_px"]:
                result["reason"] = "yaw_candidates_ambiguous"
                return result
        result.update(valid=True, reason="accepted", yaw_deg=best["yaw_deg"], rmse_px=best["rmse_px"],
                      t_camera_marker_m=best["t_camera_marker_m"])
        return result


class YawFusion:
    def __init__(self, settings):
        self.settings = settings
        self.angle = None
        self.last_imu_time = None
        self.last_visual_time = None
        self.history = deque(maxlen=300)
        self.roll, self.pitch = 0., 0.
        self.rate = 0.
        self.predict_only = 0.

    def imu(self, stamp, gyro_deg_s, roll_deg, pitch_deg, test_bias_deg_s=0.):
        self.roll, self.pitch = math.radians(roll_deg), math.radians(pitch_deg)
        self.rate = euler_yaw_rate(np.deg2rad(gyro_deg_s), self.roll, self.pitch) + math.radians(test_bias_deg_s)
        dt = 0. if self.last_imu_time is None else stamp-self.last_imu_time
        if dt <= 0 or dt > .20:
            if self.last_imu_time is not None and dt > .20:
                self.history.clear()
            dt = 0.
        if self.angle is not None:
            self.angle = wrap(self.angle + self.rate*dt)
        self.predict_only += self.rate*dt
        self.last_imu_time = stamp
        self.history.append((stamp, self.predict_only, self.roll, self.pitch))

    def at(self, stamp):
        if not self.history or stamp < self.history[0][0] or stamp > self.history[-1][0] + .05:
            return None
        times = [v[0] for v in self.history]
        return tuple(float(np.interp(stamp, times, [v[i] for v in self.history])) for i in (1, 2, 3))

    def visual(self, measured_rad, stamp, now):
        past = self.at(stamp)
        if past is None:
            return False, "no_imu_history"
        current_measurement = wrap(measured_rad + self.predict_only-past[0])
        if self.angle is None:
            self.angle = current_measurement
        else:
            innovation = wrap(current_measurement-self.angle)
            if abs(innovation) > math.radians(self.settings["max_innovation_deg"]):
                return False, "innovation_rejected"
            self.angle = wrap(self.angle+self.settings["visual_gain"]*innovation)
        self.last_visual_time = now
        return True, "fused"


def rate_command(target, estimate, settings):
    error = wrap(target-estimate)
    if abs(error) < math.radians(settings["deadband_deg"]):
        return 0.
    cap = math.radians(settings["max_rate_deg_s"])
    return float(np.clip(settings["kp_per_s"]*error, -cap, cap))
