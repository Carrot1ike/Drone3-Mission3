"""Sensor-only stable hover fixture before Mission3, not a Mission2 route.

The runner preserves estimator, PID, actuator and physics states. Only the
mission phase clock starts when the declared readiness window is complete.
No pose, motor force or live vehicle truth is accepted here.
"""
import math
import numpy as np
from .command import Command
from .track import wrap


class HoverHandoff:
    def __init__(self,entry,settings):
        self.c=dict(settings)
        self.target=np.r_[entry["xy_m"],self.c.get("target_height_m",1.)]
        self.heading=math.radians(entry["heading_deg"])
        self.since=None
        self.last_time=None
        self.last_floor_s=None
        self.latest=None

    def update(self,now,own):
        if not math.isfinite(now) or now<0 or (self.last_time is not None and now<=self.last_time):
            raise ValueError("handoff timestamps must increase and be nonnegative")
        self.last_time=now
        p=np.asarray(own["position_world_m"],float)
        v=np.asarray(own["velocity_world_m_s"],float)
        rpy=np.asarray(own["rpy_rad"],float)
        finite=bool(np.isfinite(p).all() and np.isfinite(v).all() and np.isfinite(rpy).all())
        floor=bool(own.get("flow_fusion_reason")=="floor_patch_clear" and own.get("range_accurate",False))
        if floor:self.last_floor_s=now
        # A brief quality dropout does not prove a changed surface. Continue
        # only the static-floor hypothesis, with accurate range and a finite
        # stale budget. The actual handoff still requires a current floor fix.
        floor_age=None if self.last_floor_s is None else now-self.last_floor_s
        grace=bool(not floor and floor_age is not None
                   and floor_age<=self.c.get("floor_quality_grace_s",0.)+1e-8
                   and own.get("range_accurate",False)
                   and own.get("surface_hypothesis")=="static_floor_candidate"
                   and own.get("flow_fusion_reason")=="floor_patch_overlap_or_invalid")
        aligned=finite and (
            abs(p[2]-self.target[2])<self.c["height_tolerance_m"]
            and np.linalg.norm(p[:2]-self.target[:2])<self.c["position_xy_tolerance_m"]
            and abs(v[2])<self.c["vertical_speed_limit_m_s"]
            and np.linalg.norm(v[:2])<self.c["horizontal_speed_limit_m_s"]
            and math.hypot(rpy[0],rpy[1])<math.radians(self.c["tilt_limit_deg"])
            and abs(wrap(own["heading_rad"]-self.heading))<math.radians(self.c["heading_tolerance_deg"]))
        stable=bool((floor or grace) and aligned)
        if stable:
            if self.since is None:self.since=now
        else:self.since=None
        ready=bool(floor and self.since is not None and now-self.since>=self.c["stable_duration_s"])
        self.latest={"t":float(now),"ready":ready,"floor_datum_qualified":floor,
                     "floor_quality_grace_active":grace,"last_qualified_floor_age_s":floor_age,
                     "stable_since_s":self.since,"sensor_estimated_position_m":p.tolist(),
                     "sensor_estimated_velocity_m_s":v.tolist(),"sensor_estimated_rpy_rad":rpy.tolist(),
                     "scope":"sensor-only continuous-hover fixture; not live Mission2 execution"}
        if not finite:
            return Command("HANDOFF_PREROLL",[0.,0.,0.],0.,done=True,failure="handoff_nonfinite_estimate"),False
        velocity=np.clip(self.c["position_gain_per_s"]*(self.target-p),
                         -self.c["velocity_limit_m_s"],self.c["velocity_limit_m_s"])
        yaw_rate=float(np.clip(1.2*wrap(self.heading-own["heading_rad"]),-.2,.2))
        command=Command("HANDOFF_PREROLL",velocity.tolist(),yaw_rate)
        if not ready and now>=self.c["max_duration_s"]:
            command.done=True;command.failure="handoff_preroll_not_stable"
        return command,ready
