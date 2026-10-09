"""Plant 전용 차량 시각/속도. 제어기·추정기에 전달하지 않는다."""
import math


class VehicleMotion:
    def __init__(self, settings, track, initial_arc):
        self.nominal=float(settings["speed_m_s"])
        self.track=track
        self.initial_arc=float(initial_arc)
        self.options=settings.get("motion")
        self.start_time=None if self.options else 0.
        if self.options:
            if self.options.get("start_trigger") != "hover_ready" or self.options.get("end_behavior") != "stop":
                raise ValueError("straight motion requires hover_ready and stop")
            self.acceleration=float(self.options["acceleration_m_s2"])
            if not math.isfinite(self.nominal) or self.nominal <= 0 or not math.isfinite(self.acceleration) or self.acceleration <= 0:
                raise ValueError("finite positive vehicle speed and acceleration required")
            self.distance=track.length-self.initial_arc
            if self.distance <= 0:raise ValueError("no remaining straight travel")
            self.peak=min(self.nominal,math.sqrt(self.acceleration*self.distance))
            self.ramp=self.peak/self.acceleration
            self.ramp_distance=.5*self.acceleration*self.ramp**2
            self.cruise=max(0.,(self.distance-2*self.ramp_distance)/self.peak)
            self.duration=2*self.ramp+self.cruise

    def start(self, now):
        if self.options and self.start_time is None:
            self.start_time=float(now)
            return True
        return False

    def sample(self, now):
        if not self.options:
            # Preserve the historical continuous stadium calculation exactly.
            return self.initial_arc+self.nominal*now,self.nominal,"continuous"
        if self.start_time is None:return self.initial_arc,0.,"waiting_for_hover"
        t=max(0.,float(now)-self.start_time)
        if t < self.ramp:
            return self.initial_arc+.5*self.acceleration*t*t,self.acceleration*t,"accelerating"
        if t < self.ramp+self.cruise:
            return self.initial_arc+self.ramp_distance+self.peak*(t-self.ramp),self.peak,"cruising"
        if t < self.duration:
            remaining=self.duration-t
            return self.track.length-.5*self.acceleration*remaining**2,self.acceleration*remaining,"decelerating"
        return self.track.length,0.,"stopped_at_endpoint"
