"""Bounded failed-flight command delivery; no contact or safe recovery claim."""
import math


def finite_control_state(own):
    try:
        values = [*own['position_world_m'], *own['velocity_world_m_s'],
                  *own['rpy_rad'], *own['gyro_body_rad_s'], own['heading_rad']]
        return all(len(own[k]) == 3 for k in ('position_world_m', 'velocity_world_m_s',
                                            'rpy_rad', 'gyro_body_rad_s')) and all(math.isfinite(x) for x in values)
    except (KeyError, TypeError, ValueError):
        return False


class AbortDelivery:
    def __init__(self, now, reason, max_duration_s=3., dwell_s=.5):
        if not all(math.isfinite(x) for x in (now, max_duration_s, dwell_s)) or not reason or max_duration_s < dwell_s or dwell_s <= 0:
            raise ValueError('finite start and bounded positive dwell required')
        self.start = now
        self.reason = reason
        self.max_duration = max_duration_s
        self.dwell = dwell_s
        self.last_observe = None
        self.stable_since = None
        self.packet_count = 0
        self.physics_steps = 0
        self.finish = None
        self.latest = None

    def note_wire(self, now):
        if now >= self.start:
            self.packet_count += 1

    def note_physics_step(self, now):
        if now > self.start:
            self.physics_steps += 1

    def observe(self, now, own, sample_stamp):
        if not math.isfinite(now) or now < self.start or (self.last_observe is not None and now <= self.last_observe):
            raise ValueError('observation time must increase from failure start')
        gap = None if self.last_observe is None else now - self.last_observe
        self.last_observe = now
        if gap is not None and gap > .03:
            self.stable_since = None
        finite = finite_control_state(own)
        fresh = math.isfinite(sample_stamp) and 0 <= now - sample_stamp <= .03
        speed = math.sqrt(sum(v*v for v in own['velocity_world_m_s'])) if finite else None
        tilt = math.hypot(*own['rpy_rad'][:2]) if finite else None
        gyro = math.sqrt(sum(v*v for v in own['gyro_body_rad_s'])) if finite else None
        stable = (finite and fresh and own.get('range_accurate') is True
                  and own.get('flow_valid') is True and own.get('flow_fusion_usable') is True
                  and own.get('flow_fusion_reason') == 'floor_patch_clear'
                  and speed <= .05 and tilt <= math.radians(10) and gyro <= math.radians(5))
        if stable:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        elapsed = now - self.start
        stable_elapsed = 0. if self.stable_since is None else now - self.stable_since
        delivered = self.packet_count > 0 and self.physics_steps > 0
        if self.finish is None and delivered:
            if stable and stable_elapsed >= self.dwell:
                self.finish = 'sensor_stationary_hold_observed'
            elif elapsed >= self.max_duration - 1e-9:
                self.finish = 'bounded_delivery_elapsed_without_stationary_evidence'
        self.latest = {'t': now, 'sample_stamp_s': sample_stamp, 'finite': finite,
                       'fresh': fresh, 'stable': stable, 'estimated_speed_m_s': speed,
                       'estimated_tilt_rad': tilt, 'estimated_gyro_norm_rad_s': gyro,
                       'stable_dwell_s': stable_elapsed}
        return self.finish

    def summary(self):
        return {'schema': 'mission3-abort-delivery/1', 'failure_latched': self.reason,
                'started_at_s': self.start, 'packet_count_after_failure': self.packet_count,
                'physics_steps_after_failure': self.physics_steps, 'finish': self.finish,
                'last_observation': self.latest,
                'settings_class': 'S', 'max_delivery_s': self.max_duration, 'stationary_dwell_s': self.dwell,
                'deadline_clock_roundoff_tolerance_s': 1e-9,
                'safe_recovery_or_motor_stop_confirmed': False,
                'scope': 'bounded zero-velocity delivery and sensor stationary observation; failure retained, airborne motors remain commanded'}
