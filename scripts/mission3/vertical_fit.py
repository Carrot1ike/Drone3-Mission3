"""Causal scalar range/IMU velocity fit; no contact or actual-state input."""
import numpy as np


def instantaneous_range_velocity(measurements, imu, now):
    """Subtract integrated acceleration displacement, then fit initial speed.

    Right-end stamped interval-average acceleration is a timing hypothesis.
    Bias and gravity-compensation errors are not identified by this fit.
    """
    ranges = np.asarray(measurements, dtype=float)
    samples = np.asarray(imu, dtype=float)
    if (ranges.ndim != 2 or ranges.shape[1] != 2 or len(ranges) < 6
            or samples.ndim != 2 or samples.shape[1] != 2 or len(samples) < 2
            or not np.isfinite(now) or not np.all(np.isfinite(ranges))
            or not np.all(np.isfinite(samples))):
        raise ValueError('insufficient or nonfinite measured history')
    mt, height = ranges.T
    it, acceleration = samples.T
    if (not np.all(np.diff(mt) > 0) or not np.all(np.diff(it) > 0)
            or np.any(np.diff(it) > .2) or mt[0] < it[0] or mt[-1] > now
            or it[-1] > now
            or now > it[-1] + 1e-8):
        raise ValueError('unordered, uncovered, gapped or future measured timestamps')
    dt = np.diff(it)
    velocity = np.r_[0., np.cumsum(acceleration[1:] * dt)]
    displacement = np.r_[0., np.cumsum(velocity[:-1] * dt + .5 * acceleration[1:] * dt ** 2)]

    def integrate(times):
        index = np.minimum(np.searchsorted(it, times, side='right') - 1, len(it) - 2)
        elapsed = times - it[index]
        return (displacement[index] + velocity[index] * elapsed
                + .5 * acceleration[index + 1] * elapsed ** 2,
                velocity[index] + acceleration[index + 1] * elapsed)

    position_delta, _ = integrate(mt)
    center = mt - mt.mean()
    corrected = height - position_delta
    initial_speed = float(center @ (corrected - corrected.mean()) / (center @ center))
    _, speed_delta = integrate(np.array([now]))
    return initial_speed + float(speed_delta[0])
