#!/usr/bin/env python3
"""Ideal one-dimensional command-budget diagnostic; never a control input.

This assumes a perfectly tracked command and perfectly known straight target
motion. It is not a physical reachability proof: actual speed, estimator path,
turning, camera visibility, and calibration need separate verification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def trajectory(t, v0, acceleration, maximum):
    t = np.asarray(t, float)
    saturation = (maximum-v0)/acceleration
    ramp = np.minimum(t, saturation)
    displacement = v0*ramp + .5*acceleration*ramp*ramp + maximum*np.maximum(t-saturation, 0.)
    if v0 < 0:
        zero = -v0/acceleration
        path = np.where(t <= zero, -displacement, displacement+v0*v0/acceleration)
    else:
        path = displacement.copy()
    return displacement, path


def best_case(v0, acceleration, maximum, target_speed, path_budget, time_budget):
    low, high = 0., time_budget
    if trajectory(high, v0, acceleration, maximum)[1] > path_budget:
        for _ in range(80):
            middle = (low+high)/2
            if trajectory(middle, v0, acceleration, maximum)[1] <= path_budget: low = middle
            else: high = middle
        end = low
    else: end = high
    displacement, path = trajectory(end, v0, acceleration, maximum)
    closing = max(0., float(displacement-target_speed*end))
    # Independent numerical integration checks the absolute path across a
    # backwards-to-forwards reversal, rather than using net displacement.
    dt = time_budget/200000
    t = np.arange(200001)*dt
    mid = (t[:-1]+t[1:])/2
    v = np.minimum(v0+acceleration*mid, maximum)
    numerical_path = np.r_[0., np.cumsum(np.abs(v))*dt]
    numerical_displacement = np.r_[0., np.cumsum(v)*dt]
    mask = numerical_path <= path_budget+1e-10
    numeric_closing = float(np.max(numerical_displacement[mask]-target_speed*t[mask]))
    if abs(numeric_closing-closing) > .0001:
        raise RuntimeError('analytic versus numerical budget mismatch')
    return {'initial_parallel_command_m_s':v0, 'path_budget_m':path_budget,
            'time_used_s':end, 'absolute_path_m':float(path), 'net_forward_displacement_m':float(displacement),
            'maximum_gap_reduction_m':closing, 'numerical_maximum_gap_reduction_m':numeric_closing,
            'reversal_time_s':max(0., -v0/acceleration),
            'time_until_command_faster_than_target_s':max(0., (target_speed-v0)/acceleration)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    if a.out.exists(): raise FileExistsError('preserve prior diagnostic')
    config = json.loads(a.config.read_text())
    c = config['control']
    target_speed = config['vehicle_plant_only']['speed_m_s']
    limits = {'maximum_m_s':c['max_speed_m_s'], 'acceleration_m_s2':c['max_accel_m_s2'],
              'target_speed_m_s':target_speed, 'coarse_time_budget_s':c.get('coarse_motion_duration_s',20.)}
    if not 0 <= target_speed < limits['maximum_m_s']: raise ValueError('faster-than-target fixture required')
    results = [best_case(v, limits['acceleration_m_s2'], limits['maximum_m_s'], target_speed, budget,
                        limits['coarse_time_budget_s'])
               for v in (-.15, 0., .15, target_speed)
               for budget in (c.get('coarse_approach_distance_m',1.5), c.get('coarse_live_path_limit_m',4.))]
    payload = {'scope':'offline ideal straight-command comparison only; target speed is user scenario setting, never estimator/controller input',
               'limits_are':'S software settings and user vehicle scenario, not manufacturer aircraft performance',
               'assumptions':['perfect command tracking and path measurement', 'target and drone move along one line',
                              'instant best gaze and continuous observations', 'maximum allowed positive acceleration then maximum speed',
                              'no tilt, optical, dynamics, contact, or uncertain world-frame restrictions'],
               'limits':limits, 'results':results,
               'input_sha256':{str(a.config):hashlib.sha256(a.config.read_bytes()).hexdigest(),
                               'scripts/mission3/controller.py':hashlib.sha256(Path('scripts/mission3/controller.py').read_bytes()).hexdigest(),
                               str(Path(__file__)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
               'finding':'initial opposite motion consumes both time and absolute path; blind base budget may never reduce the moving-target gap. This does not prove native-pixel acquisition or physical impossibility and does not authorize extending blind motion.'}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(payload, indent=2)+'\n')
    print(json.dumps(payload, indent=2))


if __name__ == '__main__': main()
