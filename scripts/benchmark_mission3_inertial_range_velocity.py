#!/usr/bin/env python3
"""Offline instantaneous vertical fit; no live input, new flight or bias oracle."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import shutil

import numpy as np


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def inertial_fit(measurements, imu, now):
    """Remove measured-acceleration displacement before a two-parameter fit.

    IMU interval averages are right-end stamped. Integrate piecewise constant
    acceleration over each interval, including fractional range timestamps.
    A constant unknown vertical acceleration bias remains a model error.
    """
    mt, height = np.asarray(measurements, float).T
    it, acceleration = np.asarray(imu, float).T
    if (len(mt) < 6 or len(it) < 2 or not np.all(np.diff(mt) > 0)
            or not np.all(np.diff(it) > 0) or mt[0] < it[0] or mt[-1] > now
            or now > it[-1] + 1e-8 or not np.all(np.isfinite([*mt, *height, *it, *acceleration, now]))):
        raise ValueError('invalid, future or insufficient measured timestamps')
    dt = np.diff(it)
    v = np.r_[0., np.cumsum(acceleration[1:] * dt)]
    z = np.r_[0., np.cumsum(v[:-1] * dt + .5 * acceleration[1:] * dt ** 2)]

    def integrate(times):
        index = np.minimum(np.searchsorted(it, times, side='right') - 1, len(it) - 2)
        elapsed = times - it[index]
        return (z[index] + v[index] * elapsed + .5 * acceleration[index + 1] * elapsed ** 2,
                v[index] + acceleration[index + 1] * elapsed)

    displacement, _ = integrate(mt)
    center = mt - mt.mean()
    denominator = float(center @ center)
    legacy = float(center @ (height - height.mean()) / denominator)
    corrected = height - displacement
    initial_velocity = float(center @ (corrected - corrected.mean()) / denominator)
    _, dv = integrate(np.array([now]))
    return legacy, initial_velocity + float(dv[0])


def checks():
    # Analytic independent constant-acceleration trajectories and irregular
    # producer times: the old linear slope refers to the window midpoint.
    outputs = []
    for a in (-.04, 0., .06):
        for origin in (0., 123.):
            t = np.array([.025, .05, .075, .125, .20, .275, .35, .375])
            imu_t = np.arange(0., .401, .01)
            values = .70 - .03 * t + .5 * a * t ** 2
            old, new = inertial_fit(list(zip(origin + t, values)),
                                    list(zip(origin + imu_t, np.full(len(imu_t), a))), origin + .4)
            expected = -.03 + a * .4
            assert abs(new - expected) < 1e-11, (a, old, new, expected)
            outputs.append({'a_m_s2': a, 'time_origin_s': origin, 'legacy_fit_m_s': old,
                            'instantaneous_fit_m_s': new, 'analytic_current_velocity_m_s': expected})
    try:
        inertial_fit(list(zip(t, values)), list(zip(imu_t, np.full(len(imu_t), a))), .30)
    except ValueError:
        pass
    else:
        raise AssertionError('future range timestamp accepted')
    return outputs


def analyze(name):
    run = Path(name)
    rows = [json.loads(x) for x in (run / 'sensors.jsonl').open()]
    summary = json.loads((run / 'adapter-summary.json').read_text())
    end = summary['motor_off_at_s'] or next(x['time_s'] for x in summary['events']
                 if x.get('to') == 'BLIND_RECOVER')
    terminal = next(x['t'] for x in summary['events'] if x.get('phase') == 'TOUCHDOWN_DESCEND')
    history, imu, fits = [], [], []
    window = json.loads((run / 'experiment.json').read_text())['mission']['observer'].get('vertical_range_velocity_window_s', .4)
    previous_range = None
    for x in rows:
        t = x['t']
        if t > end + 1e-8:
            break
        packet = x['observer_packet']
        imu.append((t, packet['accel_heading_m_s2'][2]))
        while len(imu) > 2 and imu[1][0] < t - window - .05:
            imu.pop(0)
        value = packet['range_m']
        jump = value is not None and previous_range is not None and abs(value - previous_range) > .10
        previous_range = value
        surface = x['estimate']['surface_hypothesis']
        usable = surface in ('static_floor_candidate', 'moving_surface_candidate') and not jump
        if not usable:
            history.clear()
            continue
        if packet['range_new']:
            world_height = value + (.30 if surface == 'moving_surface_candidate' else 0.)
            history.append((packet['range_stamp_s'], world_height))
        while len(history) > 2 and t - history[0][0] > window:
            history.pop(0)
        if t >= terminal and packet['range_new'] and len(history) > 5:
            old, new = inertial_fit(history, imu, t)
            fits.append({'receipt_s': t, 'latest_range_stamp_s': packet['range_stamp_s'],
                         'range_samples': len(history), 'window_span_s': history[-1][0] - history[0][0],
                         'linear_window_slope_m_s': old, 'imu_aided_instantaneous_fit_m_s': new,
                         'logged_filtered_vertical_velocity_m_s': x['estimate']['velocity_world_m_s'][2]})
    # Truth is first read after all sensor-only candidate fits have finished.
    truth = [json.loads(x) for x in (run / 'truth.jsonl').open()]
    ts = [x['t'] for x in truth]
    vz = [x['aircraft']['velocity_world_m_s'][2] for x in truth]
    for x in fits:
        x['actual_vz_OFFLINE_m_s'] = float(np.interp(x['receipt_s'], ts, vz))
    rms = lambda field: float(np.sqrt(np.mean([(x[field] - x['actual_vz_OFFLINE_m_s']) ** 2 for x in fits])))
    return {'run': name, 'seed': json.loads((run / 'experiment.json').read_text())['seed'],
            'scope': 'Saved baseline surface classification and range window. Candidate fit only; no filtered-state replay or flight.',
            'all_terminal_new_range_fits': len(fits), 'window_s': window,
            'rmse_m_s': {f: rms(f) for f in ('linear_window_slope_m_s',
                  'imu_aided_instantaneous_fit_m_s', 'logged_filtered_vertical_velocity_m_s')},
            'last_fit': fits[-1], 'samples': fits,
            'source_sha256': {n: sha(run / n) for n in ('sensors.jsonl', 'truth.jsonl', 'experiment.json', 'adapter-summary.json')}}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', action='append', required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, args.out / Path(__file__).name)
    tests = checks()
    with ProcessPoolExecutor(max_workers=min(4, len(args.run)), mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(analyze, args.run))
    report = {'analytic_fixture_checks': tests, 'future_measurement_rejected': True,
              'analysis_sha256': sha(__file__), 'cases': results, 'live_connection': False,
              'limitations': ['IMU interval timing/tilt/range quality are S/U; unknown acceleration bias is uncorrected.',
                   'Fits use baseline surface classification and windows, not an alternative closed-loop trajectory.',
                   'Constant-velocity regression estimates a window slope; compensated fit aims at receipt-time velocity.',
                   'Original .08 velocity blend, .12 height blend and contact/prediction gates were not changed.',
                   'No landing success or reliable uncertainty improvement follows from this offline fixture.']}
    (args.out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps([{k: x[k] for k in ('seed', 'rmse_m_s', 'last_fit')} for x in results], indent=2))


if __name__ == '__main__':
    main()
