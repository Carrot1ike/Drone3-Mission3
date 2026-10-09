#!/usr/bin/env python3
"""Predeclared 0.4/0.8/1.2-second sensitivity on saved vertical inputs."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import shutil

import numpy as np

from benchmark_mission3_inertial_range_velocity import checks, inertial_fit

POLICIES = [('linear', .4), ('inertial', .4), ('linear', .8),
            ('inertial', .8), ('linear', 1.2), ('inertial', 1.2)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def replay(sensor, start, end, pad_height, model, window):
    history, imu, previous_range, state, result = [], [], None, None, []
    max_z = max_v = 0.
    last_qualified = None
    for x in sensor:
        t, packet, saved = x['t'], x['observer_packet'], x['estimate']
        imu.append((t, packet['accel_heading_m_s2'][2]))
        while len(imu) > 2 and imu[1][0] < t - window - .05:
            imu.pop(0)
        value = packet['range_m']
        jump = value is not None and previous_range is not None and abs(value - previous_range) > .10
        previous_range = value
        surface = saved['surface_hypothesis']
        usable = surface in ('static_floor_candidate', 'moving_surface_candidate') and not jump
        if usable:
            height = value + (pad_height if surface == 'moving_surface_candidate' else 0.)
            if packet['range_new']:
                history.append((packet['range_stamp_s'], height))
                last_qualified = t
            while len(history) > 2 and t - history[0][0] > window:
                history.pop(0)
        else:
            history.clear()
        if t < start - 1e-8:
            continue
        if state is None:
            assert abs(t - start) < 1e-8
            state = [saved['position_world_m'][2], saved['velocity_world_m_s'][2]]
            previous_time = t
        else:
            dt = t - previous_time
            state[1] += packet['accel_heading_m_s2'][2] * dt
            if usable and packet['range_new']:
                state[0] += .12 * (height - state[0])
                if len(history) > 5:
                    fit = inertial_fit(history, imu, t)[model == 'inertial']
                    state[1] += .08 * (fit - state[1])
            state[0] += state[1] * dt
            previous_time = t
        max_z = max(max_z, abs(state[0] - saved['position_world_m'][2]))
        max_v = max(max_v, abs(state[1] - saved['velocity_world_m_s'][2]))
        result.append({'t_s': t, 'z_m': state[0], 'vz_m_s': state[1]})
    return {'model': model, 'window_s': window, 'max_logged_z_difference_m': max_z,
            'max_logged_vz_difference_m_s': max_v, 'last_qualified_s': last_qualified,
            'samples': result}


def analyze(name):
    run = Path(name)
    experiment = json.loads((run / 'experiment.json').read_text())
    command = rows(run / 'commands.jsonl')
    start = next(x['t'] for x in command if x['phase'] == 'TOUCHDOWN_DESCEND')
    end = next(x['t'] for x in command if x['t'] >= start and
               x['phase'] in ('MOTOR_OFF_HOLD', 'BLIND_RECOVER', 'BLIND_RISE', 'ABORT'))
    assert not any(x.get('valid') and start < x['t'] <= end
                   for x in rows(run / 'observations.jsonl'))
    sensor = [x for x in rows(run / 'sensors.jsonl') if x['t'] <= end + 1e-8]
    pad_height = experiment['mission']['landing_geometry']['pad_height_m']
    candidates = [replay(sensor, start, end, pad_height, model, window) for model, window in POLICIES]
    baseline = candidates[0]
    assert baseline['max_logged_z_difference_m'] < 1e-9
    assert baseline['max_logged_vz_difference_m_s'] < 1e-9
    # No actual state has been read during fitting or policy construction.
    truth = rows(run / 'truth.jsonl')
    times = np.array([x['t'] for x in truth])
    z = np.array([x['aircraft']['center_world_m'][2] for x in truth])
    vz = np.array([x['aircraft']['velocity_world_m_s'][2] for x in truth])
    for candidate in candidates:
        values = candidate.pop('samples')
        t = np.array([x['t_s'] for x in values])
        actual = [np.interp(t, times, z), np.interp(t, times, vz)]
        loss = t > candidate['last_qualified_s']
        candidate['terminal_rmse'] = {}
        candidate['post_loss_rmse'] = {}
        for axis, real in zip(('z_m', 'vz_m_s'), actual):
            error = np.array([x[axis] for x in values]) - real
            candidate['terminal_rmse'][axis] = float(np.sqrt(np.mean(error ** 2)))
            candidate['post_loss_rmse'][axis] = float(np.sqrt(np.mean(error[loss] ** 2)))
        candidate['end'] = {**values[-1], 'actual_z_OFFLINE_m': actual[0][-1],
                            'actual_vz_OFFLINE_m_s': actual[1][-1]}
    return {'run': name, 'seed': experiment['seed'], 'start_s': start, 'end_s': end,
            'baseline_replayed_exactly': True, 'policies': candidates,
            'source_sha256': {n: sha(run / n) for n in ('experiment.json', 'sensors.jsonl',
                 'commands.jsonl', 'observations.jsonl', 'truth.jsonl')}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='append', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    helpers = [Path(__file__), Path(__file__).with_name('benchmark_mission3_inertial_range_velocity.py')]
    for helper in helpers:
        shutil.copyfile(helper, args.out / helper.name)
    plan = {'runs': args.run, 'policies_declared_before_processing': POLICIES,
            'helper_sha256': {p.name: sha(p) for p in helpers}, 'analytic_checks': checks(),
            'scope': 'Spent-development saved-input window sensitivity; no closed-loop flight or success.',
            'limits': ['Seeded logged terminal state and baseline surface classification retained.',
                'All six candidates keep .12 height and .08 velocity gains; no gate, deadline or bias oracle changes.',
                'Longer windows trade noise reduction against uncorrected IMU bias and surface/timing errors.',
                'No calibrated covariance or manufacturer qualification follows.']}
    (args.out / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    with ProcessPoolExecutor(max_workers=min(4, len(args.run)),
                             mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(analyze, args.run))
    assert all(sha(p) == plan['helper_sha256'][p.name] for p in helpers)
    (args.out / 'summary.json').write_text(json.dumps({**plan, 'cases': results}, indent=2) + '\n')
    print(json.dumps([{k: x[k] for k in ('seed', 'policies')} for x in results], indent=2))


if __name__ == '__main__':
    main()
