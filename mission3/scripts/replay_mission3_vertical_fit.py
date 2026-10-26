#!/usr/bin/env python3
"""Saved-input vertical filter comparison, never a closed-loop flight verdict."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import shutil

import numpy as np

from benchmark_mission3_inertial_range_velocity import checks, inertial_fit


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def analyze(name):
    run = Path(name)
    experiment = json.loads((run / 'experiment.json').read_text())
    commands = rows(run / 'commands.jsonl')
    start = next(x['t'] for x in commands if x['phase'] == 'TOUCHDOWN_DESCEND')
    end = next(x['t'] for x in commands if x['t'] >= start and
               x['phase'] in ('MOTOR_OFF_HOLD', 'BLIND_RECOVER', 'BLIND_RISE', 'ABORT'))
    observation = rows(run / 'observations.jsonl')
    assert not any(x.get('valid') and start < x['t'] <= end for x in observation), (
        'Post-seed visual corrections require a broader replay', name)
    pad_height = experiment['mission']['landing_geometry']['pad_height_m']
    window = experiment['mission']['observer'].get('vertical_range_velocity_window_s', .4)
    sensor = [x for x in rows(run / 'sensors.jsonl') if x['t'] <= end + 1e-8]
    history, imu, previous_range = [], [], None
    states, traces, last_qualified = None, [], None
    max_z = max_v = 0.
    for x in sensor:
        t = x['t']
        packet, saved = x['observer_packet'], x['estimate']
        imu.append((t, packet['accel_heading_m_s2'][2]))
        while len(imu) > 2 and imu[1][0] < t - window - .05:
            imu.pop(0)
        value = packet['range_m']
        jump = (value is not None and previous_range is not None
                and abs(value - previous_range) > .10)
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
        if states is None:
            assert abs(t - start) < 1e-8, (t, start)
            states = [[saved['position_world_m'][2], saved['velocity_world_m_s'][2]]
                      for _ in range(2)]
            previous_time = t
        else:
            dt = t - previous_time
            assert 0 < dt <= .2
            fits = None
            if usable and packet['range_new'] and len(history) > 5:
                fits = inertial_fit(history, imu, t)
            for index, state in enumerate(states):
                state[1] += packet['accel_heading_m_s2'][2] * dt
                if usable and packet['range_new']:
                    state[0] += .12 * (height - state[0])
                if fits is not None:
                    state[1] += .08 * (fits[index] - state[1])
                state[0] += state[1] * dt
            previous_time = t
        max_z = max(max_z, abs(states[0][0] - saved['position_world_m'][2]))
        max_v = max(max_v, abs(states[0][1] - saved['velocity_world_m_s'][2]))
        traces.append({'t_s': t, 'baseline_surface': surface,
                       'range_fused': bool(usable and packet['range_new']),
                       'legacy_z_m': states[0][0], 'legacy_vz_m_s': states[0][1],
                       'candidate_z_m': states[1][0], 'candidate_vz_m_s': states[1][1]})
    assert max_z < 1e-9 and max_v < 1e-9, (name, max_z, max_v)
    # Read truth only after the legacy sensor-only filter has matched logs.
    truth = rows(run / 'truth.jsonl')
    times = [x['t'] for x in truth]
    for x in traces:
        x['actual_z_OFFLINE_m'] = float(np.interp(x['t_s'], times,
                  [r['aircraft']['center_world_m'][2] for r in truth]))
        x['actual_vz_OFFLINE_m_s'] = float(np.interp(x['t_s'], times,
                  [r['aircraft']['velocity_world_m_s'][2] for r in truth]))
    def errors(group):
        return {policy: {axis: float(np.sqrt(np.mean([
            (x[f'{policy}_{axis}'] - x[f'actual_{axis.replace("_m", "_OFFLINE_m", 1)}']) ** 2
            for x in group]))) for axis in ('z_m', 'vz_m_s')}
            for policy in ('legacy', 'candidate')}
    propagation = [x for x in traces if x['t_s'] > last_qualified]
    return {'run': name, 'seed': experiment['seed'], 'terminal_start_s': start,
            'end_s': end, 'last_new_qualified_range_s': last_qualified,
            'legacy_crosscheck_max_z_m': max_z, 'legacy_crosscheck_max_vz_m_s': max_v,
            'terminal_rmse': errors(traces), 'post_range_loss_rmse': errors(propagation),
            'last_state': traces[-1], 'samples': traces,
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
    plan = {'runs': args.run, 'helper_sha256': {p.name: sha(p) for p in helpers},
            'analytic_fixture_checks': checks(),
            'scope': 'Sensor-only sequential vertical filter replay with saved baseline surface decisions. Truth is offline QA only.',
            'limitations': ['Original .12 height/.08 velocity gains, range window, timestamps and measurements retained.',
                'Both filters begin from the same logged terminal state; pre-terminal feedback is excluded.',
                'Baseline classification and plant trajectory are frozen, not an alternative closed-loop flight.',
                'IMU timing, tilt, bias, near-range quality and contact dynamics remain S/U.',
                'A replay cannot establish motor-off, 3-second hold or landing success.']}
    (args.out / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    with ProcessPoolExecutor(max_workers=min(4, len(args.run)),
                             mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(analyze, args.run))
    assert all(sha(p) == plan['helper_sha256'][p.name] for p in helpers)
    (args.out / 'summary.json').write_text(json.dumps({**plan, 'cases': results}, indent=2) + '\n')
    print(json.dumps([{k: x[k] for k in ('seed', 'terminal_rmse', 'post_range_loss_rmse',
                                        'last_state')} for x in results], indent=2))


if __name__ == '__main__':
    main()
