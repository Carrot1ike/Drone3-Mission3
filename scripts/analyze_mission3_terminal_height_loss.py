#!/usr/bin/env python3
"""Sensor-only vertical propagation crosscheck, followed by offline truth QA."""
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


def rows(p):
    return [json.loads(x) for x in Path(p).open()]


def analyze(name):
    run = Path(name)
    summary = json.loads((run / 'adapter-summary.json').read_text())
    commands = rows(run / 'commands.jsonl')
    start = next((x['t'] for x in commands if x['phase'] == 'TOUCHDOWN_DESCEND'), None)
    result = {'run': name, 'seed': json.loads((run / 'experiment.json').read_text())['seed'],
              'failure': summary['failure'], 'terminal_start_s': start}
    if start is None:
        return {**result, 'analysis': 'no terminal phase; original failure retained'}
    end = min([x['t'] for x in commands if x['t'] >= start and
               x['phase'] in ('MOTOR_OFF_HOLD', 'BLIND_RECOVER', 'BLIND_RISE', 'ABORT')])
    sensor = [x for x in rows(run / 'sensors.jsonl') if start <= x['t'] <= end + 1e-8]
    observations = [x for x in rows(run / 'observations.jsonl') if x.get('valid') and x['t'] <= end]
    last_visual = max(x['t'] for x in observations)
    fused = [i for i, x in enumerate(sensor) if x['deck'].get('range_accurate')
             and x['deck'].get('range_valid') and x['deck'].get('range_new')
             and x['estimate']['surface_hypothesis'] in ('static_floor_candidate', 'moving_surface_candidate')]
    if not fused:
        return {**result, 'analysis': 'no qualified range in selected terminal; unsupported propagation decomposition'}
    index = fused[-1]
    # Use the last held valid sample too; held observations do not fuse again.
    while index + 1 < len(sensor) and sensor[index + 1]['deck'].get('range_accurate'):
        index += 1
    base = sensor[index]
    assert last_visual <= base['t'] + 1e-8, 'later image corrections require an explicit replay'
    seq = sensor[index + 1:]
    assert all(x['estimate']['surface_hypothesis'] == 'ambiguous' for x in seq)
    z0 = base['estimate']['position_world_m'][2]
    v0 = base['estimate']['velocity_world_m_s'][2]
    z, v, time, acceleration_displacement = z0, v0, base['t'], 0.
    max_z_error = max_v_error = 0.
    checkpoints = []
    for x in seq:
        dt = x['t'] - time
        v += x['observer_packet']['accel_heading_m_s2'][2] * dt
        z += v * dt
        acceleration_displacement += (v - v0) * dt
        max_z_error = max(max_z_error, abs(z - x['estimate']['position_world_m'][2]))
        max_v_error = max(max_v_error, abs(v - x['estimate']['velocity_world_m_s'][2]))
        time = x['t']
        if not checkpoints or time - checkpoints[-1]['t_s'] >= .25 - 1e-8:
            checkpoints.append({'t_s': time, 'height_above_pad_m': z - .30,
                                'vz_m_s': v, 'initial_velocity_displacement_m': v0 * (time - base['t']),
                                'integrated_acceleration_displacement_m': acceleration_displacement})
    assert max_z_error < 1e-9 and max_v_error < 1e-9, (max_z_error, max_v_error)
    # Truth is first read only after sensor-only propagation has matched logs.
    truth = rows(run / 'truth.jsonl')
    ts = np.asarray([x['t'] for x in truth])
    at = lambda t, key: float(np.interp(t, ts, [x['aircraft'][key][2] for x in truth]))
    return {**result, 'analysis': 'exact sensor-only propagation on saved no-range/no-vision interval',
            'interval_start_s': base['t'], 'interval_end_s': time, 'terminal_end_s': end,
            'last_metric_receipt_s': last_visual, 'last_new_qualified_range_s': sensor[fused[-1]]['t'],
            'last_qualified_raw_range_m': sensor[fused[-1]]['deck']['range_m'],
            'initial_estimated_height_above_pad_m': z0 - .30, 'initial_estimated_vz_m_s': v0,
            'end_estimated_height_above_pad_m': z - .30, 'end_estimated_vz_m_s': v,
            'initial_velocity_displacement_m': v0 * (time - base['t']),
            'integrated_acceleration_displacement_m': acceleration_displacement,
            'max_logged_z_replay_error_m': max_z_error, 'max_logged_vz_replay_error_m_s': max_v_error,
            'checkpoints': checkpoints,
            'offline_truth_only': {'initial_height_above_pad_m': at(base['t'], 'center_world_m') - .30,
                'initial_vz_m_s': at(base['t'], 'velocity_world_m_s'),
                'end_height_above_pad_m': at(time, 'center_world_m') - .30,
                'end_vz_m_s': at(time, 'velocity_world_m_s')},
            'source_sha256': {n: sha(run / n) for n in ('adapter-summary.json', 'sensors.jsonl',
                   'commands.jsonl', 'observations.jsonl', 'truth.jsonl', 'experiment.json')}}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', action='append', required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, args.out / Path(__file__).name)
    with ProcessPoolExecutor(max_workers=min(4, len(args.run)),
                             mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(analyze, args.run))
    output = {'scope': 'Read-only sensor-only vertical interval propagation followed by offline truth QA. No live correction.',
              'analysis_sha256': sha(__file__), 'all_cases': results,
              'limitations': ['Qualified range flag is the existing synthetic contract, not real VL53L1X quality.',
                  'Initial velocity and subsequent measured acceleration contributions are arithmetic, not isolated causes.',
                  'No contact-based ZUPT, oracle bias, widened contact gate or prediction time was adopted.',
                  'Bias/noise, attitude, contact impulse and pre-loss visual/range fit require separate causal tests.']}
    (args.out / 'summary.json').write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps([{k: x.get(k) for k in ('seed', 'interval_start_s', 'interval_end_s',
                'initial_estimated_vz_m_s', 'end_estimated_vz_m_s', 'max_logged_z_replay_error_m')} for x in results]))


if __name__ == '__main__':
    main()
