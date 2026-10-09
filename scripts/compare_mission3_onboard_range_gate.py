#!/usr/bin/env python3
"""Read-only paired log comparison, not a saved-sensor EKF replay."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import shutil

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def compare(case):
    old, new = Path(case['baseline_run']), Path(case['run'])
    raw = {}
    for name in ('commands.jsonl', 'truth.jsonl', 'observations.jsonl', 'motors.jsonl',
                 'rpyt_packets.jsonl', 'deck_samples.jsonl', 'estimates.jsonl'):
        first = None
        with (old / name).open() as a, (new / name).open() as b:
            index = 0
            while True:
                left, right = a.readline(), b.readline()
                if not left and not right:
                    break
                if left != right:
                    first = {'row_index': index,
                             'baseline_t': json.loads(left).get('t') if left else None,
                             'candidate_t': json.loads(right).get('t') if right else None,
                             'row_missing': not left or not right}
                    break
                index += 1
        raw[name] = {'baseline_sha256': sha(old / name), 'candidate_sha256': sha(new / name),
                     'byte_identical': first is None, 'first_changed_row': first}
    a, b = rows(old / 'estimates.jsonl'), rows(new / 'estimates.jsonl')
    aa = {round(x['t'], 8): x for x in a}
    pairs = [(aa[round(x['t'], 8)], x) for x in b if round(x['t'], 8) in aa]
    stats = {}
    for context in ('firmware_parallel_attitude', 'pc_independent_attitude'):
        left = np.asarray([x[context]['quaternion_wxyz'] for x, y in pairs])
        right = np.asarray([y[context]['quaternion_wxyz'] for x, y in pairs])
        # q and -q are the same attitude; chord distance is stable near zero.
        right *= np.where(np.sum(left * right, axis=1) < 0, -1., 1.)[:, None]
        chord = np.linalg.norm(left / np.linalg.norm(left, axis=1)[:, None]
                               - right / np.linalg.norm(right, axis=1)[:, None], axis=1)
        angles = np.rad2deg(4 * np.arcsin(np.clip(chord / 2, 0, 1)))
        z = np.asarray([y[context]['position_world_m'][2] - x[context]['position_world_m'][2] for x, y in pairs])
        cov = np.asarray([np.asarray(y[context]['covariance_diagonal'])
                          - np.asarray(x[context]['covariance_diagonal']) for x, y in pairs])
        stats[context] = {'max_attitude_difference_deg': float(angles.max()),
                          'max_abs_vertical_position_difference_m': float(np.abs(z).max()),
                          'max_abs_covariance_diagonal_difference': np.abs(cov).max(axis=0).tolist(),
                          'quaternion_lists_exactly_equal': all(x[context]['quaternion_wxyz']
                                                               == y[context]['quaternion_wxyz'] for x, y in pairs)}
    position = np.asarray([np.asarray(y['mission_estimate']['position_world_m'])
                           - np.asarray(x['mission_estimate']['position_world_m']) for x, y in pairs])
    stats['mission_estimate'] = {'max_position_difference_m': float(np.linalg.norm(position, axis=1).max()),
                                 'all_estimates_exactly_equal': all(x['mission_estimate'] == y['mission_estimate'] for x, y in pairs)}
    summaries = [json.loads((run / 'adapter-summary.json').read_text()) for run in (old, new)]
    evaluations = [json.loads((run / 'offline-evaluation.json').read_text()) for run in (old, new)]
    low = [x for x in rows(new / 'deck_samples.jsonl') if x['kind'] == 'tof_40hz'
           and x.get('range_m') is not None and 0 <= x['range_m'] < .04]
    return {'seed': case['seed'], 'baseline_run': str(old), 'candidate_run': str(new),
            'matched_saved_estimate_times': len(pairs), 'baseline_estimates': len(a), 'candidate_estimates': len(b),
            'raw_files': raw, 'saved_state_difference': stats,
            'first_candidate_raw_below_004_s': low[0]['t'] if low else None,
            'producer_raw_below_004_count': len(low),
            'candidate_measurement_counts': summaries[1]['onboard_estimator_measurement_counts'],
            'baseline_duration_s': summaries[0]['duration_s'], 'candidate_duration_s': summaries[1]['duration_s'],
            'baseline_failure': summaries[0]['failure'], 'candidate_failure': summaries[1]['failure'],
            'baseline_contact_status': evaluations[0]['simulation_contact_success']['status'],
            'candidate_contact_status': evaluations[1]['simulation_contact_success']['status'],
            'evaluation_sha256': [sha(run / 'offline-evaluation.json') for run in (old, new)]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('series', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads((args.series / 'plan.json').read_text())
    args.out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, args.out / Path(__file__).name)
    with ProcessPoolExecutor(max_workers=min(4, len(plan['cases'])),
                             mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(compare, plan['cases']))
    output = {'scope': 'Offline comparison of completed saved logs at shared timestamps; no sensor replay or live control modification.',
              'plan_sha256': sha(args.series / 'plan.json'), 'analysis_sha256': sha(__file__),
              'max_workers': 4, 'all_preplanned_cases': results,
              'limitations': ['Raw lower-range guard is S; RangeStatus and real surface quality remain unresolved.',
                              'Changing onboard position does not imply changing the PC IMU-only attitude context.',
                              'Three byte-identical plant/control retained flights are not three new landing conditions.',
                              'No causal performance gain or hardware qualification follows from this log comparison.']}
    (args.out / 'summary.json').write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps([{k: x[k] for k in ('seed', 'saved_state_difference', 'candidate_failure',
                       'producer_raw_below_004_count')} for x in results], indent=2))


if __name__ == '__main__':
    main()
