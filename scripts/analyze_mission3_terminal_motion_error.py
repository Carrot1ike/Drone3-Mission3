#!/usr/bin/env python3
"""Offline error sensitivity, not a counterfactual flight or a live input."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def analyze(out):
    out.mkdir(parents=True, exist_ok=False)
    inputs, records = {}, []
    for version, policies in [('v21', ('staged', 'coordinated')), ('v22', ('coordinated',))]:
        for policy in policies:
            for seed in (571, 619):
                run = Path(f'artifacts/mission3/physical/20261009-{policy}-approach-{version}-entry1-seed{seed}')
                for name in ('truth.jsonl', 'commands.jsonl', 'offline-evaluation.json'):
                    inputs[str(run / name)] = sha(run / name)
                truth, cmds = rows(run / 'truth.jsonl'), rows(run / 'commands.jsonl')
                times = np.array([x['t'] for x in truth])
                evaluation = json.loads((run / 'offline-evaluation.json').read_text())
                contact = evaluation['checks']['touchdown'].get('first_touchdown_time_s')
                samples = []
                for phase in ('LOWER_ALIGN', 'TRACK', 'BLIND_APPROACH', 'TOUCHDOWN_DESCEND'):
                    command = next((x for x in cmds if x['phase'] == phase), None)
                    if command is None:
                        continue
                    actual = truth[int(np.argmin(np.abs(times - command['t'])))]
                    assert abs(actual['t'] - command['t']) < 1e-8
                    relative = np.array(actual['pad']['center_world_m']) - np.array(actual['aircraft']['center_world_m'])
                    estimated = np.array(command['vehicle']['relative_pad_world_m'])
                    error = (np.array(command['vehicle']['vehicle_velocity_world_m_s'])
                             - np.array(actual['vehicle']['velocity_world_m_s']))
                    remaining = contact - command['t']
                    samples.append({'phase': phase, 'time_s': command['t'],
                                    'camera_age_s': command['vehicle']['age_s'],
                                    'velocity_source': command['vehicle'].get('velocity_source'),
                                    'actual_relative_pad_xy_m': relative[:2].tolist(),
                                    'estimated_relative_pad_xy_m': estimated[:2].tolist(),
                                    'relative_pose_error_xy_m': (estimated - relative)[:2].tolist(),
                                    'estimated_velocity_xy_m_s': command['vehicle']['vehicle_velocity_world_m_s'][:2],
                                    'velocity_error_xy_m_s': error[:2].tolist(),
                                    'seconds_to_first_contact': remaining,
                                    'constant_current_velocity_error_times_remaining_xy_m': (error[:2] * remaining).tolist(),
                                    'interpretation': 'D local constant-error sensitivity; not causal decomposition or live truth input'})
                actual = truth[int(np.argmin(np.abs(times - contact)))]
                error = np.array(actual['aircraft']['center_world_m']) - np.array(actual['pad']['center_world_m'])
                records.append({'run': str(run), 'version': version, 'policy': policy, 'seed': seed,
                                'first_contact_xy_error_vector_m': error[:2].tolist(),
                                'score_distance_m': evaluation['score']['distance_m'], 'samples': samples})
    assert all(sha(path) == digest for path, digest in inputs.items())
    result = {'scope': 'offline6 completed spent training paths; equal logged times, capture/readout/covariance S/U',
              'observed_live_velocity_method': 'default world_position_regression; relative_kinematics option not used',
              'inputs_sha256': inputs, 'inputs_unchanged': True, 'records': records,
              'qualified_successes': 0, 'analyzer_sha256': sha(__file__),
              'limit': 'Bias sensitivity and pose error do not isolate feedback/contact response or show hypothetical policy efficacy. No future truth/data supplied to control.'}
    (out / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    shutil.copy2(__file__, out / Path(__file__).name)
    print(json.dumps({'runs': len(records), 'input_files_unchanged': len(inputs), 'qualified_successes': 0}))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True, type=Path)
    analyze(p.parse_args().out)
