#!/usr/bin/env python3
"""Saved-input command counterfactual only; no simulated efficacy claim."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from mission3.controller import Mission3


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(run):
    cfg = json.loads((run / 'controller-settings.json').read_text())
    manifest = json.loads((run / 'run_manifest.json').read_text())
    entry = next(x for x in cfg['entry']['positions'] if x['id'] == 1)
    alternative = deepcopy(cfg)
    alternative['control'].update(coarse_translate_turn_error_deg=40.,
                                  coarse_turn_translation_speed_m_s=.35,
                                  coarse_translation_axis='fresh_decoded_bearing')
    baseline = Mission3(cfg, entry, manifest['handoff_time_s'])
    candidate = Mission3(alternative, entry, manifest['handoff_time_s'])
    samples = []
    phase_errors = []
    for line in (run / 'commands.jsonl').open():
        stored = json.loads(line)
        if stored['t'] < manifest['handoff_time_s'] - 1e-8:
            continue
        b = baseline.update(stored['t'], deepcopy(stored['own']), deepcopy(stored['vehicle']))
        c = candidate.update(stored['t'], deepcopy(stored['own']), deepcopy(stored['vehicle']))
        if b.phase != stored['phase']:
            phase_errors.append(stored['t'])
        if b.coarse_guard is not None:
            samples.append({'t': stored['t'], 'baseline_guard': b.coarse_guard,
                            'candidate_guard': c.coarse_guard,
                            'baseline_velocity_m_s': b.velocity_world_m_s,
                            'candidate_velocity_m_s': c.velocity_world_m_s,
                            'baseline_logged_velocity_error_m_s': float(np.linalg.norm(
                                np.asarray(b.velocity_world_m_s) - stored['velocity_world_m_s']))})
    assert not phase_errors
    same = max((x['baseline_logged_velocity_error_m_s'] for x in samples), default=0.)
    assert same < 1e-9
    return {'run': str(run), 'coarse_samples': len(samples),
            'same_input_baseline_phase_errors': phase_errors,
            'coarse_baseline_velocity_replay_max_error_m_s': same,
            'previously_stopped_candidate_translate_samples': int(sum(
                x['baseline_guard']['desired_forward_m_s'] == 0.
                and x['candidate_guard']['desired_forward_m_s'] > 0. for x in samples)),
            'fresh_decoded_bearing_direction_samples': int(sum(
                x['candidate_guard'].get('fresh_decoded_translation_used', False) for x in samples)),
            'raw_sha256': {name: sha(run / name) for name in ('commands.jsonl',
                            'controller-settings.json', 'run_manifest.json')}, 'samples': samples}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='append', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    results = [analyze(run) for run in args.run]
    args.out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, args.out / Path(__file__).name)
    report = {'scope': 'saved own/vehicle image estimates only, no truth; counterfactual does not change saved trajectory or images',
              'declared_candidate': {'turn_error_deg': 40., 'turn_speed_m_s': .35,
                                     'translation_axis': 'fresh_decoded_bearing'},
              'source_sha256': sha(Path(__file__)),
              'controller_sha256': sha(Path('scripts/mission3/controller.py')),
              'runs': results, 'limits': 'Not a new flight, pixel detection, metric pose or landing. '
              'Freshness/path/pixel proximity/20s/3D .4/accel .15 gates stay in existing implementation.'}
    (args.out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps([{k: v for k, v in row.items() if k not in ('samples', 'raw_sha256')}
                      for row in results], indent=2))
