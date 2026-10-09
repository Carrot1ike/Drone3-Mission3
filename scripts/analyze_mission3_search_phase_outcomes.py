#!/usr/bin/env python3
"""All-case phase/time/uncertainty diagnostics from saved measurements only."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def load(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(series, output):
    plan, summary = load(series / 'plan.json'), load(series / 'summary.json')
    assert summary['all_cases_completed']
    results = []
    for case in plan['cases']:
        run = Path(case['run'])
        settings = load(run / 'controller-settings.json')
        raw = load(run / 'adapter-summary.json')
        events = raw['events']
        metric = next((e['time_s'] for e in events if e.get('to') == 'ACQUIRE'), None)
        handoff = next(e['t'] for e in events if e.get('phase') == 'HANDOFF_IN')
        first_id = None
        decoded, rejected_small = 0, 0
        for line in (run / 'observations.jsonl').open():
            obs = json.loads(line)
            if obs.get('corners') is not None:
                decoded += 1
                if first_id is None:
                    first_id = obs['capture_stamp_s']
                rejected_small += obs.get('reason') == 'marker_too_small'
        phases = {}
        at_metric, last = None, None
        for line in (run / 'commands.jsonl').open():
            command = json.loads(line)
            last = command
            if at_metric is None and metric is not None and command['t'] >= metric:
                at_metric = command
            own = command['own']
            phase = phases.setdefault(command['phase'], {
                'first_s': command['t'], 'last_s': command['t'], 'samples': 0,
                'min_position_std_m': own['position_std_m'],
                'max_position_std_m': own['position_std_m'],
                'qualified_floor_flow_samples': 0})
            phase['last_s'] = command['t']
            phase['samples'] += 1
            phase['min_position_std_m'] = min(phase['min_position_std_m'], own['position_std_m'])
            phase['max_position_std_m'] = max(phase['max_position_std_m'], own['position_std_m'])
            phase['qualified_floor_flow_samples'] += own.get('flow_fusion_usable', False) and own.get('flow_fusion_reason') == 'floor_patch_clear'
        results.append({
            'policy': case['policy'], 'seed': case['seed'], 'run': str(run),
            'failure': raw['failure'], 'duration_s': raw['duration_s'],
            'handoff_s': handoff, 'first_decoded_capture_s': first_id,
            'first_metric_acceptance_s': metric,
            'metric_elapsed_from_handoff_s': None if metric is None else metric - handoff,
            'after_metric_until_finish_s': None if metric is None else raw['duration_s'] - metric,
            'decoded_frames': decoded, 'marker_too_small_decoded_frames': rejected_small,
            'accepted_frames': raw['accepted_frames'],
            'position_std_at_first_metric_m': None if at_metric is None else at_metric['own']['position_std_m'],
            'last_position_std_m': last['own']['position_std_m'],
            'position_std_abort_limit_m': settings['control']['odometry_uncertainty_limit_m'],
            'search_timeout_s': settings['control'].get('search_timeout_s', 300.),
            'mission_timeout_s': settings['control']['mission_timeout_s'],
            'phases': phases, 'events': events,
            'raw_sha256': {name: sha(run / name) for name in (
                'commands.jsonl', 'observations.jsonl', 'adapter-summary.json', 'controller-settings.json')},
            'simulation_contact_status': load(run / 'offline-evaluation.json')['simulation_contact_success']['status']})
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, output / Path(__file__).name)
    result = {'scope': 'post-run full predeclared series; saved sensor/estimate commands only, no truth input',
              'series': str(series), 'attempts': len(results),
              'plan_sha256': sha(series / 'plan.json'), 'summary_sha256': sha(series / 'summary.json'),
              'analysis_sha256': sha(Path(__file__)), 'runs': results,
              'limits': 'uncertainty is an uncalibrated S model, not measured probability; no gate changes; no battery/SOC validation'}
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps([{k: row[k] for k in ('policy', 'seed', 'failure', 'simulation_contact_status',
                     'first_metric_acceptance_s', 'position_std_at_first_metric_m', 'last_position_std_m')} for row in results], indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('series', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    analyze(args.series, args.out)
