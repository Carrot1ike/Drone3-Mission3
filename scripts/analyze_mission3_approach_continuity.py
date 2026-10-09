#!/usr/bin/env python3
"""Read-only, post-flight comparison; never imported by a live controller."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def analyze(series, out):
    plan = load(series / 'plan.json')
    whole = load(series / 'summary.json')
    assert whole['all_cases_completed']
    out.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})
    seeds = sorted({c['seed'] for c in plan['cases']})
    fig, axes = plt.subplots(len(seeds), 3, figsize=(15, 4.5 * len(seeds)),
                             squeeze=False, layout='constrained')
    inputs = {str(series / name): sha(series / name) for name in ('plan.json', 'summary.json')}
    records = []
    for case in plan['cases']:
        run = Path(case['run'])
        for name in ('truth.jsonl', 'commands.jsonl', 'motors.jsonl', 'observations.jsonl',
                     'sensors.jsonl', 'deck_samples.jsonl', 'estimates.jsonl',
                     'rpyt_packets.jsonl', 'camera.avi', 'camera_frames.jsonl',
                     'run_manifest.json', 'offline-evaluation.json', 'adapter-summary.json',
                     'command-limits-audit.json', 'camera-serialization-validation.json'):
            inputs[str(run / name)] = sha(run / name)
        cmds, truth = rows(run / 'commands.jsonl'), rows(run / 'truth.jsonl')
        ev = load(run / 'offline-evaluation.json')
        caps = load(run / 'command-limits-audit.json')
        start = next((c['t'] for c in cmds if c['phase'] == 'LOWER_ALIGN'), None)
        end = ev['checks']['touchdown'].get('first_touchdown_time_s')
        result = {k: case[k] for k in ('policy', 'seed', 'run', 'split')}
        result.update(contact_pass=ev['simulation_contact_success']['status'] == 'pass',
                      command_envelope_pass=caps['command_envelope_pass'],
                      first_contact_error_m=ev.get('score', {}).get('distance_m'),
                      lower_align_start_s=start, first_contact_s=end,
                      actual_vertical_stop_s=None, measured_interval_s=None)
        if start is not None and end is not None and end > start:
            t = np.array([r['t'] for r in truth])
            p = np.array([r['aircraft']['center_world_m'] for r in truth])
            pad = np.array([r['pad']['center_world_m'] for r in truth])
            vz = np.array([r['aircraft']['velocity_world_m_s'][2] for r in truth])
            dt = np.maximum(0., np.minimum(np.r_[t[1:], t[-1]], end) - np.maximum(t, start))
            valid = dt > 0
            stop_s = float(dt[np.abs(vz) < .005].sum())
            result.update(actual_vertical_stop_s=stop_s,
                          measured_interval_s=float(dt.sum()),
                          actual_height_drop_m=float(p[valid][0, 2] - p[valid][-1, 2]),
                          actual_min_vertical_speed_m_s=float(vz[valid].min()))
            phase_times = {}
            for a, b in zip(cmds[:-1], cmds[1:]):
                overlap = max(0., min(b['t'], end) - max(a['t'], start))
                if overlap:
                    phase_times[a['phase']] = phase_times.get(a['phase'], 0.) + overlap
            result['phase_duration_s'] = phase_times
            i = seeds.index(case['seed'])
            label = '기존 단계 방식' if case['policy'] == 'staged' else '동시 접근 후보'
            color = '#2869b3' if case['policy'] == 'staged' else '#db6335'
            # Same interval definition; no resampling of live camera/controller inputs.
            axes[i, 0].plot(t[valid] - start, p[valid, 2], color=color,
                            label=f'{label} (정지 {stop_s:.2f}s)')
            axes[i, 1].plot(t[valid] - start, vz[valid], color=color, label=label)
            headings = np.array([r['vehicle']['heading_rad'] for r in truth])
            behind = -(p[:, 0] - pad[:, 0]) * np.cos(headings) - (p[:, 1] - pad[:, 1]) * np.sin(headings)
            axes[i, 2].plot(behind[valid], p[valid, 2], color=color,
                            label=f'{label} (착지 {result["first_contact_error_m"] * 100:.2f}cm)')
        records.append(result)
    pairs = []
    for seed in seeds:
        base = next(r for r in records if r['seed'] == seed and r['policy'] == 'staged')
        candidate = next(r for r in records if r['seed'] == seed and r['policy'] == 'coordinated')
        reduction = (1. - candidate['actual_vertical_stop_s'] / base['actual_vertical_stop_s']
                     if base['actual_vertical_stop_s'] and candidate['actual_vertical_stop_s'] is not None else None)
        increment = (candidate['first_contact_error_m'] - base['first_contact_error_m']
                     if candidate['first_contact_error_m'] is not None and base['first_contact_error_m'] is not None else None)
        pairs.append({'seed': seed, 'vertical_stop_relative_reduction': reduction,
                      'first_contact_error_increment_m': increment,
                      'stop_reduction_pass': reduction is not None and reduction >= .2,
                      'error_increment_pass': increment is not None and increment <= .01})
    for i, seed in enumerate(seeds):
        axes[i, 0].set(title=f'seed {seed}: 실제 높이', xlabel='LOWER_ALIGN 시작 이후 (s)', ylabel='바닥 기준 높이 (m)')
        axes[i, 1].set(title=f'seed {seed}: 실제 수직 속도', xlabel='LOWER_ALIGN 시작 이후 (s)', ylabel='수직 속도 (m/s)')
        axes[i, 1].axhspan(-.005, .005, color='gray', alpha=.15)
        axes[i, 2].set(title=f'seed {seed}: 차량 기준 접근 경로', xlabel='패드 중심 뒤쪽 거리 (m)', ylabel='바닥 기준 높이 (m)')
        axes[i, 2].invert_xaxis()
        for j in (0, 2):
            axes[i, j].axhline(.30, ls=':', color='gray', label='패드 상면 .30m')
        for ax in axes[i]:
            ax.grid(alpha=.2)
            ax.legend(fontsize=9)
    fig.suptitle('미션3 접근 연속성 | 원 물리 로그, 재사용 training 2개 조건\n'
                 '조건부 모델 접촉 · 제조사 일치/실기 성공 0 · 정지는 |실제 수직속도| < .005m/s', fontsize=14)
    fig.savefig(out / 'approach-continuity.png', dpi=150)
    plt.close(fig)
    assert all(sha(p) == h for p, h in inputs.items()), 'input changed during analysis'
    accepted = (all(r['contact_pass'] and r['command_envelope_pass'] for r in records)
                and all(p['stop_reduction_pass'] and p['error_increment_pass'] for p in pairs))
    summary = {'scope': 'same spent training conditions; no population/holdout/real hardware claim',
               'measurement': 'left-sample time integral clipped to [first LOWER_ALIGN, first actual contact); abs(actual vz)<.005m/s',
               'records': records, 'pairs': pairs, 'predeclared_all_checks_pass': accepted,
               'decision': 'eligible for independent development review' if accepted else 'candidate rejected under predeclared gates; preserve baseline',
               'qualified_successes': 0, 'inputs_sha256': inputs,
               'analyzer_sha256': sha(__file__), 'figure_sha256': sha(out / 'approach-continuity.png')}
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    shutil.copy2(__file__, out / Path(__file__).name)
    print(json.dumps({'pairs': pairs, 'predeclared_all_checks_pass': accepted,
                      'decision': summary['decision']}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('series', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    analyze(args.series, args.out)
