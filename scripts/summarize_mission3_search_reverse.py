#!/usr/bin/env python3
"""Offline aggregation only; keep failures, immutable evaluations and provenance."""
import argparse
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def load(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def summarize(base):
    plan = load(base / 'plan.json')
    changed = [p for p, h in plan['core_hashes'].items() if sha(p) != h]
    assert not changed, changed
    assert all(sha(p) == h for p, h in plan['config_file_hashes'].items())
    assert sha(plan['cases'][0]['profile']) == plan['profile_file_hash']
    reference = None
    results = []
    for case in plan['cases']:
        run = Path(case['run'])
        manifest = load(run / 'run_manifest.json')
        summary = load(run / 'adapter-summary.json')
        evaluation = load(run / 'offline-evaluation.json')
        audit = load(run / 'command-limits-audit.json')
        video = load(run / 'camera-serialization-validation.json')
        assert manifest['completed'] and video['exact_pixel_replay']
        assert sha(run / manifest['config_identity_file']) == manifest['config_sha256']
        for group in ('code_sha256', 'runtime_library_sha256'):
            assert all(sha(run / 'source-snapshot' / p) == h == sha(p)
                       for p, h in manifest[group].items())
        if reference is None:
            reference = manifest['code_sha256']
        assert reference == manifest['code_sha256']
        commands = [json.loads(line) for line in (run / 'commands.jsonl').open()]
        observations = [json.loads(line) for line in (run / 'observations.jsonl').open()]
        item = {key: case[key] for key in ('policy', 'seed', 'entry', 'split', 'config', 'profile', 'run', 'log')}
        item.update(duration_s=summary['duration_s'], failure=summary['failure'],
                    decoded_frames=sum(x.get('corners') is not None for x in observations),
                    accepted_frames=summary['accepted_frames'], contact_samples=summary['truth_contact_samples'],
                    motor_off_at_s=summary['motor_off_at_s'],
                    simulation_contact_status=evaluation['simulation_contact_success']['status'],
                    score=evaluation.get('score'), events=summary['events'],
                    command_envelope_pass=audit['command_envelope_pass'],
                    max_command_accel_normal_m_s2=audit['max_command_accel_normal_m_s2'],
                    max_actual_horizontal_speed_m_s=audit['max_actual_horizontal_speed_m_s'],
                    final_position_std_m=summary['final_estimate']['position_std_m'],
                    frames=video['frames'], recording_exact=video['exact_pixel_replay'],
                    evidence_sha256={n: sha(run / n) for n in (
                        'run_manifest.json', 'adapter-summary.json', 'offline-evaluation.json',
                        'command-limits-audit.json', 'camera-serialization-validation.json')})
        results.append(item)
        if item['simulation_contact_status'] != 'pass':
            continue
        truth = [json.loads(line) for line in (run / 'truth.jsonl').open()]
        off = summary['motor_off_at_s']
        end = evaluation['checks']['hold_support']['hold_end_s']
        hold = [x for x in truth if off <= x['t'] <= end + 1e-8]
        hold_commands = [x for x in commands if off <= x['t'] <= end + 1e-8]
        surfaces = []
        previous = None
        for x in commands:
            hypothesis = x['own']['surface_hypothesis']
            if hypothesis != previous:
                surfaces.append({'t': x['t'], 'hypothesis': hypothesis})
                previous = hypothesis
        review_path = run / 'development-review-unsigned.json'
        if review_path.exists():
            existing = load(review_path)
            assert existing['evaluation_source_sha256'] == sha(run / 'offline-evaluation.json')
            assert existing['independent_signature'] is None
            continue
        review = {
            'review_type': 'main-agent engineering critique, UNSIGNED, NOT independent review',
            'author_role': 'same implementation agent', 'independent_signature': None,
            'independent_qualified_count': 0, 'manufacturer_qualified_count': 0,
            'evaluation_source_sha256': sha(run / 'offline-evaluation.json'),
            'raw_log_sha256': {n: sha(run / n) for n in ('truth.jsonl', 'commands.jsonl', 'motors.jsonl',
                               'observations.jsonl', 'deck_samples.jsonl', 'estimates.jsonl')},
            'evaluated_checks': evaluation['checks'], 'score': evaluation['score'],
            'command_envelope_pass': audit['command_envelope_pass'],
            'max_command_accel_normal_m_s2': audit['max_command_accel_normal_m_s2'],
            'hold_samples': len(hold),
            'hold_min_supporting_feet': min(sum(q['other'] == 'pad' and q['normal_force_N'] > 0
                                               for q in x['contacts']) for x in hold),
            'hold_max_relative_speed_m_s': max(norm([v - w for v, w in zip(
                x['aircraft']['velocity_world_m_s'], x['vehicle']['velocity_world_m_s'])]) for x in hold),
            'hold_max_motor_actual_force_N': max(sum(x['motor_actual_force_N']) for x in hold),
            'hold_all_straight': all(not x['vehicle']['in_turn'] for x in hold),
            'hold_accurate_range_count': sum(x['own']['range_accurate'] for x in hold_commands),
            'hold_flow_valid_count': sum(x['own']['flow_valid'] for x in hold_commands),
            'hold_max_last_vision_age_s': max(x['vehicle']['age_s'] for x in hold_commands if x['vehicle']),
            'surface_changes': surfaces,
            'final_world_position_error_m': norm([x - y for x, y in zip(
                truth[-1]['aircraft']['center_world_m'], commands[-1]['own']['position_world_m'])]),
            'feedback': [
                '접촉/모터 종료/3초/직선 조건은 별도 평가와 물리 로그에서 확인됐다. 근접 정지로 대체하지 않았다.',
                '말기 Flow/정밀 range 관측 소실 및 IMU 전달에 의존했다. 실제 센서/마찰/접촉/추력 미확정인 조건부 성과다.',
                '점수는 내부 CoM 동심원 지표다. 상대 착지 오차와 세계 위치 추정 오차를 구분한다.',
                '정상 명령 포화 판정과 접촉 판정을 별도로 보존한다. 변경한 source는 새 시험으로 확인한다.',
                '재사용 training/regression과 unused holdout의 분모를 구분하며 독립 검토 5회까지 진입2로 진행하지 않는다.']}
        review_path.write_text(json.dumps(review, ensure_ascii=False, indent=2) + '\n')
    successes = sum(x['simulation_contact_status'] == 'pass' for x in results)
    total = {
        'created_kst': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(), 'scope': plan['scope'],
        'preplan_sha256': sha(base / 'plan.json'), 'all_cases_completed': True,
        'aggregator_sha256': sha(__file__),
        'frozen_core_files': len(plan['core_hashes']), 'changed_core_files': changed,
        'source_snapshot_and_runtime_matches': True,
        'full_completed_before': plan['prior_full_completed'],
        'full_completed_after': plan['prior_full_completed'] + len(results),
        'simulation_contact_successes_in_series': successes,
        'previous_source_contact_passes_are_separate': plan['prior_contact_success'],
        'contact_and_command_envelope_passes': sum(x['simulation_contact_status'] == 'pass'
                                                 and x['command_envelope_pass'] for x in results),
        'independent_reviewed_successes': 0, 'manufacturer_qualified_successes': 0,
        'native_frames_verified': sum(x['frames'] for x in results),
        'all_command_envelope_pass': all(x['command_envelope_pass'] for x in results),
        'policy_counts': {p: {'attempts': sum(x['policy'] == p for x in results),
                            'contact_pass': sum(x['policy'] == p and x['simulation_contact_status'] == 'pass' for x in results)}
                         for p in sorted({x['policy'] for x in results})}, 'runs': results,
        'split_counts': {split: {'attempts': sum(x['split'] == split for x in results),
                        'contact_pass': sum(x['split'] == split and x['simulation_contact_status'] == 'pass' for x in results),
                        'contact_and_command_pass': sum(x['split'] == split and x['simulation_contact_status'] == 'pass'
                                                       and x['command_envelope_pass'] for x in results)}
                        for split in sorted({x['split'] for x in results})},
        'decision': 'All preplanned attempts preserved and counted by source, policy and split. '
                    'Contact and ordinary command-envelope verdicts are separate. '
                    'Unused cases observed here become development data if used for subsequent adaptation. '
                    'No independent signatures, manufacturer match or entry progression; no general success-rate claim.'}
    output = base / 'summary.json'
    if output.exists():
        raise RuntimeError('existing summary is preserved')
    output.write_text(json.dumps(total, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: total[k] for k in ('policy_counts', 'split_counts', 'native_frames_verified',
                     'contact_and_command_envelope_passes', 'all_command_envelope_pass')}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('series_dir', type=Path)
    summarize(parser.parse_args().series_dir)
