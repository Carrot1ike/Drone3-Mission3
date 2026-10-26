#!/usr/bin/env python3
"""Offline same-input/failed-flight delivery audit, never a reviewer signature."""
import argparse
import hashlib
import json
from pathlib import Path

JSON_LOGS = ['commands.jsonl', 'truth.jsonl', 'observations.jsonl', 'motors.jsonl',
             'sensors.jsonl', 'deck_samples.jsonl', 'estimates.jsonl',
             'rpyt_packets.jsonl', 'camera_frames.jsonl']


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with path.open() as f:
        for line in f:
            yield json.loads(line)


def prefix(path, before):
    h, count = hashlib.sha256(), 0
    with path.open('rb') as f:
        for line in f:
            if json.loads(line)['t'] >= before:
                break
            h.update(line); count += 1
    return {'sha256': h.hexdigest(), 'lines': count}


def norm(v):
    return sum(x*x for x in v)**.5


def audit(case):
    run, old = Path(case['run']), Path(case['reference_run'])
    load = lambda p: json.loads(p.read_text())
    summary, reference = load(run/'adapter-summary.json'), load(old/'adapter-summary.json')
    ev, ref_ev = load(run/'offline-evaluation.json'), load(old/'offline-evaluation.json')
    result = {'policy':case['policy'], 'seed':case['seed'], 'run':str(run), 'reference_run':str(old),
              'original_failure':reference['failure'], 'new_failure':summary['failure'],
              'original_contact_status':ref_ev['simulation_contact_success']['status'],
              'new_contact_status':ev['simulation_contact_success']['status'],
              'scope':'main offline regression/delivery audit only; no independent signature or real safety approval',
              'qualified':0, 'safe_recovery_approved':False}
    checks = {'original_failure_reason_retained':summary['failure']==reference['failure'],
              'original_contact_status_retained':ev['simulation_contact_success']['status']==ref_ev['simulation_contact_success']['status'],
              'normal_caps_pass':load(run/'command-limits-audit.json')['command_envelope_pass'],
              'native_pixels_exact':load(run/'camera-serialization-validation.json')['exact_pixel_replay']}
    if reference['failure'] is None:
        identical = {name:sha(run/name)==sha(old/name) for name in [*JSON_LOGS,'camera.avi']}
        result['full_raw_file_byte_identity']=identical
        checks['all10_raw_files_identical']=all(identical.values())
        checks['no_abort_delivery_on_positive']=summary['abort_delivery'] is None
    else:
        start = next(r['t'] for r in rows(old/'commands.jsonl') if r['phase']=='ABORT')
        p = {name:{'original':prefix(old/name,start),'new':prefix(run/name,start)} for name in JSON_LOGS}
        checks['all9_original_prefixes_identical']=all(x['original']==x['new'] for x in p.values())
        result['before_first_original_abort_prefixes']=p
        commands=[r for r in rows(run/'commands.jsonl') if r['t']>=start]
        wire=[r for r in rows(run/'rpyt_packets.jsonl') if r['t']>=start]
        motor_count=0;zero_pwm_count=0;last_motor=None
        for r in rows(run/'motors.jsonl'):
            if r['t']<start: continue
            motor_count+=1;zero_pwm_count+=all(x==0 for x in r['motor_output']);last_motor=r
        truth=[r for r in rows(run/'truth.jsonl') if r['t']>=start]
        delivery=summary['abort_delivery']
        checks['latched_failure_matches_original']=delivery['failure_latched']==reference['failure']
        checks['failure_start_matches_original']=delivery['started_at_s']==start
        checks['post_failure_all_zero_velocity_yaw']=all(r['phase']=='ABORT' and norm(r['velocity_world_m_s'])==0 and r['yaw_rate_rad_s']==0 for r in commands)
        checks['airborne_stop_packet_absent']=len(wire)>0 and all(r['stop'] is False for r in wire)
        checks['wire_and_physics_delivered']=len(wire)>0 and motor_count>0 and len(truth)>1 and delivery['physics_steps_after_failure']==motor_count
        checks['wire_count_matches_summary']=len(wire)==delivery['packet_count_after_failure']
        checks['bounded_finish_has_named_status']=delivery['finish'] in ('sensor_stationary_hold_observed','bounded_delivery_elapsed_without_stationary_evidence')
        checks['predeclared_3s_plus_one_physics_bound']=summary['duration_s']-start<=3.001+1e-8
        if delivery['finish']=='sensor_stationary_hold_observed':
            latest=delivery['last_observation']; since=latest['t']-latest['stable_dwell_s']
            stable_rows=[r for r in commands if since-1e-8<=r['t']<=latest['t']+1e-8]
            checks['recorded_stationary_window_contiguous']=len(stable_rows)>=26 and all(b['t']-a['t']<=.03 for a,b in zip(stable_rows,stable_rows[1:]))
            checks['recorded_window_sensor_stable']=all(r['abort_delivery']['last_observation']['stable'] for r in stable_rows)
        result['failed_flight_delivery']=delivery
        result['after_failure_offline_only']={'first_abort_s':start,'duration_after_failure_s':summary['duration_s']-start,
            'command_samples':len(commands),'wire_packets':len(wire),'motor_samples':motor_count,'zero_PWM_samples':zero_pwm_count,
            'truth_samples':len(truth),'last_actual_speed_m_s':norm(truth[-1]['aircraft']['velocity_world_m_s']) if truth else None,
            'last_actual_tilt_rpy_rad':truth[-1]['aircraft'].get('rpy_rad') if truth else None,
            'last_motor_t_s':last_motor['t'] if last_motor else None,'last_truth_t_s':truth[-1]['t'] if truth else None}
    result['checks']=checks
    result['all_predeclared_checks_pass']=all(checks.values())
    result['evidence_sha256']={str(run/name):sha(run/name) for name in ['adapter-summary.json','run_manifest.json','offline-evaluation.json',*JSON_LOGS,'camera.avi']}
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('series',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();assert not a.out.exists()
    plan=json.loads((a.series/'plan.json').read_text())
    assert (a.series/'summary.json').exists(), 'whole series aggregate must exist first'
    result={'plan_sha256':sha(a.series/'plan.json'),'audit_sha256':sha(Path(__file__)),
            'scope':'main offline audit of all predeclared regression cases; originals and reviewer outputs untouched',
            'cases':[audit(c) for c in plan['cases']], 'qualified':0,'independent_signature':None}
    result['all_predeclared_checks_pass']=all(r['all_predeclared_checks_pass'] for r in result['cases'])
    a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'all_predeclared_checks_pass':result['all_predeclared_checks_pass'],
                      'cases':[{'policy':r['policy'],'seed':r['seed'],'failed_checks':[k for k,v in r['checks'].items() if not v]} for r in result['cases']]}))
    return 0 if result['all_predeclared_checks_pass'] else 2


if __name__=='__main__':raise SystemExit(main())
