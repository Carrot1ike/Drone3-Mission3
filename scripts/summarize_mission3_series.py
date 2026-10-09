#!/usr/bin/env python3
"""완료 로그의 조건부 series 보고서. 관리 provenance를 검증하며 승격하지 않는다."""
import argparse
from copy import deepcopy
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import re
from zoneinfo import ZoneInfo
from evaluate_mission3 import ROOT, canonical_run
from mission3.evaluation import build_ledger, evaluation_digest
from run_mission3_series import used_seeds

SCHEMA='mission3-series/1'


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as file:
        for chunk in iter(lambda:file.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def read(path):
    value=json.loads(Path(path).read_text())
    if not isinstance(value,dict):raise ValueError('metadata must be a JSON object')
    return value


def resolve(path,base):
    path=Path(path)
    return (path if path.is_absolute() else base/path).resolve()


def member(base,relative):
    path=Path(relative)
    if path.is_absolute() or '..' in path.parts:raise ValueError('snapshot member must be relative')
    target=(base/path).resolve()
    if not target.is_relative_to(base.resolve()):raise ValueError('snapshot member escapes boundary')
    return target


def hash_errors(base,identity):
    errors=[]
    for key in ('code_sha256','runtime_library_sha256'):
        mapping=identity.get(key)
        if not isinstance(mapping,dict) or not mapping:
            errors.append(key+'_missing');continue
        for name,digest in mapping.items():
            try:
                if not isinstance(digest,str) or not re.fullmatch('[0-9a-f]{64}',digest):
                    raise ValueError('invalid SHA-256')
                if sha(member(base,name))!=digest:errors.append(key+':'+name+':bytes_mismatch')
            except (OSError,ValueError,TypeError) as exc:errors.append(key+':'+str(name)+':'+str(exc))
    return errors


def plan_checks(plan,path,jobs,identity):
    # Only the explicit historical runs/top-level-hash shape is legacy.
    # Removing a coordinator schema must not bypass its state requirement.
    legacy=('schema' not in plan and 'runs' in plan and 'jobs' not in plan
            and 'runtime_identity' not in plan and not (path.parent/'state.json').exists()
            and not (path.parent/'management-source').exists())
    coordinated=not legacy;errors=[];state=None
    if coordinated and plan.get('schema')!=SCHEMA:errors.append('unsupported_coordinator_plan_schema')
    digest=path.with_name('plan.sha256')
    if coordinated or digest.exists():
        try:
            if digest.read_text().strip()!=sha(path):errors.append('plan_digest_mismatch')
        except OSError:errors.append('plan_digest_missing')
    if not coordinated:return errors,state,'legacy_direct_plan'
    try:
        if resolve(plan['output'],ROOT)!=path.parent:errors.append('plan_output_mismatch')
        if resolve(plan['root'],ROOT)!=ROOT.resolve():errors.append('plan_root_mismatch')
        if sha(member(path.parent,plan['config_identity_file']))!=plan['config_sha256']:
            errors.append('plan_settings_bytes_mismatch')
        profile=resolve(plan['profile_frozen'],ROOT)
        if not profile.is_relative_to(path.parent) or sha(profile)!=plan['frozen_profile_sha256']:
            errors.append('plan_profile_bytes_mismatch')
        errors.extend(hash_errors(path.parent/'source-snapshot',identity))
        if sha(path.parent/'management-source/run_mission3_series.py')!=plan['coordinator_sha256']:
            errors.append('archived_coordinator_bytes_mismatch')
        ids=[job.get('job_id') for job in jobs]
        if any(not isinstance(value,str) or not value for value in ids) or len(set(ids))!=len(ids):
            errors.append('planned_job_ids_missing_or_duplicate')
        for job in jobs:
            if not isinstance(job.get('trial_metadata_sha256'),str) or not re.fullmatch('[0-9a-f]{64}',job['trial_metadata_sha256']):
                errors.append('planned_trial_digest_missing:'+str(job.get('job_id')))
            if not resolve(job.get('output',''),ROOT).is_relative_to(path.parent):
                errors.append('planned_worker_outside_series:'+str(job.get('job_id')))
    except (OSError,ValueError,TypeError,KeyError) as exc:errors.append('plan_freeze_binding:'+str(exc))
    try:
        state=read(path.parent/'state.json')
        if state.get('schema')!=SCHEMA or state.get('series_id')!=plan.get('series_id') or state.get('plan_sha256')!=sha(path):
            errors.append('state_plan_binding')
        if state.get('stop_reason') is not None:errors.append('state_stop_reason:'+str(state['stop_reason']))
        if state.get('complete') is not True:errors.append('state_not_complete')
        actual=state.get('jobs',[])
        if len(actual)!=len(jobs):errors.append('state_scheduled_jobs_mismatch')
        indexed={j.get('job_id'):j for j in actual}
        if len(indexed)!=len(actual):errors.append('state_duplicate_job_id')
        for job in jobs:
            row=indexed.get(job.get('job_id'))
            if row is None:errors.append('state_job_missing:'+str(job.get('job_id')));continue
            if any(row.get(k)!=job.get(k) for k in ('job_id','entry_id','seed','output','argv','trial_metadata_sha256')):
                errors.append('state_job_binding:'+str(job.get('job_id')))
            process_ok=(type(row.get('returncode')) is int and
                ((row.get('status')=='worker_succeeded' and row['returncode']==0) or
                 (row.get('status')=='worker_failed' and row['returncode']==2)))
            if not process_ok or row.get('identity_changed_during_run'):
                errors.append('state_job_integrity_or_identity_changed:'+str(job.get('job_id')))
            if row.get('manifest_check',{}).get('valid') is not True:
                errors.append('state_manifest_check_failed:'+str(job.get('job_id')))
            if row.get('status')=='worker_failed' and process_ok:
                # The physical runner returns2 for a completed mission failure.
                # Preserve that failure in N; it is distinct from a crash,
                # missing output, timeout, or source/manifest integrity failure.
                try:
                    directory=resolve(job['output'],ROOT)
                    manifest=read(directory/'run_manifest.json');summary=read(directory/'adapter-summary.json')
                    reason=manifest.get('termination_reason')
                    if (manifest.get('completed') is not True or not isinstance(reason,str) or not reason
                            or reason=='hold_window_elapsed' or summary.get('failure')!=reason
                            or row.get('manifest_check',{}).get('termination_reason')!=reason):
                        errors.append('state_worker_failed_without_preserved_mission_failure:'+str(job.get('job_id')))
                except (OSError,ValueError,KeyError,TypeError) as exc:
                    errors.append('state_failed_mission_output:'+str(job.get('job_id'))+':'+str(exc))
        for event in state.get('events',[]):
            if event.get('reason')=='identity_change' or event.get('status')=='identity_changed':
                errors.append('state_identity_change_event')
    except (OSError,ValueError,TypeError,AttributeError) as exc:errors.append('state_unreadable:'+str(exc))
    return errors,state,'coordinator_plan'


def run_checks(directory,manifest,job,plan,identity,series,split,entry):
    differences=[]
    expected={**{k:identity.get(k) for k in ('code_sha256','runtime_library_sha256')},
              'config_sha256':plan.get('config_sha256'),'seed':job.get('seed'),
              'entry_id':entry,'series_id':series,'split':split,'run_id':directory.name}
    for key,value in expected.items():
        if value is None or manifest.get(key)!=value:differences.append(key)
    differences.extend(hash_errors(directory/'source-snapshot',identity))
    try:
        settings_path=member(directory,manifest.get('config_identity_file','frozen-experiment-settings.json'))
        if sha(settings_path)!=manifest.get('config_sha256') or sha(settings_path)!=plan.get('config_sha256'):
            differences.append('frozen_settings_bytes')
        if plan.get('config_identity_file') and manifest.get('config_identity_file')!=plan['config_identity_file']:
            differences.append('config_identity_file')
        settings=read(settings_path);trial=read(directory/'experiment.json')
        trial_sha=sha(directory/'experiment.json')
        if trial_sha!=manifest.get('trial_metadata_sha256'):differences.append('trial_metadata_bytes')
        if job.get('trial_metadata_sha256') and trial_sha!=job['trial_metadata_sha256']:
            differences.append('trial_metadata_sha256')
        if {k:trial.get(k) for k in settings}!=settings or trial.get('entry_id')!=entry or trial.get('seed')!=job.get('seed'):
            differences.append('trial_settings_entry_seed')
        # Current and historical v2 public settings: no plant code executed.
        allowed=('camera','arena','track','marker','landing_geometry','entry','observer','control')
        mission=settings['mission'];physical=settings['physical']
        public={key:deepcopy(mission[key]) for key in allowed}
        public['control'].update(physical.get('controller_software',{}))
        public['observer'].update(physical.get('observer_software',{}))
        if physical.get('observer_optics_sensitivity'):
            public['camera']['observer_optics_sensitivity']=physical['observer_optics_sensitivity']
        public['control']['terminal_policy']='last_fix_bridge'
        if public!=read(member(directory,manifest.get('controller_settings_file','controller-settings.json'))):
            differences.append('controller_settings_projection')
    except (OSError,ValueError,TypeError,KeyError) as exc:differences.append('settings_trial_binding:'+str(exc))
    return differences


def time_value(value):
    result=datetime.fromisoformat(value)
    if result.tzinfo is None:raise ValueError('timestamp lacks timezone')
    return result.astimezone(timezone.utc)


def attempt_time(path,manifest):
    for key in ('started_at_utc','started_at_kst','created_at_utc','created_at_kst'):
        if manifest.get(key):return time_value(manifest[key]),key
    match=re.match(r'(\d{8}-\d{6})-',manifest.get('run_id',''))
    if match:
        return datetime.strptime(match[1],'%Y%m%d-%H%M%S').replace(tzinfo=ZoneInfo('Asia/Seoul')).astimezone(timezone.utc),'run_id_timestamp'
    trial=path.parent/'experiment.json'
    if trial.exists():return datetime.fromtimestamp(trial.stat().st_mtime,timezone.utc),'frozen_trial_filesystem_mtime'
    return None,'time_unavailable'


def seed_history(plan,jobs):
    conflicts=[];later=[];errors=[]
    scheduled={resolve(j.get('output',j.get('run_dir','')),ROOT)/'run_manifest.json' for j in jobs}
    pairs={(j.get('entry_id',j.get('entry',plan.get('entry_id',plan.get('entry')))),j.get('seed')) for j in jobs}
    created=plan.get('created_at_utc',plan.get('created_kst'))
    try:cutoff=time_value(created) if created else None
    except (ValueError,TypeError) as exc:cutoff=None;errors.append('plan_history_time:'+str(exc))
    if plan.get('training_reuse_history'):conflicts.append({'kind':'declared_training_reuse','history':plan['training_reuse_history']})
    # Explicit training paths declare prior history, including test doubles.
    for name in plan.get('training_runs',[]):
        path=resolve(name,ROOT);path=path if path.name=='run_manifest.json' else path/'run_manifest.json'
        try:
            m=read(path)
            if (m.get('entry_id'),m.get('seed')) in pairs:conflicts.append({'path':str(path),'kind':'explicit_prior_training'})
        except (OSError,ValueError) as exc:errors.append('training_metadata:'+str(path)+':'+str(exc))
    try:registry=used_seeds(ROOT)
    except (OSError,ValueError) as exc:registry={};errors.append('seed_registry:'+str(exc))
    for pair in pairs:
        for name in registry.get(pair,[]):
            path=Path(name).resolve()
            if path in scheduled:continue
            try:
                m=read(path);stamp,basis=attempt_time(path,m)
                item={'path':str(path),'entry_id':pair[0],'seed':pair[1],'split':m.get('split'),
                      'time_basis':basis,'attempt_time_utc':None if stamp is None else stamp.isoformat()}
                if cutoff is not None and stamp is not None and stamp>cutoff:later.append(item)
                else:conflicts.append(item)
            except (OSError,ValueError,TypeError) as exc:errors.append('seed_history:'+str(path)+':'+str(exc))
    return conflicts,later,errors


def review_extensions(review,manifest,directory,plan_path):
    expected={'trial_metadata_sha256':manifest.get('trial_metadata_sha256'),
              'runtime_library_sha256':manifest.get('runtime_library_sha256'),
              'plan_sha256':sha(plan_path),'series_id':manifest.get('series_id'),
              'entry_id':manifest.get('entry_id'),'seed':manifest.get('seed'),'run_dir':str(directory)}
    errors=[key for key,value in expected.items() if key in review and review[key]!=value]
    if 'evaluation_file_sha256' in review:
        try:
            if review['evaluation_file_sha256']!=sha(directory/'offline-evaluation.json'):errors.append('evaluation_file_sha256')
        except OSError:errors.append('evaluation_file_sha256')
    return errors


def summarize(plan_path):
    plan_path=Path(plan_path).resolve();plan=read(plan_path)
    jobs=plan.get('jobs',plan.get('runs',[]))
    if not jobs:raise ValueError('plan contains no scheduled jobs')
    identity=plan.get('runtime_identity',plan)
    errors,state,mode=plan_checks(plan,plan_path,jobs,identity)
    conflicts,later,history_errors=seed_history(plan,jobs);errors.extend(history_errors)
    series=plan.get('series_id',plan.get('series'));split=plan.get('split')
    items=[];completed=[];pending=[];mismatches=[];attempt_errors=[]
    review_index={};review_read_errors=[];extension_errors=[]
    for path in sorted((ROOT/'artifacts/mission3/independent-review').rglob('signedreview.json')):
        try:
            review=read(path);review_index.setdefault(review.get('run_id'),[]).append((path,review))
        except (OSError,ValueError,TypeError) as exc:
            review_read_errors.append({'path':str(path),'error':str(exc),'cannot_approve':True})
    for job in jobs:
        directory=resolve(job.get('output',job.get('run_dir','')),ROOT);mp=directory/'run_manifest.json'
        try:manifest=read(mp)
        except (OSError,ValueError) as exc:
            pending.append({'run_dir':str(directory),'reason':'manifest_missing_or_corrupt','error':str(exc)});continue
        if manifest.get('completed') is not True:
            pending.append({'run_dir':str(directory),'reason':'incomplete'});continue
        entry=job.get('entry_id',job.get('entry',plan.get('entry_id',plan.get('entry'))))
        differences=run_checks(directory,manifest,job,plan,identity,series,split,entry)
        if differences:mismatches.append({'run_id':manifest.get('run_id'),'keys':differences})
        try:evaluation=canonical_run(directory)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            attempt_errors.append({'run_dir':str(directory),'error':str(exc)})
            completed.append({'run_id':manifest.get('run_id'),'run_dir':str(directory),'contact_status':'ineligible','evaluation_error':str(exc)})
            continue
        review_pairs=review_index.get(evaluation['run_id'],[]);eligible=[];review_rows=[]
        for path,review in review_pairs:
            extra=review_extensions(review,manifest,directory,plan_path)
            review_rows.append({'path':str(path),'sha256':sha(path),'extension_errors':extra})
            if extra:extension_errors.append({'path':str(path),'run_id':evaluation['run_id'],'keys':extra})
            else:eligible.append(review)
        # Bound corrupt records remain invalid related reviews. Unbound corrupt
        # records are reported separately and can never approve an attempt.
        for broken in review_read_errors:
            if any(evaluation['run_id'] in part for part in Path(broken['path']).parts[:-1]):
                eligible.append({'schema':'corrupt-review','run_id':evaluation['run_id']})
                broken['affected_run_id']=evaluation['run_id']
        items.append({'evaluation':evaluation,'reviews':eligible})
        encoded=(json.dumps(evaluation,ensure_ascii=False,indent=2)+'\n').encode()
        completed.append({'run_id':evaluation['run_id'],'run_dir':str(directory),'seed':evaluation['seed'],
            'entry':evaluation['entry_id'],'split':evaluation['split'],'score':evaluation['score'],
            'contact_status':evaluation['simulation_contact_success']['status'],
            'contact_fail_reasons':evaluation['simulation_contact_success']['fail_reasons'],
            'evaluation_sha256':hashlib.sha256(encoded).hexdigest(),
            'canonical_evaluation_digest':evaluation_digest(evaluation),'reviews':review_rows})
    ledger=build_ledger(items);rows=ledger['attempts']
    approved=sum(r['simulation_contact_success']=='pass' and r['review_approved_contact'] for r in rows)
    invalid=sum(not r['valid'] for a in rows for r in a['reviews'])+len(extension_errors)
    fresh=plan.get('seeds_unused_at_entry',plan.get('holdout_seed_previously_used') is False) is True
    pairs=[(j.get('entry_id',j.get('entry',plan.get('entry_id',plan.get('entry')))),j.get('seed')) for j in jobs]
    outputs=[str(resolve(j.get('output',j.get('run_dir','')),ROOT)) for j in jobs]
    unique=len(set(pairs))==len(jobs) and len(set(outputs))==len(jobs)
    if plan.get('seeds')!=[pair[1] for pair in pairs]:errors.append('planned_seed_sequence_mismatch')
    if any(type(e) is not int or type(s) is not int for e,s in pairs):errors.append('planned_entry_seed_types')
    gate=(len(completed)==len(jobs) and not pending and not mismatches and not invalid
          and not errors and not conflicts and not attempt_errors and fresh and split=='holdout'
          and all(r['split']=='holdout' for r in rows) and unique
          and len({e for e,_ in pairs})==1 and approved>=5)
    return {'schema':'mission3-conditional-series-report/1','created_utc':datetime.now(timezone.utc).isoformat(),
        'producer_sha256':sha(Path(__file__)),'plan':str(plan_path),'plan_sha256':sha(plan_path),
        'provenance_mode':mode,'coordinator_state_sha256':sha(plan_path.parent/'state.json') if state is not None else None,
        'plan_state_errors':errors,'known_prior_seed_reuse':conflicts,'later_seed_uses':later,
        'history_time_limit':'metadata and frozen-trial filesystem timestamps; not cryptographic/global formal provenance',
        'scheduled':len(jobs),'completed':len(completed),'pending':pending,'attempt_errors':attempt_errors,
        'identity_mismatches':mismatches,'completed_attempts':completed,'immutable_ledger':ledger,
        'review_read_errors':review_read_errors,'review_extension_errors':extension_errors,
        'conditional_contact_passes':sum(r['simulation_contact_success']=='pass' for r in rows),
        'independently_approved_contact_passes':approved,'invalid_reviews':invalid,
        'formal_qualified_count':0,'global_formal_protocol_verified':False,
        'formal_count_reason':'local subset ledger is diagnostic only; global external formal protocol proof is not implemented',
        'conditional_research_gate_passed':bool(gate),
        'scope':'offline conditional research only; no manufacturer match, hardware approval, or formal promotion'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    if args.out.exists():p.error('existing report is preserved; choose a new output path')
    result=summarize(args.plan);args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('x') as file:file.write(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('scheduled','completed','conditional_contact_passes',
        'independently_approved_contact_passes','invalid_reviews','formal_qualified_count','conditional_research_gate_passed')}))
    return 0


if __name__=='__main__':raise SystemExit(main())
