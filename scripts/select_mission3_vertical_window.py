#!/usr/bin/env python3
"""Apply the frozen v14 development selection rule to all completed trials."""
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


def audit(case):
    run=Path(case['run'])
    evaluation=json.loads((run/'offline-evaluation.json').read_text())
    caps=json.loads((run/'command-limits-audit.json').read_text())
    command=rows(run/'commands.jsonl')
    off=json.loads((run/'adapter-summary.json').read_text())['motor_off_at_s']
    start=next((x['t'] for x in command if x['phase']=='TOUCHDOWN_DESCEND'),None)
    result={'policy':case['policy'],'seed':case['seed'],'run':str(run),
            'contact_and_caps':evaluation['simulation_contact_success']['status']=='pass'
                               and caps['command_envelope_pass']}
    if off is None or start is None:
        return {**result,'post_loss_height_rmse_m':None,'analysis':'no completed terminal/off'}
    sensor=[x for x in rows(run/'sensors.jsonl') if start<=x['t']<=off+1e-8]
    qualified=[x['t'] for x in sensor if x['observer_packet']['range_new']
        and x['estimate']['surface_hypothesis'] in ('moving_surface_candidate','static_floor_candidate')]
    last=max(qualified)
    seq=[x for x in sensor if x['t']>last]
    assert seq
    truth=rows(run/'truth.jsonl')
    times=[x['t'] for x in truth]
    measured=np.array([x['estimate']['position_world_m'][2] for x in seq])
    actual=np.interp([x['t'] for x in seq],times,[x['aircraft']['center_world_m'][2] for x in truth])
    return {**result,'last_new_qualified_range_s':last,'motor_off_s':off,
            'post_loss_samples':len(seq),'post_loss_height_rmse_m':float(np.sqrt(np.mean((measured-actual)**2))),
            'post_loss_max_abs_height_error_m':float(np.max(np.abs(measured-actual))),
            'vertical_fit_linear_fallbacks':sensor[-1]['estimate'].get('vertical_fit_linear_fallbacks'),
            'future_range_packets':sum(x['observer_packet']['range_stamp_s']>x['t']+1e-8 for x in sensor),
            'source_sha256':{p:sha(run/p) for p in ('sensors.jsonl','truth.jsonl','offline-evaluation.json','command-limits-audit.json')}}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('series',type=Path);args=parser.parse_args()
    plan=json.loads((args.series/'plan.json').read_text())
    assert (args.series/'summary.json').exists(), 'whole-series aggregate must precede selection'
    target=args.series/'selection.json';assert not target.exists()
    shutil.copyfile(__file__,args.series/Path(__file__).name)
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(audit,plan['cases']))
    policies=[]
    original_positive={433,467,479,541,577}
    for policy in sorted({x['policy'] for x in results}):
        group=[x for x in results if x['policy']==policy]
        positive={x['seed'] for x in group if x['contact_and_caps']}
        policies.append({'policy':policy,'attempts':len(group),'contact_and_caps':len(positive),
            'original_five_retained':original_positive<=positive,
            'max_post_loss_height_rmse_m':max((x['post_loss_height_rmse_m'] for x in group
                if x['post_loss_height_rmse_m'] is not None),default=None)})
    eligible=[x for x in policies if x['original_five_retained']]
    chosen=min(eligible,key=lambda x:(-x['contact_and_caps'],x['max_post_loss_height_rmse_m'])) if eligible else None
    target.write_text(json.dumps({'rule_declared_in_plan':plan['selection_criterion_before_results'],
        'preplan_sha256':sha(args.series/'plan.json'),'analysis_sha256':sha(__file__),
        'policies':policies,'chosen':chosen,'cases':results,'scope':'spent-development selection; no unused holdout conclusion',
        'qualified_count':0,'formal_count':0,'entry2_authorized':False},indent=2)+'\n')
    print(json.dumps({'policies':policies,'chosen':chosen},indent=2))


if __name__=='__main__':main()
