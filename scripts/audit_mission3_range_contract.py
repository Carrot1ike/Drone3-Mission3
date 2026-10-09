#!/usr/bin/env python3
"""Read-only raw-range eligibility diagnostic; not an estimator/control replay."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import shutil


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def inspect(run_name):
    run=Path(run_name)
    experiment=json.loads((run/'experiment.json').read_text())
    minimum=experiment['physical']['sensor']['tof_component_min_accurate_distance_m']
    samples=[json.loads(x) for x in (run/'deck_samples.jsonl').open()]
    commands=[json.loads(x) for x in (run/'commands.jsonl').open()]
    ranges=[x for x in samples if x['kind']=='tof_40hz']
    flow=[x for x in samples if x['kind']=='flow_100hz']
    counters={k:0 for k in ('range_samples','finite_raw','synthetic_accuracy_true',
          'synthetic_accuracy_true_below_minimum_raw','synthetic_accuracy_false_above_minimum_raw',
          'current_onboard_raw_eligible','raw_only_candidate_eligible','current_onboard_below_minimum_raw',
          'tof_driver_range_status_missing','flow_motion_squal_shutter_missing')}
    examples=[]
    for x in ranges:
        raw=x.get('range_m');finite=raw is not None and math.isfinite(raw)
        current=finite and 0<=raw<5.
        age=x['t']-x['range_stamp_s']
        # S candidate: uses measurement and producer stamp only, not ideal ray,
        # synthetic accuracy flag, contact truth, known pad or aircraft state.
        candidate=finite and minimum<=raw<=4. and 0<=age<=.05
        counters['range_samples']+=1;counters['finite_raw']+=int(finite)
        accurate=x.get('range_accurate') is True
        counters['synthetic_accuracy_true']+=int(accurate)
        mismatch=finite and accurate and raw<minimum
        counters['synthetic_accuracy_true_below_minimum_raw']+=int(mismatch)
        counters['synthetic_accuracy_false_above_minimum_raw']+=int(finite and not accurate and minimum<=raw<=4.)
        counters['current_onboard_raw_eligible']+=int(current)
        counters['raw_only_candidate_eligible']+=int(candidate)
        counters['current_onboard_below_minimum_raw']+=int(current and raw<minimum)
        counters['tof_driver_range_status_missing']+=int('RangeStatus' not in x and 'range_status' not in x)
        if mismatch and len(examples)<12:examples.append({'t':x['t'],'raw_range_m':raw,'minimum_S_m':minimum,'synthetic_accuracy':accurate,'producer_age_s':age})
    counters['flow_motion_squal_shutter_missing']=sum(not all(k in x for k in ('motion','squal','shutter')) for x in flow)
    timestamps=[x['range_stamp_s'] for x in ranges]
    return {'run':str(run),'seed':experiment['seed'],'scope':'offline raw eligibility comparison only, not full EKF replay/changed flight',
            'minimum_raw_m':minimum,'minimum_class':'S conservative metric gate, not certified hardware quality',
            'raw_stamp_freshness_max_s':.05,'freshness_class':'S diagnostic, real producer time contract U',
            'producer_timestamp_unique':len(set(timestamps))==len(timestamps),
            'counters':counters,'flow_samples':len(flow),'examples':examples,
            'control_flag_counts':{'range_accurate_true':sum(x['own'].get('range_accurate',False) for x in commands),
                                  'flow_valid_true':sum(x['own'].get('flow_valid',False) for x in commands)},
            'source_sha256':{n:sha(run/n) for n in ('experiment.json','deck_samples.jsonl','commands.jsonl')},
            'script_sha256':sha(__file__),
            'limitations':['No performance/contact verdict changed; no live input connected.',
                'Measurement threshold is not VL53L1X RangeStatus/reflectance/crosstalk uncertainty or pad identity.',
                'Synthetic Flow validity is not motion.motion==0xB0, squal, shutter or actual PMW3901 transfer.',
                'Native firmware raw logging omits SDK RangeStatus; software telemetry extension requires separate design.',
                'Cannot infer full EKF/attitude change from counts alone; requires frozen paired source experiment.']}


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='append',required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(__file__,args.out/Path(__file__).name)
    (args.out/'plan.json').write_text(json.dumps({'runs':args.run,'max_workers':3,'scope':'read-only, no truth/estimator connection'},indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=min(3,len(args.run)),mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(inspect,args.run))
    (args.out/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps([{k:x[k] for k in ('seed','counters')} for x in results],indent=2))


if __name__=='__main__':main()
