#!/usr/bin/env python3
"""Read-only whole raw regression, chronology inventory and frozen binding."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

RAW=('truth.jsonl','commands.jsonl','observations.jsonl','motors.jsonl','sensors.jsonl',
     'deck_samples.jsonl','estimates.jsonl','rpyt_packets.jsonl','camera_frames.jsonl','camera.avi')

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('series',type=Path);a=p.parse_args()
    plan=json.loads((a.series/'plan.json').read_text())
    summary=json.loads((a.series/'summary.json').read_text());rows=[]
    for case in plan['cases']:
        run=Path(case['run']);reference=Path(case['reference_run'])
        raw={name:{'new_sha256':sha(run/name),'reference_sha256':sha(reference/name)} for name in RAW}
        last_consumer=last_producer=None;count=fresh=held=0;max_age=0.;fallback=0
        for line in (run/'sensors.jsonl').open():
            record=json.loads(line);deck=record['deck'];stamp=float(deck['stamp_s']);producer=float(deck['range_stamp_s'])
            assert math.isfinite(stamp) and math.isfinite(producer) and producer<=stamp
            assert last_consumer is None or 0<stamp-last_consumer<=.2
            assert record['imu']['stamp_s']<=stamp and deck['flow_stamp_s']<=stamp
            if deck['range_new']:
                assert last_producer is None or producer>last_producer
                last_producer=producer;fresh+=1
            else:
                assert producer>=last_producer;held+=1
            count+=1;max_age=max(max_age,stamp-producer);last_consumer=stamp
            fallback=max(fallback,record['estimate'].get('vertical_fit_linear_fallbacks',0))
        manifest=json.loads((run/'run_manifest.json').read_text())
        source={**manifest['code_sha256'],**manifest['runtime_library_sha256']}
        assert all(sha(run/'source-snapshot'/name)==value==plan['core_hashes'][name]
                   for name,value in source.items())
        current=json.loads((run/'offline-evaluation.json').read_text())
        prior=json.loads((reference/'offline-evaluation.json').read_text())
        identical=all(item['new_sha256']==item['reference_sha256'] for item in raw.values())
        rows.append({'case':case,'raw_files':raw,'all10_raw_byte_identical':identical,
                     'contact_status':current['simulation_contact_success']['status'],
                     'reference_contact_status':prior['simulation_contact_success']['status'],
                     'contact_verdict_unchanged':current['simulation_contact_success']['status']==prior['simulation_contact_success']['status'],
                     'consumer_samples':count,'fresh_range_samples':fresh,'held_range_consumptions':held,
                     'max_range_consumer_age_s':max_age,'invalid_chronology_observed':0,
                     'maximum_recorded_linear_fit_fallbacks':fallback,
                     'source_files':len(manifest['code_sha256']),'runtime_files':len(manifest['runtime_library_sha256']),
                     'source_snapshot_matches_plan':True})
    result={'created_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'scope':'same four spent valid-input physical flights, separate source regression; no efficacy or independent signature',
            'preplan_sha256':sha(a.series/'plan.json'),'summary_sha256':sha(a.series/'summary.json'),
            'contract_at_plan_sha256':sha(a.series/'contract-at-plan.md'),
            'contract_snapshot_matches_preplan':sha(a.series/'contract-at-plan.md')==plan['contract']['sha256'],
            'all_cases_completed':summary['all_cases_completed'],'cases':rows,
            'all_four_10_raw_byte_identical':all(r['all10_raw_byte_identical'] for r in rows),
            'all_contact_verdicts_unchanged':all(r['contact_verdict_unchanged'] for r in rows),
            'all_caps_pass':summary['all_command_envelope_pass'],'native_frames_exact':summary['native_frames_verified'],
            'full_completed_before':plan['prior_full_completed'],
            'full_completed_after':plan['prior_full_completed']+len(rows),
            'qualified':0,'hardware_approval':False,
            'past_signature_reused':False,'guard_rejection_is_not_flight_recovery':True,
            'audit_script_sha256':sha(Path(__file__))}
    assert all(result[k] for k in ('all_four_10_raw_byte_identical','all_contact_verdicts_unchanged',
                                 'all_caps_pass','contract_snapshot_matches_preplan'))
    with (a.series/'whole-raw-chronology-regression-audit.json').open('x') as f:
        f.write(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='cases'},ensure_ascii=False))

if __name__=='__main__':main()
