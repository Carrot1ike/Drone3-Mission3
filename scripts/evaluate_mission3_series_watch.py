#!/usr/bin/env python3
"""완료된 동결 시험만 오프라인 평가한다. 제어 입력·실행 계획을 바꾸지 않는다."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from evaluate_mission3 import canonical_run
from run_mission3_series import read_plan, manifest_check


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    args=parser.parse_args()
    path=args.plan.resolve();plan=read_plan(path)
    cutoff=datetime.fromisoformat(plan['cutoff_utc'])
    finished=set()
    while datetime.now(timezone.utc)<cutoff:
        state=json.loads((path.parent/'state.json').read_text())
        for job in state['jobs']:
            if job['job_id'] in finished or job['status'] not in ('worker_succeeded','worker_failed'):
                continue
            check=manifest_check(plan,job)
            if not check['valid']:
                print(json.dumps({'job':job['job_id'],'evaluation_skipped':check}),flush=True)
                finished.add(job['job_id']);continue
            directory=Path(job['output']);out=directory/'offline-evaluation.json'
            if not out.exists():
                result=canonical_run(directory)
                with out.open('x') as file:
                    file.write(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            else:
                result=json.loads(out.read_text())
            print(json.dumps({'job':job['job_id'],
                'contact':result['simulation_contact_success']['status'],
                'score':result.get('score'),
                'scope':'stored offline result only; independent summary reevaluates raw logs'},ensure_ascii=False),flush=True)
            finished.add(job['job_id'])
        if state.get('complete') is True:
            return 0 if len(finished)==len(plan['jobs']) else 2
        time.sleep(2)
    print('session cutoff reached; pending cases remain pending',flush=True)
    return 2


if __name__=='__main__':raise SystemExit(main())
