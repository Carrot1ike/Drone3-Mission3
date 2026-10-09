#!/usr/bin/env python3
"""Bounded processes for an immutable predeclared trial set and its audits."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from zoneinfo import ZoneInfo


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def execute(case, identities, deadline):
    if datetime.now(ZoneInfo('Asia/Seoul')) >= deadline:
        return {'case': case, 'not_started': 'deadline', 'all_checks_completed': False}
    for p, digest in identities.items():
        if sha(p) != digest:
            raise RuntimeError(f'frozen source changed: {p}')
    run = Path(case['run'])
    if run.exists():
        raise RuntimeError(f'original run already exists: {run}')
    start = datetime.now(ZoneInfo('Asia/Seoul')).isoformat()
    with Path(case['log']).open('x') as log:
        code = subprocess.run(case['argv'], stdout=log, stderr=subprocess.STDOUT).returncode
    if code not in (0, 2) or not (run / 'adapter-summary.json').exists():
        return {'case': case, 'start_kst': start, 'flight_exit_code': code, 'all_checks_completed': False}
    checks = [
        [sys.executable, 'scripts/evaluate_mission3.py', 'run', str(run), '--out', str(run / 'offline-evaluation.json')],
        [sys.executable, 'scripts/audit_mission3_limits.py', str(run)],
        [sys.executable, 'scripts/validate_mission3_camera_recording.py', str(run)],
    ]
    outcomes = []
    with Path(case['log'].replace('.log', '-checks.log')).open('x') as log:
        for argv in checks:
            status = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT).returncode
            outcomes.append({'argv': argv, 'exit_code': status})
            if status:
                break
    return {'case': case, 'start_kst': start, 'flight_exit_code': code, 'checks': outcomes,
            'all_checks_completed': len(outcomes) == 3 and all(x['exit_code'] == 0 for x in outcomes)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('series', type=Path)
    args = p.parse_args()
    plan = json.loads((args.series / 'plan.json').read_text())
    identities = {**plan['core_hashes'], **plan['config_file_hashes']}
    deadline = datetime.fromisoformat(plan['deadline_kst'])
    shutil.copyfile(__file__, args.series / Path(__file__).name)
    results = []
    with ThreadPoolExecutor(max_workers=min(4, plan['max_physics_workers'])) as pool:
        tasks = {pool.submit(execute, case, identities, deadline): i for i, case in enumerate(plan['cases'])}
        for task in as_completed(tasks):
            result = task.result()
            result['preplan_index'] = tasks[task]
            results.append(result)
            with (args.series / 'execution-progress.jsonl').open('a') as out:
                out.write(json.dumps(result, ensure_ascii=False) + '\n')
            print(json.dumps({'index': tasks[task], 'policy': result['case']['policy'],
                              'seed': result['case']['seed'], 'flight_exit_code': result.get('flight_exit_code'),
                              'all_checks_completed': result['all_checks_completed']}), flush=True)
    (args.series / 'execution-results.json').write_text(json.dumps({'plan_sha256': sha(args.series / 'plan.json'),
          'launcher_sha256': sha(__file__), 'results': sorted(results, key=lambda x: x['preplan_index'])}, indent=2) + '\n')


if __name__ == '__main__':
    main()
