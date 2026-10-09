#!/usr/bin/env python3
"""변경 전후 판단을 저장된 센서/영상 추정 입력에서 정확 비교한다.

참값 파일을 읽지 않고, 새 비행/착륙 성공이나 독립 검토로 세지 않는다.
물리 접촉 adapter의 시작 직전까지 비교하며 그 뒤는 별도 물리 회귀 범위다.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict, fields
import hashlib
import importlib
import json
from pathlib import Path
import sys
import types
import numpy as np
from mission3.controller import Mission3
from mission3.settings import ROOT


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(before, batch, output):
    package=types.ModuleType('_mission3_hierarchy_before')
    package.__path__=[str(before/'scripts/mission3')]
    sys.modules[package.__name__]=package
    old=importlib.import_module(package.__name__+'.controller')
    records=[];inputs={};errors=[]
    for run in sorted(batch.glob('d*-r*/run')):
        paths=[run/n for n in ('controller-settings.json','run_manifest.json','commands.jsonl')]
        identity={str(p.relative_to(ROOT)):digest(p) for p in paths}
        inputs.update(identity)
        cfg=json.loads(paths[0].read_text());manifest=json.loads(paths[1].read_text())
        entry=next(e for e in cfg['entry']['positions'] if e['id']==manifest['entry_id'])
        start=manifest['handoff_time_s']
        previous=old.Mission3(deepcopy(cfg),entry,start_time_s=start)
        current=Mission3(deepcopy(cfg),entry,start_time_s=start)
        count=0;record_count=0;last_phase=None;geometry=False
        command_sha=hashlib.sha256();mismatches=[]
        for line in paths[2].open():
            sample=json.loads(line)
            if sample['t']<start-1e-8:
                for mission in (previous,current):
                    mission.last_v=np.asarray(sample['velocity_world_m_s'])
                    mission.last_yaw_rate=sample['yaw_rate_rad_s']
                continue
            a=previous.update(sample['t'],deepcopy(sample['own']),deepcopy(sample['vehicle']))
            b=current.update(sample['t'],deepcopy(sample['own']),deepcopy(sample['vehicle']))
            count+=1;last_phase=b.phase
            if asdict(a)!=asdict(b) or previous.events!=current.events:
                mismatches.append({'t':sample['t'],'before':asdict(a),'after':asdict(b),'before_events':previous.events,'after_events':current.events})
                break
            command_sha.update(json.dumps(asdict(b),sort_keys=True,allow_nan=False).encode())
            if a.phase=='GEOMETRY_READY':
                # Saved physical command has already continued touchdown here.
                geometry=True;break
            if a.phase!='ABORT':
                expected={f.name:sample.get(f.name,None if f.name in ('coarse_guard','search_guard','approach_reference') else getattr(a,f.name)) for f in fields(a)}
                if asdict(a)!=expected:
                    mismatches.append({'t':sample['t'],'error':'before_controller_does_not_reproduce_original_saved_command','before':asdict(a),'saved':expected});break
                record_count+=1
            if a.done:break
        rows={'case':run.parent.name,'commands_compared':count,'original_saved_commands_exact':record_count,'events_exact':previous.events==current.events,'last_phase':last_phase,'geometry_handoff_reached':geometry,'command_sequence_sha256':command_sha.hexdigest(),'pass':not mismatches,'mismatches':mismatches}
        records.append(rows)
        if mismatches:errors.append(run.parent.name)
        assert all(digest(p)==identity[str(p.relative_to(ROOT))] for p in paths),'historical input changed during verification'
    result={'scope':'저장된 센서/영상 추정 입력을 사용한 변경 전후 판단·명령·이벤트 동일성 검증. 참값 입력·새 비행·독립 서명·성공 수 추가 없음. GEOMETRY_READY 뒤 물리 지원 adapter는 별도 회귀 대상.','cases':records,'case_count':len(records),'commands_compared':sum(r['commands_compared'] for r in records),'pass':bool(records) and not errors,'errors':errors,'input_sha256':inputs}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('case_count','commands_compared','pass','errors')},ensure_ascii=False))
    return 0 if result['pass'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--before',type=Path,required=True);p.add_argument('--batch',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();raise SystemExit(verify(a.before.resolve(),a.batch.resolve(),a.output.resolve()))
