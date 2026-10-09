#!/usr/bin/env python3
"""추적기의 점진적 ID 외형 변화 반례. 합성 S 입력, 제어와 연결하지 않음.

원 native 프레임을 읽기만 하며 시뮬레이터 truth를 읽지 않는다. 가상
20FPS에서 code 영역을 점진적으로 다른 ID로 바꾸는 합성 반례다. 실제
가림/센서 변화 모델이나 실제 오연결 확률을 재현한 결과는 아니다.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import cv2
import numpy as np
from benchmark_mission3_marker_tracking import advance, grid
from stress_mission3_marker_tracking import variant


def fixed_reference_ncc(anchor,current,initial_quad,current_quad):
    """첫 decoded code의 고정 외형과 비교하는 offline 추가 진단."""
    H=cv2.getPerspectiveTransform(initial_quad.astype(np.float32),current_quad.astype(np.float32))
    low=np.maximum(np.floor(initial_quad.min(axis=0)-2).astype(int),0)
    high=np.minimum(np.ceil(initial_quad.max(axis=0)+3).astype(int),320)
    yy,xx=np.mgrid[low[1]:high[1],low[0]:high[0]]
    p=np.stack([xx,yy],axis=-1).astype(np.float32).reshape(-1,1,2)
    mapped=cv2.perspectiveTransform(p,H).reshape(len(yy),len(yy[0]),2)
    aligned=cv2.remap(current,mapped[:,:,0],mapped[:,:,1],cv2.INTER_LINEAR).astype(float)
    before=anchor[low[1]:high[1],low[0]:high[0]].astype(float)
    u=before-before.mean();v=aligned-aligned.mean();norm=np.linalg.norm(u)*np.linalg.norm(v)
    return float(np.sum(u*v)/norm) if norm>1e-6 else -1.


def worker(args):
    source,initial_quad,case,window,wrong_id,fade_steps=args;cv2.setNumThreads(1)
    anchor=variant(source,initial_quad,'redraw_target_id0',0)
    wrong=variant(source,initial_quad,f'replace_wrong_id{wrong_id}',0)
    previous=anchor.copy();quad=initial_quad.copy();points=grid(quad)
    records=[];expiry=False
    for i in range(1,23):
        stamp=i/20
        if stamp>1.:
            expiry=True;records.append({'virtual_s':stamp,'status':'last_decoded_age_expired'});break
        mix=min(1.,i/fade_steps)
        current=np.rint((1-mix)*anchor.astype(float)+mix*wrong.astype(float)).astype(np.uint8)
        result,status,details=advance(previous,current,points,quad,window)
        reference=None
        if result is not None:reference=fixed_reference_ncc(anchor,current,initial_quad,result[0])
        records.append({'virtual_s':stamp,'synthetic_wrong_id':wrong_id,'wrong_id_mix':mix,
                        'status':status,'accepted_unconfirmed_continuity':result is not None,
                        'fixed_decoded_reference_ncc':reference,'fixed_reference_threshold_S':.85,
                        'fixed_reference_would_reject':reference is not None and reference<.85,**details})
        if result is None:break
        quad,points=result;previous=current
    return {'case':case,'window_px':window,'synthetic_wrong_id':wrong_id,'fade_steps':fade_steps,
            'records':records,'last_decode_expiry_applied':expiry,
            'accepted_fully_wrong_frames':sum(r.get('wrong_id_mix')==1. and r.get('accepted_unconfirmed_continuity',False) for r in records),
            'first_fixed_reference_reject_s':next((r['virtual_s'] for r in records if r.get('fixed_reference_would_reject')),None)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stress-plan',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    parent=json.loads(a.stress_plan.read_text());jobs=[];source_hashes={}
    for run,sources in parent['source_hashes'].items():
        run=Path(run)
        tracking=next(Path(f) for f in sources if f.endswith('win15-age1.json'))
        records=json.loads(tracking.read_text())['records']
        anchors=[x for x,y in zip(records,records[1:]) if x['status']=='decoded_reset' and y['status']=='tracked_partial_bearing' and y['frame']==x['frame']+1]
        needed={r['frame'] for r in anchors}
        meta=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
        capture=cv2.VideoCapture(str(run/'camera.avi'));index=0;images={}
        while True:
            ok,bgr=capture.read()
            if not ok:break
            if index in needed:
                if bgr.shape!=(320,320,3) or not np.array_equal(bgr[:,:,0],bgr[:,:,1]) or not np.array_equal(bgr[:,:,1],bgr[:,:,2]):raise RuntimeError('native format mismatch')
                gray=bgr[:,:,0].copy()
                if hashlib.sha256(gray.tobytes()).hexdigest()!=meta[index]['controller_input_sha256']:raise RuntimeError('source pixel mismatch')
                images[index]=gray
            index+=1
        capture.release()
        if index!=len(meta) or set(images)!=needed:raise RuntimeError('recorded source mismatch')
        source_hashes[str(run)]=sources
        for r in anchors:
            case={'run':str(run),'frame':r['frame'],'source_capture_s':r['capture_s']}
            jobs.extend((images[r['frame']],np.array(r['corners'],np.float32),case,w,identity,fade)
                        for w in (7,15) for identity in (1,2,17) for fade in (5,10))
    plan={'planned_jobs':len(jobs),'workers':4,'parent_plan_sha256':hashlib.sha256(a.stress_plan.read_bytes()).hexdigest(),
          'source_used_pixels_sha_exact':True,'source_hashes':source_hashes,
          'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'dependency_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                                (Path('scripts/benchmark_mission3_marker_tracking.py'),Path('scripts/stress_mission3_marker_tracking.py'))},
          'scope':'synthetic gradual appearance counterexample at20FPS, last decode age1s; frozen background/native320; no truth or controller; all values S',
          'fixed_reference_guard':'offline diagnostic only, threshold .85S; not enabled on tracker/controller'}
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:results=list(pool.map(worker,jobs))
    (a.out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    summary={'plan':plan,'completed_jobs':len(results),
             'cases_with_any_accepted_fully_wrong_frame':sum(r['accepted_fully_wrong_frames']>0 for r in results),
             'accepted_fully_wrong_frames':sum(r['accepted_fully_wrong_frames'] for r in results),
             'cases_with_fixed_reference_warning':sum(r['first_fixed_reference_reject_s'] is not None for r in results),
             'cases_with_age_expiry':sum(r['last_decode_expiry_applied'] for r in results),
             'decision':'incremental FB/NCC continuity may pass gradual identity change; fixed decoded reference is only an additional appearance check, not a new ID decode or proven physical robustness. Keep LK offline until image association, age, source provenance and partial-state semantics are separately validated.'}
    (a.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
