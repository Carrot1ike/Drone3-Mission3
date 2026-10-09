#!/usr/bin/env python3
"""완료된 무손실 영상의 디코더 비교. 모든 workers 종료 후 truth 평가.

카메라 native320 입력/SHA를 유지한다. ArUco 내부 표준 cell rectification
설정 비교는 별도 후보이며 입력 확대/새 픽셀/제어기 입력이 아니다.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import time
import cv2
import numpy as np
from benchmark_mission3_native_detection import VARIANTS,build_evaluation_reference


PARAMETERS=[*VARIANTS,
    {'name':'small_window_c3_cell8','parameters':{**VARIANTS[2]['parameters'],'perspectiveRemovePixelPerCell':8}},
    {'name':'small_window_c7_cell8','parameters':{**VARIANTS[4]['parameters'],'perspectiveRemovePixelPerCell':8}}]
CELL_SUBPIX=[{'name':f'small_window_c{c}_cell8_subpix',
             'parameters':{**VARIANTS[index]['parameters'],'perspectiveRemovePixelPerCell':8,
                           'cornerRefinementMethod':cv2.aruco.CORNER_REFINE_SUBPIX}}
            for c,index in ((3,2),(7,4))]


def worker(args):
    run,out,variant,dictionary,target_id=args
    cv2.setNumThreads(1)
    metadata=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
    parameters=cv2.aruco.DetectorParameters()
    for key,value in variant['parameters'].items():setattr(parameters,key,value)
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,dictionary)),parameters)
    cap=cv2.VideoCapture(str(run/'camera.avi'));index=0;records=[];elapsed=0.;rejected=0;target=0;wrong=0
    while True:
        ok,bgr=cap.read()
        if not ok:break
        if (index>=len(metadata) or bgr.shape!=(320,320,3)
                or not np.array_equal(bgr[:,:,0],bgr[:,:,1]) or not np.array_equal(bgr[:,:,1],bgr[:,:,2])):
            raise RuntimeError('native camera contract/count mismatch')
        gray=bgr[:,:,0].copy()
        if hashlib.sha256(gray.tobytes()).hexdigest()!=metadata[index]['controller_input_sha256']:
            raise RuntimeError('source sensor pixels changed')
        before=time.perf_counter();corners,ids,quads=detector.detectMarkers(gray);elapsed+=time.perf_counter()-before
        rejected+=len(quads)
        if ids is not None:
            detections=[]
            for code,quad in zip(ids.ravel(),corners):
                quad=quad.reshape(4,2);side=np.linalg.norm(np.roll(quad,-1,axis=0)-quad,axis=1)
                target+=int(code==target_id);wrong+=int(code!=target_id)
                detections.append({'id':int(code),'corners':quad.tolist(),'short_side_px':float(side.min())})
            records.append({'frame':index,'capture_s':metadata[index]['t'],'detections':detections})
        index+=1
    cap.release()
    if index!=len(metadata):raise RuntimeError('missing native frame')
    payload={'run':str(run.resolve()),'variant':variant,'native_frames':index,'target_id_detections':target,
             'wrong_id_detections':wrong,'rejected_quads':rejected,'mean_detection_ms':1000*elapsed/index,
             'detections':records,'truth_input':False,'source_sha_validated_all_frames':True}
    filename=out/(variant['name']+'.json');filename.write_text(json.dumps(payload,indent=2)+'\n')
    print(f'{run.name}/{variant["name"]}: frames={index}/target={target}/wrong={wrong}',flush=True)
    return {'run':str(run),'variant':variant['name'],'result':str(filename)}


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=4)
    p.add_argument('--preset-group',choices=('original8','cell-subpix2'),default='original8')
    a=p.parse_args()
    if not 1<=a.workers<=4:raise ValueError('bounded workers required')
    a.out.mkdir(parents=True,exist_ok=False)
    variants=PARAMETERS if a.preset_group=='original8' else CELL_SUBPIX
    plan={'runs':[str(r.resolve()) for r in a.runs],'workers':a.workers,'variants':variants,
          'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'imported_benchmark_sha256':hashlib.sha256(Path(__file__).with_name('benchmark_mission3_native_detection.py').read_bytes()).hexdigest(),
          'opencv_version':cv2.__version__,'source_hashes':{},
          'scope':'paired failed reused training only; full native pixel SHA replay; dictionary/id/error-correction unchanged; no live adoption; truth only after every worker finishes; detector cell rectification does not add sensor information'}
    jobs=[]
    for run in a.runs:
        check=json.loads((run/'camera-serialization-validation.json').read_text())
        if not check['exact_pixel_replay']:raise RuntimeError('archival sensor pixel validation required')
        plan['source_hashes'][str(run.resolve())]={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in ('camera.avi','camera_frames.jsonl','experiment.json','camera-serialization-validation.json')}
        cfg=json.loads((run/'experiment.json').read_text())['mission'];out=a.out/run.name;out.mkdir()
        jobs.extend((run,out,v,cfg['marker']['dictionary'],cfg['marker']['id']) for v in variants)
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('spawn')) as pool:
        completed=list(pool.map(worker,jobs))
    # Truth is opened only after all image/CPU workers have completed.
    references={};configs={}
    for run in a.runs:
        reference,cfg=build_evaluation_reference(run)
        references[str(run)]=reference;configs[str(run)]=cfg
        (a.out/run.name/'offline-evaluation-reference.json').write_text(json.dumps(reference)+'\n')
    results=[]
    for case in completed:
        payload=json.loads(Path(case['result']).read_text());reference=references[case['run']];cfg=configs[case['run']]
        matched=unmatched=0;confirmation=0;last=None
        for row in payload['detections']:
            for detection in row['detections']:
                if detection['id']!=cfg['marker']['id']:continue
                expected=reference[row['frame']]['projected_target_corners']
                agrees=False
                if expected is not None:
                    expected=np.array(expected);quad=np.array(detection['corners'])
                    side=np.linalg.norm(np.roll(quad,-1,axis=0)-quad,axis=1)
                    eside=np.linalg.norm(np.roll(expected,-1,axis=0)-expected,axis=1).max()
                    agrees=bool(np.linalg.norm(quad.mean(axis=0)-expected.mean(axis=0))<=max(3.,.3*eside) and .5<=side.max()/eside<=2.)
                matched+=int(agrees);unmatched+=int(not agrees)
                if agrees and detection['short_side_px']>=cfg['observer']['coarse_bearing_min_side_px']:
                    if last is not None and row['capture_s']-last<.7:confirmation+=1
                    last=row['capture_s']
        item={k:v for k,v in payload.items() if k!='detections'}
        item.update(projection_matched_target_detections=matched,unmatched_target_detections=unmatched,
                    offline_two_matched_id_within_07s_pairs=confirmation,
                    scope='post-worker conditional projection association; paired training only; pair count omits bearing agreement and is not actual coarse/control/pose success')
        results.append(item)
    (a.out/'summary.json').write_text(json.dumps({'plan':plan,'results':results,'decision':'offline comparison only; no threshold/physics/controller adoption or contact success'},indent=2)+'\n')


if __name__=='__main__':main()
