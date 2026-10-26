#!/usr/bin/env python3
"""Short image-only bearing continuity prototype, not online ArUco decoding.

Native pixels and previously decoded target corners only. Lucas-Kanade forward/
backward + partial affine consistency are S diagnostics, not MedianFlow/TLD or
identity re-decoding. Truth is read only after all sensor workers complete.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import cv2
import numpy as np
from benchmark_mission3_native_detection import build_evaluation_reference


def grid(quad):
    unit=np.array([[0,0],[1,0],[1,1],[0,1]],np.float32)
    points=np.array([[u,v] for v in (.15,.5,.85) for u in (.15,.5,.85)],np.float32)
    H=cv2.getPerspectiveTransform(unit,np.asarray(quad,np.float32))
    return cv2.perspectiveTransform(points.reshape(-1,1,2),H)


def advance(previous,current,points,quad,window):
    opts={'winSize':(window,window),'maxLevel':2,
          'criteria':(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01),'minEigThreshold':1e-4}
    tracked,status,_=cv2.calcOpticalFlowPyrLK(previous,current,points,None,**opts)
    if tracked is None:return None,'forward_missing',{}
    back,reverse,_=cv2.calcOpticalFlowPyrLK(current,previous,tracked,None,**opts)
    if back is None:return None,'backward_missing',{}
    error=np.linalg.norm(back-points,axis=2).ravel()
    good=(status.ravel()==1)&(reverse.ravel()==1)&np.isfinite(error)&(error<=.5)
    if np.count_nonzero(good)<6:return None,'forward_backward_reject',{'valid_points':int(good.sum())}
    before=points.reshape(-1,2)[good];after=tracked.reshape(-1,2)[good]
    A,inliers=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=.75,maxIters=500,confidence=.99)
    if A is None or np.mean(inliers)<.75:return None,'affine_inlier_reject',{}
    residual=np.linalg.norm(before@A[:,:2].T+A[:,2]-after,axis=1)
    scale=float(np.hypot(A[0,0],A[1,0]))
    if np.median(residual)>.5 or not .8<=scale<=1.25:return None,'affine_motion_reject',{'scale':scale}
    newquad=np.asarray(quad)@A[:,:2].T+A[:,2]
    sides=np.linalg.norm(np.roll(newquad,-1,axis=0)-newquad,axis=1)
    if np.any(newquad<3) or np.any(newquad>316) or sides.min()<7 or sides.max()>36:
        return None,'native_edge_or_size_reject',{'min_side_px':float(sides.min())}
    # Previous-image patch must retain photometric appearance under the fitted
    # transform. This is diagnostic image warping, never input enlargement.
    low=np.maximum(np.floor(np.min(quad,axis=0)-2).astype(int),0)
    high=np.minimum(np.ceil(np.max(quad,axis=0)+3).astype(int),320)
    yy,xx=np.mgrid[low[1]:high[1],low[0]:high[0]]
    mapped=np.stack([xx,yy],axis=-1)@A[:,:2].T+A[:,2]
    warped=cv2.remap(current,mapped[:,:,0].astype(np.float32),mapped[:,:,1].astype(np.float32),cv2.INTER_LINEAR)
    template=previous[low[1]:high[1],low[0]:high[0]].astype(float)
    u=template-template.mean();v=warped.astype(float)-warped.mean()
    norm=np.linalg.norm(u)*np.linalg.norm(v);ncc=float(np.sum(u*v)/norm) if norm>1e-6 else -1.
    details={'valid_points':int(good.sum()),'median_fb_px':float(np.median(error[good])),
             'median_fit_px':float(np.median(residual)),'scale':scale,'patch_ncc':ncc,
             'min_side_px':float(sides.min()),'max_side_px':float(sides.max())}
    if ncc<.85:return None,'patch_ncc_reject',details
    return (newquad.astype(np.float32),grid(newquad)),'tracked_partial_bearing',details


def worker(args):
    run,detections,out,window,max_age=args;cv2.setNumThreads(1)
    meta=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
    decoded=json.loads(detections.read_text());target=json.loads((run/'experiment.json').read_text())['mission']['marker']['id']
    byframe={r['frame']:next((d for d in r['detections'] if d['id']==target),None) for r in decoded['detections']}
    cap=cv2.VideoCapture(str(run/'camera.avi'));previous=None;quad=points=None;last_decode=None;records=[];index=0
    while True:
        ok,bgr=cap.read()
        if not ok:break
        gray=bgr[:,:,0].copy()
        if (bgr.shape!=(320,320,3) or not np.array_equal(bgr[:,:,0],bgr[:,:,1]) or not np.array_equal(bgr[:,:,1],bgr[:,:,2])
                or hashlib.sha256(gray.tobytes()).hexdigest()!=meta[index]['controller_input_sha256']):
            raise RuntimeError('native source image mismatch')
        stamp=meta[index]['t'];known=byframe.get(index)
        if known is not None:
            quad=np.array(known['corners'],np.float32);points=grid(quad);last_decode=stamp
            records.append({'frame':index,'capture_s':stamp,'status':'decoded_reset','corners':quad.tolist(),'decode_age_s':0.})
        elif points is not None:
            age=stamp-last_decode
            if age>max_age:
                points=quad=None;records.append({'frame':index,'capture_s':stamp,'status':'decode_age_expired','decode_age_s':age})
            else:
                result,status,details=advance(previous,gray,points,quad,window)
                item={'frame':index,'capture_s':stamp,'status':status,'decode_age_s':age,**details}
                if result is None:points=quad=None
                else:quad,points=result;item['corners']=quad.tolist()
                records.append(item)
        previous=gray;index+=1
    cap.release()
    if index!=len(meta):raise RuntimeError('camera/source count mismatch')
    payload={'run':str(run),'window_px':window,'decode_age_limit_s':max_age,'native_frames':index,'source_sha_all_exact':True,
             'records':records,'scope':'offline image partial bearing prototype only; no new decoded ID/metric pose/vehicle state/control; thresholds S, no truth input'}
    file=out/f'win{window}-age{max_age:g}.json';file.write_text(json.dumps(payload,indent=2)+'\n')
    return str(file)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,action='append',required=True);p.add_argument('--detections',type=Path,action='append',required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if len(a.run)!=len(a.detections):raise ValueError('one frozen decoded source per run')
    a.out.mkdir(parents=True,exist_ok=False);jobs=[]
    plan={'runs':[str(r.resolve()) for r in a.run],'workers':4,'variants':[(7,.5),(7,1.),(15,.5),(15,1.)],
          'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'source_hashes':{},'scope':'reused paired failed training; native source pixels; no live adoption; truth after all workers; source ROI seeded solely from decoded target ID'}
    for run,detections in zip(a.run,a.detections):
        out=a.out/run.name;out.mkdir()
        if not json.loads((run/'camera-serialization-validation.json').read_text())['exact_pixel_replay']:raise RuntimeError('verified FFV1 required')
        plan['source_hashes'][str(run)]={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (run/'camera.avi',run/'camera_frames.jsonl',detections)}
        jobs.extend((run,detections,out,w,age) for w,age in plan['variants'])
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:finished=list(pool.map(worker,jobs))
    references={str(run):build_evaluation_reference(run)[0] for run in a.run}
    summaries=[]
    for filename in finished:
        item=json.loads(Path(filename).read_text());ref=references[item['run']];assess=[];statuses={}
        for record in item['records']:
            statuses[record['status']]=statuses.get(record['status'],0)+1
            if record['status']!='tracked_partial_bearing':continue
            expected=ref[record['frame']]['projected_target_corners'];error=None
            if expected is not None:error=float(np.linalg.norm(np.mean(record['corners'],axis=0)-np.mean(expected,axis=0)))
            assess.append({'capture_s':record['capture_s'],'decode_age_s':record['decode_age_s'],'offline_center_error_px':error,
                           'offline_target_inside_guarded_projection':expected is not None})
        Path(filename).with_suffix('.offline-evaluation.json').write_text(json.dumps(assess,indent=2)+'\n')
        errors=[r['offline_center_error_px'] for r in assess if r['offline_center_error_px'] is not None]
        summaries.append({k:v for k,v in item.items() if k!='records'}|{'status_counts':statuses,'partial_points':len(assess),
             'offline_median_center_error_px':None if not errors else float(np.median(errors)),
             'offline_max_center_error_px':None if not errors else max(errors),
             'partial_points_without_valid_projection':sum(r['offline_center_error_px'] is None for r in assess)})
    (a.out/'summary.json').write_text(json.dumps({'plan':plan,'results':summaries,'decision':'research-only; no new ID/metric/contact success; temporal image association can drift despite forward-backward consistency'},indent=2)+'\n')


if __name__=='__main__':main()
