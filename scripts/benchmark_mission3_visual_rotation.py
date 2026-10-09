#!/usr/bin/env python3
"""원본 영상의 조건부 회전 관측 진단. 참값은 추정 완료 뒤 평가만 한다.

단순 pure-rotation homography prototype이며 Jia/Evans implicit EKF/VIO가
아니다. 카메라 보정/rolling shutter/배경 정지성 미검증: 제어에 미채택.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import cv2
import numpy as np
from mission3.scene import R_HEADING_CAMERA
from mission3.physical_estimator import rotation_rpy


GAPS_S = (.1, .25, .5, 1.)
SETTINGS = {'corners':500, 'quality':.01, 'min_distance_px':5.,
            'forward_backward_px':.75, 'homography_ransac_px':1.,
            'minimum_inliers':20, 'minimum_inlier_fraction':.7,
            'minimum_hull_area_px2':4096., 'max_singular_deviation':.06,
            'rotation_median_error_px':.75, 'estimated_speed_gate_m_s':.03,
            'estimated_tilt_gate_deg':2., 'no_images_resized':True}


def rotation_fit(first, second, K, D, Rbc):
    if first.shape != (320,320) or second.shape != (320,320):
        raise ValueError('native mono320 contract required')
    p=cv2.goodFeaturesToTrack(first,maxCorners=SETTINGS['corners'],
        qualityLevel=SETTINGS['quality'],minDistance=SETTINGS['min_distance_px'])
    if p is None or len(p)<SETTINGS['minimum_inliers']:return {'status':'too_few_features'}
    q,st,_=cv2.calcOpticalFlowPyrLK(first,second,p,None,winSize=(21,21),maxLevel=3)
    if q is None:return {'status':'lk_failed'}
    back,bst,_=cv2.calcOpticalFlowPyrLK(second,first,q,None,winSize=(21,21),maxLevel=3)
    if back is None:return {'status':'backward_lk_failed'}
    good=(st.ravel()!=0)&(bst.ravel()!=0)&(np.linalg.norm(p-back,axis=2).ravel()<SETTINGS['forward_backward_px'])
    good &= np.all((q.reshape(-1,2)>=1.)&(q.reshape(-1,2)<=318.),axis=1)
    p,q=p[good].reshape(-1,2),q[good].reshape(-1,2)
    if len(p)<SETTINGS['minimum_inliers']:return {'status':'too_few_bidirectional_matches','matches':len(p)}
    a=cv2.undistortPoints(p[:,None,:],K,D).reshape(-1,2)
    b=cv2.undistortPoints(q[:,None,:],K,D).reshape(-1,2)
    H,mask=cv2.findHomography(a,b,cv2.RANSAC,SETTINGS['homography_ransac_px']/K[0,0])
    if H is None:return {'status':'homography_failed'}
    m=mask.ravel()!=0;n=int(m.sum())
    if n<SETTINGS['minimum_inliers'] or n/len(p)<SETTINGS['minimum_inlier_fraction']:
        return {'status':'inlier_gate','matches':len(p),'inliers':n}
    area=cv2.contourArea(cv2.convexHull(p[m].astype(np.float32)))
    if area<SETTINGS['minimum_hull_area_px2']:return {'status':'spatial_coverage_gate','inliers':n,'hull_area_px2':area}
    det=float(np.linalg.det(H))
    if not math.isfinite(det) or det<=0:return {'status':'nonpositive_homography_determinant'}
    H=H/np.cbrt(det)
    U,s,Vt=np.linalg.svd(H)
    deviation=float(np.max(np.abs(s-1.)))
    if deviation>SETTINGS['max_singular_deviation']:
        return {'status':'nonrotation_homography','singular_deviation':deviation}
    R=U@Vt
    if np.linalg.det(R)<0:return {'status':'reflection'}
    rays=np.c_[a[m],np.ones(n)]@R.T
    if np.any(rays[:,2]<=0):return {'status':'negative_rotation_depth'}
    pixels=cv2.projectPoints(rays,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
    residual=float(np.median(np.linalg.norm(pixels-q[m],axis=1)))
    if residual>SETTINGS['rotation_median_error_px']:
        return {'status':'rotation_pixel_residual','median_error_px':residual}
    Rbody=Rbc@R@Rbc.T
    return {'status':'rotation_candidate','yaw_increment_rad':-math.atan2(Rbody[1,0],Rbody[0,0]),
            'rotation_body_2_from_1':Rbody.tolist(),'inliers':n,'matches':len(p),
            'hull_area_px2':area,'median_error_px':residual,'singular_deviation':deviation}


def worker(job):
    run,out,gap=job;cv2.setNumThreads(1);cv2.setRNGSeed(0)
    cfg=json.loads((run/'experiment.json').read_text())['physical']
    lens=cfg['synthetic_lens'];K=np.array(lens['K']);D=np.array(lens['distortion'])
    Rbc=rotation_rpy(*np.deg2rad(cfg['mounts']['camera_rpy_offset_deg']))@R_HEADING_CAMERA
    rows=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
    commands=[json.loads(s) for s in (run/'commands.jsonl').read_text().splitlines()]
    times=np.array([r['t'] for r in commands])
    def own_at(t):return commands[max(0,int(np.searchsorted(times,t,side='right')-1))]
    cap=cv2.VideoCapture(str(run/'camera.avi'));images=[]
    while True:
        ok,bgr=cap.read()
        if not ok:break
        if bgr.shape!=(320,320,3) or not np.array_equal(bgr[:,:,0],bgr[:,:,1]) or not np.array_equal(bgr[:,:,1],bgr[:,:,2]):
            raise RuntimeError('native mono channel serialization contract violated')
        images.append(bgr[:,:,0].copy())
    cap.release()
    if len(images)!=len(rows):raise RuntimeError('camera frame/timestamp count differs')
    lag=max(1,round(gap*20.));records=[]
    for i in range(lag,len(rows),lag):
        t0,t1=rows[i-lag]['t'],rows[i]['t'];c0,c1=own_at(t0),own_at(t1)
        record={'t0':t0,'t1':t1,'dt':t1-t0,'phase0':c0['phase'],'phase1':c1['phase']}
        if c0['phase'] not in ('ENTRY_SCAN','ENTRY_SEARCH') or c1['phase']!=c0['phase']:
            record['status']='outside_unobserved_search_phase'
        else:
            start=int(np.searchsorted(times,t0,side='right')-1);end=int(np.searchsorted(times,t1,side='right'))
            samples=commands[max(0,start):end]
            speed=max(np.linalg.norm(s['own']['velocity_world_m_s']) for s in samples)
            tilt=max(math.hypot(*s['own'].get('rpy_rad',[0,0,0])[:2]) for s in samples)
            if speed>SETTINGS['estimated_speed_gate_m_s'] or tilt>math.radians(SETTINGS['estimated_tilt_gate_deg']):
                record.update(status='translation_or_tilt_excludes_pure_rotation',estimated_max_speed_m_s=float(speed))
            else:
                record.update(rotation_fit(images[i-lag],images[i],K,D,Rbc))
                if record['status']=='rotation_candidate':
                    delta=c1['own']['heading_rad']-c0['own']['heading_rad']
                    delta=math.atan2(math.sin(delta),math.cos(delta))
                    record['gyro_pc_heading_increment_rad']=delta
                    record['gyro_minus_visual_rate_deg_s']=math.degrees((delta-record['yaw_increment_rad'])/(t1-t0))
        records.append(record)
    name=f'{run.name}-gap{gap:g}'
    result={'run':str(run.resolve()),'gap_s':gap,'settings':SETTINGS,'records':records,
        'scope':'conditional rotation candidates only; no truth used; K/D/extrinsics assumed, global-frame render, not gyro calibration or control adoption'}
    (out/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    verdicts={}
    for r in records:verdicts[r['status']]=verdicts.get(r['status'],0)+1
    rates=[r['gyro_minus_visual_rate_deg_s'] for r in records if r['status']=='rotation_candidate']
    summary={'run':run.name,'gap_s':gap,'result':name+'.json','pairs':len(records),'verdicts':verdicts,
             'median_gyro_minus_visual_rate_deg_s':float(np.median(rates)) if rates else None}
    print(json.dumps(summary),flush=True)
    return summary


def evaluate_after_estimation(run, result):
    # Truth file is opened only after all image/sensor-only workers have ended.
    truth=[json.loads(s) for s in (run/'truth.jsonl').read_text().splitlines()]
    times=np.array([r['t'] for r in truth]);errors=[];records=[]
    for r in result['records']:
        if r['status']!='rotation_candidate':continue
        a=truth[min(np.searchsorted(times,r['t0']),len(truth)-1)]['aircraft']
        b=truth[min(np.searchsorted(times,r['t1']),len(truth)-1)]['aircraft']
        Rtrue=rotation_rpy(*b['rpy_rad']).T@rotation_rpy(*a['rpy_rad'])
        Rest=np.array(r['rotation_body_2_from_1'])
        rotation_error=math.degrees(math.acos(float(np.clip((np.trace(Rest@Rtrue.T)-1)/2,-1,1))))
        yaw=b['rpy_rad'][2]-a['rpy_rad'][2];yaw=math.atan2(math.sin(yaw),math.cos(yaw))
        rate_error=math.degrees((r['yaw_increment_rad']-yaw)/r['dt'])
        errors.append(rate_error);records.append({'t0':r['t0'],'t1':r['t1'],'rotation_error_deg':rotation_error,'visual_yaw_rate_error_deg_s':rate_error})
    return {'run':run.name,'gap_s':result['gap_s'],'candidate_pairs':len(records),
        'median_visual_yaw_rate_error_deg_s':float(np.median(errors)) if errors else None,
        'visual_yaw_rate_rmse_deg_s':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        'records':records,'scope':'offline model-truth comparison, not calibrated confidence, no control input'}


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path)
    p.add_argument('--out',required=True,type=Path);p.add_argument('--workers',type=int,default=4)
    a=p.parse_args()
    if not 1<=a.workers<=4:raise ValueError('bounded1..4workers required')
    a.out.mkdir(parents=True,exist_ok=False)
    hashes={str(run.resolve()):{name:hashlib.sha256((run/name).read_bytes()).hexdigest()
        for name in ['camera.avi','camera_frames.jsonl','commands.jsonl','experiment.json']} for run in a.runs}
    plan={'runs':[str(r.resolve()) for r in a.runs],'gaps_s':GAPS_S,'settings':SETTINGS,'input_sha256':hashes,
          'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'opencv_version':cv2.__version__,
          'workers':a.workers,'scope':'reused failed-training records; no resize/scene/gyro-noise or gate change, no calibration adopted'}
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    jobs=[(run,a.out,gap) for run in a.runs for gap in GAPS_S]
    with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('spawn')) as pool:
        summaries=list(pool.map(worker,jobs))
    evaluations=[]
    for s in summaries:
        result=json.loads((a.out/s['result']).read_text())
        evaluations.append(evaluate_after_estimation(Path(result['run']),result))
    (a.out/'rotation-evaluation.json').write_text(json.dumps(evaluations,indent=2)+'\n')
    compact=[{k:v for k,v in e.items() if k!='records'} for e in evaluations]
    (a.out/'summary.json').write_text(json.dumps({'plan':plan,'results':summaries,'offline_evaluation':compact},indent=2)+'\n')


if __name__=='__main__':main()
