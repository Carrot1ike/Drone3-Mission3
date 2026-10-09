#!/usr/bin/env python3
"""Frozen native-image replay. Independent CPU processes, no live control."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import time
import cv2
import numpy as np
from mission3.scene import R_HEADING_CAMERA, R_VEHICLE_MARKER
from mission3.track import rotation
from mission3.physical_estimator import rotation_rpy


VARIANTS=[
    {'name':'current_subpix','parameters':{'cornerRefinementMethod':cv2.aruco.CORNER_REFINE_SUBPIX}},
    {'name':'default_no_refinement','parameters':{}},
    {'name':'small_window_c3','parameters':{'adaptiveThreshWinSizeMin':3,'adaptiveThreshWinSizeMax':13,
        'adaptiveThreshWinSizeStep':2,'adaptiveThreshConstant':3.,'minMarkerPerimeterRate':.02}},
    {'name':'small_window_c0','parameters':{'adaptiveThreshWinSizeMin':3,'adaptiveThreshWinSizeMax':13,
        'adaptiveThreshWinSizeStep':2,'adaptiveThreshConstant':0.,'minMarkerPerimeterRate':.02}},
    {'name':'small_window_c7','parameters':{'adaptiveThreshWinSizeMin':3,'adaptiveThreshWinSizeMax':13,
        'adaptiveThreshWinSizeStep':2,'adaptiveThreshConstant':7.,'minMarkerPerimeterRate':.02}},
    {'name':'april_corner','parameters':{'cornerRefinementMethod':cv2.aruco.CORNER_REFINE_APRILTAG}},
]


def build_evaluation_reference(run):
    """Truth is used only here/after detection to assess matching pixels."""
    experiment=json.loads((run/'experiment.json').read_text())
    cfg,physical=experiment['mission'],experiment['physical']
    lens=physical['synthetic_lens'];K=np.array(lens['K']);D=np.array(lens['distortion'])
    Rbc=rotation_rpy(*np.deg2rad(physical['mounts']['camera_rpy_offset_deg']))@R_HEADING_CAMERA
    half=cfg['marker']['code_side_m']/2
    points=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
    slopes=np.tan(np.deg2rad([cfg['camera']['horizontal_fov_deg'],cfg['camera']['vertical_fov_deg']])/2)
    truth=[json.loads(line) for line in (run/'truth.jsonl').open()]
    stamps=np.array([r['t'] for r in truth])
    frames=[json.loads(line) for line in (run/'camera_frames.jsonl').open()]
    reference=[]
    for frame in frames:
        stamp=frame['t'];i=min(np.searchsorted(stamps,stamp),len(truth)-1);r=truth[i]
        if abs(r['t']-stamp)>1e-5:raise RuntimeError('camera/truth evaluation timestamps disagree')
        Rb=rotation_rpy(*r['aircraft']['rpy_rad']);Rv=rotation(r['vehicle']['heading_rad'])
        camera=np.array(r['aircraft']['center_world_m'])+Rb@np.array(physical['mounts']['camera_position_body_m'])
        marker=np.array(r['pad']['center_world_m'])+Rv@np.array(cfg['marker']['known_marker_to_pad_vehicle_m'])
        item={'t':stamp,'projected_target_corners':None}
        Rcm=Rbc.T@Rb.T@Rv@R_VEHICLE_MARKER;translation=Rbc.T@Rb.T@(marker-camera)
        camera_points=points@Rcm.T+translation
        if (np.dot(Rv@np.array([-1.,0.,0.]),camera-marker)>0 and np.all(camera_points[:,2]>=.30)
                and np.all(np.abs(camera_points[:,:2]/camera_points[:,2,None])<=slopes)):
            corners=cv2.projectPoints(points,cv2.Rodrigues(Rcm)[0],translation,K,D)[0].reshape(4,2)
            if np.all(corners>=0) and np.all(corners<=319):item['projected_target_corners']=corners.tolist()
        reference.append(item)
    return reference,cfg


def worker(args):
    run,out,variant,reference,target_id,dictionary=args
    cv2.setNumThreads(1)
    parameters=cv2.aruco.DetectorParameters()
    for key,value in variant['parameters'].items():setattr(parameters,key,value)
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,dictionary)),parameters)
    capture=cv2.VideoCapture(str(run/'camera.avi'))
    records=[];index=0;wrong_id=target=matched=unmatched=rejected=0;elapsed=0.
    while True:
        ok,bgr=capture.read()
        if not ok:break
        if bgr.shape!=(320,320,3) or not np.array_equal(bgr[:,:,0],bgr[:,:,1]) or not np.array_equal(bgr[:,:,1],bgr[:,:,2]):
            raise RuntimeError('serialized native mono frame contract violated; no resizing/conversion')
        gray=bgr[:,:,0].copy()
        before=time.perf_counter();corners,ids,rejected_corners=detector.detectMarkers(gray);elapsed+=time.perf_counter()-before
        rejected+=len(rejected_corners)
        # Evaluation reference is consulted AFTER detection, never passed to detector.
        reference_corners=reference[index]['projected_target_corners']
        detections=[]
        if ids is not None:
            for code,quad in zip(ids.ravel(),corners):
                quad=quad.reshape(4,2);side=np.linalg.norm(np.roll(quad,-1,axis=0)-quad,axis=1)
                record={'id':int(code),'corners':quad.tolist(),'short_side_px':float(side.min()),'matches_offline_projection':False}
                if code!=target_id:wrong_id+=1
                else:
                    target+=1
                    if reference_corners is not None:
                        expected=np.array(reference_corners);distance=np.linalg.norm(expected.mean(axis=0)-quad.mean(axis=0))
                        expected_side=np.linalg.norm(np.roll(expected,-1,axis=0)-expected,axis=1).max()
                        record['matches_offline_projection']=bool(distance<=max(3.,.3*expected_side)
                            and .5 <= side.max()/expected_side <= 2.)
                    if record['matches_offline_projection']:matched+=1
                    else:unmatched+=1
                detections.append(record)
        if detections:records.append({'frame':index,'t':reference[index]['t'],'detections':detections})
        index+=1
    capture.release()
    if index!=len(reference):raise RuntimeError('native frame count mismatch')
    result={'variant':variant,'native_frames':index,'target_id_detections':target,'projection_matched_target_detections':matched,
        'unmatched_target_detections':unmatched,'wrong_id_detections':wrong_id,'rejected_quads':rejected,
        'total_detector_cpu_wall_s':elapsed,'mean_detection_ms':1000*elapsed/index,'detections':records,
        'scope':'offline native replay; projection match is conditional-model evaluation only, not calibrated confidence'}
    (out/(variant['name']+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(f"{variant['name']}: frames={index}, target={target}, matched={matched}, unmatched={unmatched}, wrong={wrong_id}",flush=True)
    return {key:value for key,value in result.items() if key!='detections'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=4);a=parser.parse_args()
    if a.out.exists():raise RuntimeError('preserve existing benchmark')
    a.out.mkdir(parents=True)
    reference,cfg=build_evaluation_reference(a.run)
    (a.out/'evaluation-reference.json').write_text(json.dumps(reference)+'\n')
    plan={'run':str(a.run.resolve()),'native_camera_sha256':hashlib.sha256((a.run/'camera.avi').read_bytes()).hexdigest(),
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'opencv_version':cv2.__version__,
        'workers':a.workers,'variants':VARIANTS,'scope':'training comparison; no frame resize, dictionary or scene changes; raw mono extraction verified'}
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    jobs=[(a.run,a.out,variant,reference,cfg['marker']['id'],cfg['marker']['dictionary']) for variant in VARIANTS]
    with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(worker,jobs))
    (a.out/'summary.json').write_text(json.dumps({'plan':plan,'results':results},indent=2)+'\n')


if __name__=='__main__':main()
