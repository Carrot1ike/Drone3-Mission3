#!/usr/bin/env python3
"""Lossless-record native corner geometry; no new detector/flight/state update."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import numpy as np
from mission3.height_bearing_geometry import relative_marker, conditional_covariance
from mission3.scene import R_HEADING_CAMERA
from mission3.track import rotation
from yaw_core import rotation_rpy

SENSITIVITIES=[{'name':'low','pixel_std':.5,'height_std_m':.01,'tilt_std_deg':.5},
               {'name':'middle','pixel_std':1.,'height_std_m':.03,'tilt_std_deg':1.},
               {'name':'high','pixel_std':2.,'height_std_m':.05,'tilt_std_deg':2.}]


def worker(job):
    run,out=job
    cfg=json.loads((run/'experiment.json').read_text());mission,physical=cfg['mission'],cfg['physical']
    lens=physical['observer_optics_sensitivity'];K=np.array(lens['K']);D=np.array(lens['distortion'])
    Rbc=rotation_rpy(*np.deg2rad(physical['mounts']['camera_rpy_offset_deg']))@R_HEADING_CAMERA
    mount=physical['mounts']['camera_position_body_m']
    marker_height=mission['landing_geometry']['pad_height_m']+mission['marker']['known_marker_to_pad_vehicle_m'][2]
    commands=[json.loads(s) for s in (run/'commands.jsonl').read_text().splitlines()]
    stamps=np.array([r['t'] for r in commands]);results=[]
    observations=[json.loads(s) for s in (run/'observations.jsonl').read_text().splitlines()]
    for obs in observations:
        if 'corners' not in obs:continue
        t=obs['capture_stamp_s'];i=max(0,int(np.searchsorted(stamps,t,side='right')-1));own=commands[i]['own']
        item={'capture_s':t,'receipt_s':obs['receipt_s'],'native_min_side_px':obs.get('min_side_px'),
              'original_observer_reason':obs['reason'],'original_metric_valid':obs['valid']}
        if (t-stamps[i]>.03 or own['surface_hypothesis']!='static_floor_candidate'
            or not own.get('range_accurate',False) or own.get('range_m') is None):
            item['status']='floor_tof_height_reference_unavailable'
        elif obs.get('min_side_px',0.)<mission['observer']['coarse_bearing_min_side_px']:
            item['status']='below_original_decoded_coarse_side_gate'
        else:
            try:
                # Use floor-ToF height, not visually anchored own z. Datum/tilt
                # dependence and nearest-return sensor model remain conditional.
                h=float(own['range_m']);roll,pitch=own['rpy_rad'][:2]
                point,info=relative_marker(obs['corners'],h,roll,pitch,marker_height,K,D,Rbc,mount)
                sensitivity=[]
                for c in SENSITIVITIES:
                    P,_=conditional_covariance(obs['corners'],h,roll,pitch,marker_height,K,D,Rbc,mount,
                                              c['pixel_std'],c['height_std_m'],c['tilt_std_deg'])
                    sensitivity.append(dict(c,relative_xy_covariance_m2=P.tolist(),
                        maximum_relative_xy_std_m=float(math.sqrt(np.linalg.eigvalsh(P).max()))))
                item.update(status='conditional_height_bearing',relative_marker_heading_frame_m=point.tolist(),
                    floor_tof_body_height_m=h,own_pose_reference={'xy_m':own['position_world_m'][:2],'heading_rad':own['heading_rad']},
                    geometry=info,sensitivities=sensitivity)
            except (ValueError, np.linalg.LinAlgError) as exc:
                item.update(status='geometry_rejected',reason=str(exc))
        results.append(item)
    target=out/(run.name+'.json');target.write_text(json.dumps({'run':str(run.resolve()),'results':results},indent=2)+'\n')
    counts={}
    for item in results:counts[item['status']]=counts.get(item['status'],0)+1
    return {'run':run.name,'result':target.name,'decoded_frames':len(results),'status_counts':counts}


def evaluate_only_after_workers(run,result):
    cfg=json.loads((run/'experiment.json').read_text())['mission'];offset=np.array(cfg['marker']['known_marker_to_pad_vehicle_m'])
    truth=[json.loads(s) for s in (run/'truth.jsonl').read_text().splitlines()];times=np.array([r['t'] for r in truth]);errors=[]
    for obs in result['results']:
        if obs['status']!='conditional_height_bearing':continue
        row=truth[min(np.searchsorted(times,obs['capture_s']),len(truth)-1)]
        vehicle=row['vehicle'];air=row['aircraft'];marker=np.array(row['pad']['center_world_m'])+rotation(vehicle['heading_rad'])@offset
        actual=(rotation(air['rpy_rad'][2]).T@(marker-np.array(air['center_world_m'])))[:2]
        fitted=np.array(obs['relative_marker_heading_frame_m'])[:2];reference=obs['own_pose_reference']
        worldfit=np.array(reference['xy_m'])+(rotation(reference['heading_rad'])@np.r_[fitted,0.])[:2]
        errors.append({'capture_s':obs['capture_s'],'relative_xy_error_m':float(np.linalg.norm(fitted-actual)),
            'world_xy_error_including_unresolved_own_pose_m':float(np.linalg.norm(worldfit-marker[:2])),
            'original_metric_valid':obs['original_metric_valid'],
            'middle_relative_std_m':obs['sensitivities'][1]['maximum_relative_xy_std_m']})
    return {'run':run.name,'conditional_points':len(errors),'previously_nonmetric_points':sum(not r['original_metric_valid'] for r in errors),
        'median_relative_xy_error_m':float(np.median([r['relative_xy_error_m'] for r in errors])) if errors else None,
        'max_relative_xy_error_m':max((r['relative_xy_error_m'] for r in errors),default=None),
        'median_world_xy_error_m':float(np.median([r['world_xy_error_including_unresolved_own_pose_m'] for r in errors])) if errors else None,
        'median_middle_relative_std_m':float(np.median([r['middle_relative_std_m'] for r in errors])) if errors else None,'records':errors}


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=4)
    a=p.parse_args()
    if not 1<=a.workers<=4:raise ValueError('bounded1..4workers required')
    a.out.mkdir(parents=True,exist_ok=False)
    for run in a.runs:
        if not json.loads((run/'camera-serialization-validation.json').read_text())['exact_pixel_replay']:
            raise ValueError('verified lossless capture contract required')
    plan={'runs':[str(r.resolve()) for r in a.runs],'workers':a.workers,'sensitivities':SENSITIVITIES,
          'input_sha256':{str(r.resolve()):{n:hashlib.sha256((r/n).read_bytes()).hexdigest() for n in ('observations.jsonl','commands.jsonl','experiment.json','camera-serialization-validation.json')} for r in a.runs},
          'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'geometry_sha256':hashlib.sha256((Path(__file__).parent/'mission3/height_bearing_geometry.py').read_bytes()).hexdigest(),
          'scope':'known marker-height conditional geometry from existing decoded pixels+qualified floor ToF/IMU; original detector/pose gates preserved, no vehicle heading/speed/pad estimate, no control/state update. S error sensitivities are not hardware probabilities; mount/lens/ToF transfer correlations U.'}
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(worker,[(r,a.out) for r in a.runs]))
    evaluated=[evaluate_only_after_workers(run,json.loads((a.out/result['result']).read_text())) for run,result in zip(a.runs,results)]
    (a.out/'offline-evaluation.json').write_text(json.dumps(evaluated,indent=2)+'\n')
    (a.out/'summary.json').write_text(json.dumps({'plan':plan,'results':results,'offline_evaluation':[{k:v for k,v in e.items() if k!='records'} for e in evaluated],'decision':'research only; no live adoption or absolute yaw/landing success claim'},indent=2)+'\n')


if __name__=='__main__':main()
