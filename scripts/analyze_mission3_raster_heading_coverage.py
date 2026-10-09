#!/usr/bin/env python3
"""Offline ideal-odometry raster projections, never decoded frames or flights."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import hashlib
import itertools
import json
import math
import multiprocessing
from pathlib import Path
import shutil

import cv2
import numpy as np

from aruco_pose import pinhole_matrix
from mission3.controller import Mission3
from mission3.scene import R_HEADING_CAMERA, R_VEHICLE_MARKER
from mission3.track import rotation, wrap
from yaw_core import rotation_rpy

VARIANTS=list(itertools.product((6.,8.,10.),(.75,3.,7.5),(.15,),('entry','leg')))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def rows(path):return [json.loads(x) for x in Path(path).open()]


def ideal_path(settings,entry,variant):
    cfg=deepcopy(settings);forward,lateral,speed,gaze=variant
    cfg['control'].update(search_forward_extent_m=forward,search_lateral_extent_m=lateral,
                          search_translation_speed_m_s=speed)
    controller=Mission3(cfg,entry)
    p=np.array([*entry['xy_m'],cfg['entry']['height_m']],float)
    yaw=math.radians(entry['heading_deg']);velocity=np.zeros(3);samples=[];applied_yaw=0.
    for tick in range(15001):
        t=tick*.02
        own={'position_world_m':p.tolist(),'velocity_world_m_s':velocity.tolist(),
             'heading_rad':yaw,'position_std_m':.1}
        command=controller.update(t,own,None)
        if gaze=='leg' and command.phase=='ENTRY_SEARCH':
            error=controller.entry_search_targets[controller.entry_search_index][:2]-p[:2]
            desired=math.atan2(error[1],error[0]) if np.linalg.norm(error)>.1 else yaw
            rate=float(np.clip(1.2*wrap(desired-yaw),-math.radians(10),math.radians(10)))
            command.yaw_rate_rad_s=float(np.clip(rate,applied_yaw-math.radians(20)*.02,
                                               applied_yaw+math.radians(20)*.02))
            controller.last_yaw_rate=command.yaw_rate_rad_s
        applied_yaw=command.yaw_rate_rad_s
        if tick%5==0:samples.append((t,p.copy(),yaw,command.phase))
        velocity=np.array(command.velocity_world_m_s)
        p+=velocity*.02;yaw=wrap(yaw+command.yaw_rate_rad_s*.02)
    return samples


def analyze(name):
    run=Path(name);experiment=json.loads((run/'experiment.json').read_text())
    settings=json.loads((run/'controller-settings.json').read_text())
    entry=next(x for x in settings['entry']['positions'] if x['id']==experiment['entry_id'])
    handoff=json.loads((run/'run_manifest.json').read_text())['handoff_time_s']
    truth=rows(run/'truth.jsonl');ts=np.array([x['t'] for x in truth])
    pad_positions=np.array([x['pad']['center_world_m'] for x in truth])
    vehicle_yaw=np.unwrap([x['vehicle']['heading_rad'] for x in truth])
    cfg=experiment['mission'];mount=experiment['physical']['mounts']
    optics=cfg['camera'].get('optics_sensitivity',{})
    K=np.array(optics.get('K',pinhole_matrix(320,320,cfg['camera']['vertical_fov_deg'])))
    D=np.array(optics.get('distortion',[0.]*5))
    Rbc=rotation_rpy(*np.deg2rad(mount['camera_rpy_offset_deg']))@R_HEADING_CAMERA
    half=cfg['marker']['code_side_m']/2
    local=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
    offset=np.array(cfg['marker']['known_marker_to_pad_vehicle_m']);reports=[]
    for variant in VARIANTS:
        path=ideal_path(settings,entry,variant); times=np.array([x[0]+handoff for x in path])
        assert np.all(times>=ts[0]) and np.all(times<=ts[-1]+.03)
        pads=np.column_stack([np.interp(times,ts,pad_positions[:,i]) for i in range(3)])
        headings=np.interp(times,ts,vehicle_yaw);eligible=[];coarse=[];peak=0.
        for sample,pad,heading in zip(path,pads,headings):
            t,p,yaw,phase=sample;Rwb=rotation(yaw);Rwv=rotation(heading)
            center=pad+Rwv@offset;camera=p+Rwb@mount['camera_position_body_m']
            Rwm=Rwv@R_VEHICLE_MARKER;Rwc=Rwb@Rbc
            corner=(local@Rwm.T+center-camera)@Rwc
            if Rwm[:,2]@(camera-center)<=0 or not np.all(corner[:,2]>.001):continue
            uv=cv2.projectPoints(corner,np.zeros(3),np.zeros(3),K,D)[0].reshape(4,2)
            depth=((center-camera)@Rwc)[2]
            if not(np.all(uv>=3) and np.all(uv<=316) and depth>=cfg['camera']['published_depth_of_field_near_m']):continue
            side=float(np.linalg.norm(uv-np.roll(uv,1,axis=0),axis=1).min());peak=max(peak,side)
            if side>=7:coarse.append(t)
            if side>=18:eligible.append(t)
        reports.append({'forward_m':variant[0],'lateral_m':variant[1],'speed_m_s':variant[2],'gaze_proxy':variant[3],
            'projected_18px_samples':len(eligible),'projected_7px_samples':len(coarse),
            'first_18px_relative_mission_s':eligible[0] if eligible else None,
            'first_7px_relative_mission_s':coarse[0] if coarse else None,'max_eligible_min_side_px':peak})
    return {'run':name,'seed':experiment['seed'],'entry':experiment['entry_id'],
            'original_vehicle_phase_preserved_in_offline_projection':True,'variants':reports,
            'source_sha256':{p:sha(run/p) for p in ('experiment.json','controller-settings.json','truth.jsonl','run_manifest.json')}}


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='append',required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False);shutil.copyfile(__file__,args.out/Path(__file__).name)
    files=[Path(__file__),Path('scripts/mission3/controller.py'),Path('scripts/mission3/scene.py'),Path('scripts/mission3/track.py')]
    identities={str(x):sha(x) for x in files}
    for x in files[1:]:shutil.copyfile(x,args.out/x.name)
    plan={'runs':args.run,'variants_declared_before_processing':VARIANTS,'source_sha256':identities,
       'scope':'Offline S kinematic search paths and saved original vehicle trajectories only, no pixels/decoder/sensors/physics.',
       'limits':['Perfect self-position/heading and direct velocity following replace real odometry/dynamics; optimistic geometry. Leg gaze is an offline proxy, not an implemented controller option.',
                'No marker observations are fed into path generation, so coarse pursuit and post-discovery behavior are excluded.',
                'No occlusion, distortion calibration, PSF, exposure or capture/transport timing validation.',
                '7px/18px projections are opportunities, not actual decoding/pose qualification or landing.',
                'Entry-relative targets ignore track/cart phase; no observation station or initial vehicle phase modification.',
                'Source/runtime qualification and hardware success remain0; all selected seeds are spent development.']}
    (args.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=min(4,len(args.run)),mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(analyze,args.run))
    assert all(sha(path)==value for path,value in identities.items())
    (args.out/'summary.json').write_text(json.dumps({**plan,'cases':results},indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
