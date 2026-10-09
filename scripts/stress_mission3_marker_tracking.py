#!/usr/bin/env python3
"""Offline synthetic identity/occlusion stress on native recorded frame pairs.

Replacements are explicitly synthetic S inputs; recordings are read-only.
Tracking acceptance is never called an ArUco identity decode or flight success.
No simulator truth is read. This probes limitations, not a false-positive rate.
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


MODES = ['baseline','black_occlusion','white_occlusion','gray_occlusion','random_occlusion',
         'half_black_occlusion','redraw_target_id0','replace_wrong_id1','replace_wrong_id2','replace_wrong_id17']


def variant(current, quad, mode, seed):
    if mode == 'baseline': return current.copy()
    mask = np.zeros((320,320), np.uint8)
    cv2.fillConvexPoly(mask, np.rint(quad).astype(np.int32), 255)
    image = current.copy()
    if mode in ('black_occlusion','white_occlusion','gray_occlusion','random_occlusion','half_black_occlusion'):
        mask = cv2.dilate(mask, np.ones((3,3),np.uint8))
        if mode == 'half_black_occlusion':
            mask[:,int(np.mean(quad[:,0])):] = 0
        values = {'black_occlusion':0,'white_occlusion':255,'gray_occlusion':128,'half_black_occlusion':0}
        if mode == 'random_occlusion':
            replacement = np.random.default_rng(seed).integers(0,256,(320,320),dtype=np.uint8)
            image[mask>0] = replacement[mask>0]
        else: image[mask>0] = values[mode]
        return image
    identity = 0 if mode=='redraw_target_id0' else int(mode.rsplit('id',1)[1])
    asset = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),identity,96)
    H = cv2.getPerspectiveTransform(np.array([[0,0],[95,0],[95,95],[0,95]],np.float32),quad.astype(np.float32))
    replacement = cv2.warpPerspective(asset,H,(320,320),flags=cv2.INTER_LINEAR)
    image[mask>0] = replacement[mask>0]
    return image


def worker(args):
    first, second, quad, case, window, mode = args
    cv2.setNumThreads(1)
    prediction, status, details = advance(first,second,grid(quad),quad,window)
    if prediction is None:
        return {'case':case,'window_px':window,'mode':mode,'eligible_baseline':False,'status':status}
    changed = variant(second,prediction[0],mode,case['first_frame'])
    result, status, details = advance(first,changed,grid(quad),quad,window)
    return {'case':case,'window_px':window,'mode':mode,'eligible_baseline':True,
            'accepted_unconfirmed_continuity':result is not None,'status':status,**details,
            'original_second_sha256':hashlib.sha256(second.tobytes()).hexdigest(),
            'stress_second_sha256':hashlib.sha256(changed.tobytes()).hexdigest(),
            'corners':None if result is None else result[0].tolist()}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tracking',type=Path,action='append',required=True)
    p.add_argument('--out',type=Path,required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    jobs=[];sources={};pair_count=0
    for tracking in a.tracking:
        d=json.loads(tracking.read_text());run=Path(d['run'])
        records=d['records'];pairs=[]
        for before,after in zip(records,records[1:]):
            if (before['status']=='decoded_reset' and after['status']=='tracked_partial_bearing'
                    and after['frame']==before['frame']+1):pairs.append((before,after))
        needed={q['frame'] for pair in pairs for q in pair}
        meta=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
        images={};capture=cv2.VideoCapture(str(run/'camera.avi'));index=0
        while True:
            ok,bgr=capture.read()
            if not ok:break
            if index in needed:
                if (bgr.shape!=(320,320,3) or not np.array_equal(bgr[:,:,0],bgr[:,:,1])
                        or not np.array_equal(bgr[:,:,1],bgr[:,:,2])):raise RuntimeError('native format mismatch')
                gray=bgr[:,:,0].copy()
                if hashlib.sha256(gray.tobytes()).hexdigest()!=meta[index]['controller_input_sha256']:
                    raise RuntimeError('source pixel mismatch')
                images[index]=gray
            index+=1
        capture.release()
        if index!=len(meta) or set(images)!=needed:raise RuntimeError('source count mismatch')
        sources[str(run)]={str(f):hashlib.sha256(f.read_bytes()).hexdigest()
                          for f in (tracking,run/'camera.avi',run/'camera_frames.jsonl')}
        for before,after in pairs:
            case={'run':str(run),'first_frame':before['frame'],'second_frame':after['frame'],
                  'first_capture_s':before['capture_s'],'second_capture_s':after['capture_s']}
            pair_count+=1
            jobs.extend((images[before['frame']],images[after['frame']],np.array(before['corners'],np.float32),case,w,mode)
                        for w in (7,15) for mode in MODES)
    plan={'pairs':pair_count,'planned_jobs':len(jobs),'workers':4,'modes':MODES,'windows_px':[7,15],
          'source_used_pixels_sha_exact':True,'source_hashes':sources,
          'code_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                         (Path(__file__),Path('scripts/benchmark_mission3_marker_tracking.py'))},
          'scope':'synthetic native-frame S counterexamples, two reused failed training scenes; no truth, no hardware appearance certification, no flight control'}
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(worker,jobs))
    (a.out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    summary=[]
    for window in (7,15):
        for mode in MODES:
            rows=[r for r in results if r['window_px']==window and r['mode']==mode]
            eligible=[r for r in rows if r['eligible_baseline']]
            summary.append({'window_px':window,'mode':mode,'planned_pairs':len(rows),
                            'eligible_baseline_pairs':len(eligible),
                            'accepted_unconfirmed_continuity':sum(r['accepted_unconfirmed_continuity'] for r in eligible),
                            'rejected':sum(not r['accepted_unconfirmed_continuity'] for r in eligible)})
    payload={'plan':plan,'completed_jobs':len(results),'results':summary,
             'decision':'offline only; FB/NCC continuity is not identity re-decoding. Any accepted wrong-ID or occluded variant is a limitation, not a successful target observation. Synthetic pair counts cannot estimate real-world false-association probability.'}
    (a.out/'summary.json').write_text(json.dumps(payload,indent=2)+'\n')
    print(json.dumps(payload,indent=2))


if __name__ == '__main__':main()
