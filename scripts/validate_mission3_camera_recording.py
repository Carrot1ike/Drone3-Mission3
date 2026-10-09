#!/usr/bin/env python3
"""Checksum validation of the saved mono input; no truth or control changes."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from mission3.recording import frame_sha256


def validate(run):
    rows=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
    cap=cv2.VideoCapture(str(run/'camera.avi'));index=0;missing=0;different=[]
    while True:
        ok,bgr=cap.read()
        if not ok:break
        if index>=len(rows):raise RuntimeError('saved camera has extra frames')
        if bgr.shape!=(320,320,3) or not np.array_equal(bgr[:,:,0],bgr[:,:,1]) or not np.array_equal(bgr[:,:,1],bgr[:,:,2]):
            raise RuntimeError('native grayscale contract violated')
        expected=rows[index].get('controller_input_sha256')
        if expected is None:missing+=1
        elif frame_sha256(bgr[:,:,0])!=expected:different.append(index)
        index+=1
    cap.release()
    return {'frames':index,'metadata_frames':len(rows),'missing_source_hashes':missing,
            'mismatched_frame_indices':different,'exact_pixel_replay':index==len(rows) and missing==0 and not different,
            'scope':'archival byte fidelity only, not camera hardware/transport/rolling-shutter validation'}


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path)
    a=p.parse_args();out=a.out or a.run/'camera-serialization-validation.json'
    if out.exists():raise RuntimeError('preserve prior validation')
    result=validate(a.run);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['exact_pixel_replay'] else 2


if __name__=='__main__':raise SystemExit(main())
