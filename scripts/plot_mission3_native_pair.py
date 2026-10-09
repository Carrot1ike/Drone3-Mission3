#!/usr/bin/env python3
"""Native 센서 replay 비교 그림. 확대 crop은 표시 전용이며 detector 입력 아님."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
from PIL import Image,ImageDraw,ImageFont


def plot(runs,out,times):
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
    canvas=Image.new('RGB',(1320,1150),'#101820');draw=ImageDraw.Draw(canvas)
    draw.text((20,10),'Native 320x320 sensor replay: same training seed433 / v4 vs v5',font=font,fill='white')
    draw.text((20,35),'NO landing. Display crop uses nearest enlargement ONLY; decoder always reads original 320x320.',font=small,fill='#cdd5dd')
    provenance=[]
    for row,run in enumerate(runs):
        validation=json.loads((run/'camera-serialization-validation.json').read_text())
        if not validation['exact_pixel_replay']:raise RuntimeError('native archive not verified')
        metadata=[json.loads(s) for s in (run/'camera_frames.jsonl').read_text().splitlines()]
        observations=[json.loads(s) for s in (run/'observations.jsonl').read_text().splitlines()]
        indices={min(range(len(metadata)),key=lambda i:abs(metadata[i]['t']-t)):t for t in times}
        if len(indices)!=len(times):raise RuntimeError('distinct timestamps required')
        frames={};cap=cv2.VideoCapture(str(run/'camera.avi'))
        for i in range(max(indices)+1):
            ok,bgr=cap.read()
            if not ok:raise RuntimeError('camera frame missing')
            if i in indices:
                if bgr.shape!=(320,320,3) or not (bgr[:,:,0]==bgr[:,:,1]).all() or not (bgr[:,:,1]==bgr[:,:,2]).all():
                    raise RuntimeError('native mono contract violated')
                gray=bgr[:,:,0]
                if hashlib.sha256(gray.tobytes()).hexdigest()!=metadata[i]['controller_input_sha256']:
                    raise RuntimeError('source pixel SHA mismatch')
                frames[indices[i]]=(metadata[i]['t'],Image.fromarray(gray))
        cap.release()
        y=80+row*515
        draw.text((20,y),'v4: waits for second ID' if row==0 else 'v5: brakes/gazes after first ID',font=font,fill='#7dcfff')
        for col,target in enumerate(times):
            timestamp,frame=frames[target];x=10+col*330
            obs=min(observations,key=lambda r:abs(r.get('capture_stamp_s',-1)-timestamp))
            if abs(obs['capture_stamp_s']-timestamp)>.001:raise RuntimeError('observation association unavailable')
            draw.text((x+3,y+25),f't={timestamp:.2f}s / '+('ID0 decoded' if 'corners' in obs else 'ID missing'),font=small,fill='#9ece6a' if 'corners' in obs else '#f7768e')
            canvas.paste(frame.convert('RGB'),(x,y+48))
            nearest=Image.Resampling.NEAREST if hasattr(Image,'Resampling') else Image.NEAREST
            crop=frame.crop((280,185,320,220)).resize((160,140),nearest)
            canvas.paste(crop.convert('RGB'),(x,y+375))
            for line,text in enumerate(('display crop 4x','original region','x280:320','y185:220')):
                draw.text((x+170,y+390+22*line),text,font=small,fill='white')
            provenance.append({'run':str(run.resolve()),'capture_s':timestamp,
                'source_frame_sha256':hashlib.sha256(frame.tobytes()).hexdigest(),
                'decoded':bool('corners' in obs),'min_side_px':obs.get('min_side_px')})
    canvas.save(out/'native-pixel-loss-comparison.png')
    (out/'native-pixel-loss-comparison.json').write_text(json.dumps({
        'scope':'display-only native frame comparison; no new recognition inputs or truth data',
        'display_crop':'40x35 original pixels shown160x140 nearest; not decoder input',
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'frames':provenance},indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();plot((a.baseline,a.candidate),a.out,(159.02,159.22,159.32,159.42))
