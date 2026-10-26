"""Read-only live review. Evaluation poses never enter the flight controller."""
import argparse
from pathlib import Path
import json
import threading
import time
import sys
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'.venv/lib/python3.10/site-packages'))
sys.path.insert(0,str(ROOT/'tools'))
import cv2
from mission2_review_media import build_replay, project_marker
from .telemetry_view import render as compose

HTML='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Mission 2 · Live</title>
<style>body{margin:0;background:#10151d;color:#e4edf6;font:16px system-ui}header{padding:18px 24px;display:flex;justify-content:space-between}strong{color:#52deb3}img,video{display:block;width:100%;height:auto}p{padding:0 24px;color:#9eafc3}code{color:#e2ba78}</style>
<header><b>MISSION 2 · 인지 / 판단 / 제어</b><strong id="live">연결 중</strong></header>
<img id="screen"><video id="replay" controls playsinline muted style="display:none"></video>
<p id="result">외부 시점 / 실제 사용 영상 / 검출과 판단 근거 · 원본 기체 기준 hover 명령</p>
<p><code id="detail"></code></p>
<script>
let replayRun=null;
async function tick(){try{
 const s=await (await fetch('/status',{cache:'no-store'})).json();
 const label=document.querySelector('#live'),screen=document.querySelector('#screen'),video=document.querySelector('#replay');
 document.querySelector('#detail').textContent=s.run||'';
 if(s.ended&&s.video_available){
  label.textContent='비행 종료 · 기록 영상 재생';screen.style.display='none';video.style.display='block';
  const key=s.playback_key||s.run;
  if(replayRun!==key){replayRun=key;video.src='/video.mp4?run='+encodeURIComponent(key);video.play().catch(()=>{});}
  const r=s.result||{};document.querySelector('#result').textContent=s.comparison_summary||`독립 판정: 코스 ${(r.ordered_sections||[]).length}/8 통과 · 장애물 접촉 ${r.nonfloor_contacts??'확인 중'} · 착륙 ${r.landing?.passed?'통과':'별도 기준 미통과'} | 재생 20fps / 실제 입력 ${(r.input_fps||0).toFixed(1)}fps`;
 }else{
  label.textContent=s.ended?'비행 종료 · 전체 영상 저장 중':s.stage||s.error||'준비 중';
  document.querySelector('#result').textContent=s.ended?'시험 종료 · 독립 평가와 영상 저장 중':'시험 진행 중 · 통과 수와 접촉 여부는 종료 후 독립 평가';
  if(replayRun){video.pause();replayRun=null;}screen.style.display='block';video.style.display='none';
  screen.src='/frame.jpg?t='+Date.now();
 }
}catch(e){document.querySelector('#live').textContent='연결 재시도 중';}}
setInterval(tick,100);tick();</script></html>'''

def lastrow(path):
    try:
        with path.open('rb') as f:
            f.seek(0,2);size=f.tell();f.seek(max(0,size-50000));lines=f.read().splitlines()
        for line in reversed(lines):
            try:return json.loads(line)
            except ValueError:pass
    except OSError:pass
    return None

def main():
    p=argparse.ArgumentParser();p.add_argument('--pointer',type=Path,required=True);p.add_argument('--port',type=int,default=8088)
    args=p.parse_args();cv2.setNumThreads(1)
    state={'stage':'WAITING'};jpeg=[None]
    def render():
        loaded=None;renderer=None
        while True:
            try:
                pointer=json.loads(args.pointer.read_text());run=Path(pointer['run']).resolve()
                if state.get('ended') and state.get('run')==str(run):
                    state['video_available']=(run/'telemetry_video/validation.json').exists()
                    if (run/'result_summary.json').exists():state['result']=json.loads((run/'result_summary.json').read_text())
                    if pointer.get('comparison_video'):
                        state.update(video_available=Path(pointer['comparison_video']).exists(),
                            playback_key=pointer['comparison_video'],comparison_summary=pointer.get('comparison_summary'))
                    else:
                        state.pop('playback_key',None);state.pop('comparison_summary',None)
                    time.sleep(.3);continue
                runtime_state=json.loads((run/'runtime/runtime_state/state.json').read_text())
                runtime=Path(runtime_state['log_dir']);truth=lastrow(runtime/'evaluation/ground_truth.jsonl')
                if not truth or truth.get('event')!='ground_truth':time.sleep(.25);continue
                if loaded!=runtime:
                    if renderer:renderer.close()
                    job=dict(baseline='/home/razer/Desktop/Drone3',runtime_log_dir=str(runtime),truth=[truth],
                             width=640,height=420,evaluation_setup_present=(runtime/'evaluation/evaluation_setup.json').exists())
                    _,model,data,apply,report=build_replay(job)
                    model.vis.global_.offwidth=max(640,model.vis.global_.offwidth)
                    model.vis.global_.offheight=max(420,model.vis.global_.offheight)
                    renderer=mujoco.Renderer(model,height=420,width=640)
                    camera=mujoco.MjvCamera();camera.azimuth=135;camera.elevation=-25;camera.distance=1.7
                    loaded=runtime
                position,_=apply(truth);camera.lookat[:]=position+np.array([0,0,.1])
                renderer.update_scene(data,camera=camera)
                external=renderer.render()[:,:,::-1].copy()
                average=mujoco.mjv_averageCamera(renderer.scene.camera[0],renderer.scene.camera[1])
                marker=project_marker(average,position,640,420)
                if marker:
                    point=tuple(np.rint(marker).astype(int))
                    cv2.circle(external,point,12,(80,230,255),2)
                    cv2.putText(external,'DRONE POSITION',(point[0]+16,point[1]-10),0,.45,(80,230,255),1,cv2.LINE_AA)
                frame=lastrow(run/'host/frames.jsonl');decision=lastrow(run/'host/decisions.jsonl');command=lastrow(run/'host/commands.jsonl')
                # Show the exact decision-associated image, not a newer unrelated frame.
                d=decision or {};frame=d.get('frame') or frame
                input_image=np.load(run/'host'/frame['image_file'])['image'] if frame else None
                age=time.monotonic()-(frame or {}).get('received_monotonic',time.monotonic())
                stats=json.loads((run/'host/receive_stats.json').read_text()) if (run/'host/receive_stats.json').exists() else {}
                summary_path=run/'host/summary.json';ended=summary_path.exists()
                stage=d.get('stage','WAITING')
                if ended:
                    summary=json.loads(summary_path.read_text());elapsed=summary['finished_monotonic']-summary['started_monotonic']
                else:
                    first=lastrow(run/'host/events.jsonl') or {};elapsed=0.
                    try:
                        with (run/'host/decisions.jsonl').open() as f:initial=json.loads(f.readline())
                        elapsed=(frame or {}).get('received_monotonic',0)-initial['frame']['received_monotonic']
                    except (OSError,ValueError):pass
                canvas=compose(external,input_image,d,command,run.name,elapsed,age,stats.get('fps',0),ended)
                jpeg[0]=cv2.imencode('.jpg',canvas,[cv2.IMWRITE_JPEG_QUALITY,85])[1].tobytes()
                state.clear();state.update(stage=stage,run=str(run),image_age_s=age,ended=ended,
                    video_available=(run/'telemetry_video/validation.json').exists())
            except Exception as exc:state['error']=str(exc)
            time.sleep(.03)
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith('/video.mp4'):
                pointer=json.loads(args.pointer.read_text());run=Path(pointer['run']).resolve()
                path=Path(pointer['comparison_video']) if pointer.get('comparison_video') else run/'telemetry_video/mission2_perception_decisions.mp4'
                if not path.exists() or not (path.parent/'validation.json').exists():self.send_error(404);return
                size=path.stat().st_size;start=0;end=size-1;partial=False
                requested=self.headers.get('Range','')
                if requested.startswith('bytes='):
                    first,last=requested[6:].split('-',1)
                    try:
                        start=int(first) if first else max(0,size-int(last));end=min(size-1,int(last)) if first and last else size-1;partial=True
                    except ValueError:self.send_error(416);return
                if start>end or start>=size:self.send_error(416);return
                self.send_response(206 if partial else 200);self.send_header('Content-Type','video/mp4')
                self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
                if partial:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
                self.end_headers()
                try:
                    with path.open('rb') as stream:
                        stream.seek(start);remaining=end-start+1
                        while remaining:
                            data=stream.read(min(1024*1024,remaining));self.wfile.write(data);remaining-=len(data)
                except (BrokenPipeError,ConnectionResetError):pass
                return
            if self.path.startswith('/frame.jpg'):
                payload=jpeg[0] or b'';ctype='image/jpeg'
            elif self.path.startswith('/status'):payload=json.dumps(state).encode();ctype='application/json'
            else:payload=HTML.encode();ctype='text/html; charset=utf-8'
            self.send_response(200);self.send_header('Content-Type',ctype);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(payload)
        def log_message(self,*_):pass
    threading.Thread(target=render,daemon=True).start()
    print(f'http://127.0.0.1:{args.port}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()

if __name__=='__main__':main()
