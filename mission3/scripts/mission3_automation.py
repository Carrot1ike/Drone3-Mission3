#!/usr/bin/env python3
"""Local Mission3 GUI/CLI batch launcher. Only owned child groups are stopped.

No scheduled chat automation, radio link, internet writes or independent-review
signatures. Failed/interrupted/not-started cases stay in the predeclared denominator.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from datetime import datetime
import hashlib
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import json
import math
import mimetypes
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlparse, unquote
from zoneinfo import ZoneInfo

ROOT=Path(__file__).resolve().parents[1]
PYTHON=Path(sys.executable)  # 자식 비행/평가/렌더도 실행 중인 같은 환경을 사용합니다.
CONFIG=ROOT/'config/mission3-30x5-straight-v1.json'
PROFILE=ROOT/'config/mission3-rpyt-30x5-straight-v1.json'
OUTPUT=ROOT/'artifacts/mission3/automation'
UI=ROOT/'assets/mission3/automation.html'


def write_json(path, data):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    temp.replace(path)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def count_contacts(cases):return sum(c.get('contact_status')=='pass' for c in cases)


def distances(start,end):
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in (start,end)):
        raise ValueError('거리에는 유한 숫자를 입력하세요')
    if not .5 <= end <= start <= 1.25:raise ValueError('거리 범위는 1.25~0.50m입니다')
    count=round((start-end)/.05)
    if abs(start-end-count*.05)>1e-8:raise ValueError('거리 차이는 5cm 배수여야 합니다')
    return [round(start-i*.05,2) for i in range(count+1)]


def validate_options(options):
    result={'start_distance':1.25,'end_distance':.5,'repeats':1,'workers':4,'seed':1001,'render_video':False}
    if not isinstance(options,dict) or set(options)-set(result):raise ValueError('알 수 없는 실행 설정')
    result.update(options)
    distances(result['start_distance'],result['end_distance'])
    for key,lo,hi in (('repeats',1,10),('workers',1,4),('seed',1,2147483640)):
        value=result[key]
        if type(value) is not int or not lo<=value<=hi:raise ValueError(f'{key}: {lo}~{hi} 정수 필요')
    if type(result['render_video']) is not bool:raise ValueError('영상 옵션은 true/false')
    return result


class BatchManager:
    def __init__(self):
        self.lock=threading.RLock();self.cancel=threading.Event()
        self.active={};self.thread=None;self.batch=None
        self.state={'status':'idle','batch_id':None,'cases':[],'qualified_successes':0}
        if OUTPUT.exists():
            for folder in sorted(OUTPUT.iterdir(),reverse=True):
                progress=folder/'progress.json'
                if not progress.is_file():continue
                saved=json.loads(progress.read_text())
                if saved.get('status') in ('finished','stopped','planned'):
                    self.batch=folder;self.state=saved
                    self.state['conditional_contacts']=count_contacts(saved['cases'])
                break

    def snapshot(self):
        with self.lock:
            result=deepcopy(self.state)
            for case in result['cases']:
                log=self.batch/case['case_id']/'flight.log'
                if log.exists():
                    with log.open('rb') as f:
                        f.seek(max(0,log.stat().st_size-2000));case['log_tail']=f.read().decode('utf8','replace')
            return result

    def persist(self):
        with self.lock:write_json(self.batch/'progress.json',self.state)

    def start(self,options,execute=True):
        options=validate_options(options)
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('실행 중입니다. 먼저 중지하거나 완료를 기다리세요')
            stamp=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d-%H%M%S-%f')
            self.batch=OUTPUT/stamp;self.batch.mkdir(parents=True)
            cfg=json.loads(CONFIG.read_text());profile=json.loads(PROFILE.read_text())
            cases=[]
            for d in distances(options['start_distance'],options['end_distance']):
                for repetition in range(options['repeats']):
                    case_id=f'd{round(d*100):03d}-r{repetition+1:02d}'
                    folder=self.batch/case_id;folder.mkdir()
                    variant=deepcopy(cfg);variant['entry']['positions'][0]['xy_m']=[2.,round(2.5-d,2)]
                    variant['case_geometry']={'lateral_distance_m':d,'longitudinal_ahead_m':1.,'heading_east_ccw_deg':70.}
                    write_json(folder/'config.json',variant)
                    # Common seed across distances avoids conflating distance with RNG seed.
                    cases.append({'case_id':case_id,'distance_m':d,'repetition':repetition+1,
                                  'seed':options['seed']+repetition,'status':'queued'})
            write_json(self.batch/'profile.json',profile)
            files=[Path(__file__),ROOT/'scripts/run_mission3_physical.py',ROOT/'scripts/evaluate_mission3.py',
                   ROOT/'scripts/audit_mission3_limits.py',ROOT/'scripts/validate_mission3_camera_recording.py',
                   ROOT/'scripts/review_mission3_physical.py',ROOT/'scripts/yaw_core.py',ROOT/'scripts/aruco_pose.py',
                   ROOT/'scripts/mission3_runtime.bash',ROOT/'scripts/check_mission3_environment.py',
                   ROOT/'requirements/mission3-lock.txt',ROOT/'config/mission3-bootstrap.json',
                   ROOT/'scripts/mission3_rpyt_protocol.py',UI,ROOT/'scripts/run_mission3_automation.sh',
                   *sorted((ROOT/'scripts/mission3').glob('*.py')),
                   *sorted((ROOT/'scripts/mission3').glob('*_host.c'))]
            for folder in ('mission3-flight-controller','mission3-firmware-host','mission3-position-bridge'):
                files.extend(sorted(p for p in (ROOT/'build'/folder).glob('*') if p.is_file()))
            if (ROOT/'build/mission3-setup.json').is_file():
                files.append(ROOT/'build/mission3-setup.json')
            files.extend([CONFIG,PROFILE,ROOT/'config/provided-hardware.json',ROOT/'config/mission3-contact-sensitivity.json'])
            identities={str(p):sha(p) for p in files}
            for p in files:
                destination=self.batch/'source-snapshot'/p.relative_to(ROOT)
                destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(p.read_bytes())
            identities.update({str(self.batch/'profile.json'):sha(self.batch/'profile.json')})
            identities.update({str(self.batch/c['case_id']/'config.json'):sha(self.batch/c['case_id']/'config.json') for c in cases})
            self.identities=identities;self.cancel.clear()
            self.state={'status':'running' if execute else 'planned','batch_id':stamp,'options':options,
                'planned_count':len(cases),'completed_count':0,'cases':cases,'qualified_successes':0,
                'scope':'새 30×5m 조건부 개발 시험. 독립 승인·제조사 일치·실기 성공은 별도이며 자동 계상하지 않음.'}
            write_json(self.batch/'plan.json',{'created_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
                'options':options,'cases':deepcopy(cases),'identities':identities,'max_independent_processes':options['workers'],
                'trial_wall_timeout_s':600,'plan_is_training_not_unused_holdout':True,
                'failure_policy':'미인식 화각 통과/직선 끝/제어 안전 실패; 모든 중단·실패·미시작을 보존',
                'provenance':'기존 경기장 결과와 합산 금지; 자동 평가는 독립 검토 서명이 아님'})
            self.persist()
            if execute:
                self.thread=threading.Thread(target=self.run_batch,name='mission3-batch',daemon=False)
                self.thread.start()
            return stamp

    def stop(self):
        with self.lock:
            self.cancel.set()
            if self.state['status']=='running':self.state['status']='stopping';self.persist()
            for process in list(self.active.values()):
                if process.poll() is None:
                    try:os.killpg(process.pid,signal.SIGINT)
                    except ProcessLookupError:pass

    def verify(self):
        for name,digest in self.identities.items():
            if sha(name)!=digest:raise RuntimeError(f'고정 소스/설정 변경: {name}')

    def command(self,case,argv,logname,timeout=600):
        if self.cancel.is_set():return None
        env={**os.environ,'PYTHONNOUSERSITE':'1','MUJOCO_GL':os.environ.get('MUJOCO_GL','egl'),'OPENBLAS_NUM_THREADS':'1',
             'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','PYTHONPATH':str(ROOT/'scripts')}
        with (self.batch/case['case_id']/logname).open('xb') as log:
            with self.lock:
                if self.cancel.is_set():return None
                process=subprocess.Popen(argv,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                self.active[case['case_id']]=process
            deadline=time.monotonic()+timeout;cancel_at=None;timed_out=False
            while process.poll() is None:
                if self.cancel.is_set() or time.monotonic()>deadline:
                    timed_out=timed_out or time.monotonic()>deadline
                    if cancel_at is None:
                        cancel_at=time.monotonic()
                        try:os.killpg(process.pid,signal.SIGINT)
                        except ProcessLookupError:pass
                    elif time.monotonic()-cancel_at>5.:
                        try:os.killpg(process.pid,signal.SIGKILL)
                        except ProcessLookupError:pass
                time.sleep(.1)
            code=process.wait()
        with self.lock:self.active.pop(case['case_id'],None)
        if timed_out:raise TimeoutError('단일 작업 벽시계600초 초과; 원자료 보존')
        return code

    def execute_case(self,case):
        folder=self.batch/case['case_id'];run=folder/'run'
        with self.lock:
            if self.cancel.is_set():case['status']='not_started';self.persist();return
            case['status']='running';self.persist()
        try:
            self.verify()
            argv=[str(PYTHON),str(ROOT/'scripts/run_mission3_physical.py'),'--config',str(folder/'config.json'),
                  '--profile',str(self.batch/'profile.json'),'--entry','1','--seed',str(case['seed']),
                  '--out',str(run),'--video','--split','training','--series',self.state['batch_id']]
            code=self.command(case,argv,'flight.log');case['flight_exit_code']=code
            if self.cancel.is_set():case['status']='interrupted';return
            if code not in (0,2) or not (run/'adapter-summary.json').exists():raise RuntimeError(f'물리 실행 오류 exit={code}')
            self.verify()
            checks=[['evaluate_mission3.py','run',str(run),'--out',str(run/'offline-evaluation.json')],
                    ['audit_mission3_limits.py',str(run)],['validate_mission3_camera_recording.py',str(run)],
                    ['review_mission3_physical.py',str(run)]]
            case['status']='evaluating';case['check_exit_codes']=[];self.persist()
            for index,check in enumerate(checks):
                result=self.command(case,[str(PYTHON),str(ROOT/'scripts'/check[0]),*check[1:]],f'check-{index}.log')
                case['check_exit_codes'].append(result)
                if self.cancel.is_set():case['status']='interrupted';return
                if result:raise RuntimeError(f'검증 작업 오류 {check[0]} exit={result}')
            evaluation=json.loads((run/'offline-evaluation.json').read_text())
            summary=json.loads((run/'adapter-summary.json').read_text())
            limits=json.loads((run/'command-limits-audit.json').read_text())
            case.update(contact_status=evaluation['simulation_contact_success']['status'],
                mission_failure=summary['failure'],decoded=summary.get('decoded_target_seen',False),
                accepted_frames=summary['accepted_frames'],duration_s=summary['duration_s'],
                motor_off_at_s=summary['motor_off_at_s'],score=evaluation.get('score'),
                command_limits_pass=limits['command_envelope_pass'],
                max_actual_speed_m_s=limits['max_actual_speed_m_s'],
                first_image_guidance_time_s=limits['first_image_guidance_time_s'])
            import imageio_ffmpeg
            ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
            encoder=[ffmpeg,'-nostdin','-y','-i',str(run/'camera.avi'),'-an','-c:v','libx264',
                     '-threads','1','-pix_fmt','yuv420p','-movflags','+faststart',str(run/'camera-h264.mp4')]
            if self.command(case,encoder,'camera-video.log',180)!=0:raise RuntimeError('카메라 재생 영상 변환 실패')
            case['camera_url']=f'/files/{self.state["batch_id"]}/{case["case_id"]}/run/camera-h264.mp4'
            case['metrics_url']=f'/files/{self.state["batch_id"]}/{case["case_id"]}/run/physical-review-metrics.png'
            if self.state['options']['render_video']:
                if self.command(case,[str(PYTHON),str(ROOT/'scripts/review_mission3_physical.py'),str(run),
                                      '--video','--video-fps','5','--workers','1'],'third-view.log')!=0:
                    raise RuntimeError('제3자 영상 렌더 실패')
                encoder[4]=str(run/'physical-review.mp4');encoder[-1]=str(run/'physical-review-h264.mp4')
                if self.command(case,encoder,'third-video.log',180)!=0:raise RuntimeError('제3자 영상 변환 실패')
                case['video_url']=f'/files/{self.state["batch_id"]}/{case["case_id"]}/run/physical-review-h264.mp4'
            self.verify()
            case['status']='completed'
        except Exception as exc:
            case['status']='interrupted' if self.cancel.is_set() else 'error';case['error']=str(exc)
        finally:
            with self.lock:self.persist()

    def run_batch(self):
        try:
            with ThreadPoolExecutor(max_workers=self.state['options']['workers']) as pool:
                futures=[pool.submit(self.execute_case,c) for c in self.state['cases']]
                for future in as_completed(futures):
                    future.result()
                    with self.lock:
                        self.state['completed_count']=sum(c['status']=='completed' for c in self.state['cases']);self.persist()
            with self.lock:
                self.state['status']='stopped' if self.cancel.is_set() else 'finished'
                self.state['completed_count']=sum(c['status']=='completed' for c in self.state['cases'])
                self.state['conditional_contacts']=count_contacts(self.state['cases'])
                self.state['errors']=sum(c['status']=='error' for c in self.state['cases'])
                self.state['ended_kst']=datetime.now(ZoneInfo('Asia/Seoul')).isoformat();self.persist()
            lines=['# 미션3 직선 경기장 자동 시험',f'\n배치 `{self.state["batch_id"]}` · 계획 {self.state["planned_count"]}회 · 완료 {self.state["completed_count"]}회',
                   '\n조건부 접촉은 원로그 자동 평가이며 독립 승인·제조사 일치·실기 성공을 의미하지 않는다. 정식 성공 수 0.',
                   '\n|옆거리 m|seed|작업 상태|접촉|인식|실패 원인|','|---|---|---|---|---|---|']
            for c in self.state['cases']:
                lines.append(f'|{c["distance_m"]:.2f}|{c["seed"]}|{c["status"]}|{c.get("contact_status","미평가")}|{c.get("decoded","미평가")}|{c.get("mission_failure") or c.get("error") or "—"}|')
            (self.batch/'report.md').write_text('\n'.join(lines)+'\n')
        except Exception as exc:
            with self.lock:self.state['status']='error';self.state['error']=str(exc);self.persist()


def serve(manager,port):
    token=secrets.token_urlsafe(24)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send_json(self,data,status=200):
            payload=json.dumps(data,ensure_ascii=False).encode()
            self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
        def do_GET(self):
            path=unquote(urlparse(self.path).path)
            if path=='/api/state':return self.send_json(manager.snapshot())
            if path=='/':
                payload=UI.read_text().replace('__TOKEN__',token).encode()
                self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload);return
            if not path.startswith('/files/'):return self.send_error(404)
            target=(OUTPUT/path.removeprefix('/files/')).resolve()
            if not target.is_relative_to(OUTPUT.resolve()) or not target.is_file():return self.send_error(404)
            size=target.stat().st_size;start=0;end=size-1;partial=False
            header=self.headers.get('Range','')
            if header.startswith('bytes='):
                try:
                    lo,hi=header[6:].split('-');start=int(lo);end=min(int(hi) if hi else end,end)
                    if not 0<=start<=end<size:raise ValueError()
                    partial=True
                except ValueError:return self.send_error(416)
            self.send_response(206 if partial else 200);self.send_header('Content-Type',mimetypes.guess_type(target.name)[0] or 'application/octet-stream')
            self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
            if partial:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
            self.end_headers()
            try:
                with target.open('rb') as f:
                    f.seek(start);remaining=end-start+1
                    while remaining:
                        chunk=f.read(min(65536,remaining))
                        if not chunk:break
                        self.wfile.write(chunk);remaining-=len(chunk)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_POST(self):
            origin=self.headers.get('Origin')
            if self.headers.get('X-Mission3-Token')!=token or (origin and origin not in (f'http://127.0.0.1:{port}',f'http://localhost:{port}')):
                return self.send_json({'error':'로컬 실행 화면에서 요청하세요'},403)
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<=length<=8192:raise ValueError('요청 크기 초과')
                data=json.loads(self.rfile.read(length) or '{}')
                if self.path=='/api/start':return self.send_json({'batch_id':manager.start(data)})
                if self.path=='/api/stop':manager.stop();return self.send_json({'status':'stopping'})
                return self.send_error(404)
            except (ValueError,RuntimeError) as exc:return self.send_json({'error':str(exc)},400)
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    print(f'http://127.0.0.1:{port}',flush=True)
    try:server.serve_forever()
    finally:
        manager.stop()
        if manager.thread:manager.thread.join(10.)
        server.server_close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',action='store_true');parser.add_argument('--plan-only',action='store_true')
    parser.add_argument('--start-distance',type=float,default=1.25);parser.add_argument('--end-distance',type=float,default=.5)
    parser.add_argument('--workers',type=int,default=4);parser.add_argument('--repeats',type=int,default=1)
    parser.add_argument('--seed',type=int,default=1001);parser.add_argument('--render-video',action='store_true')
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args();manager=BatchManager()
    if args.batch or args.plan_only:
        options={key:getattr(args,key) for key in ('start_distance','end_distance','workers','repeats','seed','render_video')}
        manager.start(options,execute=not args.plan_only);print(manager.batch,flush=True)
        if manager.thread:
            try:manager.thread.join()
            except KeyboardInterrupt:manager.stop();manager.thread.join()
        print(json.dumps(manager.snapshot(),ensure_ascii=False,indent=2))
        return 1 if manager.state.get('errors') or manager.state['status'] in ('error','stopped') else 0
    try:serve(manager,args.port)
    except KeyboardInterrupt:pass
    return 0


if __name__=='__main__':raise SystemExit(main())
