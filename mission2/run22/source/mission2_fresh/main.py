"""Own one official TCP image consumer and one cflib command connection."""
import argparse
import json
from pathlib import Path
import signal
import sys
import threading
import multiprocessing
import time
import warnings
from collections import deque
import numpy as np
import cv2

BASE=Path('/home/razer/Desktop/Drone3')
sys.path.insert(0,str(BASE/'virtual_aideck/runtime/host_protocol'))
from e2e_client import Client
from .commands import command_worker
from .mission import Mission

def run(args):
    cv2.setNumThreads(1)
    warnings.filterwarnings('ignore',category=DeprecationWarning)
    args.out.mkdir(parents=True,exist_ok=False)
    streams={key:(args.out/(key+'.jsonl')).open('w',buffering=1) for key in ('frames','decisions')}
    lock=threading.Lock();stop=threading.Event();context=multiprocessing.get_context("spawn")
    updates=context.Queue();worker_stop=context.Event();ready=context.Queue()
    worker=context.Process(target=command_worker,args=(args.out,updates,worker_stop,ready,args.ring_mask))
    mission=Mission(ring_mask=args.ring_mask,exit_width=args.exit_width,wall_scan=args.wall_scan);started=time.monotonic();error=None;last_print=0
    def write(name,row):
        with lock:streams[name].write(json.dumps(row,allow_nan=False)+'\n')
    def halt(*_):stop.set()
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt)
    client=None
    latest=[None];received_event=threading.Event();receiver_error=[None]
    receipts=deque(maxlen=100)
    def receive():
        i=0;last_stats=0
        while not stop.is_set():
            try:
                metadata,pixels=client.receive();stamp=time.monotonic()
                image=np.frombuffer(pixels,np.uint8).reshape(244,324)
                association=metadata.get('association') or {};state=association.get('ego_state')
                filename=f'frame_{i:06d}.npz';np.savez_compressed(args.out/filename,image=image)
                frame=dict(sequence=metadata['frame_sequence'],session_id=metadata['session_id'],
                           capture_tick_us=metadata['gap8_capture_end_tick_us'],received_monotonic=stamp,
                           image_file=filename,association_valid=association.get('state_association_valid',False),state=state)
                write('frames',frame);i+=1;receipts.append(stamp)
                if stamp-last_stats>1:
                    fps=(len(receipts)-1)/(receipts[-1]-receipts[0]) if len(receipts)>1 else 0.
                    temporary=args.out/'receive_stats.tmp'
                    temporary.write_text(json.dumps(dict(received_frames=i,fps=fps,last_receipt=stamp)))
                    temporary.replace(args.out/'receive_stats.json');last_stats=stamp
                with lock:latest[0]=(metadata,image,frame)
                received_event.set()
            except BaseException as exc:
                receiver_error[0]=f'{type(exc).__name__}: {exc}';received_event.set();break
    try:
        worker.start()
        connection=ready.get(timeout=40)
        if not connection['ok']:raise RuntimeError(connection['error'])
        client=Client(timeout=10)
        receiver=threading.Thread(target=receive,daemon=True);receiver.start()
        i=0
        while not stop.is_set() and time.monotonic()-started<args.duration:
            if worker_stop.is_set() or not worker.is_alive():raise RuntimeError("command process exited")
            if not received_event.wait(1):continue
            received_event.clear()
            with lock:packet=latest[0];latest[0]=None
            if packet is None:
                if receiver_error[0]:raise RuntimeError(receiver_error[0])
                continue
            metadata,image,frame=packet;received=frame['received_monotonic']
            association=metadata.get('association') or {};state=association.get('ego_state')
            if not state or not association.get('state_association_valid'):continue
            command=mission.step(image,state,received)
            identity=[metadata['session_id'],state['stm32_boot_id'],state['reset_generation']]
            write('decisions',dict(frame=frame,policy_completed_monotonic=time.monotonic(),command=command,
                stage=mission.stage,ring_index=mission.index,target=mission.target,obstacles=mission.obstacles,
                decision=mission.decision,odometry=mission.position.tolist(),
                passage=mission.passage,passage_phase=mission.passage_phase,
                travel_m=mission.travel,straight_distance_m=mission.distance))
            updates.put((command,frame,identity))
            if received-last_print>1:
                print(json.dumps(dict(stage=mission.stage,ring=mission.index,height=round(state['ekf_height_m'],3),
                    tof=round(state['tof_distance_m'],3),travel=round(mission.travel,3),target=mission.target,
                    passage={k:v for k,v in (mission.passage or {}).items() if k in ('depth','left','side','gap_m','rear_depth')})),flush=True)
                last_print=received
            if mission.stage=='DONE':break
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}';print(error,flush=True)
    finally:
        stop.set()
        worker_stop.set()
        if worker.pid is not None:worker.join(timeout=5)
        if worker.is_alive():
            worker.terminate();worker.join(timeout=3)
            error=error or 'command process did not close normally'
        if client is not None:client.close()
        if "receiver" in locals():receiver.join(timeout=2)
        updates.close();updates.cancel_join_thread()
        summary=dict(started_monotonic=started,finished_monotonic=time.monotonic(),stage=mission.stage,
                     ring_cycles=mission.index,transitions=mission.history,error=error,
                     physical_success='requires independent scoring',
                     tof_fusion='historical kalman.ringPass during ring crossing' if args.ring_mask else 'not modified by host',
                     ring_pass_profile=bool(args.ring_mask))
        (args.out/'summary.json').write_text(json.dumps(summary,indent=2))
        for stream in streams.values():stream.close()
    return 1 if error else 0

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--duration',type=float,default=300)
    p.add_argument('--ring-mask',type=int,choices=[0,1],default=0)
    p.add_argument('--exit-width',type=float,default=None,help='explicit allowed exit opening width in metres')
    p.add_argument('--wall-scan',action='store_true',help='stop and observe left/right before each wall decision')
    sys.exit(run(p.parse_args()))
