"""Offline physical scoring and synchronized external/received-camera recording."""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from eval_mission2_course import (course_from_scene,section_transits,continuous_flight_evidence,
                                 emitted_flight_commands,ground_landing_assessment)
from eval_score import controlled_intervals,restrict_control_window
from render_mission2_review import Replay,Movie,ffmpeg_binary,decoded_info

def rows(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

def evaluate(run):
    runtime=Path(json.loads((run/'runtime/runtime_state/state.json').read_text())['log_dir'])
    truth=[r for r in rows(runtime/'evaluation/ground_truth.jsonl') if r.get('event')=='ground_truth']
    commands=rows(run/'host/commands.jsonl');summary=json.loads((run/'host/summary.json').read_text())
    trial=dict(host_started_monotonic=summary['started_monotonic'],host_exit_monotonic=summary['finished_monotonic'])
    intervals=controlled_intervals(commands,trial)
    emitted=controlled_intervals(emitted_flight_commands(commands),trial)
    metadata=json.loads((runtime/'evaluation/model_metadata.json').read_text())
    manifest=json.loads((runtime/'arena/arena_manifest.json').read_text())
    layout=manifest.get('evaluation_layout')
    sections=course_from_scene(runtime/'arena/scene.xml',metadata,infer_wall_opening=bool(layout))
    controlled=restrict_control_window(truth,intervals)
    scores=[section_transits(controlled,s) for s in sections]
    ordered=[];continuity_checks=[]
    for event in sorted((e for s in scores for e in s['transits'] if e['geometry_pass']),key=lambda e:e['rear_clear_sim_time_s']):
        if len(ordered)<8 and event['name']==sections[len(ordered)]['name']:
            continuity=continuous_flight_evidence(truth,commands,emitted,
                ordered[0]['start_host_monotonic'] if ordered else event['start_host_monotonic'],event['rear_clear_host_monotonic'])
            continuity_checks.append(dict(through_section=event['name'],**continuity))
            if continuity['observed']:ordered.append(event)
    contacts=[r for r in truth if any(not {'arena_floor','floor','ground'}.intersection(c['geoms']) for c in r.get('contacts',[]))]
    lifecycle=json.loads((runtime/'evaluation/evaluation_lifecycle.json').read_text())
    complete=lifecycle.get('wrapper_closed') and lifecycle.get('source_main_returned') and not lifecycle.get('infrastructure_failure')
    landing=ground_landing_assessment(truth,commands,ordered,trial)
    result=dict(ordered_sections=[e['name'] for e in ordered],ordered_events=ordered,section_results=scores,
        continuity_checks=continuity_checks,
        all_log_nonfloor_contact_samples=len(contacts),lifecycle_complete=bool(complete),
        host_stage=summary['stage'],host_error=summary['error'],ground_landing=landing,
        full_course_success=bool(len(ordered)==8 and not contacts and complete and not summary['error'] and summary['stage']=='DONE'),
        evaluation_only=True,evaluation_layout=layout,wall_opening_inferred_from_scene=bool(layout),geometry_scorer='unchanged tools/eval_mission2_course.py section_transits',
        command_continuity='same original scorer continuous_flight_evidence')
    (run/'physical_score.json').write_text(json.dumps(result,indent=2))
    return runtime,truth,commands,summary,result

def render(run,runtime,truth,commands,summary,result,fps):
    out=run/'video';out.mkdir(exist_ok=False)
    frames=rows(run/'host/frames.jsonl');decisions=rows(run/'host/decisions.jsonl')
    args=SimpleNamespace(baseline=Path('/home/razer/Desktop/Drone3'),sim_python=None,width=640,
        image_height=420,preflight_assets=None)
    replay=Replay(args,runtime,truth,out,{},trial_started_monotonic=summary['started_monotonic'])
    movie=Movie(out/'mission2_review.mp4',(1280,660),fps,ffmpeg_binary())
    frame_times=[x['received_monotonic'] for x in frames]
    decision_times=[x['policy_completed_monotonic'] for x in decisions]
    command_times=[x['sent_monotonic'] for x in commands]
    truth_times=[x['host_monotonic'] for x in truth]
    start=summary['started_monotonic'];end=summary['finished_monotonic'];count=int(np.ceil((end-start)*fps))
    cached=None;input_image=None
    def at(items,times,t):
        i=bisect_right(times,t)-1
        return items[i] if i>=0 else None
    try:
        for index in range(count):
            t=start+index/fps
            frame=at(frames,frame_times,t);decision=at(decisions,decision_times,t) or {}
            command=at(commands,command_times,t) or {};gi=bisect_right(truth_times,t)-1
            canvas=np.zeros((660,1280,3),np.uint8);canvas[:]=(27,22,16)
            if gi>=0:canvas[45:465,:640]=replay.frame(gi)
            if frame and frame['image_file']!=cached:
                input_image=cv2.cvtColor(np.load(run/'host'/frame['image_file'])['image'],cv2.COLOR_GRAY2BGR);cached=frame['image_file']
            if input_image is not None:canvas[45:465,640:]=cv2.resize(input_image,(640,420))
            def put(text,x,y,color=(225,230,235),scale=.6):cv2.putText(canvas,text,(x,y),0,scale,color,1,cv2.LINE_AA)
            state=(frame or {}).get('state') or {};sent=command.get('command') or {}
            put('EXTERNAL - RECORDED POSE REPLAY',18,28);put('ONBOARD - ACTUAL RECEIVED IMAGE',660,28)
            put(f"{run.name} | {t-start:.1f}s | {decision.get('stage','CONNECTING')} | ring stages {decision.get('ring_index',0)}/3",20,500,(100,235,190),.8)
            put(f"EKF Z {state.get('ekf_height_m',0):.3f} m  |  ToF {state.get('tof_distance_m',0):.3f} m | velocity {np.round(state.get('body_velocity_flu',[0,0,0]),3)}",20,535)
            put(f"SENT vx {sent.get('vx',0):+.3f}  vy {sent.get('vy',0):+.3f} m/s | height target {sent.get('height_m',0):.3f} m | yaw rate {sent.get('yawrate',0):+.1f} deg/s",20,568)
            passage=decision.get('passage') or {}
            put(f"ToF hold requested {sent.get('ring_pass_mask',0)} / ACK {command.get('ring_pass_ack',0)} | passage {decision.get('passage_phase','')}  depth {passage.get('depth',0):.2f} m  lateral {passage.get('left',0):+.2f} m",20,600)
            crossed=[s['name'] for s in result['section_results'] if any(e['geometry_pass'] and e['rear_clear_host_monotonic']<=t for e in s['transits'])]
            put('Verified crossings: '+', '.join(crossed),20,635,scale=.45)
            if index==min(count-1,int(20*fps)):cv2.imwrite(str(out/'poster.jpg'),canvas)
            movie.write(canvas)
            if index%int(fps*30)==0:print(f'video {index}/{count}',flush=True)
    finally:movie.close();replay.close()
    check=decoded_info(out/'mission2_review.mp4')
    (out/'validation.json').write_text(json.dumps(dict(expected_frames=count,decoded=check,
        fps=fps,whole_host_timeline=True,external='kinematic replay, no simulation steps',
        internal='received mono8, preceding actual frame held'),indent=2))
    print(str(out/'mission2_review.mp4'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--video',action='store_true');p.add_argument('--fps',type=float,default=20.)
    a=p.parse_args();data=evaluate(a.run.resolve());print(json.dumps(data[-1],indent=2))
    if a.video:render(a.run.resolve(),*data,a.fps)
