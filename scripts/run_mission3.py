#!/usr/bin/env python3
"""미션3 영상/가상센서 실행. 실제 기체 접촉 성공은 집계하지 않는다."""
import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import os
import time
from pathlib import Path
from zoneinfo import ZoneInfo
os.environ.setdefault("MUJOCO_GL","egl")
import cv2
import numpy as np
from mission3.settings import load,controller_settings,ROOT
from mission3.scene import GeometryPlant
from mission3.estimator import SelfMotion
from mission3.vision import Observer
from mission3.controller import Mission3


def run(cfg,out,entry_id,seed,arc=None,strict_focus=False,timeout=None,record=True,wall_deadline=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    cfg=json.loads(json.dumps(cfg))
    if wall_deadline is not None and datetime.now(ZoneInfo("Asia/Seoul"))>=wall_deadline:
        raise RuntimeError("작업 마감 후 새 시험은 시작하지 않습니다")
    if timeout is not None:
        cfg["control"]["mission_timeout_s"]=timeout
    public=controller_settings(cfg)
    entry=next(e for e in public["entry"]["positions"] if e["id"]==entry_id)
    plant=GeometryPlant(cfg,out,entry_id,seed,arc)
    own=SelfMotion(public,entry)
    observer=Observer(public,own)
    mission=Mission3(public,entry)
    dt=1/cfg["timing_experiment"]["control_hz"]
    image_dt=1/cfg["timing_experiment"]["render_fps"]
    pending=deque();next_image=0.;rendered=accepted=0;verdicts={}
    files={name:(out/name).open("w") for name in ("observations.jsonl","commands.jsonl","sensors.jsonl","truth.jsonl")}
    video=cv2.VideoWriter(str(out/"camera.avi"),cv2.VideoWriter_fourcc(*"MJPG"),cfg["timing_experiment"]["render_fps"],(320,320),isColor=False) if record else None
    (out/"experiment.json").write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+"\n")
    (out/"controller-settings.json").write_text(json.dumps(public,ensure_ascii=False,indent=2)+"\n")
    before="WAIT"
    command=None
    last_result=None
    try:
        while plant.time <= cfg["control"]["mission_timeout_s"]+.1:
            if wall_deadline is not None and datetime.now(ZoneInfo("Asia/Seoul"))>=wall_deadline:
                command=mission.abort("session_wall_deadline",plant.time,dt)
                break
            sample=plant.sensor_packet(dt)
            own_state=own.update(sample)
            files["sensors.jsonl"].write(json.dumps({"raw":sample,"estimate":own_state})+"\n")
            if plant.time>=next_image-1e-8:
                gray,scope=plant.image(strict_focus)
                rendered+=1
                if gray is not None:
                    if video is not None:video.write(gray)
                    pending.append((plant.time+cfg["timing_experiment"]["transport_delay_s"],plant.time,gray))
                    if not(out/f"camera-{mission.phase}.png").exists():
                        cv2.imwrite(str(out/f"camera-{mission.phase}.png"),gray)
                else:
                    verdicts[scope]=verdicts.get(scope,0)+1
                next_image+=image_dt
            while pending and pending[0][0]<=plant.time+1e-8:
                receipt,stamp,gray=pending.popleft()
                result=observer.frame(gray,stamp,plant.time)
                accepted+=int(result["valid"])
                verdicts[result["reason"]]=verdicts.get(result["reason"],0)+1
                result.update(time_s=plant.time,phase=mission.phase)
                files["observations.jsonl"].write(json.dumps(result)+"\n")
                last_result=result
            state=observer.state(plant.time)
            command=mission.update(plant.time,own_state,state)
            own.terminal_hint=state if mission.phase in ("BLIND_APPROACH","BLIND_DESCEND") else None
            files["commands.jsonl"].write(json.dumps({"time_s":plant.time,"own":own_state,"vehicle":state,**asdict(command)})+"\n")
            files["truth.jsonl"].write(json.dumps(plant.truth())+"\n")
            if command.phase!=before:
                print(f"t={plant.time:.2f} {before}->{command.phase} {mission.events[-1]['reason']}",flush=True)
                before=command.phase
            if command.done:break
            plant.step(command.velocity_world_m_s,command.yaw_rate_rad_s,dt)
        truth=plant.truth()
        state=observer.state(plant.time)
        summary={"entry_id":entry_id,"seed":seed,"initial_track_arc_m_plant_only":plant.initial_arc,
                 "duration_s":plant.time,"phase":command.phase,"failure":command.failure,
                 "scope":"image+virtuallevelprobe optical-point kinematics; no aircraft dynamics/contact/motors",
                 "initial_uncertainty_injected":bool(cfg["entry"].get("inject_initial_uncertainty",False)),
                 "geometry_ready":mission.geometry_ready,"physical_success":False,"physical_success_count":0,
                 "strict_published_focus_envelope":strict_focus,"rendered_frames":rendered,"accepted_frames":accepted,
                 "verdicts":verdicts,"events":mission.events,"surface_events":own.events,
                 "vehicle_speed_source":"image+sensor odometry estimates, not configured cart speed",
                 "controller_has_plant_vehicle_settings":False,
                 "final_truth_evaluation_only":truth,"final_estimate":state,
                 "final_optical_to_pad_3d_distance_m":float(np.linalg.norm(np.asarray(truth["pad_world_m"])-plant.position)),
                 "final_optical_to_pad_horizontal_error_m":float(np.linalg.norm(np.asarray(truth["pad_world_m"])[:2]-plant.position[:2])),
                 "final_optical_height_above_pad_m":float(plant.position[2]-truth["pad_world_m"][2]),
                 "estimator_self_position_error_m":float(np.linalg.norm(own.position-plant.position)),
                 "aircraft_commands_published":False,"hardware_landing_validated":False,
                 "input_source_hashes":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in list((ROOT/"scripts/mission3").glob("*.py"))+[Path(__file__).resolve()]}}
        (out/"evaluation.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n")
        print(json.dumps({k:summary[k] for k in ("phase","failure","duration_s","rendered_frames","accepted_frames","verdicts","geometry_ready","physical_success")},ensure_ascii=False,indent=2),flush=True)
    finally:
        if video is not None:video.release()
        plant.close()
        for f in files.values():f.close()
    return summary


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--entry",type=int,default=1,choices=range(1,9))
    parser.add_argument("--seed",type=int,default=1)
    parser.add_argument("--arc",type=float)
    parser.add_argument("--timeout",type=float)
    parser.add_argument("--strict-focus",action="store_true")
    parser.add_argument("--terminal-policy",choices=("image","last_fix_bridge"))
    parser.add_argument("--no-video",action="store_true")
    parser.add_argument("--perturb-entry",action="store_true",help="inject experimental initial position/yaw uncertainty")
    parser.add_argument("--respect-session-deadline",action="store_true",help="stop at configured KST wall-clock deadline")
    parser.add_argument("--out",type=Path)
    args=parser.parse_args()
    now=datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out=args.out or ROOT/"artifacts/mission3"/f"{now}-entry{args.entry}-seed{args.seed}"
    cfg=load();cfg["entry"]["inject_initial_uncertainty"]=args.perturb_entry
    if args.terminal_policy:cfg["control"]["terminal_policy"]=args.terminal_policy
    deadline=datetime.fromisoformat(cfg["deadline_kst"]) if args.respect_session_deadline else None
    result=run(cfg,out,args.entry,args.seed,args.arc,args.strict_focus,args.timeout,not args.no_video,deadline)
    print(f"산출물 {out}",flush=True)
    return 0 if result["geometry_ready"] else 2


if __name__=="__main__":raise SystemExit(main())
