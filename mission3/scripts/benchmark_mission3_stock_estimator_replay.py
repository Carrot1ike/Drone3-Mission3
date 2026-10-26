#!/usr/bin/env python3
"""Offline exact-core/approximate-input shadow; never controls an aircraft.

Consumes saved body IMU/deck only. The IMU log is sampled100Hz, not the original
1kHz sequence: held samples and scheduling are an explicit replay limitation.
Only after inference finishes are truth and project estimates read for plots.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from zoneinfo import ZoneInfo
import csv
import numpy as np
from mission3.firmware import FirmwareCore
from mission3.settings import ROOT


def replay(sensors,entry,height,mode):
    core=FirmwareCore(np.r_[entry["xy_m"],height],math.radians(entry["heading_deg"]))
    results=[];noise_tick=0;counts={"flow":0,"tof":0,"bad_range":0}
    coefficient=math.log(.2/.0025)/(4.-2.5)
    for row in sensors:
        stamp=row["t"];tick=int(round(stamp*1000))
        for n in range(noise_tick+1,tick+1):core.process_noise(n)
        noise_tick=tick
        imu=row["imu"];deck=row["deck"]
        if tick>0:
            core.predict(tick,imu["specific_force_body_m_s2"],imu["gyro_body_rad_s"])
        if deck.get("range_new"):
            distance=deck.get("range_m")
            if distance is not None and math.isfinite(distance) and distance>=0 and distance<5.:
                if mode=="stock_driver_like" or deck.get("range_accurate",True):
                    # zranger2 RangeMilliMeter is uint16; virtual rounding is S.
                    distance=round(distance*1000)/1000
                    std=.0025*(1+math.exp(coefficient*(distance-2.5)))
                    core.tof(int(round(deck["range_stamp_s"]*1000)),distance,std);counts["tof"]+=1
            else:counts["bad_range"]+=1
        if deck["flow_valid"]:
            dpixel=np.rint(deck["flow_raw_dpixel"])
            if np.max(np.abs(dpixel))<100:
                core.flow(tick,dpixel,deck["flow_dt_s"],(2.,2.),np.rad2deg(imu["gyro_body_rad_s"]));counts["flow"]+=1
        core.finalize()
        results.append({"t":stamp,**core.state()})
    return results,counts


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path);parser.add_argument("--out",type=Path)
    args=parser.parse_args();run=args.run.resolve()
    out=args.out or ROOT/"artifacts/mission3/stock-estimator-shadow"/datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out.mkdir(parents=True,exist_ok=True)
    experiment=json.loads((run/"experiment.json").read_text());cfg=experiment["mission"]
    entry=next(e for e in cfg["entry"]["positions"] if e["id"]==experiment["entry_id"])
    sensors=[json.loads(x) for x in (run/"sensors.jsonl").read_text().splitlines()]
    modes={};counts={}
    for mode in ("stock_driver_like","component_accuracy_gated"):
        modes[mode],counts[mode]=replay(sensors,entry,cfg["entry"]["height_m"],mode)
        (out/f"{mode}.jsonl").write_text("".join(json.dumps(r)+"\n" for r in modes[mode]))
    # Evaluator-only comparison: no truth/phase/project mean is an EKF input.
    truth=[json.loads(x) for x in (run/"truth.jsonl").read_text().splitlines()]
    commands=[json.loads(x) for x in (run/"commands.jsonl").read_text().splitlines()]
    tt=np.array([r["t"] for r in truth]);ct=np.array([r["t"] for r in commands])
    times=np.array([r["t"] for r in sensors])
    position=np.array([r["aircraft"]["center_world_m"] for r in truth])
    velocity=np.array([r["aircraft"]["velocity_world_m_s"] for r in truth])
    actual=np.column_stack([np.interp(times,tt,position[:,i]) for i in range(3)])
    actual_v=np.column_stack([np.interp(times,tt,velocity[:,i]) for i in range(3)])
    metrics={}
    phase_columns={}
    for mode,rows in modes.items():
        p=np.array([r["position_world_m"] for r in rows]);v=np.array([r["velocity_world_m_s"] for r in rows])
        per_phase={}
        for phase in ("WATCH","LINEUP","APPROACH","TRACK","BLIND_APPROACH","TOUCHDOWN_DESCEND","MOTOR_OFF_HOLD"):
            indices=np.clip(np.searchsorted(ct,times,side="right")-1,0,len(commands)-1)
            mask=np.array([commands[i]["phase"]==phase for i in indices])
            if mask.any():
                per_phase[phase]={"samples":int(mask.sum()),"median_z_error_m":float(np.median(p[mask,2]-actual[mask,2])),
                                  "median_vxy_error_m_s":float(np.median(np.linalg.norm(v[mask,:2]-actual_v[mask,:2],axis=1)))}
        metrics[mode]=per_phase;phase_columns[mode]=(p,v)
    with (out/"height-velocity-comparison.csv").open("w") as stream:
        writer=csv.writer(stream);writer.writerow(["t_s","actual_z_m","project_z_m","stock_z_m","gated_z_m","actual_vx_m_s","stock_vx_m_s","gated_vx_m_s"])
        for i,stamp in enumerate(times):
            writer.writerow([stamp,actual[i,2],sensors[i]["estimate"]["position_world_m"][2],phase_columns["stock_driver_like"][0][i,2],
                             phase_columns["component_accuracy_gated"][0][i,2],actual_v[i,0],phase_columns["stock_driver_like"][1][i,0],phase_columns["component_accuracy_gated"][1][i,0]])
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"]="NanumGothic"
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    axes[0].plot(times,actual[:,2],label="실제 높이 (평가용)")
    axes[0].plot(times,[s["estimate"]["position_world_m"][2] for s in sensors],label="프로젝트 표면 추정")
    axes[0].plot(times,phase_columns["stock_driver_like"][0][:,2],label="공식 EKF shadow / driver 유사 range")
    axes[0].plot(times,phase_columns["component_accuracy_gated"][0][:,2],label="공식 EKF shadow / 4cm gate")
    axes[1].plot(times,actual_v[:,0],label="실제 vx (평가용)")
    axes[1].plot(times,phase_columns["stock_driver_like"][1][:,0],label="공식 EKF shadow vx")
    axes[0].set_ylabel("높이 [m]");axes[1].set_ylabel("vx [m/s]");axes[1].set_xlabel("미션 시간 [s]")
    for ax in axes:ax.legend(loc="upper right");ax.grid(alpha=.2)
    fig.suptitle("공식 추정기 코드의 오프라인 shadow: 100Hz IMU 로그, 실제 RTOS/기체 재현 아님")
    fig.tight_layout();fig.savefig(out/"height-velocity-comparison.png",dpi=150);plt.close(fig)
    paths=[Path(__file__),ROOT/"scripts/mission3/firmware.py",ROOT/"build/mission3-firmware-host/libmission3_estimator.so",run/"sensors.jsonl"]
    report={"run":str(run.relative_to(ROOT)),"input_count":len(sensors),"measurement_counts":counts,"phase_metrics":metrics,
            "inference_inputs":["known entry/height/heading","body IMU","deck samples"],
            "truth_read_only_after_inference":True,"controller_in_loop":False,"source_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            "limitations":["body IMU saved100Hz, not original1kHz; held samples approximate","Flowvalid is generic virtual texture/range policy, not actual motion register","raw counts rounded S; missing shutter/SQUAL","exact core code but no FreeRTOS/queue timing/real sensor noise"],
            "qualified_success_count":0,"scope":"offline estimator structural mismatch diagnostic only"}
    (out/"report.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(metrics,indent=2));print(out)

if __name__=="__main__":main()
