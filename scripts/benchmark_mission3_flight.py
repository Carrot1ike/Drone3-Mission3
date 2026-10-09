#!/usr/bin/env python3
"""Conditional plant/PID/estimator benchmark; never a Mission3 landing run."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import time
from zoneinfo import ZoneInfo
os.environ.setdefault("MUJOCO_GL","egl")
import numpy as np
from yaw_core import quaternion_rpy
from mission3.settings import ROOT,load
from mission3.physical_scene import PhysicalPlant
from mission3.flight_controller import StockPid
from mission3.firmware import FirmwareCore


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--state",choices=("oracle_diagnostic","firmware"),default="firmware")
    ap.add_argument("--duration",type=float,default=12.)
    ap.add_argument("--response",type=float)
    ap.add_argument("--inertia-scale",type=float)
    ap.add_argument("--profile",type=Path,default=ROOT/"config/mission3-physical-sensitivity.json")
    ap.add_argument("--seed",type=int,default=1)
    ap.add_argument("--out",type=Path)
    args=ap.parse_args()
    cfg=load();physical=json.loads(args.profile.read_text())
    if args.response is not None:physical["motor"]["response_time_constant_s"]=args.response
    if args.inertia_scale is not None:physical["aircraft_contact_case"]["inertia_scale"]=args.inertia_scale
    stamp=datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out=args.out or ROOT/"artifacts/mission3/flight-bench"/f"{stamp}-{args.state}-seed{args.seed}"
    out.mkdir(parents=True,exist_ok=True)
    plant=PhysicalPlant(cfg,physical,out,1,args.seed,initial_arc_m=30.)
    pid=StockPid();ekf=FirmwareCore(position=(1,2,1),yaw=0.)
    files={n:(out/n).open("w") for n in ("truth.jsonl","sensors.jsonl","motors.jsonl","estimates.jsonl","commands.jsonl")}
    (out/"experiment.json").write_text(json.dumps({"mission":cfg,"physical":physical,"args":{"state":args.state,"duration":args.duration,"seed":args.seed}},indent=2)+"\n")
    state=ekf.state();sumacc=np.zeros(3);sumgyro=np.zeros(3);n=0
    dt=plant.model.opt.timestep;failure=None;max_tilt=0.;start=time.monotonic()
    max_v_error=0.;position_errors=[];zmin=1.;zmax=1.
    try:
        for tick in range(1,int(args.duration/dt)+1):
            imu=plant.imu();gyro=np.asarray(imu["gyro_body_rad_s"]);acc=np.asarray(imu["specific_force_body_m_s2"])
            sumacc+=acc;sumgyro+=gyro;n+=1
            ekf.process_noise(tick)
            if tick%10==0:
                ekf.predict(tick,sumacc/n,sumgyro/n);sumacc*=0;sumgyro*=0;n=0
                deck=plant.deck(.01)
                if deck["flow_valid"]:ekf.flow(tick,deck["flow_raw_dpixel"],.01,gyro_body_deg_s=np.rad2deg(gyro))
                files["sensors.jsonl"].write(json.dumps({"t":plant.time,**imu,"deck":deck})+"\n")
            if tick%25==0:
                tof=plant.deck(.01)
                if tof["range_valid"]:ekf.tof(tick,tof["range_m"],physical["sensor"]["tof_noise_std_m"])
            ekf.finalize();state=ekf.state()
            if args.state=="oracle_diagnostic":
                pos,vel,rpy=plant.position,plant.velocity,plant.rpy
            else:
                pos,vel=np.asarray(state["position_world_m"]),np.asarray(state["velocity_world_m_s"])
                q=np.asarray(state["quaternion_wxyz"]);rpy=np.array(quaternion_rpy(np.r_[q[1:],q[0]]))
            # Feedback comes from declared state source. Oracle is explicitly
            # isolated to sign/plant debugging, not mission control or scoring.
            vcmd=np.zeros(3);yawcmd=0.;phase="HOVER"
            if 4<=plant.time<7:vcmd[0]=.25;phase="FORWARD"
            elif 7<=plant.time<9:yawcmd=15.;phase="YAW"
            vcmd[2]=np.clip(.8*(1-pos[2]),-.2,.2)
            output=pid.update(tick,pos,vel,np.rad2deg(rpy)*[1,-1,1],np.rad2deg(gyro),acc/9.81,vcmd,yawcmd,
                              supply_voltage=physical["motor"]["supply_voltage_V"])
            plant.step(output,pid.arm_length,pid.thrust_to_torque)
            tilt=float(np.rad2deg(np.linalg.norm(plant.rpy[:2])));max_tilt=max(max_tilt,tilt)
            zmin=min(zmin,float(plant.position[2]));zmax=max(zmax,float(plant.position[2]))
            position_errors.append(float(np.linalg.norm(np.asarray(state["position_world_m"])-plant.position)))
            max_v_error=max(max_v_error,float(np.linalg.norm(vel-plant.velocity)))
            if tick%20==0:
                truth=plant.truth()
                files["truth.jsonl"].write(json.dumps(truth)+"\n")
                files["estimates.jsonl"].write(json.dumps({"t":plant.time,**state})+"\n")
                files["motors.jsonl"].write(json.dumps({"t":plant.time,"motor_output":output["motor_pwm"],**output})+"\n")
                files["commands.jsonl"].write(json.dumps({"t":plant.time,"phase":phase,"velocity_world_m_s":vcmd.tolist(),"yaw_rate_deg_s":yawcmd,"state_source":args.state})+"\n")
                if any(c["other"]=="floor" for c in truth["contacts"]):failure="floor_contact";break
            if tilt>60:failure="attitude_exceeds_60deg";break
            if not np.all(np.isfinite(plant.data.qpos)):failure="nonfinite_state";break
        result={"scope":"conditional flight benchmark, not Mission3", "state_source":args.state,"mission_landing":False,"manufacturer_matched":False,
                "duration_s":plant.time,"failure":failure,"max_tilt_deg":max_tilt,"z_min_m":zmin,"z_max_m":zmax,
                "final_truth_evaluation_only":plant.truth(),"final_estimate":state,"max_estimator_position_error_m":max(position_errors),
                "max_controller_velocity_state_error_m_s":max_v_error,"runtime_s":time.monotonic()-start,
                "code_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),ROOT/"scripts/mission3/physical_scene.py",ROOT/"scripts/mission3/flight_controller.py",ROOT/"scripts/mission3/flight_controller_host.c"]}}
        (out/"evaluation.json").write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps({k:result[k] for k in ("state_source","duration_s","failure","max_tilt_deg","z_min_m","z_max_m","max_estimator_position_error_m","max_controller_velocity_state_error_m_s","runtime_s")},indent=2));print(out)
    finally:
        plant.close()
        for f in files.values():f.close()
    return 2 if failure else 0


if __name__=="__main__":raise SystemExit(main())
