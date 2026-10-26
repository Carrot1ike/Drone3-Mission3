#!/usr/bin/env python3
"""Contact observer airborne negative controls, not Mission3 attempts.

Vary conditional plant mass/gain/lag while the observer's initial model stays
fixed. No contact truth reaches the candidate decision. Pad remains far away.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
from zoneinfo import ZoneInfo
os.environ.setdefault("MUJOCO_GL","egl")
import numpy as np
from mission3.settings import ROOT,load,controller_settings
from mission3.physical_scene import PhysicalPlant
from mission3.physical_estimator import PhysicalMotion
from mission3.flight_controller import StockPid
from mission3.touchdown import TouchdownObserver


def case(cfg,profile,out,mass,gain,lag,seed):
    plant_profile=deepcopy(profile)
    plant_profile["aircraft_contact_case"]["mass_kg"]=mass
    plant_profile["motor"].update(force_gain=gain,response_time_constant_s=lag)
    public=controller_settings(cfg);entry=public["entry"]["positions"][0]
    plant=PhysicalPlant(cfg,plant_profile,out,1,seed,initial_arc_m=30.)
    own=PhysicalMotion(public,entry,profile,plant.camera_rotation)
    pid=StockPid();observer=TouchdownObserver(profile["contact_observer"])
    dt=plant.model.opt.timestep;latest_range=latest_flow=state=None
    trace=(out/"trace.jsonl").open("w");false_candidates=[];harness_failure=None
    calibrated=False;max_residual=-float("inf");zmin=1.
    try:
        for tick in range(1,7001):
            now=plant.time;sample=plant.imu();own.imu(tick,sample)
            before_probe=now<6.
            permitted=(before_probe and state is not None and state["flow_fusion_reason"]=="floor_patch_clear"
                       and own.position[2]>.65 and math.hypot(own.roll,own.pitch)<math.radians(10))
            observer.calibrate_freeflight(sample,permitted);evidence=observer.imu(sample)
            if tick%25==1:latest_range=plant.range_packet()
            if tick%10==1:
                latest_flow=plant.flow_packet(.01)
                deck={**latest_flow,**latest_range,"stamp_s":now,
                      "range_new":latest_range["range_stamp_s"]>getattr(own,"last_tof_stamp",-1.)}
                state,_=own.state_from_deck(deck)
                if deck["range_new"]:own.last_tof_stamp=latest_range["range_stamp_s"]
            candidate=observer.contact_candidate(now,own.roll,own.pitch,not before_probe)
            # Never use the contact candidate to teleport/support the body.
            vcmd=[0.,0.,float(np.clip(.8*(1-own.position[2]),-.2,.2)) if before_probe else -.025]
            multiplier=1. if before_probe else 1-.35*min(1.,(now-6.)/.5)
            output=pid.update(tick,*own.lower_state(),vcmd,0.,thrust_multiplier=multiplier,
                              supply_voltage=profile["motor"]["supply_voltage_V"])
            observer.predict_motor(output["motor_static_force_N"],dt)
            plant.step(output,pid.arm_length,pid.thrust_to_torque)
            zmin=min(zmin,plant.position[2]);calibrated|=observer.calibration_valid
            if not before_probe:max_residual=max(max_residual,evidence["support_residual_m_s2"])
            if tick%20==1:
                truth=plant.truth()
                if candidate and not truth["contacts"]:false_candidates.append(now)
                trace.write(json.dumps({"t":now,"candidate":candidate,"estimated_state":state,"observer":evidence,
                    "collective_multiplier":multiplier,"motor_output":output,"truth_evaluation_only":truth})+"\n")
                if truth["contacts"]:harness_failure="unexpected_environment_contact";break
            if np.linalg.norm(plant.rpy[:2])>math.radians(60):harness_failure="tilt_test_harness_stop";break
        result={"scope":"conditional contact-observer airborne negative control; not a mission attempt or manufacturer validation",
            "plant_mass_kg":mass,"plant_force_gain":gain,"plant_motor_lag_s":lag,
            "observer_initial_parameters_unchanged":profile["contact_observer"],"calibrated":calibrated,
            "final_gain_per_kg":observer.force_to_accel,"calibration_residual_rms_m_s2":observer.calibration_rms,
            "airborne_false_candidate_samples":len(false_candidates),"first_false_candidate_s":false_candidates[0] if false_candidates else None,
            "max_probe_support_residual_m_s2":max_residual,"z_min_m":float(zmin),"harness_failure":harness_failure,"duration_s":plant.time,
            "negative_control_pass":not false_candidates and harness_failure is None}
        (out/"evaluation.json").write_text(json.dumps(result,indent=2)+"\n")
        return result
    finally:trace.close();plant.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--out",type=Path)
    a=parser.parse_args();cfg=load();profile=json.loads((ROOT/"config/mission3-physical-sensitivity.json").read_text())
    now=datetime.now(ZoneInfo("Asia/Seoul"))
    if now>=datetime.fromisoformat(cfg["deadline_kst"]):raise RuntimeError("session deadline elapsed")
    out=a.out or ROOT/"artifacts/mission3/contact-observer-negative"/now.strftime("%Y%m%d-%H%M%S-%f")
    out.mkdir(parents=True,exist_ok=True)
    inputs={"mission":cfg,"profile":profile,"code_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [Path(__file__),*list((ROOT/"scripts/mission3").glob("*.py"))]}}
    (out/"experiment.json").write_text(json.dumps(inputs,indent=2)+"\n")
    results=[]
    for mass in (.029,.035,.041):
        for gain in (.8,1.,1.2):
            for lag in (.02,.05):
                if datetime.now(ZoneInfo("Asia/Seoul"))>=datetime.fromisoformat(cfg["deadline_kst"]):break
                label=f"mass{mass:.3f}-gain{gain:.1f}-lag{lag:.2f}"
                result=case(cfg,profile,out/label,mass,gain,lag,1);results.append(result)
                print(label,result["negative_control_pass"],"cal",result["calibrated"],"false",result["airborne_false_candidate_samples"],flush=True)
    (out/"matrix.json").write_text(json.dumps({"results":results,"cases":len(results),
        "pass":sum(r["negative_control_pass"] for r in results),"not_mission_success":True},indent=2)+"\n")
    print(out,flush=True)


if __name__=="__main__":main()
