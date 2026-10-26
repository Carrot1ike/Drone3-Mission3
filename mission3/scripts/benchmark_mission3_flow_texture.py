#!/usr/bin/env python3
"""Virtual downward-image sensitivity fixture; no flight/entry success count.

Aircraft poses are explicitly prescribed for two-frame sensor diagnostics.
Analytical surface velocity is evaluation-only. A generic OpenCV correlator
is compared with the mathematical ray sensor, never claimed to be PMW3901.
"""
import os
os.environ.setdefault("MUJOCO_GL","egl")
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import cv2
import mujoco
import numpy as np
from mission3.settings import ROOT,load
from mission3.physical_scene import PhysicalPlant
from mission3.flow_texture import FlowTextureProbe


def run(out,profile):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    physical=json.loads(profile.read_text());cfg=load()
    texture=physical["sensor"]["texture_quality_sensitivity"]
    cases=[];panels=[]
    for location,xoffset in (("floor",-1.),("pad_center",0.),("pad_edge",.09)):
        for height in (.40,.20,.11,.085,.06,.03):
            for relative_speed in (0.,.25):
                key=f"{location}-h{height:.3f}-vr{relative_speed:.2f}"
                folder=out/key;folder.mkdir()
                position=[25+xoffset,6.25,.30+height]
                plant=PhysicalPlant(cfg,physical,folder,1,1,initial_arc_m=10.,initial_position=position)
                probe=FlowTextureProbe(texture)
                body_speed=.25+relative_speed
                measurements=[];images=[];ray=[]
                try:
                    # These writes belong only to this explicit sensor fixture.
                    # Full mission aircraft physics never uses this trajectory.
                    for i in range(2):
                        stamp=i*.01
                        plant.data.time=stamp
                        plant.data.qpos[plant.aq]=position[0]+body_speed*stamp
                        plant.data.qvel[plant.av:plant.av+3]=[body_speed,0,0]
                        plant.update_vehicle();mujoco.mj_forward(plant.model,plant.data)
                        image=plant.flow_probe_image();images.append(image)
                        cv2.imwrite(str(folder/f"native-probe-{i}.png"),image)
                        measurements.append(probe.measure(image,stamp))
                        ray.append(plant.patch_sensor(physical["sensor"]["flow_half_angle_deg"],physical["sensor"]["patch_grid_side"]))
                    model=np.asarray(ray[-1]["angular_rad_s"])
                    predicted=measurements[-1]["diagnostic_angular_rad_s"]
                    error=None if predicted is None else float(np.linalg.norm(np.asarray(predicted)-model))
                    distance=float(min(ray[-1]["distances_m"]))
                    envelope=bool(distance>=physical["sensor"]["flow_component_min_working_range_m"])
                    result={"case":key,"location":location,"body_height_above_pad_m":height,
                            "body_world_velocity_m_s":[body_speed,0,0],"vehicle_velocity_m_s":[.25,0,0],
                            "mathematical_ray_angular_rad_s":model.tolist(),"minimum_ray_distance_m":distance,
                            "component_range_qualified":envelope,"generic_image_measurements":measurements,
                            "generic_vs_ray_angular_error_rad_s":error,
                            "conditional_flow_valid":bool(envelope and measurements[-1]["usable"]),
                            "aircraft_state_prescribed":True,"controller_in_loop":False,
                            "manufacturer_matched":False,"mission_success":False}
                    (folder/"result.json").write_text(json.dumps(result,indent=2)+"\n");cases.append(result)
                    if relative_speed==.25:
                        panel=cv2.cvtColor(images[-1],cv2.COLOR_GRAY2BGR)
                        # Display enlargement only; diagnostic used native64.
                        panel=cv2.resize(panel,(180,180),interpolation=cv2.INTER_NEAREST)
                        tile=np.full((216,180,3),245,np.uint8);tile[:180]=panel
                        cv2.putText(tile,f"{location} h={height:.3f}",(4,195),cv2.FONT_HERSHEY_SIMPLEX,.38,(0,0,0),1)
                        cv2.putText(tile,f"texture {measurements[-1]['usable']}",(4,211),cv2.FONT_HERSHEY_SIMPLEX,.38,(0,0,0),1)
                        panels.append(tile)
                finally:plant.close()
    montage=np.vstack([np.hstack(panels[i:i+6]) for i in range(0,len(panels),6)])
    cv2.imwrite(str(out/"native-probe-display-montage.png"),montage)
    counts={"cases":len(cases),"range_qualified":sum(c["component_range_qualified"] for c in cases),
            "image_quality_usable":sum(c["generic_image_measurements"][-1]["usable"] for c in cases),
            "conditional_flow_valid":sum(c["conditional_flow_valid"] for c in cases),
            "range_ok_but_texture_bad":sum(c["component_range_qualified"] and not c["generic_image_measurements"][-1]["usable"] for c in cases)}
    paths=[Path(__file__),ROOT/"scripts/mission3/flow_texture.py",ROOT/"scripts/mission3/physical_scene.py",profile]
    report={"kind":"generic virtual downward image diagnostic","counts":counts,"cases":cases,
            "settings":texture,"source_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            "actual_pmw3901_model_verified":False,"aircraft_state_prescribed":True,"controller_in_loop":False,
            "mission_success_count":0,"qualified_success_count":0,"claims":"texture diagnostic fixture only; no chip SQUAL/shutter calibration or flight validation"}
    (out/"report.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(counts,indent=2));print(out)

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile",type=Path,default=ROOT/"config/mission3-flow-texture-sensitivity.json")
    parser.add_argument("--out",type=Path)
    args=parser.parse_args()
    out=args.out or ROOT/"artifacts/mission3/flow-texture"/datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    run(out,args.profile)
