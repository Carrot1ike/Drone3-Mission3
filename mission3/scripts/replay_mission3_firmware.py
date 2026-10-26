#!/usr/bin/env python3
"""Manufacturer estimator surface-transition bench; not aircraft simulation."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
from mission3.firmware import FirmwareCore
from mission3.settings import ROOT

def run(out,scale=1.,mode="stock",velocity_error=0.,surface_error=0.,recognition_delay=.05):
    out.mkdir(parents=True,exist_ok=True)
    case={"scope":"official C predict, process noise, scalar updates, finalize; prescribed sensor fixture; no flight controller or physics",
          "rates_hz":{"host_queue":1000,"process_noise":1000,"predict":100,"flow":100,"tof":40},
          "source":"src/bitcraze-mission3-firmware pinned f6e0f0a3, unmodified",
          "scale_factor":scale,"noise":False,"mode":mode,
          "estimated_velocity_error_m_s":velocity_error,"estimated_surface_height_error_m":surface_error,
          "recognition_delay_s":recognition_delay,
          "fixture":"optical/sensor height .50m, velocity .25m/s, at 3s observe .30m high pad moving .25m/s; at 7s return floor",
          "sensor_offsets":"zero experimental probe; not actual mount",
          "surface_id_to_core":False,"vehicle_velocity_to_core":False,
          "adapter_scope":"policy/measurement-equation fixture using delayed corrupted assumed estimates, not a validated scene observer",
          "physical_success":False}
    core=FirmwareCore()
    rows=[]
    for ms in range(10001):
        t=ms*.001;onpad=3<=t<7
        height=.5-(.3 if onpad else 0.)
        relative_velocity=.0 if onpad else .25
        estimated_onpad=3+recognition_delay<=t<7+recognition_delay
        estimated_surface=.3+surface_error if estimated_onpad else 0.
        estimated_surface_velocity=.25+velocity_error if estimated_onpad else 0.
        if ms and ms%10==0:core.predict(ms)
        core.process_noise(ms)
        if ms%25==0:
            std=.0025*(1+np.exp(2.9214*(height-2.5)))
            distance=height+estimated_surface if mode=="corrected" else height
            if not(mode=="gate_moving" and estimated_onpad):core.tof(ms,distance,float(std))
        if ms%10==0:
            raw=.01*35/.71674/.1*relative_velocity/height*scale
            # Driver quality/outlier gate is a fixture assumption: textured
            # reliable surface and count below100. It is not simulated texture.
            if mode=="corrected":
                world_height=max(height+estimated_surface,.10)
                raw=raw*height/world_height+.01*35/.71674/.1*estimated_surface_velocity/world_height
            if abs(raw)<100 and not(mode=="gate_moving" and estimated_onpad):core.flow(ms,(raw,0.))
        core.finalize()
        if ms%10==0:
            rows.append({"time_s":t,"true_sensor_world_height_m":.5,"true_velocity_world_m_s":[.25,0.,0.],
                         "true_surface_elevation_m":.3 if onpad else 0.,
                         "true_surface_velocity_world_m_s":[.25 if onpad else 0.,0.,0.],
                         "range_input_m":height,"flow_relative_velocity_fixture_m_s":relative_velocity,
                         "estimated_surface_elevation_fixture_m":estimated_surface,
                         **core.state()})
    (out/"experiment.json").write_text(json.dumps(case,indent=2)+"\n")
    (out/"replay.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
    sample=lambda t:min(rows,key=lambda r:abs(r["time_s"]-t))
    report={"scope":case["scope"],"scale_factor":scale,"mode":mode,
            "before_transition":sample(2.9),"pad_steady":sample(6.9),"floor_return":sample(9.9),
            "maximum_estimated_vertical_speed_m_s":max(abs(r["velocity_world_m_s"][2]) for r in rows),
            "physical_success":False}
    (out/"evaluation.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({"output":str(out),"mode":mode,"scale":scale,
                      "pad_z_m":report["pad_steady"]["position_world_m"][2],
                      "pad_vx_m_s":report["pad_steady"]["velocity_world_m_s"][0],
                      "peak_estimated_vz_m_s":report["maximum_estimated_vertical_speed_m_s"],
                      "scope":case["scope"]},indent=2));return report

def main():
    p=argparse.ArgumentParser();p.add_argument("--scale",type=float,default=1.);p.add_argument("--out",type=Path)
    p.add_argument("--mode",choices=("stock","corrected","gate_moving"),default="stock")
    p.add_argument("--velocity-error",type=float,default=0.);p.add_argument("--surface-error",type=float,default=0.)
    p.add_argument("--recognition-delay",type=float,default=.05)
    a=p.parse_args();now=datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    run(a.out or ROOT/"artifacts/mission3"/f"{now}-firmware-transition-{a.mode}",a.scale,a.mode,
        a.velocity_error,a.surface_error,a.recognition_delay)

if __name__=="__main__":main()
