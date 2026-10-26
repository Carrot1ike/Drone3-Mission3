#!/usr/bin/env python3
"""Conditional full Mission3: image/sensor control -> official PID -> physics.

Aircraft state and contacts are evaluation-only. Unknown assembled dynamics
remain sensitivity settings, so even supported contact is not a qualified
manufacturer-matched landing. No external resources are changed.
"""
import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time
from zoneinfo import ZoneInfo
os.environ.setdefault("MUJOCO_GL","egl")
import cv2
import numpy as np
from mission3.settings import ROOT,load,controller_settings
from mission3.physical_scene import PhysicalPlant
from mission3.physical_estimator import PhysicalMotion
from mission3.flight_controller import StockPid
from mission3.position_bridge import PCPositionPid
from mission3.vision import Observer
from mission3.controller import Mission3,Command
from mission3.abort_delivery import AbortDelivery,finite_control_state
from mission3.handoff import HoverHandoff
from mission3.recording import frame_sha256,open_lossless_writer
from mission3.touchdown import TouchdownObserver
from mission3.track import wrap
from mission3.passage import PassageStop


def frozen_experiment_settings(cfg,physical,strict_focus):
    """Model/settings identity; trial entry and RNG seed have separate identity.

    A holdout series must retain the same config hash across different seeds.
    Including the seed in that hash would make every frozen series look like
    a controller/model change. Historical manifests remain untouched.
    """
    return {"mission":cfg,"physical":physical,"strict_focus":strict_focus}


def run(cfg,physical,out,entry_id,seed,strict_focus=True,record=False,wall_deadline=None,
        split="training",series_id="conditional-flight-development"):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    public=controller_settings(cfg)
    # Explicit conditional software limits, not manufacturer capabilities.
    public["control"].update(physical.get("controller_software",{}))
    public["observer"].update(physical.get("observer_software",{}))
    if physical.get("observer_optics_sensitivity"):
        public["camera"]["observer_optics_sensitivity"]=physical["observer_optics_sensitivity"]
    public["control"]["terminal_policy"]="last_fix_bridge"
    entry=next(e for e in public["entry"]["positions"] if e["id"]==entry_id)
    architecture=physical.get("control_architecture",{})
    rpyt_mode=architecture.get("mode")=="pc_velocity_rpyt"
    if rpyt_mode:
        from mission3_rpyt_protocol import encode_firmware_rpyt
    plant=PhysicalPlant(cfg,physical,out,entry_id,seed)
    own=PhysicalMotion(public,entry,physical,plant.camera_rotation)
    observer=Observer(public,own);mission=Mission3(public,entry);pid=StockPid()
    handoff_settings=physical.get("mission_handoff")
    handoff=HoverHandoff(entry,handoff_settings) if handoff_settings else None
    handoff_active=handoff is not None
    if not handoff_active and cfg["vehicle_plant_only"].get("motion"):plant.start_vehicle_motion(0.)
    handoff_at=None if handoff_active else 0.
    touchdown=TouchdownObserver(physical["contact_observer"])
    position_pid=PCPositionPid(own.position) if rpyt_mode else None
    wire_payload=None;telemetry_force=np.zeros(4)
    if rpyt_mode:
        wire_payload=encode_firmware_rpyt(0.,0.,0.,0,stop=True)
        # Explicit cold-start zero packet, not a real in-flight Mission2 state.
        pid.update_rpyt(0,[0.,0.,math.degrees(own.heading)],[0.,0.,0.],[0.,0.,1.],wire_payload,
                        supply_voltage=physical["motor"]["supply_voltage_V"])
    files={n:(out/n).open("w") for n in ("truth.jsonl","sensors.jsonl","deck_samples.jsonl","motors.jsonl","commands.jsonl","observations.jsonl","camera_frames.jsonl","estimates.jsonl","rpyt_packets.jsonl")}
    video=open_lossless_writer(out/"camera.avi",cfg["timing_experiment"]["render_fps"]) if record else None
    if record:
        (out/"camera-recording.json").write_text(json.dumps({
            "codec":"FFV1","lossless":True,"width":320,"height":320,"encoding":"mono8",
            "hash_source":"SHA256 of exact uint8 controller-input pixels before encoding; camera_frames.jsonl",
            "scope":"local archival format only; actual AI-deck transport encoding/rate/latency unverified"},indent=2)+"\n")
    experimental={"mission":cfg,"physical":physical,"strict_focus":strict_focus,"entry_id":entry_id,"seed":seed}
    (out/"experiment.json").write_text(json.dumps(experimental,indent=2)+"\n")
    (out/"frozen-experiment-settings.json").write_text(json.dumps(frozen_experiment_settings(cfg,physical,strict_focus),indent=2)+"\n")
    (out/"controller-settings.json").write_text(json.dumps(public,indent=2)+"\n")
    paths=[Path(__file__).resolve()]+list((ROOT/"scripts/mission3").glob("*.py"))+list((ROOT/"scripts/mission3").glob("*_host.c"))+[ROOT/"scripts/yaw_core.py",ROOT/"scripts/aruco_pose.py"]
    if rpyt_mode:paths.append(ROOT/"scripts/mission3_rpyt_protocol.py")
    manifest={"schema":"mission3-run/1","run_id":out.name,"author":"main-flight-adapter","run_kind":"mission3_closed_loop","evidence_level":"L2","physics_contact":True,
              "completed":False,"entry_id":entry_id,"seed":seed,"split":split,"series_id":series_id,"code_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
              "config_sha256":hashlib.sha256((out/"frozen-experiment-settings.json").read_bytes()).hexdigest(),
              "config_identity_file":"frozen-experiment-settings.json",
              "trial_metadata_sha256":hashlib.sha256((out/"experiment.json").read_bytes()).hexdigest(),"controller_settings_file":"controller-settings.json",
              "mission_timeout_s":public["control"]["mission_timeout_s"],"handoff_time_s":0.,"vehicle_nominal_speed_m_s":cfg["vehicle_plant_only"]["speed_m_s"],
              "descent_phases":["BLIND_DESCEND","DESCEND","TOUCHDOWN_DESCEND"],"controller_in_loop":True,"started_from":"HANDOFF_IN","aircraft_center_definition":"conditional aircraft CoM/body freejoint origin",
              "aircraft_profile":{},"geom_class_map":{**plant.geom_class_map,"landing_pad":"pad","marker_board":"marker_board","floor":"floor"},
              "profile_scope":"conditional assembled mass sum/box inertia/mounts/motor response/finite ray sensor hypotheses; manufacturer match false"}
    manifest["control_architecture"]={"mode":architecture.get("mode","host_velocity_pid"),
                                      "software_settings":architecture,"hardware_link":False,
                                      "cold_start_at_height_not_live_mission2_handoff":True,
                                      "mission3_after_sensor_stable_hover_preroll":handoff_active,
                                      "live_mission2_route_executed":False}
    manifest["handoff_fixture"]={"enabled":handoff_active,"settings":handoff_settings,
        "state_reset_at_mission3_start":False,"vehicle_motion_continuous":True,
        "scope":"conditional sensor-controlled hover fixture, not a Mission2 route or hardware handoff"}
    manifest["mission3_phase_entered"]=not handoff_active
    if cfg["vehicle_plant_only"].get("motion"):
        manifest["vehicle_motion_schedule"]={"settings":cfg["vehicle_plant_only"]["motion"],
            "start_time_s":None if handoff_active else 0.,"plant_only":True,
            "scope":"user hover-ready trigger; acceleration/deceleration are explicit experiment assumptions"}
        manifest["handoff_fixture"]["vehicle_motion_continuous"]=False
    required=("assembled_mass_kg","assembled_center_of_mass_m","assembled_inertia_kg_m2","pwm_to_thrust_model","motor_response_model","aerodynamic_model","camera_K","camera_distortion","R_body_camera","camera_position_body_m","camera_capture_timestamp","flow_deck_position_body_m","flow_fov_deg","flow_min_height_m","tof_fov_deg","tof_min_distance_m","landing_contact_geometry")
    for k in required:manifest["aircraft_profile"][k]={"value":None,"source_class":"unresolved","source_ref":"docs/mission3/hardware-sensor-evidence.md"}
    conditional={"assembled_mass_kg":physical["aircraft_contact_case"]["mass_kg"],"assembled_inertia_kg_m2":plant.contact_description["derived"]["inertia_kg_m2"],
                 "motor_response_model":physical["motor"],"aerodynamic_model":physical["aerodynamics"],"camera_position_body_m":physical["mounts"]["camera_position_body_m"],"flow_deck_position_body_m":physical["mounts"]["flow_position_body_m"],"landing_contact_geometry":physical["aircraft_contact_case"]}
    for k,v in conditional.items():manifest["aircraft_profile"][k]={"value":v,"source_class":"sensitivity_assumption","source_ref":"experiment.json.physical"}
    # Preserve the exact source and loaded-library identities before the run;
    # later improvements cannot make an earlier run impossible to review.
    for pth in paths:
        target=out/"source-snapshot"/pth.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(pth,target)
    manifest["runtime_library_sha256"]={}
    libraries=[("mission3-flight-controller","libmission3_controller.so"),("mission3-firmware-host","libmission3_estimator.so")]
    if rpyt_mode:libraries.append(("mission3-position-bridge","libmission3_position_bridge.so"))
    for folder,filename in libraries:
        for name in (filename,"source-manifest.json","autoconf.h","FreeRTOS.h"):
            if not (ROOT/"build"/folder/name).exists():continue
            pth=ROOT/"build"/folder/name;target=out/"source-snapshot"/pth.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(pth,target)
            manifest["runtime_library_sha256"][str(pth.relative_to(ROOT))]=hashlib.sha256(pth.read_bytes()).hexdigest()
    (out/"run_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    pending=deque();next_image=0.;command=None;own_state=None
    passage=PassageStop(cfg) if cfg.get("trial_termination",{}).get("stop_after_unrecognized_fov_passage") else None
    decoded_target_seen=False
    failure=None;rendered=accepted=0;verdicts={};events=[];before="HANDOFF_PREROLL" if handoff_active else "WAIT"
    terminal=None;motor_off_at=None;probe_at=None;hold_sensor_samples=[];thrust_multiplier=1.
    abort_delivery=None;abort_blocked=None;abort_finish=None
    start=time.monotonic();max_tilt=0.;truth_contacts_seen=0
    latest_range=None;latest_flow=None;range_count=flow_count=0
    dt=plant.model.opt.timestep
    try:
        # A failed low terminal may finish its climb after mission timeout;
        # this is recovery only and cannot turn a timeout into success.
        recovery_harness_grace=10.
        preroll_budget=handoff_settings["max_duration_s"] if handoff else 0.
        for tick in range(1,int((public["control"]["mission_timeout_s"]+preroll_budget+recovery_harness_grace)/dt)+1):
            now=plant.time
            if wall_deadline is not None and datetime.now(ZoneInfo("Asia/Seoul"))>=wall_deadline:failure="session_wall_deadline";break
            imu=plant.imu();own.imu(tick,imu)
            calibration_permitted=(terminal is None and own_state is not None
                and own_state["flow_fusion_reason"]=="floor_patch_clear"
                and own_state.get("range_accurate",False)
                and own.position[2]>public["landing_geometry"]["pad_height_m"]+public["marker"]["board_side_m"]+.15
                and math.hypot(own.roll,own.pitch)<math.radians(10))
            if not rpyt_mode or tick%10==1:
                touchdown.calibrate_freeflight(imu,calibration_permitted);touchdown.imu(imu)
            if tick%25==1:
                latest_range=plant.range_packet();range_count+=1
                files["deck_samples.jsonl"].write(json.dumps({"t":now,"kind":"tof_40hz",**latest_range})+"\n")
            if tick%10==1:
                latest_flow=plant.flow_packet(.01);flow_count+=1
                files["deck_samples.jsonl"].write(json.dumps({"t":now,"kind":"flow_100hz",**latest_flow})+"\n")
                deck={**latest_flow,**latest_range,"stamp_s":now,
                      "range_new":own.last_stamp is None or latest_range["range_stamp_s"]>getattr(own,"last_tof_stamp",-1.)}
                own_state,packet=own.state_from_deck(deck)
                if deck["range_new"]:own.last_tof_stamp=latest_range["range_stamp_s"]
                files["sensors.jsonl"].write(json.dumps({"t":now,"imu":imu,"deck":deck,"observer_packet":packet,"estimate":own_state})+"\n")
            if tick%20==1:
                state=None
                prior_velocity_command=mission.last_v.copy()
                prior_yaw_command=mission.last_yaw_rate
                abort_finish=None
                if abort_delivery is not None:
                    command=Command("ABORT",[0.,0.,0.],0.,failure=abort_delivery.reason)
                    abort_finish=abort_delivery.observe(now,own_state,deck["stamp_s"])
                elif handoff_active:
                    command,ready=handoff.update(now,own_state)
                    if not ready:
                        limited=mission.output(command.velocity_world_m_s,command.yaw_rate_rad_s,.02,
                                               urgent=command.done)
                        command.velocity_world_m_s=limited.velocity_world_m_s
                        command.yaw_rate_rad_s=limited.yaw_rate_rad_s
                    if ready:
                        handoff_active=False;handoff_at=now
                        if cfg["vehicle_plant_only"].get("motion") and plant.start_vehicle_motion(now):
                            manifest["vehicle_motion_schedule"]["start_time_s"]=now
                            events.append({"t":now,"phase":"VEHICLE_START","reason":"sensor-stable hover triggers plant only; vehicle speed remains estimated from images"})
                        next_mission=Mission3(public,entry,start_time_s=now)
                        # The phase clock changes, the physical command history does not.
                        # Preserve slew state across this continuous-flight boundary.
                        next_mission.last_v=mission.last_v.copy()
                        next_mission.last_yaw_rate=mission.last_yaw_rate
                        mission=next_mission
                        next_image=now
                        manifest["handoff_time_s"]=now
                        manifest["mission3_phase_entered"]=True
                        manifest["handoff_fixture"]["sensor_readiness"]=handoff.latest
                        (out/"run_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
                        events.append({"t":now,"phase":"HANDOFF_IN","reason":"continuous sensor-stable hover; only mission clock initialized, no PID/motor/estimator reset"})
                        print(f"t={now:.2f} sensor-stable Mission3 handoff",flush=True)
                if not handoff_active and abort_delivery is None:
                    if now>=next_image-1e-8:
                        gray,scope=plant.image(strict_focus);rendered+=1
                        if gray is not None:
                            files["camera_frames.jsonl"].write(json.dumps({"t":now,"width":320,"height":320,"encoding":"mono8","channels":1,"resized":False,"color_converted":False,"scope":scope,
                                "controller_input_sha256":frame_sha256(gray)})+"\n")
                            if video is not None:video.write(gray)
                            pending.append((now+cfg["timing_experiment"]["transport_delay_s"],now,gray))
                            if not(out/f"camera-{before}.png").exists():cv2.imwrite(str(out/f"camera-{before}.png"),gray)
                        else:verdicts[scope]=verdicts.get(scope,0)+1
                        next_image+=1/cfg["timing_experiment"]["render_fps"]
                    while pending and pending[0][0]<=now+1e-8:
                        _,capture,gray=pending.popleft();result=observer.frame(gray,capture,now)
                        accepted+=int(result["valid"]);verdicts[result["reason"]]=verdicts.get(result["reason"],0)+1
                        decoded_target_seen=decoded_target_seen or "corners" in result
                        files["observations.jsonl"].write(json.dumps({"t":now,"phase":before,**result})+"\n")
                    if passage is not None:
                        end_reason=passage.update(plant,now,decoded_target_seen)
                        if end_reason:
                            failure=end_reason
                            events.append({"t":now,"phase":"TEST_END","reason":end_reason,"evaluation_only":passage.latest})
                            break
                    if (cfg.get("trial_termination",{}).get("stop_at_vehicle_endpoint")
                            and plant.vehicle_motion_phase=="stopped_at_endpoint"):
                        failure="vehicle_reached_straight_endpoint_without_completed_landing"
                        events.append({"t":now,"phase":"TEST_END","reason":failure,"scope":"test harness endpoint; not a flight command"})
                        break
                    state=observer.state(now)
                    own_state["inertial_support_handoff_ready"]=bool(touchdown.calibration_is_fresh(now)
                        and touchdown.latest is not None and now-touchdown.latest["t"]<=.02
                        and math.hypot(own.roll,own.pitch)<math.radians(physical["contact_observer"]["tilt_limit_deg"]))
                    if terminal is None:
                        command=mission.update(now,own_state,state)
                        if command.phase=="GEOMETRY_READY":
                            # Optical completion emits zero, but this physical
                            # adapter continues descent. Limit from the prior
                            # actual command, not from an intermediate zero.
                            mission.last_v=prior_velocity_command
                            mission.last_yaw_rate=prior_yaw_command
                            terminal=now;events.append({"t":now,"phase":"TOUCHDOWN_DESCEND","reason":"continue from near-pad geometry with body sensor support evidence; no contact truth",
                                                       "handoff_mode":mission.support_handoff_mode or "relative_flow_geometry"})
                    if terminal is not None:
                        # No geometric contact flag reaches this control branch.
                        terminal_failure=mission.terminal_safety_reason(now,own_state,state)
                        if (terminal_failure is None and mission.support_handoff_mode=="conditional_inertial_texture_bridge"
                                and (state["age_s"]>=physical["contact_observer"].get("inertial_bridge_max_prediction_s",6.)
                                     or now-terminal>=physical["contact_observer"].get("inertial_bridge_max_prediction_s",6.))):
                            terminal_failure="inertial_texture_bridge_prediction_expired"
                        if motor_off_at is not None:
                            vcmd=np.zeros(3);yawcmd=0.;phase="MOTOR_OFF_HOLD"
                            stable,hold_failed=touchdown.hold_supported(now,own.roll,own.pitch)
                            hold_sensor_samples.append({"t":now,"stable":stable,**touchdown.latest,
                                                        "range_accurate":own_state["range_accurate"],"flow_valid":own_state["flow_valid"]})
                            if hold_failed:
                                failure="motor_off_hold_sensor_support_unconfirmed";break
                        elif terminal_failure is not None:
                            reason=terminal_failure
                            mission.last_time=now
                            command=mission.start_blind_recovery(reason,now,own_state,.02)
                            terminal=None;probe_at=None;thrust_multiplier=1.
                            events.append({"t":now,"phase":command.phase,"reason":reason})
                        else:
                            rel=np.asarray(state["relative_pad_world_m"]);rv=np.asarray(state["relative_velocity_world_m_s"])
                            vcmd=np.asarray(state["vehicle_velocity_world_m_s"])+public["control"]["terminal_xy_kp_per_s"]*rel+.35*rv
                            vcmd[2]=-physical["contact_observer"]["descent_speed_m_s"];phase="TOUCHDOWN_DESCEND"
                            yawcmd=np.clip(1.5*wrap(state["vehicle_heading_rad"]-own.heading),-.2,.2)
                            c=physical["contact_observer"]
                            # Below the component envelope only IMU propagation
                            # remains. This estimated height starts a bounded
                            # thrust probe; it does not prove physical contact.
                            height=own.position[2]-public["landing_geometry"]["pad_height_m"]
                            if height<c["probe_start_body_height_above_pad_m"] and probe_at is None:
                                probe_at=now;events.append({"t":now,"phase":"SUPPORT_PROBE","reason":"IMU-propagated near-pad height; conditional collective attenuation"})
                            if probe_at is not None:
                                thrust_multiplier=1-(1-c["probe_multiplier_min"])*min(1.,(now-probe_at)/c["probe_ramp_s"])
                            near_xy=np.linalg.norm(rel[:2])<public["control"]["track_xy_gate_m"]
                            enabled=near_xy and now-terminal>.5
                            if touchdown.contact_candidate(now,own.roll,own.pitch,enabled,height):
                                motor_off_at=now;phase="MOTOR_OFF_HOLD";vcmd=np.zeros(3);yawcmd=0.
                                events.append({"t":now,"phase":phase,"reason":"sustained IMU specific-force minus commanded motor-model force support candidate; no near-range precision or contact truth"})
                        if terminal is not None:
                            command=mission.output(vcmd,yawcmd,.02,urgent=motor_off_at is not None)
                            command.phase=phase
                            command.done=False;command.motors_off=motor_off_at is not None
                    own.terminal_hint=state if before in ("BLIND_APPROACH","BLIND_DESCEND","TOUCHDOWN_DESCEND","MOTOR_OFF_HOLD") or command.phase in ("BLIND_APPROACH","BLIND_DESCEND","TOUCHDOWN_DESCEND","MOTOR_OFF_HOLD") else None
                if command.done:
                    failure=command.failure or "mission_terminated"
                    if finite_control_state(own_state):
                        # Latch failure, then actually deliver hover/brake commands.
                        # This does not prove safe recovery and never sends airborne STOP.
                        abort_delivery=AbortDelivery(now,failure)
                        command=Command("ABORT",[0.,0.,0.],0.,failure=failure)
                        terminal=None;thrust_multiplier=1.;own.terminal_hint=None
                        abort_finish=abort_delivery.observe(now,own_state,deck["stamp_s"])
                        events.append({"t":now,"phase":"ABORT","reason":failure,"delivery_started":True})
                    else:
                        abort_blocked={"t":now,"reason":"nonfinite_control_state_not_sent_to_PID","failure_latched":failure}
                command_record=asdict(command)
                if abort_delivery is not None:command_record["abort_delivery"]=abort_delivery.summary()
                if command.search_guard is None:command_record.pop("search_guard")
                if command.approach_reference is None:command_record.pop('approach_reference')
                files["commands.jsonl"].write(json.dumps({"t":now,"own":own_state,"vehicle":state,
                    "touchdown_observer":touchdown.latest,"collective_multiplier":thrust_multiplier,**command_record})+"\n")
                truth=plant.truth();files["truth.jsonl"].write(json.dumps(truth)+"\n")
                files["estimates.jsonl"].write(json.dumps({"t":now,"mission_estimate":own_state,"firmware_parallel_attitude":own.firmware_parallel,
                                                         "pc_independent_attitude":own.pc_attitude_state})+"\n")
                truth_contacts_seen+=bool(truth["contacts"])
                if command.phase!=before:
                    print(f"t={now:.2f} {before}->{command.phase}",flush=True);before=command.phase
                if any(c["other"]=="floor" for c in truth["contacts"]):
                    failure="floor_contact_test_harness_stop";break
                if motor_off_at is not None and now-motor_off_at>=3.02:break
                if command.done:failure=command.failure or "mission_terminated";break
            if command is None:raise RuntimeError("control state not initialized")
            if rpyt_mode:
                fresh_packet=tick%10==1
                if fresh_packet:
                    attitude_setpoint=position_pid.update(own.position,own.velocity,own.heading,command.velocity_world_m_s)
                    collective=int(round(min(architecture.get("legacy_collective_cap",60000),
                                             max(0.,attitude_setpoint["legacy_collective"]*thrust_multiplier))))
                    wire_payload=encode_firmware_rpyt(attitude_setpoint["roll_deg"],attitude_setpoint["pitch_deg"],
                                                     math.degrees(command.yaw_rate_rad_s),collective,stop=command.motors_off)
                    if abort_delivery is not None:abort_delivery.note_wire(now)
                    files["rpyt_packets.jsonl"].write(json.dumps({"t":now,**attitude_setpoint,"yaw_rate_deg_s":math.degrees(command.yaw_rate_rad_s),
                            "requested_collective":collective,"stop":command.motors_off,"wire_hex":wire_payload.hex(),
                            "source":"sensor-derived PC outer PID -> wire packet; no plant state"})+"\n")
                output=pid.update_rpyt(tick,*own.onboard_attitude_state(),wire_payload,
                                      supply_voltage=physical["motor"]["supply_voltage_V"],fresh_packet=fresh_packet)
            else:
                lower=own.lower_state()
                if abort_delivery is not None and tick%10==1:abort_delivery.note_wire(now)
                output=pid.update(tick,*lower,command.velocity_world_m_s,math.degrees(command.yaw_rate_rad_s),stop=command.motors_off,
                                  thrust_multiplier=thrust_multiplier,supply_voltage=physical["motor"]["supply_voltage_V"])
            files["motors.jsonl"].write(json.dumps({"t":now,"motor_output":output["motor_pwm"],**output})+"\n")
            if rpyt_mode:
                if tick%10==1:telemetry_force=np.asarray(output["motor_static_force_N"])
                touchdown.predict_motor(telemetry_force,dt)
            else:touchdown.predict_motor(output["motor_static_force_N"],dt)
            plant.step(output,pid.arm_length,pid.thrust_to_torque)
            if abort_delivery is not None:abort_delivery.note_physics_step(plant.time)
            tilt=float(np.rad2deg(np.linalg.norm(plant.rpy[:2])));max_tilt=max(max_tilt,tilt)
            if tilt>60:failure="attitude_exceeds_60deg_test_harness_stop";break
            if abort_finish is not None:break
        if failure is None and (motor_off_at is None or plant.time-motor_off_at<3.0):failure="mission_time_budget_without_supported_hold"
        summary={"duration_s":plant.time,"failure":failure,"phase":before,"rendered_frames":rendered,"accepted_frames":accepted,"verdicts":verdicts,
                 "sensor_delivery_counts":{"flow_100hz":flow_count,"tof_40hz":range_count},
                 "control_architecture":manifest["control_architecture"],"onboard_estimator_measurement_counts":own.onboard_measurement_counts,
                 "motor_off_at_s":motor_off_at,"truth_contact_samples":truth_contacts_seen,"max_tilt_deg":max_tilt,
                 "handoff_time_s":handoff_at,"handoff_sensor_evidence":handoff.latest if handoff else None,
                 "scope":"L2 conditional full mission controller and free rigid-body physics; manufacturer matched false",
                 "manufacturer_matched":False,"qualified_mission_success":False,"physical_success_count":0,
                 "sensor_touchdown_evidence_is_assumption":True,"events":mission.events+events,"surface_events":own.events,
                 "motor_off_hold_sensor_evidence":hold_sensor_samples,
                 "abort_delivery":abort_delivery.summary() if abort_delivery else None,
                 "abort_delivery_blocked":abort_blocked,
                 "decoded_target_seen":decoded_target_seen,
                 "passage_termination_evidence":passage.latest if passage else None,
                 "final_truth_evaluation_only":plant.truth(),"final_estimate":own_state,"runtime_s":time.monotonic()-start}
        (out/"adapter-summary.json").write_text(json.dumps(summary,indent=2)+"\n")
        manifest["completed"]=True
        manifest["termination_reason"]=failure or "hold_window_elapsed"
        (out/"run_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        print(json.dumps({k:summary[k] for k in ("phase","duration_s","failure","accepted_frames","motor_off_at_s","truth_contact_samples","max_tilt_deg","runtime_s")},indent=2),flush=True);print(out,flush=True)
    finally:
        if video is not None:video.release()
        for f in files.values():f.close()
        plant.close()
    return summary


def main():
    p=argparse.ArgumentParser();p.add_argument("--entry",type=int,default=1,choices=range(1,9));p.add_argument("--seed",type=int,default=1)
    p.add_argument("--timeout",type=float);p.add_argument("--video",action="store_true");p.add_argument("--allow-near-focus",action="store_true")
    p.add_argument("--perturb-entry",action="store_true");p.add_argument("--respect-session-deadline",action="store_true")
    p.add_argument("--split",choices=("training","holdout"),default="training")
    p.add_argument("--series",default="conditional-flight-development")
    p.add_argument("--profile",type=Path,default=ROOT/"config/mission3-physical-sensitivity.json");p.add_argument("--out",type=Path)
    p.add_argument("--config",type=Path,default=ROOT/"config/mission3.json")
    a=p.parse_args();cfg=load(a.config);physical=json.loads(a.profile.read_text())
    if a.split=="holdout" and a.series=="conditional-flight-development":
        p.error("holdout requires an explicit frozen --series identity")
    cfg["entry"]["inject_initial_uncertainty"]=a.perturb_entry
    if a.timeout is not None:cfg["control"]["mission_timeout_s"]=a.timeout
    deadline=datetime.fromisoformat(cfg["deadline_kst"]) if a.respect_session_deadline else None
    if deadline is not None and datetime.now(ZoneInfo("Asia/Seoul"))>=deadline:raise RuntimeError("session deadline elapsed")
    stamp=datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d-%H%M%S-%f")
    out=a.out or ROOT/"artifacts/mission3/physical"/f"{stamp}-entry{a.entry}-seed{a.seed}"
    result=run(cfg,physical,out,a.entry,a.seed,not a.allow_near_focus,a.video,deadline,a.split,a.series)
    return 2 if result["failure"] else 0


if __name__=="__main__":raise SystemExit(main())
