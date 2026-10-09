#!/usr/bin/env python3
"""Offline graphics only: replay saved body poses for review, never control."""
import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from pathlib import Path
import os
os.environ.setdefault("MUJOCO_GL","egl")
import cv2
import mujoco
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle

plt.rcParams.update({"font.family":"NanumGothic","axes.unicode_minus":False,"font.size":10})


def rows(path):return [json.loads(l) for l in Path(path).open()]


def render_part(args):
    return review(*args)


def review(run,video=False,playback_speed=1.,video_fps=5.,workers=1,part_index=0,parts=1,start_s=0.,end_s=None):
    if playback_speed <= 0 or video_fps <= 0:
        raise ValueError("playback speed and video FPS must be positive")
    if not math.isfinite(start_s) or start_s < 0 or (end_s is not None and (not math.isfinite(end_s) or end_s <= start_s)):
        raise ValueError("finite positive review interval required")
    run=Path(run);truth=rows(run/"truth.jsonl");cmd=rows(run/"commands.jsonl")
    t=np.array([r["t"] for r in truth]);p=np.array([r["aircraft"]["center_world_m"] for r in truth]);pad=np.array([r["pad"]["center_world_m"] for r in truth])
    own=np.array([r["own"]["position_world_m"] for r in cmd]);vel=np.array([r["aircraft"]["velocity_world_m_s"] for r in truth])
    pv=np.array([r["vehicle"]["velocity_world_m_s"] for r in truth]);error=np.linalg.norm((p-pad)[:,:2],axis=1)
    n=min(len(t),len(cmd));t,p,pad,own,vel,pv,error=t[:n],p[:n],pad[:n],own[:n],vel[:n],pv[:n],error[:n]
    ev=json.loads((run/"offline-evaluation.json").read_text()) if (run/"offline-evaluation.json").exists() else None
    summary=json.loads((run/"adapter-summary.json").read_text())
    motors=[]
    for row in (run/"motors.jsonl").open():
        r=json.loads(row)
        if len(motors)==0 or r["t"]-motors[-1][0]>=.019999:motors.append((r["t"],max(r["motor_output"])))
    mt=np.array(motors)
    if parts==1:
        fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True,layout="constrained")
        axes[0].plot(t,p[:,2],label="기체 실제 높이");axes[0].plot(t,own[:,2],label="센서 추정 높이");axes[0].axhline(.3,ls=":",color="gray",label="패드 높이 .30m");axes[0].legend()
        axes[1].plot(t,error,label="기체–패드 수평거리");axes[1].set_ylim(0,max(1.,float(error.max())*1.05));axes[1].legend()
        axes[2].plot(t,np.linalg.norm((vel-pv)[:,:2],axis=1),label="참 상대 수평속력");axes[2].plot(t,np.linalg.norm(p-own,axis=1),label="자기 위치 추정오차 (평가 전용)");axes[2].set_ylim(0,.6);axes[2].legend()
        axes[3].plot(mt[:,0],mt[:,1],label="최대 모터 PWM");axes[3].legend();axes[3].set_xlabel("시뮬레이션 시간 (s)")
        for a,label in zip(axes,("높이 (m)","수평거리 (m)","속력/오차", "모터 PWM")):
            a.set_ylabel(label);a.grid(alpha=.2)
            if summary["motor_off_at_s"] is not None:a.axvspan(summary["motor_off_at_s"],summary["motor_off_at_s"]+3,color="green",alpha=.1)
        fig.suptitle("미션 3 조건부 물리 시험 | 공식 PID → 추력·토크 → 자유 기체·패드 접촉\n미확정 기체·센서 모델 포함 / 제조사 사양 일치·실기 성공 미검증",fontweight="bold")
        fig.savefig(run/"physical-review-metrics.png",dpi=150);plt.close(fig)
    if not video:return
    if workers > 1:
        jobs=[(run,True,playback_speed,video_fps,1,i,workers,start_s,end_s) for i in range(workers)]
        # Separate EGL/Matplotlib contexts; never split one live control history.
        with ProcessPoolExecutor(max_workers=workers,mp_context=multiprocessing.get_context('spawn')) as pool:
            segments=list(pool.map(render_part,jobs))
        writer=cv2.VideoWriter(str(run/"physical-review.mp4"),cv2.VideoWriter_fourcc(*"mp4v"),video_fps,(1280,720))
        if not writer.isOpened():raise RuntimeError("video codec unavailable")
        count=0
        for segment in segments:
            capture=cv2.VideoCapture(segment['path']);read_count=0
            while True:
                ok,bgr=capture.read()
                if not ok:break
                writer.write(bgr);count+=1;read_count+=1
            capture.release()
            if read_count != segment['frames']:raise RuntimeError("parallel segment frame mismatch")
        writer.release()
        (run/"video-timing.json").write_text(json.dumps({"video_fps":video_fps,"frames":count,
            "playback_speed":playback_speed,"video_duration_s":count/video_fps,"simulation_end_s":float(t[-1]),
            "native_camera_fps":20.,"arena_m":[json.loads((run/'experiment.json').read_text())['mission']['arena'][key] for key in ('length_m','width_m')],
            "review_start_s":start_s,"review_end_s":min(float(t[-1]),end_s) if end_s is not None else float(t[-1]),
            "render_workers":workers,"segments":segments},indent=2)+"\n")
        print(run/"physical-review.mp4",flush=True)
        return
    model=mujoco.MjModel.from_xml_path(str(run/"physical-scene.xml"));data=mujoco.MjData(model)
    aq=int(model.joint("aircraft_free").qposadr[0]);vq=int(model.joint("vehicle_free").qposadr[0])
    renderer=mujoco.Renderer(model,height=320,width=480)
    third=mujoco.MjvCamera();third.type=mujoco.mjtCamera.mjCAMERA_FREE;third.distance=1.05;third.azimuth=220.;third.elevation=-22.
    camera_rows=rows(run/"camera_frames.jsonl");camera_t=np.array([r["t"] for r in camera_rows])
    capture=cv2.VideoCapture(str(run/"camera.avi"));camera_frames=[]
    while True:
        ok,bgr=capture.read()
        if not ok:break
        camera_frames.append(cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY))
    capture.release()
    if len(camera_frames)!=len(camera_rows):raise RuntimeError("native camera frame/timestamp mismatch")
    video_path=run/("physical-review.mp4" if parts==1 else f"physical-review-part-{part_index:02d}.mp4")
    writer=cv2.VideoWriter(str(video_path),cv2.VideoWriter_fourcc(*"mp4v"),video_fps,(1280,720))
    if not writer.isOpened():raise RuntimeError("video codec unavailable")
    fig=plt.figure(figsize=(12.8,7.2),dpi=100,facecolor="#f7f8fb")
    axmap=fig.add_axes([.05,.62,.51,.29]);ax3d=fig.add_axes([.05,.05,.51,.46]);axcam=fig.add_axes([.66,.38,.28,.48]);axtxt=fig.add_axes([.61,.06,.37,.30])
    last_poster=None
    cfg=json.loads((run/"experiment.json").read_text())["mission"]
    length,width=cfg["arena"]["length_m"],cfg["arena"]["width_m"]
    frame_count=0
    interval_end=min(float(t[-1]),end_s) if end_s is not None else float(t[-1])
    if start_s > interval_end:raise ValueError("review interval outside saved history")
    stamps=np.arange(start_s,interval_end+.001,playback_speed/video_fps)
    begin,end=len(stamps)*part_index//parts,len(stamps)*(part_index+1)//parts
    for stamp in stamps[begin:end]:
        i=min(np.searchsorted(t,stamp),n-1);r=truth[i];c=cmd[i]
        for a in (axmap,ax3d,axcam,axtxt):a.clear()
        axmap.set_aspect("equal");axmap.grid(alpha=.15)
        cx,cy=cfg["track"]["center_world_xy_m"]
        lo,hi=cx-cfg["track"]["straight_length_m"]/2,cx+cfg["track"]["straight_length_m"]/2
        if cfg["track"].get("kind") == "straight":
            endpoints=np.asarray([cfg["track"]["start_world_xy_m"],cfg["track"]["end_world_xy_m"]])
            axmap.plot(endpoints[:,0],endpoints[:,1],"--",color="gray",lw=1)
        else:
            rad=cfg["track"]["turn_diameter_m"]/2
            a=np.linspace(-math.pi/2,math.pi/2,100)
            for yy in (cy-rad,cy+rad):axmap.plot([lo,hi],[yy,yy],"--",color="gray",lw=1)
            axmap.plot(hi+rad*np.cos(a),cy+rad*np.sin(a),"--",color="gray",lw=1);axmap.plot(lo-rad*np.cos(a),cy+rad*np.sin(a),"--",color="gray",lw=1)
        axmap.add_patch(Rectangle((0,0),length,width,fill=False,ls=":",color="#555"));axmap.plot(p[:i+1,0],p[:i+1,1],color="#087f8c",lw=1.5)
        axmap.scatter(p[i,0],p[i,1],c="#087f8c",s=25);axmap.scatter(pad[i,0],pad[i,1],c="#e1812c",s=25)
        heading=r["aircraft"]["rpy_rad"][2];half=math.radians(43.5)
        ends=p[i,:2]+2*np.array([[math.cos(heading-half),math.sin(heading-half)],[math.cos(heading+half),math.sin(heading+half)]])
        axmap.add_patch(Polygon(np.vstack((p[i,:2],ends)),color="#087f8c",alpha=.1))
        axmap.set(xlim=(-1,length+1),ylim=(-1,width+1),title=f"평가 지도: 실내 {length:g}×{width:g}m / 삼각형은 화각 87° (인식 한계 아님)")
        # Pose writes occur in this independent saved-log visualization only.
        data.qpos[aq:aq+7]=np.r_[p[i],r["aircraft"]["quaternion_wxyz"]]
        vh=r["vehicle"]["heading_rad"];data.qpos[vq:vq+7]=np.r_[pad[i],[math.cos(vh/2),0,0,math.sin(vh/2)]]
        mujoco.mj_forward(model,data)
        third.lookat[:]=pad[i]+[0,0,.05] if error[i]<.7 else p[i]
        third.distance=1.05 if error[i]<.7 else 1.5
        renderer.update_scene(data,camera=third);ax3d.imshow(renderer.render());ax3d.set_axis_off();ax3d.set_title("제3자 시점: 기체 추적 → 근접 시 패드 접촉 관찰")
        ci=min(int(np.searchsorted(camera_t,stamp,side="right")-1),len(camera_frames)-1)
        if ci>=0:axcam.imshow(camera_frames[ci],cmap="gray",vmin=0,vmax=255)
        axcam.set_axis_off()
        age=stamp-camera_t[ci] if ci>=0 else float("inf")
        axcam.set_title(f"가상 AI-deck 원본 mono8 320×320\n마지막 제공 영상 나이 {age:.2f}s")
        feet={z["aircraft_part"] for z in r["contacts"] if z["other"]=="pad" and z["aircraft_part"].startswith("foot_")}
        motor=mt[min(np.searchsorted(mt[:,0],stamp),len(mt)-1),1]
        speed=np.linalg.norm(vel[i]);desired_speed=np.linalg.norm(c['velocity_world_m_s'])
        text=f"시간 {t[i]:.2f}s / {c['phase']}\n기체 높이 {p[i,2]:.3f}m · 패드 높이 .300m\n패드 중심까지 수평 {error[i]*100:.1f}cm\n속력: 명령 {desired_speed:.3f} / 실제 {speed:.3f}m/s\n상대 수평속력 {np.linalg.norm((vel[i]-pv[i])[:2]):.3f}m/s\n패드 접촉 발 {len(feet)}/4 · 최대 PWM {motor:.0f}\n"
        if ev:
            score=ev.get('score')
            points=score.get('points') if score else '없음'
            text+=f"오프라인 접촉 판정 {ev['simulation_contact_success']['status']} · 내부 점수 {points}\n"
        text+="조건부 관성·장착·센서·모터 응답 포함\n제조사 일치 성공 0 / 실기 검증 없음"
        axtxt.axis("off");axtxt.text(0,1,text,va="top",fontsize=11,linespacing=1.5)
        behavior = ("진입 대기 · 제한 탐색 · 마커 추종" if cfg["control"].get("search_motion_policy")=="bounded_entry_raster"
                    else "진입 위치 대기 · 마커 발견 후 이동")
        fig.suptitle(f"미션 3 | {behavior} ({playback_speed:g}배속)",x=.04,ha="left",y=.98,fontweight="bold",fontsize=16)
        fig.canvas.draw();rgba=np.asarray(fig.canvas.buffer_rgba());writer.write(cv2.cvtColor(rgba,cv2.COLOR_RGBA2BGR))
        frame_count+=1
        if frame_count==1 and part_index==0:
            fig.savefig(run/"physical-review-start.png",dpi=100)
        if frame_count%250==0:print(f"video {stamp:.1f}/{t[-1]:.1f}s",flush=True)
        if summary["motor_off_at_s"] is not None and t[i]>=summary["motor_off_at_s"]+2.9 and last_poster is None:
            fig.savefig(run/"physical-review-poster.png",dpi=100);last_poster=i
    if last_poster is None and part_index==parts-1:fig.savefig(run/"physical-review-poster.png",dpi=100)
    writer.release();renderer.close();plt.close(fig)
    if parts>1:return {"path":str(video_path),"frames":frame_count,"first_global_frame":begin,"last_global_frame_exclusive":end}
    (run/"video-timing.json").write_text(json.dumps({"video_fps":video_fps,"frames":frame_count,
        "playback_speed":playback_speed,"video_duration_s":frame_count/video_fps,
        "simulation_end_s":float(t[-1]),"review_start_s":start_s,"review_end_s":interval_end,
        "native_camera_fps":20.,"arena_m":[length,width]},indent=2)+"\n")
    print(run/"physical-review.mp4")


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("run",type=Path);p.add_argument("--video",action="store_true")
    p.add_argument("--playback-speed",type=float,default=1.);p.add_argument("--video-fps",type=float,default=5.)
    p.add_argument("--workers",type=int,default=1)
    p.add_argument("--start-s",type=float,default=0.)
    p.add_argument("--end-s",type=float)
    a=p.parse_args()
    if a.workers < 1 or a.workers > 16:p.error("workers must be between 1 and 16")
    review(a.run,a.video,a.playback_speed,a.video_fps,a.workers,start_s=a.start_s,end_s=a.end_s)
