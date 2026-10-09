#!/usr/bin/env python3
"""Offline evaluator drawings/video. Truth is never returned to control.

Graphs use saved mono images and JSONL; no simulation replay modifications.
"""
import argparse
import json
import math
from pathlib import Path
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle
import numpy as np

plt.rcParams.update({"font.family":"NanumGothic","axes.unicode_minus":False,"font.size":10})

def read(path):return [json.loads(l) for l in path.open()]

def firmware_plot(root):
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout="constrained")
    cases=(("firmware-transition-consistent","기본 제조사 EKF"),
           ("firmware-transition-corrected","표면 보정 가정 (+속도 0.03m/s, 높이 0.01m 오차)"),
           ("firmware-transition-gated","전환 인식 50ms 후 Flow/ToF 차단"))
    official_rate=(root/"firmware-1khz-stock").exists()
    if official_rate:
        cases=(("firmware-1khz-stock",cases[0][1]),("firmware-1khz-corrected",cases[1][1]),("firmware-1khz-gated",cases[2][1]))
    for folder,label in cases:
        rows=read(root/folder/"replay.jsonl");t=np.array([r["time_s"] for r in rows])
        axes[0].plot(t,[r["position_world_m"][2] for r in rows],label=label)
        axes[1].plot(t,[r["velocity_world_m_s"][0] for r in rows])
        axes[2].plot(t,[r["velocity_world_m_s"][2] for r in rows])
    axes[0].axhline(.5,color="k",linestyle=":",label="실제 높이 0.50m (고정 시험 입력)")
    axes[1].axhline(.25,color="k",linestyle=":")
    axes[2].axhline(0.,color="k",linestyle=":")
    for ax,label in zip(axes,("추정 높이 (m)","추정 세계 vx (m/s)","추정 세계 vz (m/s)")):
        ax.set_ylabel(label);ax.axvspan(3.,7.,color="gray",alpha=.1);ax.grid(alpha=.2)
    axes[0].legend(fontsize=9,loc="lower left");axes[2].set_xlabel("시험 시간 (s), 음영: 이동 패드 관측")
    fig.suptitle("제조사 Kalman C 실행: 표면 전환 시험 (공정잡음 1kHz)\n가상 측정 재생 · 기체 비행/제어/접촉 시험 아님" if official_rate else "제조사 Kalman C 실행: 표면 전환 시험\n가상 측정 재생 · 기체 비행/제어/접촉 시험 아님",fontweight="bold")
    fig.savefig(root/("firmware-1khz-comparison.png" if official_rate else "firmware-transition-comparison.png"),dpi=150);plt.close(fig)

def review(run,video=False,stride=4):
    cfg=json.loads((run/"experiment.json").read_text());ev=json.loads((run/"evaluation.json").read_text())
    truth=read(run/"truth.jsonl");commands=read(run/"commands.jsonl")
    n=min(len(truth),len(commands));truth=truth[:n];commands=commands[:n]
    t=np.array([r["time_s"] for r in truth]);p=np.array([r["optical_position_world_m"] for r in truth]);pad=np.array([r["pad_world_m"] for r in truth]);own=np.array([r["own"]["position_world_m"] for r in commands]);err=np.linalg.norm((p-pad)[:,:2],axis=1)
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout="constrained")
    axes[0].plot(t,p[:,2],label="광학점 실제 z");axes[0].plot(t,own[:,2],label="센서 추정 z");axes[0].axhline(.3,color="gray",ls=":",label="패드 높이");axes[0].legend()
    axes[1].plot(t,err,label="광학점→패드 수평거리");axes[1].set_ylim(0,max(.4,min(3.,err.max())));axes[1].legend()
    axes[2].plot(t,np.linalg.norm(p-own,axis=1),label="자기 위치 추정 오차 (평가 참값)");axes[2].legend()
    for ax,label in zip(axes,("높이 (m)","상대 거리 (m)","자기 추정 오차 (m)")):
        ax.set_ylabel(label);ax.grid(alpha=.2)
        for event in ev["events"]:ax.axvline(event["time_s"],color="gray",lw=.4)
    axes[2].set_xlabel("시뮬레이션 시간 (s)");fig.suptitle("미션 3: 영상+가상 센서 광학점 시험\n접촉·모터 모델 없음 / 물리 착륙 성공 0",fontweight="bold")
    fig.savefig(run/"review-metrics.png",dpi=150);plt.close(fig)
    if not video:return
    capture=cv2.VideoCapture(str(run/"camera.avi"));writer=cv2.VideoWriter(str(run/"review.mp4"),cv2.VideoWriter_fourcc(*"mp4v"),20.,(1280,720))
    if not capture.isOpened() or not writer.isOpened():raise RuntimeError("video codec unavailable")
    fig=plt.figure(figsize=(12.8,7.2),dpi=100,facecolor="#f7f8fb")
    axmap=fig.add_axes([.05,.43,.53,.48]);axzoom=fig.add_axes([.08,.10,.32,.28]);axcamera=fig.add_axes([.65,.34,.28,.50]);axtxt=fig.add_axes([.62,.06,.36,.27]);axtxt.axis("off")
    frame=0
    while True:
        ok,bgr=capture.read()
        if not ok:break
        if frame%stride:frame+=1;continue
        stamp=frame/cfg["timing_experiment"]["render_fps"];i=min(int(round(stamp*cfg["timing_experiment"]["control_hz"])),n-1)
        r=truth[i];cmd=commands[i];phase=cmd["phase"];heading=r["optical_heading_rad"];vh=r["vehicle_heading_rad"]
        for a in (axmap,axzoom,axcamera,axtxt):a.clear()
        for a in (axmap,axzoom):
            a.set_aspect("equal");a.grid(alpha=.15)
            # Static map drawing exists only in this offline review.
            cx,cy=cfg["track"]["center_world_xy_m"];lo=cx-10;hi=cx+10;radius=3.75
            angles=np.linspace(-math.pi/2,math.pi/2,120)
            a.plot([lo,hi],[cy-radius]*2,"--",color="gray",lw=1)
            a.plot([lo,hi],[cy+radius]*2,"--",color="gray",lw=1)
            a.plot(hi+radius*np.cos(angles),cy+radius*np.sin(angles),"--",color="gray",lw=1)
            a.plot(lo-radius*np.cos(angles),cy+radius*np.sin(angles),"--",color="gray",lw=1)
            a.plot(p[:i+1,0],p[:i+1,1],color="#087f8c",lw=1.5)
            a.scatter(p[i,0],p[i,1],c="#087f8c",s=30,zorder=5)
            a.scatter(pad[i,0],pad[i,1],c="#e1812c",s=28,zorder=5)
            # Triangle is nominal angular FOV ONLY. Length2m is a drawing scale,
            # not maximum detection range. Accepted image detections are separate.
            half=math.radians(cfg["camera"]["horizontal_fov_deg"]/2)
            ends=p[i,:2]+2*np.array([[math.cos(heading-half),math.sin(heading-half)],[math.cos(heading+half),math.sin(heading+half)]])
            a.add_patch(Polygon(np.vstack((p[i,:2],ends)),facecolor="#087f8c",alpha=.12,edgecolor="#087f8c"))
        axmap.set(xlim=(-1,51),ylim=(-1,21),xlabel="X (m)",ylabel="Y (m)")
        axmap.add_patch(Rectangle((0,0),50,20,fill=False,ls=":",color="#555"));axmap.set_title("평가용 평면도: 회색선은 알려진 지도 / 삼각형은 공칭 화각 87°")
        axzoom.set(xlim=(pad[i,0]-.7,pad[i,0]+.7),ylim=(pad[i,1]-.5,pad[i,1]+.5));axzoom.set_title("패드 부근 확대 (기체 외형 없음)")
        rot=np.array([[math.cos(vh),-math.sin(vh)],[math.sin(vh),math.cos(vh)]])
        corners=pad[i,:2]+np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])@rot.T
        axzoom.add_patch(Polygon(corners,fill=False,edgecolor="#e1812c",lw=2))
        marker=pad[i,:2]+rot@np.array([.25,0.]);a=marker+rot@np.array([0.,.1]);b=marker-rot@np.array([0.,.1]);axzoom.plot([a[0],b[0]],[a[1],b[1]],color="black",lw=2)
        axcamera.imshow(cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY),cmap="gray",vmin=0,vmax=255);axcamera.set_axis_off();axcamera.set_title("가상 AI-deck 입력: mono8 320×320")
        state=cmd.get("vehicle");speed=np.linalg.norm(state["vehicle_velocity_world_m_s"][:2]) if state and "vehicle_velocity_world_m_s" in state else None
        text=f"시간 {t[i]:.2f}s / {phase}\n수평 거리 {err[i]*100:.1f}cm · 패드 위 높이 {(p[i,2]-.3)*100:.1f}cm\n추정 차량 속도 {speed:.3f}m/s\n" if speed is not None else f"시간 {t[i]:.2f}s / {phase}\n마커 탐색 중\n"
        text+="청록 궤적: 광학점 / 주황: 이동 패드\n카메라 거리 끝은 그림 길이이며 인식 범위 아님\n기체 동역학·접촉·모터 없음 · 물리 성공 0"
        axtxt.axis("off");axtxt.text(0.,1.,text,va="top",fontsize=11,linespacing=1.6)
        fig.suptitle("미션 3 | 영상 + 가상 센서 기하 시험 (4배속)",x=.04,ha="left",y=.98,fontsize=16,fontweight="bold")
        fig.canvas.draw();rgba=np.asarray(fig.canvas.buffer_rgba());writer.write(cv2.cvtColor(rgba,cv2.COLOR_RGBA2BGR));frame+=1
    plt.close(fig);capture.release();writer.release()
    print(run/"review.mp4")

def main():
    p=argparse.ArgumentParser();p.add_argument("run",type=Path,nargs="?");p.add_argument("--video",action="store_true");p.add_argument("--firmware",type=Path)
    a=p.parse_args()
    if a.firmware:firmware_plot(a.firmware)
    if a.run:review(a.run,a.video)

if __name__=="__main__":main()
