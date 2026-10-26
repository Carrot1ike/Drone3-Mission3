"""Explain the recorded run visually, without flight I/O or control changes."""
import argparse
from bisect import bisect_right
import importlib
import json
import math
from pathlib import Path
import sys
import types
from types import SimpleNamespace
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from render_mission2_review import Replay,Movie,ffmpeg_binary,decoded_info

W,H=1920,1080
BG='#0b121c';PANEL='#131f2e';TEXT='#edf4fb';MUTED='#9badc2'
GREEN='#53e0b4';BLUE='#6bb7ff';AMBER='#ffca6a';PURPLE='#d6a0ff';RED='#ff7f89'
FONT='/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf'
BOLD='/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf'
F=244/(2*np.tan(np.deg2rad(35)));CX=161.5;CY=121.5
B=np.array([[0.,0.,1.],[-1/F,0.,CX/F],[0.,-1/F,CY/F]])
FONTS={}
NAMES={'TAKEOFF':'이륙','ENTRY_FORWARD':'입구 준비 전진','FIND':'다음 링 탐색',
 'ALIGN':'링 앞 정렬','STRAIGHT':'링 직진 통과','BRAKE':'전진 감속',
 'POST_RING_DESCENT':'블록 구간 고도로 하강','BLOCK_GAP':'블록 사이 정렬',
 'BLOCK_STRAIGHT':'블록 사이 직진','WALL_1':'첫 번째 벽',
 'WALL_2':'두 번째 벽','WALL_STRAIGHT':'벽 옆 통로 직진',
 'EXIT':'출구 정렬','EXIT_STRAIGHT':'출구 직진','LAND':'착륙 하강','DONE':'제어 종료'}
SECTIONS=['entry_gate12','ring1','ring2','ring3','two_block_passage','s_wall1','s_wall2','exit_gate23']
LABELS=['입구','링 1','링 2','링 3','블록','벽 1','벽 2','출구']


def font(size=24,bold=False):
 key=(size,bold)
 if key not in FONTS:FONTS[key]=ImageFont.truetype(BOLD if bold else FONT,size)
 return FONTS[key]


def text(draw,xy,label,size=24,color=TEXT,bold=False):
 draw.text(xy,str(label),font=font(size,bold),fill=color,stroke_width=0)


def box(draw,rect,color=PANEL,r=14):draw.rounded_rectangle(rect,radius=r,fill=color)


def wrap(draw,label,x,y,width,size=23,color=TEXT,line=33):
 current=''
 for char in label:
  if char=='\n' or draw.textlength(current+char,font=font(size))>width:
   text(draw,(x,y),current,size,color);y+=line;current='' if char=='\n' else char
  else:current+=char
 if current:text(draw,(x,y),current,size,color);y+=line
 return y


def arrow(draw,a,b,color=PURPLE,width=4):
 draw.line([a,b],fill=color,width=width)
 delta=np.subtract(b,a);n=float(np.linalg.norm(delta))
 if n<5:return
 d=delta/n;p=np.array([-d[1],d[0]])
 tip=np.array(b);base=tip-11*d
 draw.polygon([tuple(tip),tuple(base+5*p),tuple(base-5*p)],fill=color)


def dashed(draw,a,b,color=PURPLE,width=3):
 a=np.array(a,float);b=np.array(b,float);length=np.linalg.norm(b-a)
 if length<1:return
 for s in np.arange(0,length,14):
  draw.line([tuple(a+(b-a)*s/length),tuple(a+(b-a)*min(s+7,length)/length)],fill=color,width=width)


def rows(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def load_recorded(run):
 package=types.ModuleType('annotated_recorded_m2');package.__path__=[str(run/'source')]
 sys.modules[package.__name__]=package
 return importlib.import_module(package.__name__+'.mission'),importlib.import_module(package.__name__+'.passages')


def reconstruct(decisions,module):
 mission=module.Mission(1);result=[];last_reference=None
 for i,row in enumerate(decisions):
  before=mission.stage;phase=mission.passage_phase;age=row['frame']['received_monotonic']-(mission.entered or row['frame']['received_monotonic'])
  index=mission.index;side=mission.wall_side
  module.rings=lambda *_:[row['target']] if row['target'] else []
  module.blocks=module.wall=module.exit_gate=lambda *_:row['passage']
  command=mission.step(np.zeros((244,324),np.uint8),row['frame']['state'],row['frame']['received_monotonic'])
  assert mission.stage==row['stage'],(i,'stage')
  for key in ('vx','vy','height_m','yawrate'):
   assert abs(command.get(key,0)-row['command'].get(key,0))<1e-8,(i,key)
  if before in ('FIND','ALIGN') and row['target'] or before in ('BLOCK_GAP','WALL_1','WALL_2','EXIT') and row['passage']:
   last_reference=i
  result.append(dict(before=before,after=mission.stage,phase_before=phase,index_before=index,
   side=side,age=age,velocity=mission.filtered_velocity.copy(),target=None if mission.filtered_target is None else mission.filtered_target.copy(),
   stable=mission.stable,travel=mission.travel,distance=mission.distance,reference=last_reference,
   changed=before!=mission.stage))
 return result


def active_section(row,audit):
 stage=audit['before']
 if stage in ('TAKEOFF','ENTRY_FORWARD'):return 0
 if stage in ('FIND','ALIGN','STRAIGHT','BRAKE'):return min(3,audit['index_before']+1)
 if stage in ('POST_RING_DESCENT','BLOCK_GAP','BLOCK_STRAIGHT'):return 4
 if stage=='WALL_1' or stage=='WALL_STRAIGHT' and audit['side']==-1:return 5
 if stage=='WALL_2' or stage=='WALL_STRAIGHT' and audit['side']==1:return 6
 return 7


def conditions(row,audit):
 stage=audit['before'];v=audit['velocity'];target=audit['target'];obs=row.get('passage')
 changed=audit['changed'];stable=.0 if changed else audit['stable']
 if stage=='ALIGN' and row['target'] and target is not None:
  depth,left,up=target
  return [('링 전체가 보임',bool(row['target']['full'])),
   (f'좌우 {abs(left)*100:.1f} / 9 cm · 높이 {abs(up)*100:.1f} / 9 cm',abs(left)<.09 and abs(up)<.09),
   (f'거리 오차 {abs(depth-1.4)*100:.1f} / 15 cm',abs(depth-1.4)<.15),
   (f'잔류 속도 {np.linalg.norm(v):.2f} / 0.15 m/s',np.linalg.norm(v)<.15)],(.30 if changed and audit['after']=='STRAIGHT' else stable,.30)
 if stage in ('WALL_1','WALL_2') and audit['phase_before']=='approach' and obs:
  return [(f'벽 거리 오차 {abs(obs["depth"]-2.8)*100:.1f} / 15 cm',abs(obs['depth']-2.8)<.15),
    (f'전진 속도 {abs(v[0]):.2f} / 0.12 m/s',abs(v[0])<.12)],None
 if stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT') and obs:
  return [(f'좌우 오차 {abs(obs["left"])*100:.1f} / 9 cm',abs(obs['left'])<.09),
    (f'좌우 속도 {abs(v[1]):.2f} / 0.10 m/s',abs(v[1])<.1),
    (f'전진 속도 {abs(v[0]):.2f} / 0.12 m/s',abs(v[0])<.12)],(.35 if changed else stable,.35)
 return [],None


def reason(row,audit):
 stage=audit['before'];obs=row.get('passage')
 if audit['changed']:
  return f'{NAMES.get(stage,stage)} → {NAMES.get(audit["after"],audit["after"])}',GREEN
 if stage=='TAKEOFF':return '높이 1m까지 올라간 뒤 입구 구간 시작',BLUE
 if stage=='ENTRY_FORWARD':return '입구 준비 구간: 추정 이동량 1.50m까지 전진',BLUE
 if stage=='FIND':return '화면에서 다음 링 후보를 찾으며 옆으로 이동',AMBER
 if stage=='ALIGN':return ('중심·거리·잔류 속도 조건을 맞추는 중' if row['target'] else '목표 링이 사라져 탐색 단계로 복귀'),AMBER
 if stage=='BRAKE':return '1.5초 감속 후 다음 대상을 관측',BLUE
 if stage=='POST_RING_DESCENT':return '목표 고도를 1m로 낮춰 블록 구간 준비',BLUE
 if stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT'):
  if not obs:return '선택한 대상 없음: 전진·좌우 명령 0',AMBER
  if stage.startswith('WALL') and audit['phase_before']=='approach':return '벽과 약 2.8m 간격에서 전진을 멈추기',AMBER
  return '전진 명령 0 · 목표 통로 중심으로 옆 정렬',PURPLE
 if stage in ('STRAIGHT','BLOCK_STRAIGHT','WALL_STRAIGHT','EXIT_STRAIGHT'):
  return '진입 때 정한 거리까지 직진 · 영상 조향 종료',GREEN
 if stage=='LAND':return '목표 고도를 낮추며 하강 → EKF 6cm 미만에서 종료',BLUE
 return '비행 기록 연결 중',MUTED


class Annotator:
 def __init__(self,run,out):
  self.run=run;self.out=out
  self.frames=rows(run/'host/frames.jsonl');self.decisions=rows(run/'host/decisions.jsonl');self.commands=rows(run/'host/commands.jsonl')
  self.summary=json.loads((run/'host/summary.json').read_text());self.score=json.loads((run/'physical_score.json').read_text())
  self.metrics=json.loads((run/'result_summary.json').read_text())
  self.runtime=Path(json.loads((run/'runtime/runtime_state/state.json').read_text())['log_dir'])
  self.truth=[r for r in rows(self.runtime/'evaluation/ground_truth.jsonl') if r.get('event')=='ground_truth']
  self.module,self.perception=load_recorded(run);self.audit=reconstruct(self.decisions,self.module)
  (out/'decision_reconstruction.json').write_text(json.dumps(dict(decisions=len(self.audit),exact_stage_matches=len(self.audit),exact_command_matches=len(self.audit),
   tolerance=1e-8,method='Recorded source and chosen observations replayed offline; no simulator or flight commands.',
   diagnostic_masks='Recomputed from the exact decision image, labeled in the video.'),indent=2))
  self.start=self.summary['started_monotonic'];self.end=self.summary['finished_monotonic']
  self.dt=[r['policy_completed_monotonic'] for r in self.decisions];self.ct=[r['sent_monotonic'] for r in self.commands]
  self.gt=[r['host_monotonic'] for r in self.truth];self.ft=[r['received_monotonic'] for r in self.frames]
  self.images={};self.visuals={};self.replay=None
  self.crossings={s['name']:min([e['rear_clear_host_monotonic'] for e in s['transits'] if e['geometry_pass']],default=math.inf) for s in self.score['section_results']}

 def image(self,row):
  filename=row['frame']['image_file']
  if filename not in self.images:self.images[filename]=np.load(self.run/'host'/filename)['image']
  return self.images[filename]

 def visuals_for(self,i):
  if i in self.visuals:return self.visuals[i]
  row=self.decisions[i];audit=self.audit[i];im=self.image(row);state=row['frame']['state'];stage=audit['before']
  raw=cv2.cvtColor(im,cv2.COLOR_GRAY2RGB);diagnostic=np.repeat((im*.30).astype(np.uint8)[:,:,None],3,axis=2)
  rotation,_=self.perception.attitude(state);hom=np.linalg.inv(B)@rotation@B;inverse=np.linalg.inv(hom)
  kind='none';objects=[];ring=row['target'] if stage in ('FIND','ALIGN') else None
  obs=row['passage'] if stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT') else None
  diagnostic_title='현재 단계에서는 대상 인식을 사용하지 않음'
  if ring:
   kind='ring';u,v=ring['center'];radius=ring['diameter']/2*.595/.55
   angles=np.linspace(0,2*np.pi,96,endpoint=False);c=np.cos(angles);s=np.sin(angles)
   values=[];valid=[]
   for scale in (1.,.84,1.18):
    xx=np.rint(u+radius*scale*c).astype(int);yy=np.rint(v+radius*scale*s).astype(int)
    valid.append((xx>=0)&(xx<324)&(yy>=0)&(yy<244));values.append(im[np.clip(yy,0,243),np.clip(xx,0,323)].astype(float))
   good=valid[0]&valid[1]&valid[2];contrast=values[0]-np.maximum(values[1],values[2])
   for j in range(96):
    px=int(round(u+radius*c[j]));py=int(round(v+radius*s[j]))
    if 0<=px<324 and 0<=py<244:cv2.circle(diagnostic,(px,py),2,(83,224,180) if good[j] and contrast[j]>18 else (255,127,137),-1)
   diagnostic_title='테두리 명암: 민트 충족 / 빨강 미달 · 재계산'
  elif obs:
   kind=obs['kind'];gray=self.perception.level_image(im,state);diagnostic=np.repeat((gray*.25).astype(np.uint8)[:,:,None],3,axis=2)
   if kind=='exit':
    mask=cv2.morphologyEx((gray>80).astype(np.uint8),cv2.MORPH_OPEN,np.ones((35,1),np.uint8))
    diagnostic[mask>0]=(86,108,132)
    for face in obs['faces']:
     x,y,w,h=face['box'];region=np.zeros_like(mask);region[y:y+h,x:x+w]=1
     diagnostic[(mask&region)>0]=(83,224,180)
   else:
    for f in self.perception.solid_faces(im,state):
     x,y,w,h=f['box'];selected=any(f['box']==a['box'] for a in obs['faces'])
     mask=cv2.morphologyEx((np.abs(gray.astype(np.int16)-f['gray'])<5).astype(np.uint8),cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
     region=np.zeros_like(mask);region[y:y+h,x:x+w]=1
     diagnostic[(mask&region)>0]=(83,224,180) if selected else (86,108,132)
   diagnostic_title='자세 보정 + 밝기 분리 · 기록 프레임 재계산'
  else:
   diagnostic=np.repeat((self.perception.level_image(im,state)*.35).astype(np.uint8)[:,:,None],3,axis=2)
  answer=(raw,diagnostic,inverse,kind,ring,obs,diagnostic_title)
  self.visuals[i]=answer
  if len(self.visuals)>50:del self.visuals[next(iter(self.visuals))]
  return answer

 def overlay(self,canvas,rect,inverse,kind,ring,obs,diagnostic=False):
  draw=ImageDraw.Draw(canvas);x0,y0,w,h=rect
  def point(u,v,rectified=True):
   p=np.array([u,v,1.])
   if not diagnostic and rectified:p=inverse@p
   if abs(p[2])<1e-6:return (x0,y0)
   return (x0+w*float(p[0]/p[2])/324,y0+h*float(p[1]/p[2])/244)
  def line(a,b,col,width=3,rectified=True,dash=False):
   aa=point(*a,rectified);bb=point(*b,rectified)
   if dash:dashed(draw,aa,bb,col,width)
   else:draw.line([aa,bb],fill=col,width=width)
  # Draw in a local image to clip every projected primitive at image boundaries.
  if kind=='ring' and ring:
   u,v=ring['center'];r=ring['diameter']/2
   for radius,col in ((r,GREEN),(r*.595/.55*1.18,BLUE)):
    points=[point(u+radius*np.cos(a),v+radius*np.sin(a),False) for a in np.linspace(0,2*np.pi,100)]
    draw.line(points,fill=col,width=3)
   center=point(u,v,False);reference=point(CX,CY,False)
   draw.line([reference,center],fill=PURPLE,width=3)
   draw.ellipse([center[0]-6,center[1]-6,center[0]+6,center[1]+6],fill=PURPLE)
   draw.line([(reference[0]-11,reference[1]),(reference[0]+11,reference[1])],fill=BLUE,width=2)
   draw.line([(reference[0],reference[1]-11),(reference[0],reference[1]+11)],fill=BLUE,width=2)
   # Image-center line is explanatory only; metric control errors use attitude correction.
  elif obs:
   for n,face in enumerate(obs['faces']):
    x,y,ww,hh=face['box'];points=[point(u,v) for u,v in ((x,y),(x+ww,y),(x+ww,y+hh),(x,y+hh),(x,y))]
    draw.line(points,fill=GREEN,width=4)
    px,py=point(x+3,max(y+2,6));text(draw,(max(x0+4,min(x0+w-130,px)),max(y0+4,min(y0+h-28,py))),f'선택 {n+1}',20,GREEN,True)
   if kind=='wall':
    edge=obs['edge_u'];target=CX-F*obs['left']/obs['depth']
    other=2*target-edge
    line((edge,15),(edge,232),AMBER,4)
    line((target,20),(target,228),PURPLE,3,dash=True)
    line((other,20),(other,228),PURPLE,2,dash=True)
    mid=point(target,CY);arrow(draw,point(CX,CY),mid,PURPLE)
    if not diagnostic and w>=550:text(draw,(x0+14,y0+h-78),'노랑: 관측 끝 / 보라: 규격으로 계산한 통로',19,AMBER)
   elif kind in ('blocks','exit'):
    a,b=obs['faces'];left=a['box'][0]+a['box'][2];right=b['box'][0]
    line((left,20),(left,230),AMBER,3);line((right,20),(right,230),AMBER,3)
    line(((left+right)/2,20),((left+right)/2,230),PURPLE,3,dash=True)
    arrow(draw,point(CX,CY),point((left+right)/2,CY),PURPLE)
  if not ring:
   line((CX-6,CY),(CX+6,CY),BLUE,2);line((CX,CY-6),(CX,CY+6),BLUE,2)

 def render(self,t,external=None):
  canvas=Image.new('RGB',(W,H),BG);draw=ImageDraw.Draw(canvas)
  i=bisect_right(self.dt,t)-1;ci=bisect_right(self.ct,t)-1;fi=bisect_right(self.ft,t)-1
  row=self.decisions[i] if i>=0 else None;audit=self.audit[i] if i>=0 else None
  command=self.commands[ci] if ci>=0 else {};sent=command.get('command') or {}
  elapsed=max(0,t-self.start);stage=audit['before'] if audit else 'CONNECTING'
  title=NAMES.get(stage,'연결 및 데이터 수신 준비')
  if stage in ('FIND','ALIGN','STRAIGHT','BRAKE'):title=f'링 {min(audit["index_before"]+1,3)} · {title}'
  if stage=='WALL_STRAIGHT':title=f'벽 {1 if audit["side"]==-1 else 2} · 옆 통로 직진'
  text(draw,(24,20),'MISSION 2  |  인식에서 제어까지',32,TEXT,True)
  text(draw,(1010,23),title,30,GREEN,True)
  text(draw,(1670,26),f'{elapsed:05.1f} / {self.end-self.start:.1f} s',25,MUTED)
  active=active_section(row,audit) if row else 0
  for j,(key,label) in enumerate(zip(SECTIONS,LABELS)):
   x=24+j*236;done=self.crossings[key]<=t
   box(draw,(x,77,x+218,127),'#1a4d42' if done else '#26364c' if j==active else PANEL,9)
   text(draw,(x+14,89),('완료 · ' if done else '▶ ' if j==active else '· ')+label,24,GREEN if done else TEXT if j==active else MUTED,True)
  for x,label in [(24,'외부 시점 · 기록된 실제 자세 재현'),(654,'기체 영상 · 판단에 사용한 원본 프레임'),(1284,'인식 과정 · 동일 프레임 분석')]:text(draw,(x,148),label,24,TEXT,True)
  rects=[(24,187,612,461),(654,187,612,461),(1284,187,612,461)]
  for x,y,w,h in rects:draw.rectangle((x,y,x+w,y+h),fill='#060b11')
  if external is not None:canvas.paste(Image.fromarray(external[:,:,::-1]),(24,187))
  if row:
   raw,mask,inverse,kind,ring,obs,dtitle=self.visuals_for(i)
   for which,im in [(1,raw),(2,mask)]:
    panel=Image.fromarray(im).resize((612,461),Image.Resampling.BILINEAR)
    self.overlay(panel,(0,0,612,461),inverse,kind,ring,obs,diagnostic=which==2)
    canvas.paste(panel,(rects[which][0],rects[which][1]))
   draw=ImageDraw.Draw(canvas)
   # Main panel only uses current observations. Stored targets never follow new images.
   if kind=='none':
    ref=audit['reference']
    label='인식 비활성 · 이동량으로 구간 실행'
    if stage in ('TAKEOFF','ENTRY_FORWARD','BRAKE','POST_RING_DESCENT','LAND'):label='고도·속도·시간으로 현재 단계 실행'
    box(draw,(1300,202,1880,249),'#152537',7);text(draw,(1314,214),label,22,BLUE)
    if ref is not None and stage in ('STRAIGHT','BLOCK_STRAIGHT','WALL_STRAIGHT','EXIT_STRAIGHT'):
     rr=self.decisions[ref];rrraw,_,inv,k,r,o,_=self.visuals_for(ref)
     reference=Image.fromarray(rrraw).resize((405,305),Image.Resampling.BILINEAR)
     self.overlay(reference,(0,0,405,305),inv,k,r,o)
     canvas.paste(reference,(1380,271));draw=ImageDraw.Draw(canvas)
     text(draw,(1310,596),f'진입 때 사용한 과거 프레임 · {rr["frame"]["received_monotonic"]-self.start:.2f}s',22,AMBER)
   else:
    box(draw,(1294,604,1884,642),'#101b29',6);text(draw,(1304,615),dtitle,19,MUTED)
   text(draw,(666,610),f'원본 #{row["frame"]["sequence"]} · 판단 영상 나이 {(t-row["frame"]["received_monotonic"])*1000:.0f}ms',19,TEXT)
  else:
   text(draw,(760,380),'영상 연결 중',36,MUTED,True)
  for x in (24,654,1284):box(draw,(x,670,x+612,1027),PANEL)
  text(draw,(42,687),'실제 송신 명령 · 기체 상태',25,TEXT,True)
  text(draw,(672,687),'인식 결과 · 거리와 목표 계산',25,TEXT,True)
  text(draw,(1302,687),'현재 판단 · 다음 단계 조건',25,TEXT,True)
  if row:
   self.command_panel(canvas,row,audit,command,t)
   self.observation_panel(canvas,row,audit)
   self.decision_panel(canvas,row,audit)
  draw=ImageDraw.Draw(canvas)
  text(draw,(26,1042),'민트: 선택 대상   노랑: 관측 경계   보라: 계산 목표   파랑: 명령·기준   회색: 미선택 영역',19,MUTED)
  text(draw,(1130,1042),'입력 평균 13.5fps · 송신 약 50Hz · 영상 20fps',20,MUTED)
  return canvas

 def command_panel(self,canvas,row,audit,command,t):
  draw=ImageDraw.Draw(canvas);s=row['frame']['state'];c=command.get('command') or {};ack=command.get('ring_pass_ack',0)
  text(draw,(42,730),f'전진  {c.get("vx",0):+.2f} m/s',27,BLUE,True)
  text(draw,(330,730),f'좌우  {c.get("vy",0):+.2f} m/s',27,BLUE,True)
  text(draw,(42,771),f'고도 목표 {c.get("height_m",0):.2f}m  ·  회전 {c.get("yawrate",0):.1f}°/s',22,TEXT)
  origin=(130,909);scale=100
  draw.line([(48,909),(212,909)],fill='#3e526d',width=2);draw.line([(130,827),(130,991)],fill='#3e526d',width=2)
  arrow(draw,origin,(origin[0]-c.get('vy',0)*scale,origin[1]-c.get('vx',0)*scale),BLUE,5)
  bv=s['body_velocity_flu'];arrow(draw,origin,(origin[0]-bv[1]*scale,origin[1]-bv[0]*scale),GREEN,3)
  text(draw,(96,809),'전진',19,MUTED);text(draw,(45,994),'파랑 명령 / 민트 측정 속도',17,MUTED)
  text(draw,(250,815),f'EKF  {s["ekf_height_m"]:.2f}m    ToF  {s["tof_distance_m"]:.2f}m',22,TEXT)
  text(draw,(250,846),'8초 추이 · 민트 EKF / 노랑 ToF / 파랑 목표',17,MUTED)
  j=bisect_right(self.ft,t);lo=bisect_right(self.ft,t-8);fs=self.frames[lo:j]
  chart=(250,880,610,959);draw.rectangle(chart,fill='#0c1521')
  for key,col in [('ekf_height_m',GREEN),('tof_distance_m',AMBER)]:
   points=[(chart[0]+(r['received_monotonic']-(t-8))/8*(chart[2]-chart[0]),chart[3]-np.clip(r['state'][key],0,3)/3*(chart[3]-chart[1])) for r in fs if r.get('state')]
   if len(points)>1:draw.line(points,fill=col,width=2)
  goal_y=chart[3]-np.clip(c.get('height_m',0),0,3)/3*(chart[3]-chart[1]);dashed(draw,(250,goal_y),(610,goal_y),BLUE,2)
  label='ToF 반영 잠시 중지' if ack else 'ToF 정상 반영'
  text(draw,(250,974),f'{label}  요청 {c.get("ring_pass_mask",0)} / ACK {ack}',20,AMBER if ack else GREEN,True)

 def observation_panel(self,canvas,row,audit):
  draw=ImageDraw.Draw(canvas);stage=audit['before'];ring=row['target'];obs=row.get('passage');x=672;y=732
  if stage in ('FIND','ALIGN') and ring:
   depth,left,up=audit['target']
   text(draw,(x,y),f'선택: 링 {audit["index_before"]+1}  ·  중심 ({ring["center"][0]:.0f}, {ring["center"][1]:.0f}) px',24,GREEN,True)
   text(draw,(x,y+42),f'제어 거리 {depth:.2f}m  →  목표 1.40m',27,TEXT,True)
   text(draw,(x,y+84),f'좌우 오차 {left:+.2f}m  ·  높이 오차 {up:+.2f}m',23,PURPLE)
   text(draw,(x,y+126),f'명암 지지율 {ring["support"]*100:.0f}% · 전체 보임 {"예" if ring["full"] else "아니오"}',23,TEXT)
   wrap(draw,'원 후보 → 테두리가 안팎보다 밝은지 확인\n알려진 링 규격과 자세로 거리·오차 계산',x,y+172,570,22,MUTED,32)
  elif stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT') and obs:
   label={'blocks':'블록 앞면 2개','wall':'가장 가까운 벽면','exit':'출구 기둥 2개'}[obs['kind']]
   text(draw,(x,y),'선택: '+label,25,GREEN,True)
   text(draw,(x,y+42),f'거리 {obs["depth"]:.2f}m  ·  옆 목표 {obs["left"]:+.2f}m',27,TEXT,True)
   if obs['kind']=='wall':
    method={'height':'벽의 전체 높이','floor':'바닥 접점 + EKF 고도','top':'벽 윗변 + EKF 고도'}.get(obs['faces'][0]['depth_method'],'')
    text(draw,(x,y+86),'거리 근거: '+method,23,TEXT)
    text(draw,(x,y+125),'거리 ≈ 174.23 × 2.50 / 높이(px)' if obs['faces'][0]['depth_method']=='height' else '보이는 경계와 카메라 높이로 삼각 계산',23,MUTED)
    direction='오른쪽' if stage=='WALL_1' else '왼쪽'
    text(draw,(x,y+170),f'사전 설정: {direction} 우회 · 통로 폭 2m',23,AMBER,True)
    text(draw,(x,y+211),'관측한 벽 끝에서 열린 쪽으로 1m',23,PURPLE)
   elif obs['kind']=='blocks':
    text(draw,(x,y+86),f'계산된 블록 사이 폭 {obs["gap_m"]:.2f}m',24,PURPLE)
    wrap(draw,'앞면 폭 0.70m와 화면 속 폭으로 거리 계산\n두 안쪽 경계의 중앙을 목표로 선택\n블록 길이 0.70m를 더해 뒤까지 통과',x,y+135,570,23,MUTED,35)
   else:
    wrap(draw,'밝고 가느다란 세로 기둥 2개 검출\n출구 폭 1.50m로 거리 계산\n두 기둥 사이 중앙으로 정렬',x,y+105,570,24,MUTED,39)
  elif stage in ('STRAIGHT','BLOCK_STRAIGHT','WALL_STRAIGHT','EXIT_STRAIGHT'):
   text(draw,(x,y),'진입 때 정한 목표를 유지',27,GREEN,True)
   text(draw,(x,y+48),f'이동량 {audit["travel"]:.2f} / {audit["distance"]:.2f} m',29,TEXT,True)
   fraction=np.clip(audit['travel']/max(.01,audit['distance']),0,1)
   draw.rounded_rectangle((x,y+98,x+564,y+120),8,fill='#26364c');draw.rounded_rectangle((x,y+98,x+max(1,564*fraction),y+120),8,fill=GREEN)
   wrap(draw,'EKF 전진 속도를 시간에 따라 적분\n새 영상은 기록하며, 목표를 다시 조향하지 않음\n계산한 거리 도달 → 다음 단계',x,y+152,570,23,MUTED,35)
  else:
   label={'TAKEOFF':'목표 고도 1m','ENTRY_FORWARD':'입구 준비 전진 1.50m','BRAKE':'1.5초 동안 전진 감속',
    'POST_RING_DESCENT':'목표 고도 1m까지 하강','LAND':'목표 고도 0.03m까지 하강'}.get(stage,'선택된 시각 대상 없음')
   text(draw,(x,y),label,27,BLUE,True)
   wrap(draw,'이 단계는 고도·속도·경과 시간으로 진행\n표시 영상에서 새 대상을 선택하지 않음',x,y+60,570,23,MUTED,35)
   if stage=='ENTRY_FORWARD':text(draw,(x,y+160),f'추정 전진량 {audit["travel"]:.2f}m',27,TEXT)
   if stage=='LAND':wrap(draw,'완주 판정은 출구 통과까지 확인됨\n착륙 후 완전 정지는 별도 미검증',x,y+163,570,23,AMBER,35)

 def decision_panel(self,canvas,row,audit):
  draw=ImageDraw.Draw(canvas);x=1302;y=732;label,color=reason(row,audit)
  y=wrap(draw,label,x,y,566,24,color,32)+10
  checks,timer=conditions(row,audit)
  for label,passed in checks:
   draw.ellipse((x,y+3,x+20,y+23),fill=GREEN if passed else None,outline=GREEN if passed else AMBER,width=2)
   if passed:draw.line([(x+5,y+13),(x+9,y+17),(x+16,y+9)],fill=BG,width=3)
   text(draw,(x+32,y),label,22,TEXT);y+=34
  if checks:
   if audit['before'] in ('WALL_1','WALL_2') and audit['phase_before']=='approach':
    wrap(draw,'전진 = 0.5 × (거리 - 2.8) - 0.8 × 전진 속도\n명령 범위 ±0.35 m/s',x,919,566,20,MUTED,30)
   elif audit['before']=='ALIGN':text(draw,(x,913),'좌우 = 0.4 × 오차 - 0.8 × 속도  (±0.30m/s)',19,MUTED)
   else:text(draw,(x,913),'좌우 = 0.55 × 오차 - 0.8 × 속도  (±0.40m/s)',19,MUTED)
  if timer:
   value,total=timer;y=max(y+7,948)
   text(draw,(x,y),f'조건 유지  {min(value,total):.2f} / {total:.2f}s',21,GREEN if value>=total else MUTED)
   draw.rectangle((x,y+31,x+560,y+42),fill='#293b51')
   draw.rectangle((x,y+31,x+max(1,560*min(1,value/total)),y+42),fill=GREEN)
  elif not checks:
   stage=audit['before']
   if stage in ('STRAIGHT','BLOCK_STRAIGHT','WALL_STRAIGHT','EXIT_STRAIGHT'):
    text(draw,(x,y+8),f'종료까지 추정 {max(0,audit["distance"]-audit["travel"]):.2f}m',29,GREEN,True)
    text(draw,(x,y+64),'속도 적분 이동량 ≥ 진입 때 계산한 거리',22,MUTED)
    if stage=='STRAIGHT':wrap(draw,'링 직진 표시 → ACK 확인 후 전진\n이 구간만 EKF의 ToF 반영을 잠시 중지',x,y+112,566,23,AMBER,34)
   elif stage=='LAND':wrap(draw,'현재 종료 조건: EKF 높이 < 0.06m\n이 조건은 착륙 후 정지 판정과 구분',x,y+16,560,23,AMBER,36)
   else:text(draw,(x,y+25),f'단계 경과 {audit["age"]:.2f}s',28,BLUE)

 def slate(self,end=False):
  canvas=Image.new('RGB',(W,H),BG);draw=ImageDraw.Draw(canvas)
  text(draw,(110,100),'MISSION 2',37,GREEN,True)
  text(draw,(110,169),'인식 → 판단 → 제어' if not end else '출구까지 8개 구간 연속 통과',62,TEXT,True)
  text(draw,(112,270),'실제 완주 기록에 인식 결과와 제어 근거를 겹쳐 표시',30,MUTED)
  if not end:
   lines=[('01','왼쪽','실제 비행 자세로 재현한 외부 시점'),('02','가운데','판단에 사용한 기체 영상 + 선택 대상·경계·목표'),('03','오른쪽','동일 프레임의 인식 과정 + 단계 전환 조건')]
   for i,(num,label,desc) in enumerate(lines):
    y=376+i*120;box(draw,(110,y,1810,y+94));text(draw,(142,y+27),num,30,GREEN,True);text(draw,(238,y+28),label,30,TEXT,True);text(draw,(460,y+28),desc,29,TEXT)
   text(draw,(115,812),'민트  선택 대상     노랑  관측 경계     보라  계산 목표     파랑  명령·기준',29,TEXT)
   text(draw,(115,878),'벽 높이 2.5m · 통로 폭 2m · 첫 벽 오른쪽 / 다음 벽 왼쪽은 사전 설정',27,AMBER)
   text(draw,(115,940),'판단 1,598개 재현 일치 · 추가 비행 없이 기록 분석 · 입력 13.5fps / 출력 20fps',25,MUTED)
  else:
   for j,(big,label) in enumerate([('127.5초','기록 길이'),('8 / 8','구간 연속 통과'),('0건','장애물 접촉')]):
    x=110+j*580;box(draw,(x,382,x+550,590));text(draw,(x+35,418),big,57,GREEN,True);text(draw,(x+37,507),label,29,TEXT)
   text(draw,(115,668),'입구 → 링 3개 → 블록 사이 → 벽 2개 → 출구',38,TEXT,True)
   text(draw,(115,752),'영상과 연관된 상태로 인식·제어 / 약 50Hz 별도 송신',29,MUTED)
   text(draw,(115,820),'벽의 규격·우회 순서는 사전 입력값을 사용',28,AMBER)
   text(draw,(115,884),'착륙 후 완전 정지 상태는 기록 종료로 별도 미검증',28,AMBER)
   text(draw,(115,957),'외부 영상은 실제 자세 재현 · 내부 영상은 실제 수신 프레임 · 시뮬레이터 정답 위치는 제어 입력에 사용하지 않음',23,MUTED)
  return canvas


def main():
 parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--out',type=Path,required=True);parser.add_argument('--preview',action='store_true')
 args=parser.parse_args();run=args.run.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
 cv2.setNumThreads(1);a=Annotator(run,out)
 a.replay=Replay(SimpleNamespace(baseline=Path('/home/razer/Desktop/Drone3'),sim_python=None,width=612,image_height=461,preflight_assets=None),a.runtime,a.truth,out,{},trial_started_monotonic=a.start)
 try:
  if args.preview:
   for seconds in (0.,20.,25.,59.,77.,80.,96.,109.,115.5,124.):
    gi=max(0,bisect_right(a.gt,a.start+seconds)-1)
    im=a.render(a.start+seconds,a.replay.frame(gi));im.save(out/f'preview_{seconds:05.1f}.jpg',quality=92)
   a.slate().save(out/'preview_intro.jpg',quality=92);a.slate(True).save(out/'preview_end.jpg',quality=92)
   print('previews ready',flush=True);return
  path=out/'mission2_annotated.mp4';movie=Movie(path,(W,H),20.,ffmpeg_binary());count=int(math.ceil((a.end-a.start)*20));intro=100;outro=100
  try:
   first=np.array(a.slate())[:,:,::-1]
   for _ in range(intro):movie.write(first)
   for n in range(count):
    t=a.start+n/20;gi=bisect_right(a.gt,t)-1
    im=a.render(t,a.replay.frame(gi) if gi>=0 else None);movie.write(np.asarray(im)[:,:,::-1])
    if n%400==0:print(f'render {n}/{count}',flush=True)
   last=np.array(a.slate(True))[:,:,::-1]
   for _ in range(outro):movie.write(last)
  finally:movie.close()
  info=decoded_info(path);assert info['decoded_frames']==intro+count+outro
  (out/'validation.json').write_text(json.dumps(dict(decoded=info,expected_frames=intro+count+outro,flight_frames=count,intro_s=5,outro_s=5,
   all_decisions_reconstructed_exactly=True,whole_flight=True,policy_unchanged=True,live_runtime_started=False),indent=2))
  print(path,flush=True)
 finally:a.replay.close()

if __name__=='__main__':main()
