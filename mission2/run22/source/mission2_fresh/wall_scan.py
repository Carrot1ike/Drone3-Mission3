"""Yaw observations of a wall and its visible floor strips, without map priors."""
import math
import cv2
import numpy as np
from .vision import F,CX,CY,attitude
from .passages import solid_faces
from .exit_track import heading_image
from .ring_scan import RingScan


def plane_pixels(normal,point,position,yaw,height):
    """Intersect heading-image columns with an observed vertical wall plane."""
    c,s=math.cos(yaw),math.sin(yaw)
    rotation=np.array([[c,-s],[s,c]])
    rays=np.column_stack([np.ones(324),(CX-np.arange(324))/F])@rotation.T
    denominator=rays@normal
    scale=np.full(324,np.nan)
    np.divide(np.dot(point-np.asarray(position)[:2],normal),denominator,
              out=scale,where=np.abs(denominator)>1e-8)
    feet=CY+F*height/scale
    points=np.asarray(position)[:2]+rays*scale[:,None]
    valid=np.isfinite(scale)&(scale>0)&(feet>CY+12)&(feet<241)
    return feet,points,valid


class WallScan:
    def __init__(self,yaw):
        self.turn=RingScan(yaw);self.reference=None
        self.best={};self.samples={1:0,-1:0};self.selected_side=None
        self.current_faces=[];self.current_choices=[];self.turn_evidence=None
        self.last_seen=None

    def observe(self,image,state,position,t):
        gray=heading_image(image,state)
        neutral=dict(state,quaternion_xyzw=[0.,0.,0.,1.])
        faces=solid_faces(gray,neutral)
        _,yaw=attitude(state);c,s=math.cos(yaw),math.sin(yaw)
        rotation=np.array([[c,-s],[s,c]])
        fronts=[]
        for face in faces:
            if face.get('ground_edge') is None or face['box'][2]<25:continue
            edges=np.asarray(face['ground_edge'])@rotation.T+np.asarray(position)[:2]
            depth=float(edges[:,0].mean()-position[0])
            if depth<=0 or np.ptp(edges[:,0])>=.20*depth:continue
            tangent=edges[0]-edges[1]
            if np.linalg.norm(tangent)<1e-8:continue
            tangent/=np.linalg.norm(tangent)
            normal=np.array([tangent[1],-tangent[0]])
            if normal[0]<0:normal=-normal
            point=edges.mean(axis=0)
            if self.reference is not None:
                residual=np.abs((edges-self.reference['point'])@self.reference['normal']).mean()
                if residual>.20*depth or abs(face['gray']-self.reference['gray'])>9:continue
            fronts.append((depth,face,edges,point,normal))
        self.current_faces=faces;self.current_choices=[]
        if not fronts:return
        depth,face,edges,point,normal=min(fronts,key=lambda x:x[0])
        self.reference=dict(point=point,normal=normal,gray=face['gray'])
        self.last_seen=t
        feet,plane_points,valid=plane_pixels(normal,point,position,yaw,state['ekf_height_m'])
        floor=(cv2.blur(gray.astype(float),(7,7))>30).astype(float)
        for other in faces:
            x,y,w,h=other['box'];floor[y:y+h,x:x+w]=0
        profile=np.zeros(324)
        for u in np.flatnonzero(valid):
            base=round(float(feet[u]));lo=max(0,base-10);hi=min(244,base-2)
            if hi>lo:profile[u]=floor[lo:hi,u].mean()
        profile=np.convolve(profile,np.ones(7)/7,'same')>.45
        x,y,w,h=face['box']
        rear=float(max(edges[:,0]))
        for side,edge in ((1,x),(-1,x+w)):
            choice=dict(side=side,valid=False,edge_u=int(edge),reason='벽 끝이나 이어지는 바닥 미관측')
            if 3<edge<321 and valid[edge]:
                columns=range(edge-4,4,-1) if side==1 else range(edge+4,320)
                missed=0;visible=[]
                for u in columns:
                    missed=0 if profile[u] and valid[u] else missed+1
                    if missed>=6:break
                    if profile[u] and valid[u]:visible.append(u)
                if len(visible)>=7:
                    end=visible[-1];a,b=plane_points[edge],plane_points[end]
                    gap=float(np.linalg.norm(b-a));center=(a+b)/2
                    choice.update(valid=True,floor_end_u=int(end),floor_row=int((feet[edge]+feet[end])/2-6),
                        gap_m=gap,left=float(center[1]-position[1]),
                        clipped=bool(end<=6 or end>=318),point_local_m=center.tolist(),
                        rear_local_x=rear,observed_yaw_deg=math.degrees(yaw),observed_at=t,
                        reason='회전 영상의 바닥 후보를 관측 벽 평면에 투영')
                    self.samples[side]+=1
                    old=self.best.get(side)
                    quality=(not choice['clipped'],gap)
                    if old is None or quality>=(not old['clipped'],old['gap_m']):self.best[side]=choice.copy()
            self.current_choices.append(choice)

    def step(self,image,state,position,t):
        self.observe(image,state,position,t)
        yawrate,self.turn_evidence=self.turn.step([],state,position,t)
        if self.turn.done and self.best:
            self.selected_side=max(self.best,key=lambda side:self.best[side]['gap_m'])
        return yawrate

    def guidance(self,position,t):
        choice=self.best.get(self.selected_side)
        if choice is None:return None
        delta=np.asarray(choice['point_local_m'])-np.asarray(position)[:2]
        if delta[0]<=0:return None
        return dict(kind='wall',method='yaw_scan_floor_strip',depth=float(delta[0]),left=float(delta[1]),
            side=self.selected_side,gap_m=choice['gap_m'],rear_depth=float(choice['rear_local_x']-position[0]),
            rear_observed=False,faces=[],all_faces=[],side_faces=[],choices=[dict(c,historical=True) for c in self.best.values()],
            observation_age_s=t-choice['observed_at'],
            reason='좌우 45도 관측 후 같은 벽의 관측 바닥 폭 비교 · 선택 방향 유지',
            limitation='바닥 외관과 기체 이동량으로 연결한 추정; 그물 없는 공간이나 벽 뒤쪽을 보증하지 않음')

    def evidence(self,position,t):
        evidence=dict(self.turn_evidence or {},current_faces=self.current_faces,current_choices=self.current_choices,
            choices=[dict(c,samples=self.samples[side],observation_age_s=t-c['observed_at']) for side,c in self.best.items()],
            selected_side=self.selected_side,selection='같은 벽을 좌우에서 관측한 바닥 폭 비교',
            guidance=self.guidance(position,t))
        return evidence
