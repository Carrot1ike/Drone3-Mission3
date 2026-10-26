"""Observe exit posts across yaw views, with an explicitly supplied opening width."""
import math
import cv2
import numpy as np
from .vision import F,CX,CY,attitude


def heading_image(image,state):
    rotation,yaw=attitude(state);c,s=math.cos(yaw),math.sin(yaw)
    level=np.array([[c,s,0],[-s,c,0],[0,0,1]])@rotation
    pixel_to_body=np.array([[0.,0.,1.],[-1/F,0.,CX/F],[0.,-1/F,CY/F]])
    return cv2.warpPerspective(image,np.linalg.inv(pixel_to_body)@level@pixel_to_body,(324,244))


def posts(image,state):
    gray=heading_image(image,state)
    histogram=np.bincount(gray[15:115].ravel(),minlength=256).astype(float)
    histogram[:80]=0;peaks=[]
    for _ in range(6):
        peak=int(np.argmax(histogram))
        if histogram[peak]<65:break
        peaks.append(peak);histogram[max(0,peak-9):peak+10]=0
    result=[]
    for peak in peaks:
        # A single bright mask joins the post to bright textured floor. Segment
        # the observed uniform post shade before finding its ground contact.
        mask=(np.abs(gray.astype(np.int16)-peak)<5).astype(np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((35,1),np.uint8))
        _,_,stats,_=cv2.connectedComponentsWithStats(mask)
        for x,y,w,h,_ in stats[1:]:
            foot=y+h
            if w<2 or h<=65 or h<=5*w or y>=CY or not CY+12<foot<240:continue
            if x<=1 or x+w>=323:continue
            depth=F*state['ekf_height_m']/(foot-CY)
            edges=np.array([[depth,(CX-x)*depth/F],[depth,(CX-x-w)*depth/F]])
            result.append(dict(box=list(map(int,(x,y,w,h))),floor_edge=[[int(x),int(foot)],[int(x+w),int(foot)]],
                edges_heading_m=edges.tolist(),depth=float(depth)))
    return result


class ExitTrack:
    def __init__(self,width):
        if not np.isfinite(width) or width<=0:raise ValueError('exit width must be positive')
        self.width=float(width);self.entries=[];self.current=[];self.pair=None

    def update(self,image,state,position,t):
        self.current=posts(image,state)
        _,yaw=attitude(state);c,s=math.cos(yaw),math.sin(yaw)
        rotation=np.array([[c,-s],[s,c]])
        for post in self.current:
            edges=np.asarray(post['edges_heading_m'])@rotation.T+np.asarray(position)[:2]
            center=edges.mean(axis=0)
            matches=[e for e in self.entries if np.linalg.norm(e['center']-center)<.15*self.width]
            entry=min(matches,key=lambda e:np.linalg.norm(e['center']-center)) if matches else None
            if entry is None:
                entry=dict(id=len(self.entries)+1,samples=0)
                self.entries.append(entry)
            entry.update(center=center,edges=edges,last_seen=t,samples=entry['samples']+1)
        choices=[]
        for i,a in enumerate(self.entries):
            for b in self.entries[i+1:]:
                left,right=(a,b) if a['center'][1]>b['center'][1] else (b,a)
                inner_left=left['edges'][np.argmin(left['edges'][:,1])]
                inner_right=right['edges'][np.argmax(right['edges'][:,1])]
                gap=float(np.linalg.norm(inner_left-inner_right))
                center=(inner_left+inner_right)/2
                depth=float(center[0]-position[0])
                if depth<=0 or abs(inner_left[0]-inner_right[0])>.25*depth:continue
                width_error=abs(gap-self.width)/self.width
                if width_error>.30:continue
                choices.append(dict(ids=[left['id'],right['id']],center=center,gap=gap,width_error=width_error,
                                    last_seen=min(left['last_seen'],right['last_seen'])))
        if choices:self.pair=min(choices,key=lambda x:(x['width_error'],np.linalg.norm(x['center']-np.asarray(position)[:2])))
        return self.guidance(position,t)

    def guidance(self,position,t):
        if self.pair is None:return None
        pair=self.pair;delta=pair['center']-np.asarray(position)[:2]
        if delta[0]<=0:return None
        # Known opening width sets scale; post identities and position came from images.
        scale=self.width/pair['gap'];depth,left=delta*scale
        return dict(kind='exit',method='scan_post_pair_known_width',depth=float(depth),left=float(left),
            rear_depth=float(depth),gap_m=self.width,measured_gap_m=pair['gap'],width_scale=scale,
            post_ids=pair['ids'],pair_observation_age_s=float(t-pair['last_seen']),
            faces=self.current,all_faces=self.current,side_faces=[],
            reason=f'좌우 관측 기둥 연결 · 출구 폭 {self.width:.2f}m로 거리 보정',
            limitation='정지 스캔과 기체 이동량으로 관측을 연결한 추정; 현재 양쪽 기둥 동시 검출과 다름')

    def evidence(self,position,t):
        return dict(known_width_m=self.width,current_posts=len(self.current),
            current_faces=self.current,
            posts=[dict(id=e['id'],point_local_m=e['center'].tolist(),samples=e['samples'],
                        observation_age_s=float(t-e['last_seen'])) for e in self.entries],
            pair=self.guidance(position,t))
