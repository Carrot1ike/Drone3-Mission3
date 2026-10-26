"""Monochrome geometry. No scene, map coordinates, truth or previous mission code."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

F = 244/(2*np.tan(np.deg2rad(35)))
CX, CY = 161.5, 121.5

def attitude(state):
    r=Rotation.from_quat(state['quaternion_xyzw']).as_matrix()
    yaw=np.arctan2(r[1,0],r[0,0])
    return r, yaw

def level_ray(u,v,state):
    r,yaw=attitude(state)
    c,s=np.cos(yaw),np.sin(yaw)
    return np.array([[c,s,0],[-s,c,0],[0,0,1]])@r@np.array([1.,-(u-CX)/F,-(v-CY)/F])

def rings(image,state):
    # Hough proposes circles; the physical bright annulus must also be present.
    # Score radial brightness against BOTH neighboring surfaces to reject floor texture.
    gray=cv2.GaussianBlur(image,(5,5),1)
    padded=cv2.copyMakeBorder(gray,70,70,90,90,cv2.BORDER_CONSTANT,value=0)
    proposals=cv2.HoughCircles(padded,cv2.HOUGH_GRADIENT,1.,18,param1=90,param2=35,
                               minRadius=12,maxRadius=160)
    if proposals is None:return []
    theta=np.linspace(0,2*np.pi,96,endpoint=False)
    cos,sin=np.cos(theta),np.sin(theta)
    result=[]
    for u,v,edge_radius in proposals[0][:18]:
        u-=90;v-=70
        best=None
        for dx,dy in [(0,0),(-2,0),(2,0),(0,-2),(0,2)]:
            for ratio in np.linspace(.90,1.11,15):
                radius=float(edge_radius*ratio)
                def sample(scale):
                    xx=np.rint(u+dx+radius*scale*cos).astype(int)
                    yy=np.rint(v+dy+radius*scale*sin).astype(int)
                    valid=(xx>=0)&(xx<324)&(yy>=0)&(yy<244)
                    return image[np.clip(yy,0,243),np.clip(xx,0,323)].astype(float),valid
                mid,a=sample(1);inside,b=sample(.84);outside,c=sample(1.18)
                valid=a&b&c;contrast=mid-np.maximum(inside,outside)
                fraction=np.mean((contrast>18)&valid)
                available=float(valid.mean())
                upper=valid&(sin<0)
                upper_fraction=float(np.mean(contrast[upper]>18)) if upper.sum()>15 else 0.
                merit=fraction+.25*upper_fraction+.001*float(np.clip(contrast,-20,80)[valid].mean()) if available>.25 else 0
                if best is None or merit>best[0]:best=(merit,fraction,available,upper_fraction,float(u+dx),float(v+dy),radius)
        merit,fraction,available,upper_fraction,u,v,radius=best
        if fraction<.24 or fraction/max(available,.01)<.44 or upper_fraction<.68:continue
        if any(np.linalg.norm(np.subtract(x['center'],[u,v]))<15 for x in result):continue
        ray=level_ray(u,v,state);depth=F*1.19/(2*radius)
        if ray[0]<=0:continue
        inner=2*radius*.55/.595
        full=(radius>14 and u-radius*1.08>3 and u+radius*1.08<321
              and v-radius*1.08>3 and v+radius*1.08<241 and fraction>.38)
        result.append(dict(center=[u,v],axes=[inner,inner],angle=0.,diameter=inner,
            depth=float(depth),left=float(ray[1]/ray[0]*depth),up=float(ray[2]/ray[0]*depth),
            full=bool(full),support=fraction,score=merit))
    return sorted(result,key=lambda x:x['diameter'],reverse=True)

def vertical_obstacles(image,state,kind='block'):
    """Solid faces above the visual horizon; depth from observed floor contact."""
    r,_=attitude(state)
    yy,xx=np.indices(image.shape)
    vertical=r[2,0]-r[2,1]*(xx-CX)/F-r[2,2]*(yy-CY)/F
    # Sky is black; broad solid surfaces remain after removing thin fence wires.
    threshold=75 if kind=='block' else 35
    mask=((image>threshold)&(vertical>.025)).astype(np.uint8)*255
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    result=[]
    for x,y,w,h,area in stats[1:]:
        if w<12 or h<28 or w>300:continue
        # Follow the same uniform face below the horizon until textured floor starts.
        xa=x+max(3,w//4);xb=x+w-max(3,w//4)
        if xb<=xa:continue
        reference=float(np.median(image[y+max(2,h//3):y+h,xa:xb]))
        bottom=y+h
        misses=0
        for v in range(min(y+h,243),244):
            fraction=float(np.mean(abs(image[v,xa:xb].astype(float)-reference)<12))
            misses=misses+1 if fraction<.55 else 0
            if misses>=3:bottom=v-2;break
            bottom=v
        if bottom>=240:continue
        ray=level_ray(x+w/2,bottom,state)
        if ray[2]>=-.025:continue
        depth=-state['ekf_height_m']/ray[2]
        if not .3<depth<15:continue
        left_ray=level_ray(x,bottom,state);right_ray=level_ray(x+w,bottom,state)
        result.append(dict(box=[int(x),int(y),int(w),int(bottom-y)],depth=float(depth),
            left_edge=float(left_ray[1]*depth),right_edge=float(right_ray[1]*depth),gray=reference))
    return sorted(result,key=lambda x:x['depth'])
