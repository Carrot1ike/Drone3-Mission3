"""Local motion from tracked ground texture, with original ego-velocity fallback.

No fence/grid detector, scene coordinates, or obstacle dimensions are used.
"""
import cv2
import numpy as np
from .passages import level_image
from .vision import F,CX,CY


class FloorOdometry:
    def __init__(self):
        self.previous=None;self.height=None;self.position=np.zeros(2)

    def step(self,image,state,ego_delta):
        gray=level_image(image,state);z=state['ekf_height_m']
        delta=np.array(ego_delta[:2],dtype=float);source='ego_velocity';tracks=[];count=0
        if self.previous is not None and z>.2 and self.height>.2:
            mask=np.zeros(gray.shape,np.uint8);mask[184:235,20:304]=255
            points=cv2.goodFeaturesToTrack(self.previous,150,.02,5,mask=mask,blockSize=5)
            if points is not None and len(points)>8:
                nxt,ok,error=cv2.calcOpticalFlowPyrLK(self.previous,gray,points,None,
                                                    winSize=(15,15),maxLevel=2)
                a=points[:,0];b=nxt[:,0]
                valid=(ok[:,0]>0)&(error[:,0]<25)&(b[:,1]>160)&(b[:,1]<240)&(b[:,0]>5)&(b[:,0]<319)
                a=a[valid];b=b[valid]
                if len(a)>8:
                    ga=np.column_stack([F*self.height/(a[:,1]-CY),(CX-a[:,0])*self.height/(a[:,1]-CY)])
                    gb=np.column_stack([F*z/(b[:,1]-CY),(CX-b[:,0])*z/(b[:,1]-CY)])
                    differences=ga-gb;median=np.median(differences,axis=0)
                    residual=np.linalg.norm(differences-median,axis=1)
                    keep=residual<max(.02,3*np.median(residual))
                    delta=np.median(differences[keep],axis=0).astype(float)
                    source='ground_texture';count=int(keep.sum())
                    tracks=np.concatenate([a[keep],b[keep]],axis=1)[::3].tolist()
        self.position+=delta;self.previous=gray;self.height=z
        return dict(source=source,inliers=count,delta_m=delta.tolist(),position_m=self.position.tolist(),
                    tracks=tracks,roi=[20,184,284,51],assumption='static textured ground plane')
