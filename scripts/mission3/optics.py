"""Published-FOV-constrained synthetic lens candidates, not calibration.

MuJoCo renders a wider pinhole. Each native output pixel samples its candidate
lens ray; this is a virtual sensor transfer, never a PC input resize repair.
"""
import math
import cv2
import numpy as np


def brown_candidate(horizontal_fov_deg=87.,diagonal_fov_deg=115.,side=320):
    horizontal_radius=math.tan(math.radians(horizontal_fov_deg)/2)
    diagonal_radius=math.tan(math.radians(diagonal_fov_deg)/2)
    A=np.array([[horizontal_radius,horizontal_radius**3],[diagonal_radius,diagonal_radius**3]])
    f,fk=np.linalg.solve(A,np.array([side/2,side/math.sqrt(2)]))
    k1=fk/f
    return {"K":[[float(f),0.,side/2],[0.,float(f),side/2],[0.,0.,1.]],
            "distortion":[float(k1),0.,0.,0.,0.],
            "horizontal_fov_deg":horizontal_fov_deg,"vertical_fov_deg":horizontal_fov_deg,"diagonal_fov_deg":diagonal_fov_deg,
            "source_class":"sensitivity candidate derived from published angular boundary values; not measured lens K/D"}


class SyntheticLens:
    def __init__(self,settings):
        self.c=settings;self.K=np.asarray(settings["K"],float);self.D=np.asarray(settings["distortion"],float)
        self.source_fov_deg=float(settings.get("render_vertical_fov_deg",100.))
        f=160./math.tan(math.radians(self.source_fov_deg)/2)
        source=np.array([[f,0,160.],[0,f,160.],[0,0,1.]])
        rows,cols=np.mgrid[0:320,0:320];pixels=np.stack([cols,rows],axis=-1).reshape(-1,1,2).astype(np.float64)
        # Iterative inverse avoids the 5-iteration OpenCV default near corners.
        rays=cv2.undistortPointsIter(pixels,self.K,self.D,None,None,
                                    (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,80,1e-12)).reshape(320,320,2)
        self.map_x=(rays[:,:,0]*source[0,0]+source[0,2]).astype(np.float32)
        self.map_y=(rays[:,:,1]*source[1,1]+source[1,2]).astype(np.float32)
        if not(np.all(self.map_x>=0)&np.all(self.map_x<320)&np.all(self.map_y>=0)&np.all(self.map_y<320)):
            raise ValueError("synthetic render camera does not cover all native sensor rays")

    def sample(self,gray):
        if gray.shape!=(320,320) or gray.dtype!=np.uint8:raise ValueError("synthetic source raster mismatch")
        return cv2.remap(gray,self.map_x,self.map_y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
