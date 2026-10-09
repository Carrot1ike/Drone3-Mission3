"""Generic downward image diagnostic, explicitly not the PMW3901 algorithm.

Virtual frame resolution, pinhole optics and quality thresholds are S settings.
The actual chip image array, illumination, shutter/SQUAL and correlation engine
remain unresolved. This probe can expose untextured/aperture/aliasing risks; it
cannot certify a real deck measurement or manufacture hardware calibration.
"""
import cv2
import numpy as np


class FlowTextureProbe:
    def __init__(self,settings):
        self.settings=settings
        self.previous=None
        self.previous_stamp=None

    def measure(self,gray,stamp):
        if gray.dtype!=np.uint8 or gray.ndim!=2:
            raise ValueError("virtual flow diagnostic requires native mono8 image")
        size=int(self.settings.get("virtual_resolution_px",64))
        if gray.shape!=(size,size):
            raise ValueError("virtual flow diagnostic image must not be resized")
        if self.previous_stamp is not None and stamp<=self.previous_stamp:
            raise ValueError("virtual image capture time must increase")
        values=gray.astype(np.float32)
        centered=values-float(values.mean())
        contrast=float(values.std())
        gx=cv2.Sobel(values,cv2.CV_32F,1,0,ksize=3,scale=.125)[2:-2,2:-2]
        gy=cv2.Sobel(values,cv2.CV_32F,0,1,ksize=3,scale=.125)[2:-2,2:-2]
        tensor=np.array([[np.mean(gx*gx),np.mean(gx*gy)],
                         [np.mean(gx*gy),np.mean(gy*gy)]],float)
        eigenvalues=np.linalg.eigvalsh(tensor)
        usable=(contrast>=self.settings.get("min_intensity_std",2.)
                and eigenvalues[0]>=self.settings.get("min_gradient_eigenvalue",.25))
        reason="two_axis_virtual_texture" if usable else "insufficient_virtual_texture_or_aperture"
        shift=None;response=None;angular=None
        if usable and self.previous is not None:
            window=cv2.createHanningWindow((size,size),cv2.CV_32F)
            # Copies: OpenCV may window its inputs in place.
            displacement,response=cv2.phaseCorrelate(self.previous.copy(),centered.copy(),window)
            shift=list(map(float,displacement));response=float(response)
            if not np.isfinite(shift).all() or not np.isfinite(response):
                usable=False;reason="virtual_correlation_nonfinite"
            elif response<self.settings.get("min_correlation_response",.2):
                usable=False;reason="virtual_correlation_weak"
            elif max(map(abs,shift))>size*self.settings.get("max_shift_fraction",.2):
                usable=False;reason="virtual_correlation_shift_limit"
            f=size/(2*np.tan(np.deg2rad(self.settings["virtual_fov_deg"])/2))
            dt=stamp-self.previous_stamp
            # Camera looks along body -Z, pixel right +X, pixel down -Y.
            angular=[-shift[0]/(f*dt),shift[1]/(f*dt)]
        self.previous=centered
        self.previous_stamp=float(stamp)
        return {"usable":bool(usable),"reason":reason,"intensity_std":contrast,
                "gradient_eigenvalues":eigenvalues.tolist(),"shift_px":shift,
                "correlation_response":response,"diagnostic_angular_rad_s":angular,
                "source_class":"sensitivity_assumption",
                "algorithm":"generic OpenCV phase correlation and image gradient diagnostic",
                "actual_pmw3901_quality_verified":False}
