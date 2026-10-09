"""Official PC velocity PID produces CF attitude angles and collective."""
import ctypes as ct
import math
import numpy as np
from .settings import ROOT

class PCPositionPid:
    def __init__(self,position):
        self.lib=ct.CDLL(str(ROOT/"build/mission3-position-bridge/libmission3_position_bridge.so"))
        f=ct.c_float;p=ct.POINTER(f)
        self.lib.bridge_init.argtypes=[f]*3;self.lib.bridge_init.restype=None
        self.lib.bridge_update.argtypes=[p,p,f,p];self.lib.bridge_update.restype=None
        for name in ("bridge_roll_deg","bridge_pitch_deg","bridge_collective"):
            getattr(self.lib,name).argtypes=[];getattr(self.lib,name).restype=f
        self.lib.bridge_init(*position)

    def update(self,position,velocity,heading_rad,velocity_command):
        values=[np.asarray(v,float) for v in (position,velocity,velocity_command)]
        if any(v.shape!=(3,) or not np.isfinite(v).all() for v in values) or not math.isfinite(heading_rad):
            raise ValueError("finite estimated state and velocity command required")
        vectors=[(ct.c_float*3)(*v) for v in values]
        self.lib.bridge_update(vectors[0],vectors[1],math.degrees(heading_rad),vectors[2])
        return {"roll_deg":float(self.lib.bridge_roll_deg()),"pitch_deg":float(self.lib.bridge_pitch_deg()),
                "legacy_collective":float(self.lib.bridge_collective()),
                "source":"official position_controller_pid.c on PC; sensor-derived state only"}
