"""Conditional coordinated approach reference, not a hardware capability."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CoordinatedApproach:
    start_s: float
    duration_s: float
    start_standoff_m: float
    end_standoff_m: float
    start_height_m: float
    end_height_m: float

    def __post_init__(self):
        values=tuple(vars(self).values())
        if not all(math.isfinite(x) for x in values) or self.duration_s<=0:
            raise ValueError('finite positive-duration approach reference required')
        if self.start_s<0 or self.end_standoff_m<=0 or self.start_standoff_m<self.end_standoff_m:
            raise ValueError('approach reference must retain positive standoff and approach forward')
        if self.start_height_m<self.end_height_m:
            raise ValueError('coordinated approach must not command a reference climb')

    def at(self,now):
        if not math.isfinite(now):raise ValueError('finite approach clock required')
        u=min(1.,max(0.,(now-self.start_s)/self.duration_s))
        p=u**3*(10.+u*(-15.+6.*u))
        dp=30.*u*u*(1.-u)**2/self.duration_s
        ddp=60.*u*(1.-u)*(1.-2.*u)/self.duration_s**2
        out={'progress':p,'complete':now>=self.start_s+self.duration_s}
        for name,first,last in (('standoff',self.start_standoff_m,self.end_standoff_m),
                                ('height',self.start_height_m,self.end_height_m)):
            delta=last-first
            out[name+'_m']=first+delta*p
            out[name+'_rate_m_s']=delta*dp
            out[name+'_accel_m_s2']=delta*ddp
        return out
