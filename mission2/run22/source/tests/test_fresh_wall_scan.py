import math
import numpy as np
import pytest
from mission2_fresh.vision import F,CX,CY
from mission2_fresh.wall_scan import WallScan,plane_pixels
from mission2_fresh.mission import Mission

def state(yaw=0):
    return dict(quaternion_xyzw=[0,0,math.sin(yaw/2),math.cos(yaw/2)],ekf_height_m=1.,
                body_velocity_flu=[0.,0.,0.],body_angular_rate_flu=[0.,0.,0.])

def scene(yaw=0,mirror=False):
    v,u=np.indices((244,324));a=(CX-u)/F;b=(CY-v)/F;c,s=np.cos(yaw),np.sin(yaw)
    dx=c-s*a;dy=s+c*a
    image=np.zeros((244,324),np.uint8)
    tf=np.divide(-1.,b,out=np.zeros_like(b),where=b<0)
    x,y=tf*dx,tf*dy
    floor=(b<0)&(x>0)&(x<12)&(np.abs(y)<2.5)
    image[floor]=(100+10*((np.floor(x[floor]*8)+np.floor(y[floor]*8)).astype(int)%2)).astype(np.uint8)
    tw=np.divide(3.,dx,out=np.zeros_like(dx),where=dx>0)
    wy,wz=tw*dy,1+tw*b
    low,high=(-2.5,.5) if mirror else (-.5,2.5)
    wall=(dx>0)&(wy>=low)&(wy<=high)&(wz>=0)&(wz<=2.)
    image[wall]=55
    return image

@pytest.mark.parametrize('yaw',[-math.pi/4,0,math.pi/4])
def test_projection_uses_observed_plane_and_handles_yaw(yaw):
    feet,points,valid=plane_pixels(np.array([1.,0.]),np.array([3.,0.]),np.zeros(3),yaw,1.)
    assert valid.any()
    assert np.allclose(points[valid,0],3.)
    assert np.isfinite(feet[valid]).all()

@pytest.mark.parametrize('mirror,expected',[(False,-1),(True,1)])
def test_scan_observes_both_views_before_selecting_side(mirror,expected):
    scan=WallScan(0.);position=np.zeros(3);t=0.
    for degree in [0,15,30,45,45,30,0,-30,-45,-45,-30,0,0]:
        t+=.4;yaw=math.radians(degree)
        scan.step(scene(yaw,mirror),state(yaw),position,t)
        if not scan.turn.done:assert scan.selected_side is None
    assert scan.turn.done
    assert scan.selected_side==expected
    guide=scan.guidance(position,t)
    assert guide['gap_m']>1.
    assert guide['side']==expected
    assert 'map' not in guide
    assert guide['method']=='yaw_scan_floor_strip'

def test_wall_scan_outputs_no_translation_or_ring_mask():
    m=Mission(ring_mask=1,wall_scan=True)
    m.stage='BLOCK_STRAIGHT';m.transition('WALL_1',0.,state())
    assert m.stage=='WALL_SCAN_BRAKE'
    m.step(scene(),state(),0.)
    m.step(scene(),state(),2.)
    assert m.stage=='WALL_SCAN'
    command=m.step(scene(),state(),2.1)
    assert command['vx']==command['vy']==0
    assert command['ring_pass_mask']==0
    assert command['yawrate']>0

def test_blank_scan_cannot_invent_a_passage():
    scan=WallScan(0.)
    scan.observe(np.zeros((244,324),np.uint8),state(),np.zeros(3),0.)
    assert not scan.best and scan.guidance(np.zeros(3),0.) is None
