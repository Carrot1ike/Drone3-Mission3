import math
import numpy as np
import pytest
from mission2_fresh.ring_scan import RingScan
from mission2_fresh.mission import Mission
from mission2_fresh.vision import F,CX,CY


def state(deg):
    a=math.radians(deg)/2
    return dict(quaternion_xyzw=[0.,0.,math.sin(a),math.cos(a)],ekf_height_m=1.,body_velocity_flu=[0.,0.,0.])


def observation(point,deg):
    a=math.radians(deg);c,s=math.cos(a),math.sin(a)
    x,y,z=point;d=c*x+s*y;l=-s*x+c*y
    return dict(depth=d,left=l,up=z,center=[CX-F*l/d,CY-F*z/d],diameter=F*1.1/d,
                full=True,support=.9)


def test_scan_visits_both_sides_and_returns_before_selecting():
    scan=RingScan(0.);position=np.zeros(3)
    rate,_=scan.step([],state(0),position,0.)
    assert rate>0 and not scan.done
    scan.step([],state(45),position,3.)
    scan.step([],state(45),position,3.3)
    rate,_=scan.step([],state(45),position,3.4)
    assert rate<0 and not scan.done
    scan.step([],state(-45),position,8.)
    scan.step([],state(-45),position,8.3)
    rate,_=scan.step([],state(-45),position,8.4)
    assert rate>0 and not scan.done
    scan.step([],state(0),position,12.)
    rate,evidence=scan.step([],state(0),position,12.3)
    assert scan.done and rate==0.
    assert [p['goal_deg'] for p in evidence['visited']]==[45.,-45.,0.]


def test_scan_combines_yaw_views_and_selects_nearest_3d_ring():
    scan=RingScan(0.);near=np.array([2.,-1.5,.2]);far=np.array([5.,1.,.5])
    for deg,t in [(0,0.),(-45,1.),(-20,2.)]:
        scan.step([observation(near,deg),observation(far,deg)],state(deg),np.zeros(3),t)
    assert len(scan.candidates)==2
    assert all(c['samples']==3 for c in scan.candidates)
    assert scan.closest(np.zeros(3))['point']==pytest.approx(near)


def test_entry_scan_only_commands_rotation_and_keeps_tof_enabled():
    mission=Mission(ring_mask=1);mission.stage='SCAN';mission.scan=RingScan(0.)
    command=mission.step(np.zeros((244,324),np.uint8),state(0),1.)
    assert command['vx']==command['vy']==0.
    assert command['yawrate']>0.
    assert command['ring_pass_mask']==0


def test_hover_marker_only_during_ring_straight():
    mission=Mission(ring_mask=1);mission.stage='STRAIGHT';mission.mask=1;mission.distance=.3
    image=np.zeros((244,324),np.uint8)
    command=mission.step(image,state(0),0.)
    assert command['ring_pass_mask']==1
    mission.travel=.4
    command=mission.step(image,state(0),.1)
    assert command['ring_pass_mask']==0 and mission.stage=='BRAKE'


def test_scan_brakes_rotation_near_target_using_measured_rate():
    scan=RingScan(0.);moving=state(40)
    moving['body_angular_rate_flu']=[0.,0.,math.radians(20.)]
    rate,evidence=scan.step([],moving,np.zeros(3),0.)
    assert rate<0  # Angle is still short of +45, but existing rotation needs braking.
    assert evidence['measured_yaw_rate_deg_s']==pytest.approx(20.)
