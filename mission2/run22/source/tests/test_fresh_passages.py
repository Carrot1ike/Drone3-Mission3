import cv2
import numpy as np
import pytest
from mission2_fresh.passages import F,CX,CY,blocks,wall,exit_gate
from mission2_fresh.mission import Mission


def state(z=1.):
    return dict(quaternion_xyzw=[0.,0.,0.,1.],ekf_height_m=z,body_velocity_flu=[0.,0.,0.])


def project(x,left,height,z=1.):
    return [round(CX-F*left/x),round(CY-F*(height-z)/x)]


def rectangle(image,depth,left,width,height,shade,z=1.):
    pts=np.array([project(depth,l,h,z) for l,h in [(left+width/2,0),(left-width/2,0),
                (left-width/2,height),(left+width/2,height)]])
    cv2.fillConvexPoly(image,pts,shade)


def floor_image():
    image=np.zeros((244,324),np.uint8)
    yy,xx=np.indices((122,324))
    image[122:]=38+((xx//8+yy//8)%2)*9
    return image


def block(image,depth,left,width,height,length):
    inner=left-np.sign(left)*width/2
    pts=np.array([project(x,inner,h) for x,h in [(depth,0),(depth+length,0),
                (depth+length,height),(depth,height)]])
    cv2.fillConvexPoly(image,pts,65)
    rectangle(image,depth,left,width,height,112)


@pytest.mark.parametrize('width,height,length',[(.45,1.8,.4),(1.0,3.4,1.2),(.7,2.5,.7)])
def test_block_dimensions_are_measured_not_supplied(width,height,length):
    image=np.zeros((244,324),np.uint8)
    block(image,2.5,.75+width/2,width,height,length)
    block(image,2.5,-.75-width/2,width,height,length)
    found=blocks(image,state())
    assert found is not None
    assert abs(found['depth']-2.5)<.1
    assert abs(found['gap_m']-1.5)<.1
    assert abs(found['rear_depth']-(2.5+length))<.25


@pytest.mark.parametrize('height',[1.7,2.5,3.6])
def test_wall_direction_changes_when_image_is_mirrored(height):
    image=floor_image()
    rectangle(image,3.,.9,3.,height,55)
    first=wall(image,state())
    second=wall(image[:,::-1].copy(),state())
    assert first['side']==-1 and second['side']==1
    assert abs(first['depth']-3.)<.1 and abs(second['depth']-3.)<.1
    assert abs(first['left']+second['left'])<.05


def test_hidden_floor_does_not_use_known_object_height():
    image=np.zeros((244,324),np.uint8)
    rectangle(image,.8,.2,1.,2.5,55)
    assert wall(image,state()) is None


@pytest.mark.parametrize('width,height',[(.8,1.9),(1.5,2.6),(2.4,3.0)])
def test_exit_distance_uses_floor_contacts_without_width_prior(width,height):
    image=np.zeros((244,324),np.uint8)
    for left in [(width+.07)/2,-(width+.07)/2]:rectangle(image,2.,left,.07,height,130)
    found=exit_gate(image,state())
    assert found is not None and abs(found['depth']-2.)<.08
    assert abs(found['left'])<.02
    assert abs(found['gap_m']-width)<.06


def test_wall_lateral_alignment_and_count_do_not_select_side():
    image=floor_image()
    rectangle(image,2.8,.9,3.,2.1,55)
    for stage in ['WALL_1','WALL_2']:
        mission=Mission();mission.stage=stage;mission.index=3
        mission.passage_phase='align'
        command=mission.step(image,state(),1.)
        assert command['vx']==0. and command['vy']<0.
        assert 'ring_pass_mask' not in command


def test_floor_odometry_uses_texture_translation_without_fence():
    from mission2_fresh.floor_odometry import FloorOdometry
    rng=np.random.default_rng(72)
    previous=cv2.GaussianBlur(rng.integers(0,255,(244,324),dtype=np.uint8),(3,3),0)
    u,v=np.meshgrid(np.arange(324,dtype=np.float32),np.arange(244,dtype=np.float32))
    displacement=.06
    current=cv2.remap(previous,u-displacement*(v-CY),v,cv2.INTER_LINEAR)
    odometry=FloorOdometry();odometry.step(previous,state(),[0,0,0])
    observed=odometry.step(current,state(),[0,0,0])
    assert observed['source']=='ground_texture'
    assert abs(observed['delta_m'][1]-displacement)<.01
    assert abs(observed['delta_m'][0])<.01
