import math
import cv2
import numpy as np
import pytest
from mission2_fresh.exit_track import ExitTrack,posts
from mission2_fresh.vision import F,CX,CY


def state(yaw=0.):
    a=math.radians(yaw)/2
    return dict(quaternion_xyzw=[0.,0.,math.sin(a),math.cos(a)],ekf_height_m=1.,body_velocity_flu=[0.,0.,0.])


def scene(width=1.8,center=0.,depth=2.,yaw=0.,post_width=.07):
    image=np.zeros((244,324),np.uint8)
    yy,xx=np.indices((122,324));image[122:]=95+(xx//8+yy//8)%3*10
    a=math.radians(yaw);c,s=math.cos(a),math.sin(a)
    for middle in [center+(width+post_width)/2,center-(width+post_width)/2]:
        points=[]
        for left,h in [(middle+post_width/2,0),(middle-post_width/2,0),
                       (middle-post_width/2,2.6),(middle+post_width/2,2.6)]:
            d=c*depth+s*left;l=-s*depth+c*left
            if d<=0:break
            points.append([round(CX-F*l/d),round(CY-F*(h-1.)/d)])
        if len(points)==4:cv2.fillConvexPoly(image,np.array(points),130)
    return image


@pytest.mark.parametrize('width',[.8,1.8,2.4])
def test_known_opening_width_sets_scale_without_position_prior(width):
    tracker=ExitTrack(width);found=tracker.update(scene(width),state(),np.zeros(3),0.)
    assert found is not None
    assert found['depth']==pytest.approx(2.,abs=.07)
    assert found['left']==pytest.approx(0.,abs=.03)
    assert found['gap_m']==width


def test_one_post_is_ambiguous_until_second_view():
    tracker=ExitTrack(1.8)
    # Gate is off the initial optical axis. Yaw changes visibility, not its location.
    first=tracker.update(scene(center=1.6,yaw=0),state(0),np.zeros(3),0.)
    assert len(tracker.current)==1 and first is None
    second=tracker.update(scene(center=1.6,yaw=45),state(45),np.zeros(3),1.)
    assert second is not None
    assert second['depth']==pytest.approx(2.,abs=.12)
    assert second['left']==pytest.approx(1.6,abs=.12)


def test_nearby_wide_pixel_post_is_not_discarded():
    found=posts(scene(depth=1.8,post_width=.25),state())
    assert len(found)==2
    assert all(p['box'][2]>18 for p in found)


def test_blank_image_cannot_invent_exit_from_width_alone():
    tracker=ExitTrack(1.8)
    assert tracker.update(np.zeros((244,324),np.uint8),state(),np.zeros(3),0.) is None
