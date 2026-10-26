import numpy as np
import pytest
from mission2_fresh.ring_track import RingTrack
from mission2_fresh.vision import F, CX, CY
from mission2_fresh.mission import Mission


def state():
    return dict(quaternion_xyzw=[0.,0.,0.,1.],ekf_height_m=1.,body_velocity_flu=[0.,0.,0.])


def ring(depth,left,up=0.):
    return dict(center=[CX-F*left/depth,CY-F*up/depth],depth=depth,left=left,up=up,
                diameter=F*1.10/depth,axes=[F*1.10/depth]*2,full=True,support=.9,score=1.,angle=0.)


def test_clipped_front_ring_does_not_switch_to_far_ring():
    track=RingTrack();first=track.update([ring(3.,-2.),ring(6.,-.7)],state(),np.zeros(3),0.)
    assert first['observation_source']=='current_image'
    predicted=track.update([ring(4.5,-.7)],state(),np.array([1.5,0,0]),2.)
    assert predicted['observation_source']=='ego_prediction'
    assert predicted['depth']==pytest.approx(1.5)
    assert predicted['left']==pytest.approx(-2.)
    assert predicted['full'] is False
    assert predicted['center'][0]>324
    assert track.evidence(2.)['last_observation_age_s']==2.


def test_reacquisition_uses_measured_motion_and_current_pixels():
    track=RingTrack();track.update([ring(3.,-2.)],state(),np.zeros(3),0.)
    measured=track.update([ring(1.5,-.4)],state(),np.array([1.5,-1.6,0]),4.)
    assert measured['observation_source']=='current_image'
    assert measured['full'] is True
    assert track.evidence(4.)['last_observation_age_s']==0.


def test_entry_observation_survives_find_and_cannot_commit_from_prediction(monkeypatch):
    import mission2_fresh.mission as module
    current=[ring(3.,-2.)]
    monkeypatch.setattr(module,'rings',lambda *_:current)
    mission=Mission(entry_scan=False);mission.stage='ENTRY_FORWARD';mission.travel=1.6
    image=np.zeros((244,324),np.uint8)
    mission.step(image,state(),0.)
    assert mission.stage=='FIND' and mission.ring_track.landmark is not None
    current=[]
    for t in np.arange(.1,1.,.1):mission.step(image,state(),float(t))
    assert mission.stage=='ALIGN'
    assert mission.target['observation_source']=='ego_prediction'
    assert mission.decision['checks']['full'] is False
    assert mission.stable==0.
    mission.stage='BRAKE';mission.entered=0.
    mission.step(image,state(),2.)
    assert mission.ring_track.landmark is None
