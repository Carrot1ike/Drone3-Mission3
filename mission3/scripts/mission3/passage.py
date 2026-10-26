"""시험 장치 전용 화각 통과 판정. 참값은 비행 명령에 쓰지 않는다.

판 전체가 수평 화각을 지나간 뒤 0.25초 유예하여 이미 전송 중인 영상을
처리한다. 제조사 실제 K/D/장착이 미확정이므로 이 판정도 조건부이다.
"""
import math
import numpy as np
from .track import rotation


class PassageStop:
    def __init__(self, cfg, grace_s=.25):
        self.cfg=cfg
        self.grace=grace_s
        self.seen_horizontal_overlap=False
        self.outside_since=None
        self.latest=None

    def update(self, plant, now, decoded):
        if decoded:return None
        cfg=self.cfg
        side=cfg["marker"]["board_side_m"]
        offset=np.asarray(cfg["marker"]["known_marker_to_pad_vehicle_m"])
        corners=np.array([[0.,y,z] for y in (-side/2,side/2) for z in (-side/2,side/2)])+offset
        world=plant.pad+corners@rotation(plant.vehicle_heading).T
        camera=plant.position+plant.rotation@np.asarray(plant.physical["mounts"]["camera_position_body_m"])
        points=(world-camera)@plant.rotation@plant.camera_rotation
        positive=bool(np.all(points[:,2]>1e-6))
        if not positive:
            self.outside_since=None
            return None
        normalized=points[:,:2]/points[:,2,None]
        half=math.tan(math.radians(cfg["camera"]["horizontal_fov_deg"]/2))
        xmin,xmax=float(normalized[:,0].min()),float(normalized[:,0].max())
        if xmin <= half and xmax >= -half:self.seen_horizontal_overlap=True
        # Eastbound rear-facing board exits the right image edge in this fixture.
        passed=self.seen_horizontal_overlap and xmin > half
        if not passed:self.outside_since=None
        elif self.outside_since is None:self.outside_since=now
        self.latest={"t":now,"horizontal_overlap_seen":self.seen_horizontal_overlap,
            "whole_board_outside_right":bool(passed),"normalized_x_min":xmin,"normalized_x_max":xmax,
            "normalized_half_fov":half,"outside_since_s":self.outside_since,
            "grace_s":self.grace,"scope":"plant-truth test termination only; no controller/estimator input"}
        if passed and now-self.outside_since >= self.grace-1e-8:
            return "unrecognized_marker_passed_camera_fov"
        return None
