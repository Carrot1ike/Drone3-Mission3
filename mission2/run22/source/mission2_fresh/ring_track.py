"""Keep an observed ring through image clipping, using onboard motion only."""
import math
import numpy as np
from .vision import F, CX, CY, attitude


class RingTrack:
    def __init__(self):
        self.landmark = None
        self.last_seen = None

    def predict(self, state, position):
        if self.landmark is None:
            return None
        rotation, yaw = attitude(state)
        delta = self.landmark - position
        camera = rotation.T @ delta
        if camera[0] <= 0:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        level = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ delta
        diameter = F * 1.10 / level[0]
        return dict(center=[float(CX-F*camera[1]/camera[0]), float(CY-F*camera[2]/camera[0])],
                    axes=[float(diameter)]*2, diameter=float(diameter), angle=0.,
                    depth=float(level[0]), left=float(level[1]), up=float(level[2]),
                    full=False, support=0., score=0., observation_source='ego_prediction')

    def update(self, candidates, state, position, t):
        predicted = self.predict(state, position)
        selected = None
        if self.landmark is None:
            selected = candidates[0] if candidates else None
        elif predicted is not None:
            # Same image association tolerances as the previous ALIGN selector,
            # now relative to motion-predicted pixels instead of the old image.
            matches = [x for x in candidates
                       if .7 < x['diameter']/predicted['diameter'] < 1.4
                       and np.linalg.norm(np.subtract(x['center'], predicted['center'])) < 45]
            if matches:
                selected = min(matches, key=lambda x: np.linalg.norm(np.subtract(x['center'], predicted['center'])))
        if selected is not None:
            selected = dict(selected, observation_source='current_image')
            _, yaw = attitude(state)
            c, s = math.cos(yaw), math.sin(yaw)
            d, l, u = selected['depth'], selected['left'], selected['up']
            self.landmark = np.asarray(position) + [c*d-s*l, s*d+c*l, u]
            self.last_seen = t
            return selected
        return predicted

    def evidence(self, t):
        return dict(locked=self.landmark is not None,
                    last_observation_age_s=None if self.last_seen is None else float(t-self.last_seen),
                    landmark_local_m=None if self.landmark is None else self.landmark.tolist(),
                    source='camera + integrated onboard velocity; no map or simulator pose')
