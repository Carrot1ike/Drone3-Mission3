"""Observe left/right after entry and choose the closest observed ring."""
import math
import numpy as np
from .vision import attitude


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RingScan:
    offsets = (45., -45., 0.)
    rate_limit_deg_s = 30.
    angle_gain = .8
    rate_damping = .6

    def __init__(self, yaw):
        self.reference = yaw
        self.leg = 0
        self.settled_since = None
        self.candidates = []
        self.visited = []
        self.done = False

    def step(self, detections, state, position, t):
        _, yaw = attitude(state)
        c, s = math.cos(yaw), math.sin(yaw)
        for detection in detections:
            d, l, u = (detection[k] for k in ('depth','left','up'))
            point = np.asarray(position) + [c*d-s*l,s*d+c*l,u]
            # Association tolerance is one allowed ring radius. Candidates and
            # metric positions come from pixels and onboard state, never a map.
            matches = [x for x in self.candidates if np.linalg.norm(point-x['point']) < .55]
            entry = min(matches,key=lambda x:np.linalg.norm(point-x['point'])) if matches else None
            quality = float(detection['full']) + detection['support'] - .25*abs(math.atan2(l,d))
            if entry is None:
                entry = dict(id=len(self.candidates)+1,point=point,samples=0,quality=-1.,last_seen=t)
                self.candidates.append(entry)
            entry['samples'] += 1
            if quality > entry['quality']:
                entry.update(point=point,quality=quality,last_seen=t,observed_yaw_deg=math.degrees(yaw),
                             observation=dict(detection))
        relative = math.degrees(wrap(yaw-self.reference))
        goal = self.offsets[self.leg]
        error = wrap(self.reference+math.radians(goal)-yaw)
        if abs(math.degrees(error)) < 2.:
            if self.settled_since is None:self.settled_since=t
            if t-self.settled_since >= .25:
                self.visited.append(dict(goal_deg=goal,observed_deg=relative,time=t))
                self.settled_since=None
                if self.leg == 2:self.done=True
                else:self.leg+=1
        else:self.settled_since=None
        rotation,_=attitude(state)
        measured_rate_deg=math.degrees(float((rotation @ state.get('body_angular_rate_flu',[0.,0.,0.]))[2]))
        yawrate = 0. if self.done else float(np.clip(
            self.angle_gain*math.degrees(error)-self.rate_damping*measured_rate_deg,
            -self.rate_limit_deg_s,self.rate_limit_deg_s))
        evidence=[]
        for entry in self.candidates:
            evidence.append(dict(id=entry['id'],distance_m=float(np.linalg.norm(entry['point']-position)),
                point_local_m=entry['point'].tolist(),samples=entry['samples'],quality=entry['quality'],
                last_observation_age_s=float(t-entry['last_seen'])))
        evidence.sort(key=lambda x:x['distance_m'])
        return yawrate, dict(reference_yaw_deg=math.degrees(self.reference),relative_yaw_deg=relative,
            measured_yaw_rate_deg_s=measured_rate_deg,rate_limit_deg_s=self.rate_limit_deg_s,
            angle_gain=self.angle_gain,rate_damping=self.rate_damping,
            target_offset_deg=self.offsets[self.leg],visited=self.visited.copy(),complete=self.done,
            candidates=evidence,selection='관측한 후보의 3차원 추정 거리 비교 · 미관측 링은 알 수 없음')

    def closest(self, position):
        return min(self.candidates,key=lambda x:np.linalg.norm(x['point']-position)) if self.candidates else None
