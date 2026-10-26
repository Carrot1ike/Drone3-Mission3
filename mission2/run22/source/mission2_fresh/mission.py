"""Small stop/align/straight mission, consuming only image-associated ego state."""
import math
import numpy as np
from .vision import rings, attitude
from .passages import blocks, wall, exit_gate

from .floor_odometry import FloorOdometry
from .ring_track import RingTrack
from .ring_scan import RingScan
from .exit_track import ExitTrack
from .wall_scan import WallScan

def clip(value,limit):return float(np.clip(value,-limit,limit))

class Mission:
    clearance_margin_m = .20

    def __init__(self,ring_mask=0,entry_scan=True,exit_width=None,wall_scan=False):
        self.stage='TAKEOFF';self.entered=None;self.last=None
        self.index=0;self.stable=0.;self.travel=0.;self.distance=0.
        self.height=1.
        self.target=None;self.obstacles=[];self.history=[]
        self.phase_start_height=0.;self.wall_side=None;self.wall_index=0
        self.position=np.zeros(3);self.decision={};self.committed=None
        self.motion=None;self.floor_odometry=FloorOdometry()
        self.heading=0.
        self.previous_target=None
        self.filtered_velocity=np.zeros(3)
        self.filtered_target=None
        self.passage_phase="approach"
        self.passage=None
        self.ring_track=RingTrack()
        self.ring_mask=ring_mask;self.mask=0
        self.entry_scan=entry_scan;self.scan=None;self.scan_result=None
        self.exit_tracker=ExitTrack(exit_width) if exit_width is not None else None
        self.exit_scanner=None;self.exit_scan_result=None
        self.wall_scan_enabled=wall_scan;self.wall_scanner=None;self.wall_scan_result=None
        self.pending_wall_stage=None

    def transition(self,stage,t,state):
        if self.wall_scan_enabled and stage in ('WALL_1','WALL_2') and self.stage!='WALL_SCAN':
            self.pending_wall_stage=stage;stage='WALL_SCAN_BRAKE'
        self.history.append(dict(from_stage=self.stage,to_stage=stage,time=t,ring_index=self.index))
        self.stage=stage;self.entered=t;self.stable=0.;self.travel=0.
        self.phase_start_height=state['ekf_height_m']
        if stage=='FIND':self.previous_target=None;self.filtered_target=None
        if stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT'):
            self.passage_phase='approach';self.passage=None

    def step(self,image,state,t):
        if self.entered is None:self.entered=t;self.phase_start_height=state['ekf_height_m']
        dt=min(.2,max(0.,t-self.last)) if self.last is not None else 0.;self.last=t
        r,yaw=attitude(state);vel=r@state['body_velocity_flu']
        self.position+=vel*dt
        self.motion=self.floor_odometry.step(image,state,vel*dt)
        self.decision=dict(stage_before=self.stage,perception_active=False,checks={})
        self.filtered_velocity+=(1-math.exp(-dt/.35))*(vel-self.filtered_velocity)
        vel=self.filtered_velocity
        age=t-self.entered
        self.travel+=max(0.,float(vel[0]))*dt
        vx=vy=yawrate=0.;z=self.height;self.target=None;self.obstacles=[]
        if self.stage=='TAKEOFF':
            z=min(1.,self.phase_start_height+.22*age)
            if abs(state['ekf_height_m']-1.)<.07 and age>5:self.transition('ENTRY_FORWARD',t,state)
        elif self.stage=='ENTRY_FORWARD':
            self.decision['perception_active']=True
            candidates=rings(image,state)
            self.target=self.ring_track.update(candidates,state,self.position,t)
            self.decision['ring_candidates']=candidates
            self.decision['ring_selection_reason']='입구 전진 중 가까운 링 관측·동일 목표 추적'
            vx=.25
            if self.travel>=1.5:
                self.transition('SCAN_BRAKE' if self.entry_scan else 'FIND',t,state)
        elif self.stage=='SCAN_BRAKE':
            if age>1.5:
                self.scan=RingScan(float(yaw));self.ring_track=RingTrack()
                self.transition('SCAN',t,state)
        elif self.stage=='SCAN':
            self.decision['perception_active']=True
            candidates=rings(image,state)
            self.decision['ring_candidates']=candidates
            yawrate,self.scan_result=self.scan.step(candidates,state,self.position,t)
            self.decision['ring_selection_reason']='제자리 좌45° → 우45° → 정면 · 관측 링 거리 비교'
            if self.scan.done:
                closest=self.scan.closest(self.position)
                if closest is not None:
                    self.ring_track.landmark=closest['point'].copy()
                    self.ring_track.last_seen=closest['last_seen']
                    self.scan_result['selected_id']=closest['id']
                    self.scan_result['selected_distance_m']=float(np.linalg.norm(closest['point']-self.position))
                self.transition('FIND',t,state)
        elif self.stage in ('FIND','ALIGN'):
            self.decision['perception_active']=True
            candidates=rings(image,state)
            target=self.ring_track.update(candidates,state,self.position,t)
            self.decision['ring_candidates']=candidates
            self.decision['ring_selection_reason']=('같은 링의 현재 영상 관측' if target and target['observation_source']=='current_image'
                else '관측했던 링 위치를 기체 이동량으로 추정 · 다시 보이도록 정렬' if target
                else '링 현재 관측 없음')
            self.target=target
            if target:
                self.previous_target=target
                c,s=math.cos(yaw),math.sin(yaw)
                guidance=np.array([c*target['depth']-s*target['left'],
                                   s*target['depth']+c*target['left'],target['up']])
                if self.filtered_target is None:self.filtered_target=guidance.copy()
                self.filtered_target+=(1-math.exp(-dt/.18))*(guidance-self.filtered_target)
                depth,left,up=self.filtered_target
                if self.stage=='FIND':self.transition('ALIGN',t,state)
                self.height=state['ekf_height_m']+clip(.5*up,.25)
                z=self.height
                vx=clip(.4*(depth-1.4)-.8*vel[0],.25)
                vy=clip(.4*left-.8*vel[1],.30)
                self.decision['checks']=dict(full=bool(target['full']), lateral=bool(abs(left)<.09),
                    vertical=bool(abs(up)<.09),depth=bool(abs(depth-1.4)<.15),
                    velocity=bool(np.linalg.norm(vel)<.15))
                self.decision['filtered_target']=list(map(float,(depth,left,up)))
                good=all(self.decision['checks'].values())
                self.stable=self.stable+dt if good else 0.
                if self.stable>.3:
                    self.distance=depth+.03+self.clearance_margin_m
                    self.committed=dict(kind='ring',depth=float(depth),distance=self.distance)
                    self.height=state['ekf_height_m']
                    self.mask=self.ring_mask
                    self.heading=float(yaw)
                    self.transition('STRAIGHT',t,state)
            elif self.stage=='FIND':
                # No stored initial ring side or timed lateral route.
                vy=0.
            else:
                self.transition('FIND',t,state)
        elif self.stage=='STRAIGHT':
            vx=.7
            # Translation is fixed at entry; camera no longer changes the target.
            if self.travel>=self.distance:
                self.mask=0
                self.index+=1
                self.transition('BRAKE',t,state)
        elif self.stage=='BRAKE':
            if age>1.5:
                self.heading=0.
                self.ring_track=RingTrack()
                self.transition('FIND' if self.index<3 else 'POST_RING_DESCENT',t,state)
        elif self.stage=='POST_RING_DESCENT':
            self.height=max(1.,self.phase_start_height-.2*age);z=self.height
            if abs(state['ekf_height_m']-1.)<.07 and age>3:self.transition('BLOCK_GAP',t,state)
        elif self.stage=='EXIT_SCAN_BRAKE':
            if age>1.5:
                self.exit_scanner=RingScan(float(yaw));self.transition('EXIT_SCAN',t,state)
        elif self.stage=='EXIT_SCAN':
            self.decision['perception_active']=True
            self.passage=self.exit_tracker.update(image,state,self.position,t)
            yawrate,self.exit_scan_result=self.exit_scanner.step([],state,self.position,t)
            self.exit_scan_result.update(self.exit_tracker.evidence(self.position,t))
            if self.exit_scanner.done:self.transition('EXIT',t,state)
        elif self.stage=='WALL_SCAN_BRAKE':
            self.passage=None
            if age>1.5:
                self.wall_scanner=WallScan(float(yaw));self.transition('WALL_SCAN',t,state)
        elif self.stage=='WALL_SCAN':
            self.passage=None
            self.decision['perception_active']=True
            yawrate=self.wall_scanner.step(image,state,self.position,t)
            self.wall_scan_result=self.wall_scanner.evidence(self.position,t)
            if self.wall_scanner.turn.done:self.transition(self.pending_wall_stage,t,state)
        elif self.stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT'):
            self.decision['perception_active']=True
            observation=(blocks(image,state) if self.stage=='BLOCK_GAP' else
                         (self.exit_tracker.update(image,state,self.position,t) if self.exit_tracker else exit_gate(image,state))
                         if self.stage=='EXIT' else wall(image,state))
            if self.wall_scan_enabled and self.stage in ('WALL_1','WALL_2'):
                self.wall_scanner.observe(image,state,self.position,t)
                observation=self.wall_scanner.guidance(self.position,t)
                self.wall_scan_result=self.wall_scanner.evidence(self.position,t)
            self.passage=observation
            self.obstacles=observation['faces'] if observation else []
            if observation:
                depth,offset=observation['depth'],observation['left']
                is_wall=self.stage in ('WALL_1','WALL_2')
                if is_wall and self.passage_phase=='approach':
                    # Finish the forward approach before shifting sideways.
                    vx=clip(.5*(depth-2.8)-.8*vel[0],.35)
                    if abs(depth-2.8)<.15 and abs(vel[0])<.12:
                        self.passage_phase='align';vx=0.
                else:
                    self.passage_phase='align'
                    vy=clip(.55*offset-.8*vel[1],.4)
                    self.decision['checks']=dict(lateral=bool(abs(offset)<.09),
                        sideways_stopped=bool(abs(vel[1])<.1),forward_stopped=bool(abs(vel[0])<.12))
                    good=all(self.decision['checks'].values())
                    self.stable=self.stable+dt if good else 0.
                    if self.stable>.35:
                        self.distance=observation['rear_depth']+.03+self.clearance_margin_m
                        self.committed=dict(observation,distance=self.distance)
                        next_stage=('BLOCK_STRAIGHT' if self.stage=='BLOCK_GAP' else
                                    'EXIT_STRAIGHT' if self.stage=='EXIT' else 'WALL_STRAIGHT')
                        if is_wall:self.wall_side=observation['side']
                        self.transition(next_stage,t,state)
            else:
                self.stable=0.
        elif self.stage in ('BLOCK_STRAIGHT','WALL_STRAIGHT','EXIT_STRAIGHT'):
            vx=.4
            if self.travel>=self.distance:
                if self.stage=='WALL_STRAIGHT':self.wall_index+=1
                next_stage=('WALL_1' if self.stage=='BLOCK_STRAIGHT' else
                            ('WALL_2' if self.wall_index==1 else 'EXIT') if self.stage=='WALL_STRAIGHT' else 'LAND')
                if next_stage=='EXIT' and self.exit_tracker:next_stage='EXIT_SCAN_BRAKE'
                vx=0.
                self.transition(next_stage,t,state)
        elif self.stage=='LAND':
            self.height=max(.03,self.phase_start_height-.18*age);z=self.height
            if age>5 and state['ekf_height_m']<.06:self.transition('DONE',t,state)
        elif self.stage=='DONE':return dict(mode='stop',vx=0.,vy=0.,yawrate=0.,height_m=0.)
        self.decision.update(stable_s=float(self.stable),filtered_velocity=self.filtered_velocity.tolist(),
            floor_motion=self.motion,minimal_priors=True,
            ring_tracking=self.ring_track.evidence(t),
            ring_scan=self.scan_result,
            exit_scan=self.exit_scan_result,exit_width_prior_m=self.exit_tracker.width if self.exit_tracker else None,
            wall_scan=self.wall_scan_result,clearance_margin_m=self.clearance_margin_m,
            selected_side=self.wall_side,wall_index=self.wall_index,committed=self.committed)
        yaw_error=math.atan2(math.sin(self.heading-yaw),math.cos(self.heading-yaw))
        # Only the explicitly requested scan sends a nonzero yaw rate.
        # Velocities above refer to local heading zero. Convert to current body yaw.
        c,s=math.cos(yaw),math.sin(yaw)
        return dict(mode='hover',vx=float(c*vx+s*vy),vy=float(-s*vx+c*vy),
                    yawrate=yawrate,height_m=float(z),**({'ring_pass_mask':self.mask} if self.ring_mask else {}))
