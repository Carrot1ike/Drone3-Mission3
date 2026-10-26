"""Offline GT scoring. No controller imports, control commands or runtime sockets."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

from eval_common import local_path, write_json


def norm(values):
    return math.sqrt(sum(float(value)**2 for value in values))


def score(rows, metadata, *, evidence='unknown', clearance_m=.12, settle_s=1., max_gap_s=.1):
    truth = [row for row in rows if row.get('event') == 'ground_truth']
    captures = [row for row in rows if row.get('event') == 'camera_capture']
    if not truth:
        return dict(verdict='BLOCKED', reason='no ground truth samples', powered_success=False)
    times = [row['sim_time_s'] for row in truth]
    host_times = [row['host_monotonic'] for row in truth]
    if any(b <= a for a,b in zip(times,times[1:])) or any(b <= a for a,b in zip(host_times,host_times[1:])):
        return dict(verdict='FAIL', reason='nonmonotonic clocks or mixed source sessions', powered_success=False)
    sessions = {row.get('session') for row in truth}
    if len(sessions) != 1:
        return dict(verdict='FAIL', reason='mixed source sessions', powered_success=False)
    elapsed_sim, elapsed_wall = times[-1]-times[0], host_times[-1]-host_times[0]
    gaps = [b-a for a,b in zip(times,times[1:])]
    contact_rows = [row for row in truth if row.get('contacts')]
    from eval_assessment import contact_classes
    classified_contacts=contact_classes(truth)
    max_height = max(row['drone']['position_world_m'][2] for row in truth)
    motor_rows = [row for row in truth if max(row.get('motor_rpm', [0])) > 1000]
    powered_motion = bool(evidence == 'native_instrumented_source' and motor_rows and
                          any(row['drone']['position_world_m'][2] > .3 for row in motor_rows))
    crossings, rejected = [], []
    gates = metadata.get('rings', [])
    for previous, current in zip(truth, truth[1:]):
        if previous.get('control_interval') != current.get('control_interval'):
            continue
        if current['sim_time_s'] - previous['sim_time_s'] > max_gap_s:
            continue
        p = previous['drone']['position_world_m']
        q = current['drone']['position_world_m']
        for gate in gates:
            center, normal = gate['center_world_m'], gate['normal_world']
            a = sum((x-c)*n for x,c,n in zip(p,center,normal))
            b = sum((x-c)*n for x,c,n in zip(q,center,normal))
            direction = gate.get('expected_direction', -1)
            if not (a > 0 >= b if direction == -1 else a < 0 <= b):
                continue
            fraction = a / (a-b)
            intersection = [x + fraction*(y-x) for x,y in zip(p,q)]
            difference = [x-c for x,c in zip(intersection,center)]
            along = sum(x*n for x,n in zip(difference,normal))
            radial = norm([x-along*n for x,n in zip(difference,normal)])
            touching = any(gate['name']+'_' in geom for row in (previous,current)
                           for contact in row.get('contacts',[]) for geom in contact.get('geoms',[]))
            event = dict(name=gate['name'], sim_time_s=previous['sim_time_s']+
                         fraction*(current['sim_time_s']-previous['sim_time_s']),
                         radial_error_m=radial, clearance_m=gate['aperture_radius_m']-radial,
                         contact_in_bracketing_samples=touching)
            if radial + clearance_m <= gate['aperture_radius_m'] and not touching:
                crossings.append(event)
            else:
                rejected.append(event)
    # Order and duplicates matter: only the next expected ring advances the course.
    next_gate = 0
    for event in crossings:
        if next_gate < len(gates) and event['name'] == gates[next_gate]['name']:
            next_gate += 1
    best_settle = current_settle = 0.
    previous_good = None
    previous_interval = None
    pad = metadata.get('landing_pad')
    min_relative_speed = None
    for row in truth:
        rover = row.get('rover')
        if not pad or not rover:
            continue
        rel = rover.get('drone_position_rover_m')
        dv = [a-b for a,b in zip(row['drone']['linear_velocity_world_mps'], rover['linear_velocity_world_mps'])]
        speed = norm(dv)
        min_relative_speed = speed if min_relative_speed is None else min(min_relative_speed, speed)
        pad_contact = any('landing_pad' in contact.get('geoms',[]) and contact.get('distance_m',1.) <= .002
                          and contact.get('normal_force_n',0.) > 0 for contact in row.get('contacts',[]))
        other_contact = any('landing_pad' not in contact.get('geoms',[]) for contact in row.get('contacts',[]))
        quat = row['drone']['quaternion_wxyz']
        tilt = math.acos(max(-1., min(1., 1-2*(quat[1]**2+quat[2]**2))))
        inside = rel is not None and all(abs(rel[i]-pad['center_rover_m'][i])+clearance_m <= pad['half_size_m'][i] for i in (0,1))
        at_height = rel is not None and pad['top_rover_z_m'] <= rel[2] <= pad['top_rover_z_m']+.10
        good = pad_contact and not other_contact and inside and at_height and speed <= .10 and tilt <= math.radians(15)
        now = row['sim_time_s']
        if good and previous_good is not None and now-previous_good <= max_gap_s and row.get('control_interval') == previous_interval:
            current_settle += now-previous_good
        else:
            current_settle = 0.
        previous_good = now if good else None
        previous_interval = row.get('control_interval')
        best_settle = max(best_settle,current_settle)
    rtf = elapsed_sim/elapsed_wall if elapsed_wall>0 else None
    metrics = dict(verdict='PARTIAL', evidence=evidence, samples=len(truth), sessions=list(sessions),
        sim_elapsed_s=elapsed_sim, host_elapsed_s=elapsed_wall, measured_rtf=rtf,
        max_gt_sample_gap_s=max(gaps,default=0), max_drone_height_m=max_height,
        contact_samples=len(contact_rows), contacts_by_phase=classified_contacts,
        collision_samples=classified_contacts['airborne_obstacle_collision_samples'], camera_captures=len(captures),
        powered_motion_evidence=powered_motion, ring_crossings=crossings,
        rejected_ring_crossings=rejected, sequential_rings_passed=next_gate,
        all_rings_geometry_pass=bool(gates and next_gate == len(gates)),
        pad_settle_sim_s=best_settle, pad_contact_geometry_pass=best_settle>=settle_s,
        min_relative_speed_mps=min_relative_speed,
        # Geometry/physical motion alone cannot establish closed-loop policy operation.
        powered_success=False, closed_loop_policy_assessment='requires independent native policy/log review',
        limitations=['ring metric excludes walls and other obstacles',
                     'landing metric requires force-bearing pad contact, footprint margin, low relative speed and tilt',
                     'source sim time, snapshot tick and capture tick are separate clocks; no fixed scale applied'])
    metrics['position_bounds_world_m']={
        'min':[min(r['drone']['position_world_m'][i] for r in truth) for i in range(3)],
        'max':[max(r['drone']['position_world_m'][i] for r in truth) for i in range(3)]}
    metrics['final_drone']=truth[-1]['drone']
    metrics['final_contacts']=truth[-1].get('contacts',[])
    metrics['max_motor_rpm']=max((max(r.get('motor_rpm',[0])) for r in truth),default=0)
    from eval_entry import score_entry
    metrics['official_entry_section']=score_entry(rows,metadata.get('entry_sections',[]),radius_m=clearance_m)
    return metrics


def clock_diagnostics(rows):
    """Optional normalized policy evidence; never feed this result to a controller.

    Rows: host_monotonic, snapshot_tick_ms, capture_tick_us, session, velocity_world_mps.
    The world velocity must be rotated from associated FLU using that row's attitude.
    """
    result = dict(rows=len(rows), segments=[])
    groups = []
    for row in rows:
        if not groups or row.get('session') != groups[-1][-1].get('session') or any(
            row.get(key,0) <= groups[-1][-1].get(key,0)
            for key in ('host_monotonic','snapshot_tick_ms','capture_tick_us')):
            groups.append([])
        groups[-1].append(row)
    for group in groups:
        if len(group)<2:
            continue
        first,last = group[0],group[-1]
        segment = dict(session=first.get('session'), rows=len(group),
            host_elapsed_s=last['host_monotonic']-first['host_monotonic'],
            snapshot_elapsed_s=(last['snapshot_tick_ms']-first['snapshot_tick_ms'])/1000,
            capture_elapsed_s=(last['capture_tick_us']-first['capture_tick_us'])/1e6)
        for clock, scale in [('host_monotonic',1.),('snapshot_tick_ms',.001),('capture_tick_us',.000001)]:
            if all('velocity_world_mps' in row for row in group):
                integrated = [0.,0.,0.]
                for a,b in zip(group,group[1:]):
                    dt = (b[clock]-a[clock])*scale
                    for i in range(3):
                        integrated[i] += .5*(a['velocity_world_mps'][i]+b['velocity_world_mps'][i])*dt
                segment['integrated_world_displacement_m_using_'+clock] = integrated
        result['segments'].append(segment)
    result['warning']='Capture and source clocks are diagnostic only; do not calibrate policy time from one run.'
    return result


def quaternion_rotate(q_xyzw, vector):
    x,y,z,w = q_xyzw
    q = [x,y,z]
    cross = [y*vector[2]-z*vector[1], z*vector[0]-x*vector[2], x*vector[1]-y*vector[0]]
    second = [q[1]*cross[2]-q[2]*cross[1],q[2]*cross[0]-q[0]*cross[2],q[0]*cross[1]-q[1]*cross[0]]
    return [v+2*w*c+2*s for v,c,s in zip(vector,cross,second)]


def normalize_host_frames(rows):
    normalized=[]
    for row in rows:
        state=row.get('state')
        if not state or not row.get('association_valid') or not state.get('state_valid'):
            continue
        normalized.append(dict(host_monotonic=row['received_monotonic'],
            snapshot_tick_ms=state['snapshot_tick_ms'],capture_tick_us=row['capture_tick_us'],
            session=str((row['session_id'],state['boot_id'],state['reset_generation'])),
            velocity_body=state['velocity_body'],quaternion_xyzw=state['quaternion_xyzw'],
            velocity_world_mps=quaternion_rotate(state['quaternion_xyzw'],state['velocity_body']),
            height_m=state['height_m']))
    return normalized


def compare_host_truth(host_rows, truth_rows, *, max_offset_s=.15):
    """Receipt-time nearest matching, with one fixed evaluation-only yaw alignment."""
    from bisect import bisect_left
    truth=[r for r in truth_rows if r.get('event')=='ground_truth']
    times=[r['host_monotonic'] for r in truth]
    if not truth:return dict(reason='no GT rows')
    pairs=[]
    rejected=0
    for host in host_rows:
        index=bisect_left(times,host['host_monotonic'])
        candidate=min([i for i in (index-1,index) if 0<=i<len(truth)],
                      key=lambda i:abs(times[i]-host['host_monotonic']))
        offset=times[candidate]-host['host_monotonic']
        if abs(offset)>max_offset_s:
            rejected+=1
            continue
        pairs.append((host,truth[candidate],offset))
    if len(pairs)<2:return dict(reason='insufficient receipt-time matches',matches=len(pairs),rejected=rejected)
    errors=[]
    heights=[]
    for host,gt,_ in pairs:
        w,x,y,z=gt['drone']['quaternion_wxyz']
        inverse=[-x,-y,-z,w]
        body=quaternion_rotate(inverse,gt['drone']['linear_velocity_world_mps'])
        errors.append([a-b for a,b in zip(host['velocity_body'],body)])
        heights.append(host['height_m']-gt['drone']['position_world_m'][2])
    result=dict(matches=len(pairs),rejected=rejected,max_nearest_sample_offset_s=max(abs(p[2]) for p in pairs),
        body_velocity_rmse_mps=[math.sqrt(sum(e[i]**2 for e in errors)/len(errors)) for i in range(3)],
        height_rmse_m=math.sqrt(sum(e*e for e in heights)/len(heights)),segments=[],
        alignment='nearest GT host_monotonic to host receipt time; no capture-to-source exact synchronization',
        limitations=['nearest sample offset bounds only GT sampling error; transport/state age remains unknown',
                     'GT world yaw aligns ego displacement once per source identity for evaluation only',
                     'hover displacement has little power to validate translational odometry or clock scale'])
    groups=[]
    for pair in pairs:
        if not groups or pair[0]['session']!=groups[-1][-1][0]['session']:
            groups.append([])
        groups[-1].append(pair)
    for group in groups:
        if len(group)<2:continue
        first,last=group[0],group[-1]
        h0,g0,_=first
        h1,g1,_=last
        qw,qx,qy,qz=g0['drone']['quaternion_wxyz']
        x,y,z,w=h0['quaternion_xyzw']
        yaw_gt=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        yaw_host=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        yaw=yaw_gt-yaw_host
        gt_delta=[b-a for a,b in zip(g0['drone']['position_world_m'],g1['drone']['position_world_m'])]
        segment=dict(identity=h0['session'],rows=len(group),gt_displacement_world_m=gt_delta,
            host_elapsed_s=h1['host_monotonic']-h0['host_monotonic'],
            gt_sim_elapsed_s=g1['sim_time_s']-g0['sim_time_s'],
            snapshot_elapsed_s=(h1['snapshot_tick_ms']-h0['snapshot_tick_ms'])*.001,
            capture_elapsed_s=(h1['capture_tick_us']-h0['capture_tick_us'])*1e-6,
            evaluation_yaw_alignment_rad=yaw)
        segment.update(displacement_integrals(group,yaw))
        peak=max(range(len(group)),key=lambda i:group[i][1]['drone']['position_world_m'][2])
        segment['vertical_peak_index']=peak
        segment['vertical_subintervals']={}
        for phase,part in [('ascent',group[:peak+1]),('descent',group[peak:])]:
            if len(part)>=2:
                segment['vertical_subintervals'][phase]=dict(rows=len(part),
                    gt_displacement_world_m=[b-a for a,b in zip(part[0][1]['drone']['position_world_m'],part[-1][1]['drone']['position_world_m'])],
                    **displacement_integrals(part,yaw))
        result['segments'].append(segment)
    return result


def displacement_integrals(group,yaw):
    result={}
    origin=group[0][1]['drone']['position_world_m']
    gt_delta=[b-a for a,b in zip(origin,group[-1][1]['drone']['position_world_m'])]
    for clock,scale in [('host_monotonic',1.),('snapshot_tick_ms',.001),('capture_tick_us',1e-6),('gt_sim_time_s',1.)]:
        integral=[0.,0.,0.]
        trajectory_errors=[]
        rotated=[0.,0.,0.]
        for a,b in zip(group,group[1:]):
            ha,ga,_=a
            hb,gb,_=b
            dt=(gb['sim_time_s']-ga['sim_time_s']) if clock=='gt_sim_time_s' else (hb[clock]-ha[clock])*scale
            if dt<0:continue
            va,vb=ha['velocity_world_mps'],hb['velocity_world_mps']
            for i in range(3):integral[i]+=.5*(va[i]+vb[i])*dt
            rotated=[math.cos(yaw)*integral[0]-math.sin(yaw)*integral[1],
                     math.sin(yaw)*integral[0]+math.cos(yaw)*integral[1],integral[2]]
            trajectory_errors.append([estimate-(position-initial) for estimate,position,initial in zip(
                                      rotated,gb['drone']['position_world_m'],origin)])
        result[clock]=dict(integrated_displacement_world_m=rotated,
            displacement_error_world_m=[a-b for a,b in zip(rotated,gt_delta)],
            displacement_error_norm_m=norm([a-b for a,b in zip(rotated,gt_delta)]),
            trajectory_rmse_world_m=[math.sqrt(sum(e[i]**2 for e in trajectory_errors)/len(trajectory_errors))
                                     if trajectory_errors else 0. for i in range(3)],
            trajectory_max_abs_error_world_m=[max((abs(e[i]) for e in trajectory_errors),default=0.) for i in range(3)])
    return result


def controlled_intervals(commands, trial):
    """Valid sent-command coverage, capped at source watchdog cadence and host exit."""
    start=trial.get('host_started_monotonic')
    stop=trial.get('host_exit_monotonic')
    requested=trial.get('runtime_stop',{}).get('requested_monotonic')
    if stop is None:stop=requested
    elif requested is not None:stop=min(stop,requested)
    if start is None or stop is None or stop<=start:return []
    commands=sorted(commands,key=lambda row:row['sent_monotonic'])
    intervals=[]
    for index,row in enumerate(commands):
        if row.get('verdict') not in ('active','explicit_stop') or row.get('fault'):
            continue
        timestamp=row['sent_monotonic']
        following=commands[index+1]['sent_monotonic'] if index+1<len(commands) else timestamp+.03
        begin,end=max(start,timestamp),min(stop,following,timestamp+.1)
        if end<=begin:continue
        if intervals and begin<=intervals[-1][1]+1e-9:
            intervals[-1][1]=max(end,intervals[-1][1])
        else:intervals.append([begin,end])
    return intervals


def restrict_control_window(rows, intervals):
    """Do not interpolate a crossing or sustained contact through a control gap."""
    selected=[]
    interval=0
    for row in rows:
        timestamp=row.get('host_monotonic',-math.inf)
        while interval<len(intervals) and timestamp>intervals[interval][1]:interval+=1
        if interval>=len(intervals):break
        if timestamp>=intervals[interval][0]:
            selected.append(dict(row,control_interval=interval))
    return selected


def command_phase_diagnostics(commands, truth_rows, trial):
    """Receipt-aligned physical response per contiguous command reason/verdict."""
    from bisect import bisect_left
    truth=[r for r in truth_rows if r.get('event')=='ground_truth']
    if not truth:return []
    times=[r['host_monotonic'] for r in truth]
    start=trial.get('host_started_monotonic',-math.inf)
    stop=min(trial.get('host_exit_monotonic',math.inf),trial.get('runtime_stop',{}).get('requested_monotonic',math.inf))
    commands=sorted([row for row in commands if start<=row['sent_monotonic']<=stop],key=lambda row:row['sent_monotonic'])
    groups=[]
    for row in commands:
        key=(row.get('verdict'),row.get('command',{}).get('reason',''),str(row.get('source_identity')))
        if not groups or key!=groups[-1][0] or row['sent_monotonic']-groups[-1][1][-1]['sent_monotonic']>.100000001:
            groups.append((key,[]))
        groups[-1][1].append(row)
    phases=[]
    def nearest(stamp):
        i=bisect_left(times,stamp)
        return min([j for j in (i-1,i) if 0<=j<len(truth)],key=lambda j:abs(times[j]-stamp))
    for index,(key,group) in enumerate(groups):
        if key[0] not in ('active','explicit_stop') or any(row.get('fault') for row in group):continue
        begin=group[0]['sent_monotonic']
        end=min(stop,groups[index+1][1][0]['sent_monotonic'] if index+1<len(groups) else group[-1]['sent_monotonic']+.03)
        a,b=nearest(begin),nearest(end)
        if b<=a:continue
        if max(abs(times[a]-begin),abs(times[b]-end))>.15:continue
        first,last=truth[a],truth[b]
        delta=[v-u for u,v in zip(first['drone']['position_world_m'],last['drone']['position_world_m'])]
        qw,qx,qy,qz=first['drone']['quaternion_wxyz']
        inverse=[-qx,-qy,-qz,qw]
        relative=quaternion_rotate(inverse,delta)
        def yaw(row):
            w,x,y,z=row['drone']['quaternion_wxyz']
            return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        dyaw=math.atan2(math.sin(yaw(last)-yaw(first)),math.cos(yaw(last)-yaw(first)))
        body_velocities=[]
        for row in truth[a:b+1]:
            w,x,y,z=row['drone']['quaternion_wxyz']
            body_velocities.append(quaternion_rotate([-x,-y,-z,w],row['drone']['linear_velocity_world_mps']))
        phases.append(dict(reason=key[1],verdict=key[0],command_rows=len(group),
            start_host_monotonic=begin,end_host_monotonic=end,host_elapsed_s=end-begin,
            gt_sim_elapsed_s=last['sim_time_s']-first['sim_time_s'],
            command_at_start=group[0]['command'],displacement_world_m=delta,
            displacement_initial_body_m=relative,yaw_change_deg=math.degrees(dyaw),
            mean_gt_velocity_body_mps=[sum(row[i] for row in body_velocities)/len(body_velocities) for i in range(3)],
            max_nearest_boundary_offset_s=max(abs(times[a]-begin),abs(times[b]-end)),
            limitation='receipt-aligned; physical response delay/transport uncertainty not compensated'))
    return phases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gt',type=Path,help='defaults to trial runtime_log_dir/evaluation/ground_truth.jsonl')
    parser.add_argument('--metadata',type=Path)
    parser.add_argument('--entry-scene',type=Path,help='legacy metadata fallback: actual generated scene.xml, evaluation only')
    parser.add_argument('--out',type=Path)
    parser.add_argument('--clock-log',type=Path)
    parser.add_argument('--host-frames',type=Path,help='core host frames.jsonl; offline clock/ego comparison')
    parser.add_argument('--trial',type=Path,help='eval_trial trial.json; requires valid sent command coverage')
    parser.add_argument('--assessment-scope',choices=['transport','diagnostic','hover','motion','tracking-only','approach','entry-section','passage','mission2','landing','mission3'],help='explicit parent review target; raw trial outcome remains preserved')
    parser.add_argument('--commands',type=Path,help='core commands.jsonl; defaults beside trial host frames')
    parser.add_argument('--clearance-m',type=float,default=.12)
    parser.add_argument('--settle-s',type=float,default=1.)
    args=parser.parse_args()
    if not 0<=args.clearance_m<=.5 or not .1<=args.settle_s<=60:
        parser.error('clearance-m must be 0..0.5, settle-s 0.1..60')
    if args.trial and args.host_frames is None and (args.trial.parent/'host/frames.jsonl').exists():
        args.host_frames=args.trial.parent/'host/frames.jsonl'
    if args.gt is None:
        if not args.trial:
            parser.error('--gt or --trial is required')
        trial_for_path=json.loads(args.trial.read_text())
        log_dir=trial_for_path.get('runtime_log_dir')
        if not log_dir:
            result=dict(verdict='FAIL',reason='trial has no ready runtime log directory',
                trial_status=trial_for_path.get('status'),trial_error=trial_for_path.get('error'),powered_success=False)
            write_json(args.out or args.trial.parent/'evaluation_score.json',result)
            print(json.dumps(result,indent=2))
            return
        args.gt=Path(log_dir)/'evaluation/ground_truth.jsonl'
    metadata=json.loads((args.metadata or args.gt.with_name('model_metadata.json')).read_text())
    if args.entry_scene:
        from eval_entry import metadata_from_scene
        import hashlib
        metadata['entry_sections']=metadata_from_scene(args.entry_scene)
        metadata['entry_scene_evidence']=dict(path=str(args.entry_scene.resolve()),sha256=hashlib.sha256(args.entry_scene.read_bytes()).hexdigest())
    rows=[json.loads(line) for line in args.gt.read_text().splitlines() if line.strip()]
    proof=args.gt.with_name('provenance.json')
    native=proof.exists() and json.loads(proof.read_text()).get('evidence')=='native_instrumented_source'
    all_rows=rows
    control=None
    if args.trial:
        trial=json.loads(args.trial.read_text())
        command_path=args.commands or args.trial.parent/'host/commands.jsonl'
        command_rows=[json.loads(line) for line in command_path.read_text().splitlines() if line.strip()] if command_path.exists() else []
        intervals=controlled_intervals(command_rows,trial)
        rows=restrict_control_window(sorted(rows,key=lambda row:row.get('host_monotonic',0)),intervals)
        control=dict(trial=str(args.trial.resolve()),status=trial.get('status'),
            host_returncode=trial.get('host_returncode'),host_summary=trial.get('host_summary'),
            intervals_host_monotonic=intervals,command_rows=len(command_rows),commands_missing=not command_path.exists(),
            excluded_gt_samples=sum(row.get('event')=='ground_truth' for row in all_rows)-sum(row.get('event')=='ground_truth' for row in rows),
            policy_success=False,assessment='valid sent-command interval only; shutdown/decay contacts excluded')
    result=score(rows,metadata,evidence='native_instrumented_source' if native else 'unverified_or_synthetic',
                 clearance_m=args.clearance_m, settle_s=args.settle_s)
    if control is not None:
        result['command_phases']=command_phase_diagnostics(command_rows,all_rows,trial)
        result['control_window']=control
        from eval_assessment import scoped_assessment
        decisions_path=args.trial.parent/'host/decisions.jsonl'
        decisions=[json.loads(line) for line in decisions_path.read_text().splitlines() if line.strip()] if decisions_path.exists() else []
        result['assessment_scope']=scoped_assessment(trial,decisions,result,[row for row in rows if row.get('event')=='ground_truth'],args.assessment_scope)
        if control['status']=='failed' or control['host_returncode']!=0:
            result['verdict']='PARTIAL' if result['assessment_scope']['expected_bounded_scope_end'] else 'FAIL'
            result['trial_failure_preserved']=True
    else:
        result['control_window']={'assessment':'unrestricted diagnostic; no controlled-landing conclusion allowed'}
    if metadata.get('entry_scene_evidence'):
        result['official_entry_section']['legacy_scene_evidence']=metadata['entry_scene_evidence']
    result['inputs']=dict(ground_truth=str(args.gt.resolve()), metadata=str((args.metadata or args.gt.with_name('model_metadata.json')).resolve()))
    if args.clock_log:
        result['native_clocks']=clock_diagnostics([json.loads(line) for line in args.clock_log.read_text().splitlines() if line.strip()])
    if args.host_frames:
        host=normalize_host_frames([json.loads(line) for line in args.host_frames.read_text().splitlines() if line.strip()])
        result['native_clocks']=clock_diagnostics(host)
        result['ego_vs_truth']=compare_host_truth(host,all_rows)
        if args.trial:
            from eval_motion import motion_assessment
            result['airborne_motion']=motion_assessment(host,all_rows,result.get('command_phases',[]),trial)
        if host:
            frame_rows=[row for row in all_rows if host[0]['host_monotonic']<=row.get('host_monotonic',-1)<=host[-1]['host_monotonic']]
            result['host_receipt_window_diagnostic']=score(frame_rows,metadata,evidence='native_instrumented_source' if native else 'unverified_or_synthetic',
                clearance_m=args.clearance_m,settle_s=args.settle_s)
            result['host_receipt_window_diagnostic']['limitation']='frame receipt window only; command validity/latency not established here'
    write_json(args.out or (args.trial.parent/'evaluation_score.json' if args.trial else args.gt.with_name('score.json')),result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
