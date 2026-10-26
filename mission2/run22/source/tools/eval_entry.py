"""Official gate12 finite-section GT metric, separate from Mission2 completion."""
from __future__ import annotations
import math
import xml.etree.ElementTree as ET


def section_from_boxes(boxes):
    required=('gate12_post0','gate12_post1','gate12_header')
    if not all(name in boxes for name in required):return None
    left,right=sorted((boxes[name] for name in required[:2]),key=lambda box:box['position'][1])
    header=boxes[required[2]]
    x=sum(boxes[name]['position'][0] for name in required)/3
    if any(abs(boxes[name]['position'][0]-x)>.001 for name in required):raise ValueError('entry boxes not coplanar')
    return dict(name='official_entry_gate12',evaluation_only=True,normal_world=[1,0,0],
        plane_x_world_m=x,expected_direction=-1,
        gap_y_world_m=[left['position'][1]+left['size'][1],right['position'][1]-right['size'][1]],
        gap_z_world_m=[0.,header['position'][2]-header['size'][2]],
        slab_half_depth_m=max(boxes[name]['size'][0] for name in required),
        geometry_source='actual gate12 post inner faces and header lower face; arena floor z=0',
        mission2_success=False)


def metadata_from_model(model,data):
    boxes={}
    for name in ('gate12_post0','gate12_post1','gate12_header'):
        try:geom=model.geom(name)
        except KeyError:return []
        rotation=data.geom_xmat[geom.id].reshape(3,3)
        if any(abs(float(rotation[i,j])-(1. if i==j else 0.))>1e-6 for i in range(3) for j in range(3)):
            raise ValueError('rotated official entry geometry needs explicit evaluator update')
        boxes[name]=dict(position=data.geom_xpos[geom.id].tolist(),size=geom.size.tolist())
    section=section_from_boxes(boxes)
    return [section] if section else []


def metadata_from_scene(path):
    root=ET.parse(path).getroot();boxes={}
    for geom in root.find('worldbody').findall('geom'):
        name=geom.get('name')
        if name not in ('gate12_post0','gate12_post1','gate12_header'):continue
        if any(key in geom.attrib for key in ('quat','euler','axisangle','xyaxes','zaxis')):
            raise ValueError('rotated scene entry boxes unsupported')
        boxes[name]=dict(position=[float(x) for x in geom.get('pos').split()],size=[float(x) for x in geom.get('size').split()])
    section=section_from_boxes(boxes)
    return [section] if section else []


def measured_stop(truth,begin,far_x,*,max_gap_sim_s=.1,speed_mps=.03,rate_deg_s=2.,dwell_sim_s=.30):
    """Require a continuous airborne stop after rear clearance in the same control interval."""
    origin=truth[begin];previous=None
    starts={'translation_only':None,'translation_and_attitude':None}
    best={name:0. for name in starts};observed={name:None for name in starts}
    last_violation={name:None for name in starts}
    finish=origin;boundary=None
    for r in truth[begin:]:
        if r.get('session')!=origin.get('session') or r.get('control_interval')!=origin.get('control_interval'):
            boundary='session/control interval ended';break
        now=r['sim_time_s'];p=r['drone']['position_world_m']
        gap=now-previous['sim_time_s'] if previous is not None else 0.
        hostgap=r['host_monotonic']-previous['host_monotonic'] if previous is not None else None
        if previous is not None and (not 0<gap<=max_gap_sim_s+1e-9 or hostgap<=0):
            boundary='clock/sample gap';break
        if p[0]>=far_x or p[2]<=.30 or r.get('contacts'):
            boundary='left far-side airborne contact-free domain';break
        velocity=r['drone'].get('linear_velocity_world_mps')
        rates=r['drone'].get('angular_velocity_world_radps')
        speed=math.sqrt(sum(v*v for v in velocity)) if velocity is not None and all(math.isfinite(v) for v in velocity) else math.inf
        rate=math.degrees(math.sqrt(sum(v*v for v in rates))) if rates is not None and all(math.isfinite(v) for v in rates) else math.inf
        for name in starts:
            reasons=[]
            if speed>speed_mps:reasons.append('speed missing/nonfinite/above bound')
            if name=='translation_and_attitude' and rate>rate_deg_s:reasons.append('angular rate missing/nonfinite/above bound')
            if reasons:
                starts[name]=None;last_violation[name]=dict(sim_time_s=now,reasons=reasons)
            else:
                if starts[name] is None:starts[name]=now
                best[name]=max(best[name],now-starts[name])
                if best[name]>=dwell_sim_s-1e-9 and observed[name] is None:
                    observed[name]=dict(sim_time_s=now,host_monotonic=r['host_monotonic'],
                        position_world_m=p,speed_mps=speed,angular_rate_norm_deg_s=rate if math.isfinite(rate) else None,
                        displacement_since_rear_clearance_world_m=[b-a for a,b in zip(origin['drone']['position_world_m'],p)])
        finish=r;previous=r
    return dict(criteria=dict(speed_norm_mps=speed_mps,angular_rate_norm_deg_s=rate_deg_s,
                    continuous_dwell_sim_s=dwell_sim_s,max_sample_gap_sim_s=max_gap_sim_s),
        translation_only=dict(observed=observed['translation_only'] is not None,evidence=observed['translation_only'],
            longest_dwell_sim_s=best['translation_only'],last_violation=last_violation['translation_only']),
        translation_and_attitude=dict(observed=observed['translation_and_attitude'] is not None,evidence=observed['translation_and_attitude'],
            longest_dwell_sim_s=best['translation_and_attitude'],last_violation=last_violation['translation_and_attitude']),
        observed=observed['translation_and_attitude'] is not None,censored=observed['translation_and_attitude'] is None,
        evidence_boundary=boundary or 'end of controlled evidence',
        last_observed_sim_time_s=finish['sim_time_s'],
        observed_displacement_world_m=[b-a for a,b in zip(origin['drone']['position_world_m'],finish['drone']['position_world_m'])])


def score_entry(rows,sections,*,radius_m=.12,rear_margin_m=.08,max_gap_sim_s=.1):
    truth=[row for row in rows if row.get('event')=='ground_truth']
    result=dict(scope='one official finite entry section only',full_mission2_success=False,powered_success=False,
        parent_verdict='NOT_ASSIGNED',entry_section_geometry_pass=False,local_entry_section_complete=False,
        gates_crossed=[],gates_rear_cleared=[],gates_completed_with_measured_stop=[],sections=[],
        footprint=dict(conservative_spherical_radius_m=radius_m,rear_clearance_margin_m=rear_margin_m),
        limitations=['GT metric never supplies plane/gap coordinates to policy',
            'piecewise linear swept corridor assumes motion between recorded samples; gaps >0.1 simulation seconds invalidate evidence',
            'contact-free complete rear clearance is required; outside-post bypass and bare plane crossing do not pass',
            'powered closed-loop section qualification still requires independent native policy/command review'])
    obstacles=[dict(sim_time_s=row['sim_time_s'],contacts=row['contacts']) for row in truth
        if any(not any(name in contact.get('geoms',[]) for name in ('floor','arena_floor','ground')) for contact in row.get('contacts',[]))]
    result['airborne_obstacle_contact_samples']=len(obstacles)
    result['airborne_obstacle_contact_evidence']=obstacles
    for section in sections:
        x=section['plane_x_world_m'];extent=section['slab_half_depth_m']+radius_m
        lo,hi=section['gap_y_world_m'];bottom,top=section['gap_z_world_m']
        start=None;crossing=None;violations=[];completed=[];margins=[]
        def inside(p):return lo+radius_m<=p[1]<=hi-radius_m and bottom+radius_m<=p[2]<=top-radius_m
        for i,row in enumerate(truth):
            p=row['drone']['position_world_m']
            if p[0]>x+extent:
                start=i;crossing=None;violations=[];margins=[]
            if start is None:continue
            if i>start:
                previous=truth[i-1];q=previous['drone']['position_world_m']
                dt=row['sim_time_s']-previous['sim_time_s']
                if not 0<dt<=max_gap_sim_s+1e-9 or row['host_monotonic']<=previous['host_monotonic'] or previous.get('control_interval')!=row.get('control_interval') or previous.get('session')!=row.get('session'):
                    violations.append('clock/session/control gap')
                # Check both clipped endpoints of the segment inside the inflated slab.
                fractions=[]
                if abs(p[0]-q[0])<1e-12:
                    if abs(p[0]-x)<=extent:fractions=[0.,1.]
                else:
                    a,b=sorted(((x-extent-q[0])/(p[0]-q[0]),(x+extent-q[0])/(p[0]-q[0])))
                    if max(0.,a)<=min(1.,b):fractions=[max(0.,a),min(1.,b)]
                for fraction in fractions:
                    point=[v+fraction*(w-v) for v,w in zip(q,p)]
                    margins.append(min(point[1]-lo,hi-point[1],point[2]-bottom,top-point[2])-radius_m)
                    if not inside(point):violations.append('finite gap swept footprint exceeded')
                if q[0]>x>=p[0]:
                    fraction=(q[0]-x)/(q[0]-p[0])
                    crossing=dict(sim_time_s=previous['sim_time_s']+fraction*dt,
                        position_world_m=[v+fraction*(w-v) for v,w in zip(q,p)],expected_direction=True)
                elif q[0]<x<=p[0]:violations.append('reverse crossing')
            for contact in row.get('contacts',[]):
                violations.append('physical contact during section transit')
            if p[0]<x-extent-rear_margin_m:
                event=dict(start_sim_time_s=truth[start]['sim_time_s'],rear_clear_sim_time_s=row['sim_time_s'],
                    plane_crossing=crossing,violations=sorted(set(violations)),
                    geometry_pass=bool(crossing and not violations),samples=i-start+1)
                event['min_swept_whole_vehicle_clearance_m']=min(margins) if margins else None
                event['rear_clearance_beyond_slab_m']=x-section['slab_half_depth_m']-(p[0]+radius_m)
                event['measured_stop']=measured_stop(truth,i,x-extent-rear_margin_m,max_gap_sim_s=max_gap_sim_s)
                event['local_section_complete']=event['geometry_pass'] and event['measured_stop']['observed']
                completed.append(event);start=None;crossing=None;violations=[]
        item=dict(metadata=section,completed_transits=completed,geometry_pass=any(e['geometry_pass'] for e in completed),
            local_section_complete=any(e['local_section_complete'] for e in completed),
            unfinished_transit=bool(start is not None),plane_crossing_without_rear_clearance=crossing)
        result['sections'].append(item)
        if crossing or any(e['plane_crossing'] for e in completed):result['gates_crossed'].append(section['name'])
        if item['geometry_pass']:result['gates_rear_cleared'].append(section['name'])
        if item['local_section_complete']:result['gates_completed_with_measured_stop'].append(section['name'])
    result['entry_section_geometry_pass']=bool(sections and not obstacles and all(s['geometry_pass'] for s in result['sections']))
    result['contact_free_controlled_flight']=not obstacles
    result['local_entry_section_complete']=bool(result['entry_section_geometry_pass'] and all(s['local_section_complete'] for s in result['sections']))
    return result
