"""Offline full Mission2 course scoring; never imported by or sent to policy.

Example: python tools/eval_mission2_course.py --trial /path/to/trial.json
The generated XML supplies evaluation geometry. No runtime or sockets are opened.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from eval_common import write_json
from eval_entry import section_from_boxes
from eval_score import controlled_intervals, restrict_control_window

REQUIRED_SECTIONS = ('entry_gate12', 'ring1', 'ring2', 'ring3', 'two_block_passage',
                     's_wall1', 's_wall2', 'exit_gate23')
SCORING_VERSION = 'mission2-course-v2-post-exit-ground-rest'


def course_from_scene(path, metadata, *, infer_wall_opening=False):
    """Require the intact, axis-aligned official scene rather than inventing gates."""
    root = ET.parse(path).getroot()
    if root.get('model') != 'mission_arena_mission2':
        raise ValueError('full course requires intact mission_arena_mission2')
    geoms = {g.get('name'): g for g in root.findall('./worldbody/geom')}

    def box(name):
        g = geoms[name]
        if g.get('type') != 'box' or any(k in g.attrib for k in ('quat', 'euler', 'axisangle', 'xyaxes', 'zaxis')):
            raise ValueError(f'unsupported evaluation box: {name}')
        values = {k: [float(v) for v in g.get(attr, '').split()]
                  for k, attr in (('position', 'pos'), ('size', 'size'))}
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in values.values()):
            raise ValueError(f'invalid evaluation box: {name}')
        if min(values['size']) <= 0:
            raise ValueError(f'invalid box size: {name}')
        return values

    def rectangle(name, x, half, y, z, source):
        if not (y[0] < y[1] and z[0] < z[1]):
            raise ValueError(f'empty finite passage: {name}')
        return dict(name=name, kind='rectangle', plane_x_world_m=x,
                    slab_half_depth_m=half, gap_y_world_m=y, gap_z_world_m=z,
                    expected_direction=-1, geometry_source=source)

    def gate(prefix, name):
        boxes = {'gate12_' + suffix: box(prefix + '_' + suffix)
                 for suffix in ('post0', 'post1', 'header')}
        section = section_from_boxes(boxes)
        return rectangle(name, section['plane_x_world_m'], section['slab_half_depth_m'],
                         section['gap_y_world_m'], section['gap_z_world_m'],
                         prefix + ' actual inner post faces and lower header face')

    result = [gate('gate12', 'entry_gate12')]
    rings = {g['name']: g for g in metadata.get('rings', [])}
    if set(rings) != {'ring1', 'ring2', 'ring3'}:
        raise ValueError('exactly three native ring metadata records required')
    for name in ('ring1', 'ring2', 'ring3'):
        capsules = [g for n, g in geoms.items() if n and n.startswith(name + '_')]
        if len(capsules) != 32:
            raise ValueError(f'intact 32-capsule {name} required')
        endpoints, tubes = [], []
        for g in capsules:
            if g.get('type') != 'capsule' or any(k in g.attrib for k in ('quat', 'euler', 'axisangle', 'xyaxes', 'zaxis')):
                raise ValueError('unsupported ring geometry')
            p = [float(v) for v in g.get('fromto', '').split()]
            tube = float(g.get('size', 'nan'))
            if len(p) != 6 or not all(math.isfinite(v) for v in p) or not 0 < tube < 1:
                raise ValueError('invalid ring capsule')
            endpoints.append((p[:3], p[3:])); tubes.append(tube)
        center = [sum(a[i] + b[i] for a, b in endpoints) / (2 * len(endpoints)) for i in range(3)]
        if any(abs(p[0] - center[0]) > 1e-6 for pair in endpoints for p in pair):
            raise ValueError('ring is not in a world-x plane')
        # Distance to capsule line segments, not endpoint radius: polygon chords
        # reduce the usable inscribed aperture below the nominal circle radius.
        aperture = math.inf
        for (a, b), tube in zip(endpoints, tubes):
            v = [b[i] - a[i] for i in (1, 2)]
            den = sum(x * x for x in v)
            f = min(1., max(0., sum((center[i] - a[i]) * v[i-1] for i in (1, 2)) / den)) if den else 0.
            aperture = min(aperture, math.hypot(*(a[i] + f * v[i-1] - center[i] for i in (1, 2))) - tube)
        native = rings[name]
        if (any(abs(a-b) > 1e-5 for a, b in zip(native['center_world_m'], center))
                or native.get('normal_world') != [1., 0., 0.] or native.get('expected_direction', -1) != -1
                or abs(native['aperture_radius_m'] - aperture) > 1e-5):
            raise ValueError(f'{name} XML/model metadata disagree')
        result.append(dict(name=name, kind='ring', plane_x_world_m=center[0], center_world_m=center,
                           aperture_radius_m=aperture, slab_half_depth_m=max(tubes), expected_direction=-1,
                           geometry_source='inscribed aperture of actual capsule segments'))
    a, b = sorted((box('block1'), box('block2')), key=lambda v: v['position'][1])
    if abs(a['position'][0] - b['position'][0]) > 1e-6:
        raise ValueError('two blocks not coplanar')
    result.append(rectangle('two_block_passage', a['position'][0], max(a['size'][0], b['size'][0]),
                            [a['position'][1]+a['size'][1], b['position'][1]-b['size'][1]],
                            [0., min(v['position'][2]+v['size'][2] for v in (a, b))],
                            'between inner block faces; below block tops (overflight excluded)'))
    north, south = box('fence_1_north_collision'), box('fence_1_south_collision')
    if infer_wall_opening:
        south,north=sorted((north,south),key=lambda box:box['position'][1])
    for name, side in (('s_wall1', 'north'), ('s_wall2', 'south')):
        wall = box(name); p, s = wall['position'], wall['size']
        if infer_wall_opening:
            north_gap=north['position'][1]-north['size'][1]-(p[1]+s[1])
            south_gap=p[1]-s[1]-(south['position'][1]+south['size'][1])
            side='north' if north_gap>south_gap else 'south'
        y = ([p[1]+s[1], north['position'][1]-north['size'][1]] if side == 'north'
             else [south['position'][1]+south['size'][1], p[1]-s[1]])
        result.append(rectangle(name, p[0], s[0], y, [0., p[2]+s[2]],
                                'actual wall end to arena fence inner face; below wall top (overflight excluded)'))
    result.append(gate('gate23', 'exit_gate23'))
    if any(a['plane_x_world_m'] <= b['plane_x_world_m'] for a, b in zip(result, result[1:])):
        raise ValueError('unexpected official course ordering')
    return result


def section_transits(truth, section, radius_m=.12, rear_margin_m=.08, max_gap_sim_s=.1):
    """Swept conservative spherical footprint through an inflated finite slab."""
    x = section['plane_x_world_m']; extent = section['slab_half_depth_m'] + radius_m
    start = None; crossing = None; violations = set(); margins = []; events = []; planes = []

    def clearance(p):
        if section['kind'] == 'ring':
            c = section['center_world_m']
            return section['aperture_radius_m'] - math.hypot(p[1]-c[1], p[2]-c[2]) - radius_m
        lo, hi = section['gap_y_world_m']; bottom, top = section['gap_z_world_m']
        return min(p[1]-lo, hi-p[1], p[2]-bottom, top-p[2]) - radius_m

    for i, row in enumerate(truth):
        p = row['drone']['position_world_m']
        if p[0] > x + extent:
            start = i; crossing = None; violations = set(); margins = []
        if start is None:
            continue
        if i > start:
            previous = truth[i-1]; q = previous['drone']['position_world_m']
            dt = row['sim_time_s'] - previous['sim_time_s']
            if (not 0 < dt <= max_gap_sim_s+1e-9 or row['host_monotonic'] <= previous['host_monotonic']
                    or row.get('session') != previous.get('session')
                    or row.get('control_interval') != previous.get('control_interval')):
                violations.add('clock/session/control gap')
            fractions = []
            if abs(p[0]-q[0]) < 1e-12:
                if abs(p[0]-x) <= extent:
                    fractions = [0., 1.]
            else:
                a, b = sorted(((x-extent-q[0])/(p[0]-q[0]), (x+extent-q[0])/(p[0]-q[0])))
                if max(0., a) <= min(1., b):
                    fractions = [max(0., a), min(1., b)]
            # Circular/rectangular safe corridors are convex, so clipped endpoints
            # bound the entire piecewise-linear segment within the inflated slab.
            for f in fractions:
                margin = clearance([a+f*(b-a) for a, b in zip(q, p)])
                margins.append(margin)
                if margin < -1e-9:
                    violations.add('swept whole-vehicle footprint outside finite passage')
            if q[0] > x >= p[0]:
                f = (q[0]-x)/(q[0]-p[0])
                crossing = dict(sim_time_s=previous['sim_time_s']+f*dt,
                                host_monotonic=previous['host_monotonic']+f*(row['host_monotonic']-previous['host_monotonic']))
                planes.append(crossing)
            elif q[0] < x <= p[0]:
                violations.add('reverse crossing')
        if row.get('contacts'):
            violations.add('physical contact during transit')
        if p[0] < x-extent-rear_margin_m:
            events.append(dict(name=section['name'], start_sim_time_s=truth[start]['sim_time_s'],
                               start_host_monotonic=truth[start]['host_monotonic'],
                               rear_clear_sim_time_s=row['sim_time_s'], rear_clear_host_monotonic=row['host_monotonic'],
                               session=row.get('session'), control_interval=row.get('control_interval'),
                               plane_crossing=crossing, geometry_pass=bool(crossing and not violations),
                               violations=sorted(violations), min_swept_clearance_m=min(margins) if margins else None,
                               rear_clearance_beyond_slab_m=x-section['slab_half_depth_m']-(p[0]+radius_m)))
            start = None; crossing = None
    return dict(name=section['name'], plane_crossings=planes, transits=events,
                unfinished_transit=start is not None, full_rear_clearance_pass=any(e['geometry_pass'] for e in events))


def emitted_flight_commands(commands):
    """Retain chronological barriers; count fresh or watchdog hover emissions.

    A watchdog hover is an emitted setpoint, but it is never fresh perception
    coverage for an obstacle transit. Other command events remain interval breaks.
    """
    return [dict(row, verdict='active' if row.get('verdict') in ('active', 'watchdog_hold')
                 and row.get('command', {}).get('mode') in ('hover', 'velocity')
                 else 'not_flight_command') for row in commands]


def continuous_flight_evidence(truth, commands, intervals, begin, end):
    """Prove one flight across fresh-command gaps, without treating held TTL as fresh."""
    physical = [r for r in truth if begin-1e-9 <= r['host_monotonic'] <= end+1e-9]
    relevant = [r for r in commands if begin-.1 <= r['sent_monotonic'] <= end+1e-9]
    # Only the last command before begin can have bearing on the observed flight.
    preceding = [r for r in relevant if r['sent_monotonic'] < begin]
    relevant = (preceding[-1:] if preceding else []) + [r for r in relevant if r['sent_monotonic'] >= begin]
    reasons = []
    emitted_covered = any(a <= begin+1e-9 and b >= end-1e-9 for a, b in intervals)
    if not emitted_covered:
        reasons.append('emitted-flight-command coverage missing; firmware continuity not established')
    identities = {tuple(r['source_identity']) for r in relevant
                  if isinstance(r.get('source_identity'), (list, tuple)) and len(r['source_identity']) == 3}
    epochs = {r.get('source_epoch') for r in relevant}
    epoch_changes = [dict(host_monotonic=b['sent_monotonic'], previous=a.get('source_epoch'),
                          current=b.get('source_epoch'), verdict=b.get('verdict'))
                     for a, b in zip(relevant, relevant[1:]) if a.get('source_epoch') != b.get('source_epoch')]
    if (len(identities) != 1 or any(not isinstance(r.get('source_identity'), (list, tuple))
            or len(r['source_identity']) != 3 for r in relevant)):
        reasons.append('source identity missing or changed')
    # source_epoch is CommandGate's policy-ticket generation. _invalidate bumps
    # it on repeated snapshots/transient drops even with keep_command=True. Only
    # (session, boot, reset) identifies the source; epoch changes are diagnostic.
    # In this runtime a non-null top-level fault comes from gate.abort(), whereas
    # transient hold/recovery labels live in command.reason/source_progress.
    aborts = [dict(host_monotonic=r['sent_monotonic'], fault=r['fault'], verdict=r.get('verdict'))
              for r in relevant if r.get('fault')]
    if aborts or any(r.get('verdict') not in ('active', 'watchdog_hold')
            or r.get('command', {}).get('mode') not in ('hover', 'velocity') for r in relevant):
        reasons.append('explicit stop/disarm/gate abort or unsupported command between course sections')
    if not physical or len({r.get('session') for r in physical}) != 1:
        reasons.append('continuous single-session physical evidence missing')
    if len(identities) == 1 and physical:
        session = physical[0].get('session')
        try:
            matches = int(str(session), 0) == next(iter(identities))[0]
        except (TypeError, ValueError):
            matches = False
        if not matches:
            reasons.append('command source session does not match native physical session')
    airborne = bool(physical and all(r['drone']['position_world_m'][2] > .12 and not r.get('contacts') for r in physical))
    powered = bool(physical and all(max(r.get('motor_rpm', [0]), default=0) > 1000 for r in physical))
    if not airborne:
        reasons.append('landing/contact or unobserved airborne continuity between course sections')
    if not powered:
        reasons.append('motor power ceased or missing between course sections')
    return dict(observed=not reasons, reasons=reasons, start_host_monotonic=begin, end_host_monotonic=end,
                physical_samples=len(physical), emitted_command_rows=len(relevant),
                emitted_command_coverage_complete=emitted_covered, airborne_continuous=airborne,
                motor_power_continuous=powered, source_identities=[list(v) for v in identities],
                source_epochs=sorted(epochs, key=repr), source_epoch_changes=epoch_changes,
                source_epoch_missing_rows=sum(r.get('source_epoch') is None for r in relevant),
                source_epoch_semantics='host_command_gate_generation_diagnostic_only',
                watchdog_hold_rows=sum(r.get('verdict') == 'watchdog_hold' for r in relevant),
                fresh_requalification_after_hold=[dict(host_monotonic=b['sent_monotonic'],
                    cadence_reacquiring=b.get('cadence_reacquiring'), source_identity=b.get('source_identity'))
                    for a, b in zip(relevant, relevant[1:])
                    if a.get('verdict') == 'watchdog_hold' and b.get('verdict') == 'active'],
                gate_abort_events=aborts)


def ground_landing_assessment(truth, commands, ordered, trial):
    """Independent post-exit touchdown and terminal floor rest, never host flags."""
    limits = dict(minimum_rest_sim_s=.2, max_gt_gap_sim_s=.1, max_speed_m_s=.1,
                  max_tilt_deg=10., max_rest_displacement_m=.02,
                  max_touchdown_vertical_speed_m_s=.25, max_motor_off_rpm=100.)
    reasons, touchdowns = [], []
    exit_event = ordered[-1] if len(ordered) == 8 and ordered[-1]['name'] == 'exit_gate23' else None
    if exit_event is None:
        return dict(passed=False, reasons=['ordered exit rear clearance not established'], limits=limits,
                    touchdown_events=[], stable_rest_sim_s=0.)
    begin = exit_event['rear_clear_host_monotonic']
    cutoff = min(trial.get('host_exit_monotonic', math.inf),
                 trial.get('runtime_stop', {}).get('requested_monotonic', math.inf))
    physical = [r for r in truth if begin <= r['host_monotonic'] <= cutoff]
    floor_names = {'floor', 'arena_floor', 'ground'}

    def vector(values, length):
        return (isinstance(values, (list, tuple)) and len(values) == length
                and all(type(v) in (int, float) and math.isfinite(v) for v in values))

    def floor(row):
        return any(floor_names.intersection(c.get('geoms', []))
                   and any(str(n).startswith('cf0_') for n in c.get('geoms', [])+c.get('bodies', []))
                   and type(c.get('normal_force_n')) in (int, float) and math.isfinite(c['normal_force_n'])
                   and c['normal_force_n'] > 0 and type(c.get('distance_m')) in (int, float)
                   and math.isfinite(c['distance_m']) and c['distance_m'] <= .002
                   for c in row.get('contacts', []))

    previous = None
    for row in physical:
        if floor(row) and (previous is None or not floor(previous)):
            velocity = row['drone'].get('linear_velocity_world_mps')
            prior = None if previous is None else previous['drone'].get('linear_velocity_world_mps')
            gap = None if previous is None else row['sim_time_s']-previous['sim_time_s']
            soft = bool(vector(velocity, 3) and vector(prior, 3) and gap is not None
                and 0 < gap <= .1+1e-9 and previous.get('session') == row.get('session')
                and max(abs(velocity[2]), abs(prior[2])) <= .25+1e-9)
            touchdowns.append(dict(sim_time_s=row['sim_time_s'], host_monotonic=row['host_monotonic'],
                preceding_sample_gap_sim_s=gap, contact_vz_m_s=velocity[2] if vector(velocity, 3) else None,
                preceding_vz_m_s=prior[2] if vector(prior, 3) else None, soft_touchdown_observed=soft))
        previous = row
    if not touchdowns:
        reasons.append('post-exit force-bearing drone-floor touchdown missing')
    elif not all(t['soft_touchdown_observed'] for t in touchdowns):
        reasons.append('hard or unobserved touchdown: preceding/contact vertical speed exceeds bound or evidence missing')
    identities = [r.get('source_identity') for r in commands if r.get('sent_monotonic', math.inf) <= begin
                  and isinstance(r.get('source_identity'), (list, tuple)) and len(r['source_identity']) == 3]
    identity = list(identities[-1]) if identities else None
    stops = [r for r in commands if begin < r.get('sent_monotonic', -math.inf) <= cutoff
             and r.get('verdict') == 'explicit_stop' and not r.get('fault')
             and r.get('command', {}).get('mode') == 'stop' and identity is not None
             and r.get('source_identity') is not None and list(r['source_identity']) == identity]
    stop = stops[0]['sent_monotonic'] if stops else None
    if stop is None:
        reasons.append('post-exit explicit STOP transmission from course source missing')
    rest_start = anchor = previous_good = None
    tail_rest = 0.
    last_sample = None
    for row in physical:
        velocity, quat = row['drone'].get('linear_velocity_world_mps'), row['drone'].get('quaternion_wxyz')
        rpm = row.get('motor_rpm')
        tilt = (math.degrees(math.acos(max(-1., min(1., 1-2*(quat[1]**2+quat[2]**2)))))
                if vector(quat, 4) and .99 <= math.sqrt(sum(v*v for v in quat)) <= 1.01 else math.inf)
        last_sample = dict(sim_time_s=row['sim_time_s'], host_monotonic=row['host_monotonic'],
            force_bearing_floor_contact=floor(row), position_world_m=row['drone']['position_world_m'],
            actual_speed_m_s=math.sqrt(sum(v*v for v in velocity)) if vector(velocity, 3) else None,
            tilt_deg=tilt if math.isfinite(tilt) else None,
            max_motor_rpm=max(abs(v) for v in rpm) if vector(rpm, 4) else None)
        good = (stop is not None and row['host_monotonic'] >= stop and floor(row)
                and vector(velocity, 3) and math.sqrt(sum(v*v for v in velocity)) <= .1+1e-9
                and tilt <= 10.+1e-9 and vector(rpm, 4) and max(abs(v) for v in rpm) <= 100.)
        contiguous = (previous_good is not None and
            0 < row['sim_time_s']-previous_good['sim_time_s'] <= .1+1e-9
            and row['host_monotonic'] > previous_good['host_monotonic']
            and row.get('session') == previous_good.get('session'))
        if good:
            position = row['drone']['position_world_m']
            if not contiguous or anchor is None:
                rest_start, anchor = row['sim_time_s'], position
            elif math.dist(position, anchor) > .02+1e-9:
                rest_start, anchor = row['sim_time_s'], position
            tail_rest = row['sim_time_s']-rest_start
            previous_good = row
        else:
            rest_start = anchor = previous_good = None
            tail_rest = 0.
    powered_before_stop = bool(stop is not None and any(
        r['host_monotonic'] <= stop and vector(r.get('motor_rpm'), 4)
        and max(r['motor_rpm']) > 1000 for r in physical))
    if not powered_before_stop:
        reasons.append('post-exit powered-to-motor-off transition not observed')
    if tail_rest < .2-1e-9:
        reasons.append('terminal post-STOP floor rest with low speed/tilt and decayed motors shorter than 0.2s or unavailable')
    return dict(passed=not reasons, reasons=reasons, limits=limits,
        exit_rear_clear_host_monotonic=begin, assessment_cutoff_host_monotonic=cutoff,
        post_exit_gt_samples=len(physical), touchdown_events=touchdowns,
        explicit_stop_host_monotonic=stop, explicit_stop_rows=len(stops),
        powered_before_stop=powered_before_stop, stable_rest_sim_s=tail_rest,
        terminal_sample=last_sample,
        stable_rest_start_sim_s=rest_start, stable_rest_end_sim_s=None if previous_good is None else previous_good['sim_time_s'])


def score_course(rows, sections, commands, trial, lifecycle, hooks, *, native=False,
                 radius_m=.12, rear_margin_m=.08, max_gap_sim_s=.1):
    if (not all(math.isfinite(v) for v in (radius_m, rear_margin_m, max_gap_sim_s))
            or radius_m < .12 or rear_margin_m < .08 or not 0 < max_gap_sim_s <= .1):
        raise ValueError('course evaluation cannot relax 0.12m radius, 0.08m rear margin or 0.1s maximum gap')
    truth = [r for r in rows if r.get('event') == 'ground_truth']
    errors = []
    if tuple(s['name'] for s in sections) != REQUIRED_SECTIONS:
        errors.append('full eight-section course required in official order')
    if len(truth) < 2:
        errors.append('insufficient ground truth')
    for r in truth:
        p = r.get('drone', {}).get('position_world_m', [])
        if (len(p) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in p)
                or not all(isinstance(r.get(k), (int, float)) and math.isfinite(r[k]) for k in ('sim_time_s', 'host_monotonic'))):
            raise ValueError('malformed/nonfinite ground truth')
    gaps = [b['sim_time_s']-a['sim_time_s'] for a, b in zip(truth, truth[1:])]
    sessions = {r.get('session') for r in truth}
    if len(sessions) != 1 or None in sessions:
        errors.append('mixed or missing source session')
    if any(g <= 0 or g > max_gap_sim_s+1e-9 for g in gaps) or any(b['host_monotonic'] <= a['host_monotonic'] for a, b in zip(truth, truth[1:])):
        errors.append('nonmonotonic clock or excessive sample gap')
    # Keep stops as barriers rather than permitting a transit under explicit stop.
    fresh_commands = [dict(r, verdict='not_flight_command' if r.get('verdict') == 'explicit_stop' else r.get('verdict')) for r in commands]
    intervals = controlled_intervals(fresh_commands, trial)
    emitted_intervals = controlled_intervals(emitted_flight_commands(commands), trial)
    controlled = restrict_control_window(truth, intervals) if not errors else []
    scored = [section_transits(controlled, s, radius_m, rear_margin_m, max_gap_sim_s) for s in sections]
    events = sorted((e for s in scored for e in s['transits'] if e['geometry_pass']), key=lambda e: e['rear_clear_sim_time_s'])
    ordered = []; out_of_order = []; continuity = []
    for e in events:
        expected = sections[len(ordered)]['name'] if len(ordered) < len(sections) else None
        after_previous = not ordered or e['start_sim_time_s'] >= ordered[-1]['rear_clear_sim_time_s']
        if e['name'] == expected and after_previous:
            proof = continuous_flight_evidence(truth, commands, emitted_intervals,
                ordered[0]['start_host_monotonic'] if ordered else e['start_host_monotonic'], e['rear_clear_host_monotonic'])
            continuity.append(dict(through_section=e['name'], **proof))
        else:
            proof = dict(observed=False, reasons=['not next required section or overlapping transit'])
        if e['name'] == expected and after_previous and proof['observed']:
            ordered.append(e)
        else:
            out_of_order.append(dict(name=e['name'], rear_clear_sim_time_s=e['rear_clear_sim_time_s'], expected=expected,
                                     flight_continuity_observed=proof['observed'], continuity_reasons=proof['reasons'],
                                     starts_after_previous_clearance=after_previous))
    floor_names = {'floor', 'arena_floor', 'ground'}
    contacts = [dict(sim_time_s=r['sim_time_s'], host_monotonic=r['host_monotonic'],
                     contacts=[c for c in r.get('contacts', []) if not floor_names.intersection(c.get('geoms', []))])
                for r in truth if any(not floor_names.intersection(c.get('geoms', [])) for c in r.get('contacts', []))]
    lifecycle_complete = bool(truth and lifecycle.get('wrapper_closed') is True
                              and lifecycle.get('source_main_returned') is True
                              and lifecycle.get('session') == truth[0].get('session')
                              and lifecycle.get('gt_rows') == len(truth)
                              and not lifecycle.get('infrastructure_failure', True)
                              and not lifecycle.get('hook_cleanup_errors')
                              and lifecycle.get('sim_time_regressions', 1) == 0
                              and lifecycle.get('host_time_regressions', 1) == 0
                              and hooks.get('phase') == 'closed' and not hooks.get('errors')
                              and (not hooks.get('conditions_installed') or hooks.get('conditions_closed') is True))
    last_dispatch = lifecycle.get('last_dispatch', {}).get('sim_time_s')
    if last_dispatch is None or not truth or not 0 <= last_dispatch-truth[-1]['sim_time_s'] <= max_gap_sim_s+1e-9:
        lifecycle_complete = False
    pacing = hooks.get('pacing_final_snapshot') or {}
    if pacing and (pacing.get('infrastructure_failure') or pacing.get('cleanup_errors') or not pacing.get('run_finally_completed')):
        lifecycle_complete = False
    powered = any(max(r.get('motor_rpm', [0]), default=0) > 1000 and r['drone']['position_world_m'][2] > .3 for r in controlled)
    geometry_complete = tuple(s['name'] for s in sections) == REQUIRED_SECTIONS and len(ordered) == 8
    summary = trial.get('host_summary') or {}
    healthy_host = bool(trial.get('status') == 'completed_unassessed' and trial.get('host_returncode') == 0
                        and trial.get('shutdown_trigger') == 'host_exit' and not trial.get('error')
                        and isinstance(summary.get('status'), dict) and not summary['status'].get('error')
                        and summary.get('outcome') not in (None, 'aborted'))
    diag = summary.get('course_diagnostics') or {}
    host_course_complete = bool(trial.get('mode') == 'mission2-course' and summary.get('mode') == 'mission2-course'
        and summary.get('assessment_scope') == 'mission2-course'
        and summary.get('outcome') == 'experimental_course_landing_assessed'
        and summary.get('full_mission_completed') is True and diag.get('full_mission_completed') is True
        and diag.get('phase') == 'complete' and diag.get('stage') == 'exit-crossed'
        and diag.get('course_finished') is True and diag.get('landing_assessed') is True and not diag.get('root_failure')
        and isinstance(diag.get('passed_count'), int) and diag['passed_count'] >= 3
        and isinstance(diag.get('post_ring_completed_sections'), int) and diag['post_ring_completed_sections'] >= 3)
    landing = ground_landing_assessment(truth, commands, ordered, trial)
    success = bool(geometry_complete and landing['passed'] and not errors and not contacts and native and lifecycle_complete and powered and healthy_host and host_course_complete)
    return dict(schema=2, scoring_version=SCORING_VERSION,
                scorer_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                evaluation_only=True, scope='full_mission2_entry_three_rings_blocks_two_walls_exit_ground_landing',
                verdict='PASS' if success else 'FAIL' if contacts or errors or trial.get('status') == 'failed' or geometry_complete and not landing['passed'] else 'PARTIAL',
                full_mission2_success=success, powered_success=success,
                trial_status=trial.get('status'), host_returncode=trial.get('host_returncode'),
                trial_error=trial.get('error') or (summary.get('status') or {}).get('error'),
                course_geometry_complete=geometry_complete, required_sections=[s['name'] for s in sections],
                ordered_sections_completed=[e['name'] for e in ordered], ordered_completion_events=ordered,
                next_required_section=sections[len(ordered)]['name'] if len(ordered) < len(sections) else None,
                out_of_order_or_discontinuous_transits=out_of_order, section_results=scored,
                all_log_nonfloor_contact_samples=len(contacts), all_log_nonfloor_contact_evidence=contacts,
                all_log_gt_samples=len(truth), max_gt_sample_gap_sim_s=max(gaps, default=None),
                evidence_errors=errors, lifecycle_complete=lifecycle_complete,
                source_native=native, powered_motion_evidence=powered, host_completed_without_error=healthy_host,
                host_course_completion_assessed=host_course_complete, flight_continuity=continuity,
                ground_landing_assessment=landing,
                command_coverage=dict(command_rows=len(commands), controlled_gt_samples=len(controlled),
                    intervals_host_monotonic=intervals, fresh_command_intervals_host_monotonic=intervals,
                    emitted_flight_command_intervals_host_monotonic=emitted_intervals,
                    ordered_course_single_interval=bool(ordered and len({e['control_interval'] for e in ordered}) == 1),
                    ordered_course_continuous_flight=bool(ordered and continuity[-1]['observed'])),
                footprint=dict(conservative_spherical_radius_m=radius_m, rear_clearance_margin_m=rear_margin_m),
                limitations=['Offline evaluation only; geometry/GT never supplies policy inputs.',
                             'All logged nonfloor contacts include startup and shutdown; floor contact is not counted as an obstacle collision.',
                             'Piecewise-linear swept trajectory between samples; sub-sample dynamics remain unobserved.',
                             'Walls and blocks require finite side/between passages below obstacle tops; overflight is excluded.',
                             'Command coverage proves host sent setpoints, not firmware receipt or semantic visual-policy independence.',
                             'Each obstacle transit requires one fresh-command interval; inter-section watchdog holds require same source identity, emitted setpoint cadence and continuous airborne motor evidence.',
                             'source_epoch counts command-gate ticket invalidations, not physical resets; changes/missing values are diagnostic only. Source identity/session changes and actual gate aborts remain barriers.',
                             'Timed ending, reduced-scope summary, or detection disappearance alone cannot advance any section.'])


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def evaluate_trial(path):
    trial = json.loads(path.read_text()); runtime = Path(trial['runtime_log_dir']); evaluation = runtime/'evaluation'
    scene = runtime/'arena/scene.xml'; metadata_path = evaluation/'model_metadata.json'
    metadata = json.loads(metadata_path.read_text()); sections = course_from_scene(scene, metadata)
    inputs = dict(trial=path, scene=scene, metadata=metadata_path, ground_truth=evaluation/'ground_truth.jsonl',
                  lifecycle=evaluation/'evaluation_lifecycle.json', hooks=evaluation/'evaluation_hooks.json',
                  provenance=evaluation/'provenance.json', commands=path.parent/'host/commands.jsonl')
    proof = json.loads(inputs['provenance'].read_text()) if inputs['provenance'].exists() else {}
    lifecycle = json.loads(inputs['lifecycle'].read_text()) if inputs['lifecycle'].exists() else {}
    hooks = json.loads(inputs['hooks'].read_text()) if inputs['hooks'].exists() else {}
    result = score_course(read_jsonl(inputs['ground_truth']), sections,
                          read_jsonl(inputs['commands']) if inputs['commands'].exists() else [], trial, lifecycle, hooks,
                          native=proof.get('evidence') == 'native_instrumented_source')
    result['inputs'] = {key: dict(path=str(value.resolve()), sha256=hashlib.sha256(value.read_bytes()).hexdigest() if value.exists() else None)
                        for key, value in inputs.items()}
    manifest = runtime/'arena/arena_manifest.json'
    expected_hash = json.loads(manifest.read_text()).get('scene_sha256') if manifest.exists() else None
    result['scene_manifest_hash_matches'] = expected_hash == result['inputs']['scene']['sha256'] if expected_hash else None
    if result['scene_manifest_hash_matches'] is not True:
        result['evidence_errors'].append('scene manifest missing or hash mismatch')
        result.update(full_mission2_success=False, powered_success=False, verdict='FAIL')
    result['evaluation_geometry'] = sections
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', required=True, type=Path)
    parser.add_argument('--out', type=Path, help='optional NEW output under workspace .local; default stdout only')
    args = parser.parse_args()
    try:
        result = evaluate_trial(args.trial)
    except (ValueError, KeyError, OSError, ET.ParseError) as exc:
        result = dict(verdict='FAIL', full_mission2_success=False, powered_success=False, reason=str(exc))
    if args.out:
        if args.out.exists():
            parser.error('refusing to overwrite an existing report or immutable trial input')
        write_json(args.out, result)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
