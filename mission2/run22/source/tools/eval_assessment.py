"""Scoped evidence summaries. Raw trial failures remain visible; parent owns verdict."""
from __future__ import annotations
from collections import Counter
import math


def contact_classes(truth):
    counts=Counter()
    airborne_seen=False
    for row in truth:
        high=row['drone']['position_world_m'][2]>.30
        airborne_seen=airborne_seen or high
        categories=set()
        for contact in row.get('contacts',[]):
            geoms=contact.get('geoms',[])
            if any(name in geoms for name in ('floor','arena_floor','ground')):
                categories.add('post_airborne_floor_contact_samples' if airborne_seen else 'initial_floor_contact_samples')
            elif 'landing_pad' in geoms:categories.add('pad_contact_samples')
            else:
                categories.add('airborne_obstacle_collision_samples' if airborne_seen else 'preflight_other_contact_samples')
        counts.update(categories)
    names=('initial_floor_contact_samples','post_airborne_floor_contact_samples','pad_contact_samples',
           'airborne_obstacle_collision_samples','preflight_other_contact_samples')
    return {name:counts[name] for name in names}


def scoped_assessment(trial,decisions,score,truth,scope_override=None):
    summary=trial.get('host_summary') or {}
    config=summary.get('controller_config') or {}
    requested=scope_override or trial.get('assessment_scope')
    inferred=not requested or requested=='mission3' and config.get('allow_landing') is False
    effective='tracking-only' if inferred and trial.get('mode')=='mission3' and config.get('allow_landing') is False else requested or trial.get('mode','unknown')
    scope_expected=(effective=='tracking-only' and config.get('allow_landing') is False) or effective=='approach'
    end_expected=(scope_expected and
                  summary.get('outcome')=='aborted' and summary.get('reason','').startswith('host duration expired') and
                  not summary.get('status',{}).get('error') and trial.get('shutdown_trigger')=='host_exit' and
                  not trial.get('cleanup_error'))
    statuses=[]
    for row in decisions:
        diagnostics=row.get('diagnostics') or {}
        status=diagnostics.get('status') or {}
        if row.get('accepted') and status:statuses.append(status)
    phases=Counter(str(row.get('phase','unknown')) for row in statuses)
    outcomes=Counter(str(row.get('outcome','unknown')) for row in statuses)
    roots=Counter(cause for row in statuses for cause in row.get('root_causes',[]))
    visible=sum(bool(row.get('visual_valid')) for row in statuses)
    after_initial=[row for row in statuses if row.get('phase') not in ('scan','acquire') and row.get('verdict')!='initial_stationary_settle']
    track=score.get('powered_motion_evidence',False) and phases.get('follow',0)>=10 and score.get('contacts_by_phase',{}).get('airborne_obstacle_collision_samples',0)==0
    result=dict(mode=trial.get('mode'),requested_scope=requested,effective_scope=effective,
        scope_inferred_from_landing_disabled=inferred and effective=='tracking-only',scorer_scope_override=scope_override,
        parent_verdict='NOT_ASSIGNED',tracking_evidence_present=bool(track),
        expected_bounded_scope_end=end_expected,expected_bounded_tracking_end=end_expected and effective=='tracking-only',
        mission_completion_attempted=effective in ('landing','mission2','mission3'),
        landing_enabled=config.get('allow_landing'),landing_confirmed=False,
        raw_trial_status=trial.get('status'),raw_host_returncode=trial.get('host_returncode'),
        raw_host_outcome=summary.get('outcome'),raw_host_reason=summary.get('reason'),
        accepted_decision_status_rows=len(statuses),visual_valid_rows=visible,
        visual_valid_fraction=visible/len(statuses) if statuses else None,
        post_initial_visual_valid_fraction=sum(bool(r.get('visual_valid')) for r in after_initial)/len(after_initial) if after_initial else None,
        phase_counts=dict(phases),outcome_counts=dict(outcomes),root_cause_counts=dict(roots),
        limitations=['tracking evidence is scoped and does not establish rover landing or full mission success',
                     'duration abort remains preserved; expected tracking end is separated from unexpected runtime fault',
                     'tracking criteria and final verdict require parent review'])
    rover_rows=[row for row in truth if row.get('rover') and row['drone']['position_world_m'][2]>.3 and not row.get('contacts')]
    if rover_rows and effective=='tracking-only':
        last=rover_rows[-1]['host_monotonic']
        tail=[row for row in rover_rows if row['host_monotonic']>=last-10]
        # Geometry is the documented frozen rear-facing board, evaluation only.
        desired_x=.3165-float(config.get('standoff_m',1.2))
        errors=[[row['rover']['drone_position_rover_m'][0]-desired_x,
                 row['rover']['drone_position_rover_m'][1]] for row in tail]
        result['last_10_host_s_standoff_gt']=dict(rows=len(tail),
            marker_reference='documented frozen board center rover x=+0.3165; face normal -X; evaluation only',
            desired_drone_x_rover_m=desired_x,
            mean_error_rover_xy_m=[sum(e[i] for e in errors)/len(errors) for i in (0,1)],
            rmse_rover_xy_m=[math.sqrt(sum(e[i]**2 for e in errors)/len(errors)) for i in (0,1)],
            max_abs_error_rover_xy_m=[max(abs(e[i]) for e in errors) for i in (0,1)],
            last_drone_position_rover_m=tail[-1]['rover']['drone_position_rover_m'])
    return result
