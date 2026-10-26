"""Offline airborne phase residuals, censored stop drift and apparent sensor lag."""
from __future__ import annotations
from bisect import bisect_left
import math

from eval_score import quaternion_rotate, displacement_integrals, norm


def airborne(row):
    return row['drone']['position_world_m'][2]>.30 and not row.get('contacts')


def nearest_index(times,stamp):
    index=bisect_left(times,stamp)
    return min((i for i in (index-1,index) if 0<=i<len(times)),key=lambda i:abs(times[i]-stamp))


def body_velocity(row):
    w,x,y,z=row['drone']['quaternion_wxyz']
    return quaternion_rotate([-x,-y,-z,w],row['drone']['linear_velocity_world_mps'])


def yaw(row):
    w,x,y,z=row['drone']['quaternion_wxyz']
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def match_airborne(host,truth,max_offset_s=.15):
    times=[row['host_monotonic'] for row in truth]
    pairs=[]
    for row in host:
        i=nearest_index(times,row['host_monotonic'])
        offset=times[i]-row['host_monotonic']
        if abs(offset)<=max_offset_s and airborne(truth[i]):pairs.append((row,truth[i],offset))
    return pairs


def fixed_yaw_alignment(pair):
    host,gt,_=pair
    x,y,z,w=host['quaternion_xyzw']
    return yaw(gt)-math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def apparent_velocity_lag(host,truth,*,max_lag_s=.30,step_s=.01):
    """Fixed gain=1, separately per axis; no time/velocity correction is proposed."""
    if not host or not truth:return {'verdict':'BLOCKED','reason':'missing host/GT rows'}
    times=[row['host_monotonic'] for row in truth]
    lags=[round(i*step_s,9) for i in range(-round(max_lag_s/step_s),round(max_lag_s/step_s)+1)]
    candidates=[]
    # Require every lag candidate to remain airborne and in logged coverage so
    # candidate scores use the same host samples, excluding ground transitions.
    for h in host:
        if not times[0]+max_lag_s<=h['host_monotonic']<=times[-1]-max_lag_s:continue
        targets=[]
        for lag in lags:
            index=nearest_index(times,h['host_monotonic']-lag)
            target=truth[index]
            if not airborne(target) or abs(times[index]-(h['host_monotonic']-lag))>.15:break
            targets.append(body_velocity(target))
        if len(targets)==len(lags):candidates.append((h,targets))
    if len(candidates)<20:return dict(verdict='BLOCKED',reason='insufficient common airborne lag samples',rows=len(candidates))
    axes=[]
    for axis,name in enumerate(('forward','left','up')):
        ego=[row[0]['velocity_body'][axis] for row in candidates]
        mean_ego=sum(ego)/len(ego)
        scores=[]
        for lag_index,lag in enumerate(lags):
            gt=[row[1][lag_index][axis] for row in candidates]
            mean_gt=sum(gt)/len(gt)
            std_gt=math.sqrt(sum((v-mean_gt)**2 for v in gt)/len(gt))
            std_ego=math.sqrt(sum((v-mean_ego)**2 for v in ego)/len(ego))
            errors=[a-b for a,b in zip(ego,gt)]
            corr=(sum((a-mean_ego)*(b-mean_gt) for a,b in zip(ego,gt))/len(gt)/(std_ego*std_gt)
                  if std_ego>1e-8 and std_gt>1e-8 else None)
            scores.append(dict(lag_s=lag,rmse_mps=math.sqrt(sum(v*v for v in errors)/len(errors)),
                mean_bias_mps=sum(errors)/len(errors),correlation=corr,gt_std_mps=std_gt,ego_std_mps=std_ego))
        baseline=scores[lags.index(0.)]
        best=min(scores,key=lambda row:row['rmse_mps'])
        axes.append(dict(axis=name,rows=len(candidates),excitation_sufficient=baseline['gt_std_mps']>.02,
            zero_lag=baseline,best_apparent_receipt_lag=best,
            rmse_improvement_mps=baseline['rmse_mps']-best['rmse_mps'],
            search_boundary_hit=abs(best['lag_s'])>=max_lag_s-1e-9))
    return dict(axes=axes,ground_phase_excluded=True,fitted_velocity_gain=1.,lag_grid_s=step_s,
        sign='positive lag compares ego received at t against GT at t-lag (ego appears delayed)',
        uncertainty='apparent receipt lag combines estimator response, state association and transport; no causal sensor delay or fitted calibration',
        limitations=['same airborne samples for all candidates; no ground contact/near-ground transitions',
                     'short single-run excitation; no confidence interval; nearest-GT sampling limits lag resolution',
                     'physical GT m/s and wall-timed estimator m/s may differ; lag/gain effects are not uniquely identifiable'])


def stop_drift(truth,phase,end_limit,*,speed_threshold_mps=.03,yawrate_threshold_deg_s=2.,hold_sim_s=.30,angular_scope='full_norm'):
    times=[row['host_monotonic'] for row in truth]
    begin=phase['end_host_monotonic']
    if end_limit<=begin:return dict(settled_observed=False,censored=True,reason='no all-zero settle command window')
    index=nearest_index(times,begin)
    start=truth[index]
    if not airborne(start):return dict(settled_observed=False,censored=True,reason='phase ends outside airborne domain')
    selected=[r for r in truth[index:] if begin-.04<=r['host_monotonic']<=end_limit and airborne(r)]
    if not selected:return dict(settled_observed=False,censored=True,reason='no airborne settle samples')
    settled=None
    low_begin=None
    previous=None
    longest_dwell=0.
    longest_interval=None
    last_violation=None
    speed_violations=angular_violations=gap_violations=0
    max_gap=0.
    for row in selected:
        rates=row['drone']['angular_velocity_world_radps']
        angular=0. if angular_scope=='none' else abs(math.degrees(rates[2])) if angular_scope=='yaw' else math.degrees(norm(rates))
        speed=norm(row['drone']['linear_velocity_world_mps'])
        low=(speed<=speed_threshold_mps and angular<=yawrate_threshold_deg_s)
        gap=row['sim_time_s']-previous if previous is not None else 0.
        max_gap=max(max_gap,gap)
        speed_violations+=int(speed>speed_threshold_mps)
        angular_violations+=int(angular>yawrate_threshold_deg_s)
        gap_violations+=int(gap>.100000001)
        if not low or gap>.100000001:
            low_begin=None
            last_violation=dict(sim_time_s=row['sim_time_s'],host_monotonic=row['host_monotonic'],
                speed_mps=speed,angular_rate_deg_s=angular,
                angular_components_world_deg_s=[math.degrees(value) for value in rates],
                reasons=[reason for reason,failed in [('speed',speed>speed_threshold_mps),('angular_rate',angular>yawrate_threshold_deg_s),('sample_gap',gap>.100000001)] if failed])
        if low:
            if low_begin is None:low_begin=row['sim_time_s']
            if row['sim_time_s']-low_begin>longest_dwell:
                longest_dwell=row['sim_time_s']-low_begin
                longest_interval=[low_begin,row['sim_time_s']]
            if row['sim_time_s']-low_begin>=hold_sim_s-1e-9:
                settled=row
                break
        previous=row['sim_time_s']
    finish=settled or selected[-1]
    delta=[b-a for a,b in zip(start['drone']['position_world_m'],finish['drone']['position_world_m'])]
    w,x,y,z=start['drone']['quaternion_wxyz']
    dyaw=math.atan2(math.sin(yaw(finish)-yaw(start)),math.cos(yaw(finish)-yaw(start)))
    return dict(settled_observed=settled is not None,censored=settled is None,
        measurement='through measured sustained settle' if settled else 'lower bound through next command / evidence boundary; not a stop',
        from_host_monotonic=start['host_monotonic'],until_host_monotonic=finish['host_monotonic'],
        elapsed_host_s=finish['host_monotonic']-start['host_monotonic'],
        elapsed_sim_s=finish['sim_time_s']-start['sim_time_s'],
        drift_world_m=delta,drift_initial_body_m=quaternion_rotate([-x,-y,-z,w],delta),
        residual_yaw_deg=math.degrees(dyaw),final_speed_mps=norm(finish['drone']['linear_velocity_world_mps']),
        final_yawrate_deg_s=math.degrees(finish['drone']['angular_velocity_world_radps'][2]),
        final_angular_rate_norm_deg_s=math.degrees(norm(finish['drone']['angular_velocity_world_radps'])),
        longest_qualifying_dwell_sim_s=longest_dwell,longest_qualifying_interval_sim_s=longest_interval,
        qualifying_dwell_at_measurement_end_sim_s=(finish['sim_time_s']-low_begin) if low_begin is not None else 0.,
        max_sample_gap_sim_s=max_gap,
        criterion_violation_samples=dict(speed=speed_violations,angular_rate=angular_violations,sample_gap=gap_violations),
        last_criterion_violation=last_violation,
        criterion=dict(speed_mps=speed_threshold_mps,angular_scope=angular_scope,angular_rate_deg_s=yawrate_threshold_deg_s,hold_sim_s=hold_sim_s))


def motion_assessment(host,truth_rows,phases,trial):
    truth=[row for row in truth_rows if row.get('event')=='ground_truth']
    if not truth:return dict(verdict='BLOCKED',reason='no truth rows')
    pairs=match_airborne(host,truth)
    if len(pairs)<2:return dict(verdict='BLOCKED',reason='insufficient airborne associated state/GT pairs')
    alignment=fixed_yaw_alignment(pairs[0])
    outputs=[]
    for index,phase in enumerate(phases):
        command=phase['command_at_start']
        if not any(abs(command.get(key,0))>1e-6 for key in ('vx','vy','yawrate_deg_s')):continue
        begin=phase['start_host_monotonic']
        end=phase['end_host_monotonic']
        stop_limit=end
        # Extend only through subsequent contiguous commands that request zero
        # translation AND zero yaw; never cross another active directional phase.
        for later in phases[index+1:]:
            if later['start_host_monotonic']>stop_limit+.100000001:break
            if any(abs(later['command_at_start'].get(key,0))>1e-6 for key in ('vx','vy','yawrate_deg_s')):break
            # Do not count a new descent/ramp/landing phase as a stationary
            # settling interval merely because its planar commands are zero.
            if not any(token in later['reason'] for token in ('settle','dwell','hold')):break
            stop_limit=later['end_host_monotonic']
        drift=stop_drift(truth,phase,stop_limit)
        translational=stop_drift(truth,phase,stop_limit,angular_scope='none')
        speed_yaw=stop_drift(truth,phase,stop_limit,angular_scope='yaw')
        measured_end=drift.get('until_host_monotonic',end)
        result=dict(reason=phase['reason'],command_at_start=command,stop_drift=drift,
            translation_only_settle=translational,speed_and_yaw_settle=speed_yaw,
            phase_only=None,phase_plus_observed_settle=None,phase_plus_available_zero_command_window=None,
            full_zero_command_window_end_host_monotonic=stop_limit)
        windows=[('phase_only',end),('phase_plus_available_zero_command_window',stop_limit)]
        if drift['settled_observed']:windows.append(('phase_plus_observed_settle',measured_end))
        for label,finish in windows:
            group=[pair for pair in pairs if begin<=pair[0]['host_monotonic']<=finish]
            # Do not join across an airborne/association gap or estimator reset.
            blocks=[]
            for pair in group:
                if not blocks or pair[0]['session']!=blocks[-1][-1][0]['session'] or pair[0]['host_monotonic']-blocks[-1][-1][0]['host_monotonic']>.15:
                    blocks.append([])
                blocks[-1].append(pair)
            comparisons=[]
            for block in blocks:
                if len(block)<2:continue
                gt_delta=[b-a for a,b in zip(block[0][1]['drone']['position_world_m'],block[-1][1]['drone']['position_world_m'])]
                comparisons.append(dict(rows=len(block),start_host_monotonic=block[0][0]['host_monotonic'],
                    end_host_monotonic=block[-1][0]['host_monotonic'],gt_displacement_world_m=gt_delta,
                    integrals=displacement_integrals(block,alignment)))
            result[label]=comparisons
        if translational['settled_observed']:
            final=truth[nearest_index([row['host_monotonic'] for row in truth],stop_limit)]
            # Stay within the available settle interval, never sample a new command.
            if final['host_monotonic']>stop_limit:
                final=next((row for row in reversed(truth) if row['host_monotonic']<=stop_limit),final)
            initial=truth[nearest_index([row['host_monotonic'] for row in truth],translational['until_host_monotonic'])]
            residual=[b-a for a,b in zip(initial['drone']['position_world_m'],final['drone']['position_world_m'])]
            result['observed_after_translation_settle_until_window_end']=dict(displacement_world_m=residual,
                displacement_norm_m=norm(residual),until_host_monotonic=final['host_monotonic'],
                limitation='observed remaining motion only; no bound is claimed after the next command')
        outputs.append(result)
    return dict(analysis_schema=2,ground_phase_excluded=True,domain='GT height >0.30m, no physical contacts, valid associated host state',
        fixed_evaluation_yaw_alignment_rad=alignment,phases=outputs,
        apparent_velocity_lag=apparent_velocity_lag(host,truth),
        uncertainty=['receipt-aligned nearest sample, source/state/transport clocks not exactly synchronized',
                     'settle requires measured speed/yaw rate hold; unobserved settle is censored',
                     'body sensor RMSE is instantaneous velocity comparison; snapshot integral is estimator displacement; these answer different questions',
                     'no gain, RTF multiplier or timing scale is fit or recommended'])
