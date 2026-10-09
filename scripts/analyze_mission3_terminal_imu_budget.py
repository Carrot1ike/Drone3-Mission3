#!/usr/bin/env python3
"""완료된 시험의 말기 IMU 속도오차 예산. truth 대입은 사후 진단에만 사용."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from datetime import datetime
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def rows(path, lo=-math.inf, hi=math.inf):
    result = []
    with Path(path).open() as stream:
        for line in stream:
            value = json.loads(line)
            if lo-1e-8 <= value['t'] <= hi+1e-8:
                result.append(value)
    return result


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])


def rpy(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return rz(yaw) @ np.array([[cp,sp*sr,sp*cr],[0.,cr,-sr],[-sp,cp*sr,cp*cr]])


def case(task):
    run = Path(task['run'])
    names = ('commands.jsonl','truth.jsonl','sensors.jsonl','offline-evaluation.json',
             'source-snapshot/scripts/mission3/physical_estimator.py',
             'source-snapshot/scripts/mission3/estimator.py')
    before = {str(run/name):sha(run/name) for name in names}
    evaluation = read(run/'offline-evaluation.json')
    contact_time = evaluation['checks']['touchdown']['first_touchdown_time_s']
    end = contact_time
    commands = rows(run/'commands.jsonl')
    terminal = next(c for c in commands if c['phase']=='TOUCHDOWN_DESCEND')
    start = terminal['t']
    truth = rows(run/'truth.jsonl', start-.02, end+.02)
    if task.get('precontact_only'):
        end = max(x['t'] for x in truth if x['t']<contact_time-1e-8)
        assert not any(x['contacts'] for x in truth if start-1e-8<=x['t']<=end+1e-8)
    sensors = rows(run/'sensors.jsonl', start-.02, end)
    times = np.array([x['t'] for x in truth])
    actual_vel = np.array([x['aircraft']['velocity_world_m_s'] for x in truth])
    actual_angles = np.unwrap(np.array([x['aircraft']['rpy_rad'] for x in truth]), axis=0)
    heading = truth[np.argmin(abs(times-start))]['vehicle']['heading_rad']
    basis = rz(heading).T
    interp = lambda values,t: np.array([np.interp(t,times,values[:,i]) for i in range(3)])
    prior = next(s for s in reversed(sensors) if s['t']<=start+1e-8)
    start_sensor = prior
    output_rows = []
    # Fixed terminal heading is an offline along/cross coordinate definition.
    integrals = {k:np.zeros(3) for k in ('rotation_surrogate','bias_subtraction',
                                        'averaging_noise_dynamics_residual','non_imu_update',
                                        'vehicle_ff_error','own_velocity_error')}
    total = np.zeros(3)
    previous_error = np.array(prior['estimate']['velocity_world_m_s'])-interp(actual_vel,prior['t'])
    max_step_error = 0.
    for sensor in sensors:
        if sensor['t']<=start+1e-8:
            continue
        dt = sensor['t']-prior['t']
        assert 0<dt<.01000001, dt
        estimate, old = sensor['estimate'], prior['estimate']
        h = estimate['heading_rad']
        bias = np.r_[old['horizontal_accel_residual_heading_m_s2'],0.]
        packet_acc = np.array(sensor['observer_packet']['accel_heading_m_s2'])
        raw_acc = rz(h) @ packet_acc
        subtraction = -rz(h) @ bias
        corrected = raw_acc+subtraction
        av, pav = interp(actual_vel,sensor['t']),interp(actual_vel,prior['t'])
        actual_dv = av-pav
        # Substitute sampled actual attitude only for an offline rotation
        # sensitivity. The logged body force averages the previous IMU window;
        # current 50Hz interpolated attitude is NOT its true exposure average.
        q_acc = rpy(*interp(actual_angles,sensor['t'])) @ np.array(estimate['specific_force_body_m_s2'])-np.array([0.,0.,9.81])
        rotation_surrogate = raw_acc-q_acc
        remaining = q_acc-actual_dv/dt
        observed_dv = np.array(estimate['velocity_world_m_s'])-np.array(old['velocity_world_m_s'])
        other = observed_dv-corrected*dt
        predicted = (rotation_surrogate+subtraction+remaining)*dt+other
        actual_error_delta = observed_dv-actual_dv
        assert np.max(abs(predicted-actual_error_delta))<1e-10
        max_step_error = max(max_step_error,float(np.linalg.norm(other[:2])))
        if not estimate['flow_fusion_usable']:
            # Detect unrecorded anchor/heading corrections rather than assuming
            # propagation is always pure IMU simply because Flow is unavailable.
            assert np.linalg.norm(other[:2])<1e-8, (str(run),sensor['t'],other.tolist())
        for key,value in [('rotation_surrogate',rotation_surrogate*dt),
                          ('bias_subtraction',subtraction*dt),
                          ('averaging_noise_dynamics_residual',remaining*dt),
                          ('non_imu_update',other)]:
            integrals[key]+=value
        total+=actual_error_delta
        error = np.array(estimate['velocity_world_m_s'])-av
        integrals['own_velocity_error']+=(previous_error+error)*.5*dt
        ff = np.array(terminal['vehicle']['vehicle_velocity_world_m_s'])
        cart = interp(np.array([x['vehicle']['velocity_world_m_s'] for x in truth]),sensor['t'])
        integrals['vehicle_ff_error']+=(ff-cart)*dt
        output_rows.append({'t_s':sensor['t'],'since_terminal_s':sensor['t']-start,
                            'own_cross_velocity_error_m_s':float((basis@error)[1]),
                            'rotation_cumulative_cross_delta_v_m_s':float((basis@integrals['rotation_surrogate'])[1]),
                            'bias_cumulative_cross_delta_v_m_s':float((basis@integrals['bias_subtraction'])[1]),
                            'other_cumulative_cross_delta_v_m_s':float((basis@integrals['averaging_noise_dynamics_residual'])[1]),
                            'flow_fusion_usable':estimate['flow_fusion_usable']})
        previous_error=error
        prior=sensor
    initial_error = np.array(start_sensor['estimate']['velocity_world_m_s'])-interp(actual_vel,start)
    final_error = np.array(prior['estimate']['velocity_world_m_s'])-interp(actual_vel,end)
    assert abs(prior['t']-end)<1e-8
    closure = total-(final_error-initial_error)
    assert np.linalg.norm(closure)<1e-10,closure
    check_commands = [c for c in commands if start-1e-8<=c['t']<=end+1e-8]
    relative_errors=[]
    for command in (check_commands[0],check_commands[-1]):
        actual=truth[np.argmin(abs(times-command['t']))]
        assert abs(actual['t']-command['t'])<1e-8
        true_rel=np.array(actual['pad']['center_world_m'])-np.array(actual['aircraft']['center_world_m'])
        rel_error=np.array(command['vehicle']['relative_pad_world_m'])-true_rel
        relative_errors.append(basis@rel_error)
    relative_change=relative_errors[1]-relative_errors[0]
    predicted_relative_change=basis@(integrals['vehicle_ff_error']-integrals['own_velocity_error'])
    assert all(sha(path)==digest for path,digest in before.items())
    result={'run':str(run),'version':task['version'],'policy':task['policy'],'seed':task['seed'],
            'start_s':start,'first_contact_s':contact_time,'budget_end_s':end,
            'precontact_only':bool(task.get('precontact_only')),'duration_s':end-start,
            'sensor_steps':len(output_rows),'terminal_flow_usable_steps':sum(r['flow_fusion_usable'] for r in output_rows),
            'initial_own_velocity_error_along_cross_m_s':(basis@initial_error)[:2].tolist(),
            'final_own_velocity_error_along_cross_m_s':(basis@final_error)[:2].tolist(),
            'own_velocity_error_delta_along_cross_m_s':(basis@total)[:2].tolist(),
            'component_integrals_along_cross':{k:(basis@v)[:2].tolist() for k,v in integrals.items()},
            'velocity_budget_closure_m_s':closure.tolist(),
            'max_non_imu_horizontal_velocity_update_m_s':max_step_error,
            'relative_error_change_along_cross_m':relative_change[:2].tolist(),
            'ff_minus_own_predicted_relative_error_change_along_cross_m':predicted_relative_change[:2].tolist(),
            'relative_budget_residual_along_cross_m':(relative_change-predicted_relative_change)[:2].tolist(),
            'contact_error_m':evaluation['score']['distance_m'],'inputs_sha256':before,'inputs_unchanged':True}
    return result,output_rows


def main(args):
    args.out.mkdir(parents=True,exist_ok=False)
    plan=read(args.series/'plan.json')
    tasks=[{'run':c['run'],'version':'v23','policy':c['policy'],'seed':c['seed']} for c in plan['cases']]
    tasks.extend({'run':run,'version':'v22','policy':'coordinated','seed':int(seed)}
                 for seed,run in plan['intervention_comparator_runs'].items())
    for task in tasks:task['precontact_only']=args.precontact_only
    with ProcessPoolExecutor(max_workers=min(args.workers,4),mp_context=multiprocessing.get_context('spawn')) as pool:
        results=list(pool.map(case,tasks))
    records=[]
    for record,data in results:
        path=args.out/f"{record['version']}-{record['policy']}-{record['seed']}.csv"
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=data[0].keys());writer.writeheader();writer.writerows(data)
        record['csv_sha256']=sha(path);records.append(record)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts=[f.name for f in font_manager.fontManager.ttflist if 'Noto Sans CJK' in f.name]
    if fonts:plt.rcParams['font.family']=fonts[0]
    fig,axs=plt.subplots(2,3,figsize=(15,8),layout='constrained')
    for ax,(record,data) in zip(axs.flat,results):
        t=np.array([x['since_terminal_s'] for x in data])
        initial=record['initial_own_velocity_error_along_cross_m_s'][1]
        ax.plot(t,1000*(np.array([x['own_cross_velocity_error_m_s'] for x in data])-initial),label='기체 속도오차 변화')
        ax.plot(t,1000*np.array([x['rotation_cumulative_cross_delta_v_m_s'] for x in data]),label='회전 대입 민감도')
        ax.plot(t,1000*np.array([x['bias_cumulative_cross_delta_v_m_s'] for x in data]),label='기존 잔차 차감')
        ax.plot(t,1000*np.array([x['other_cumulative_cross_delta_v_m_s'] for x in data]),label='평균/잡음/동역학 잔여')
        ax.set(title=f"{record['version']} {record['policy']} {record['seed']}",xlabel='말기 하강 시작 후 (s)',ylabel='횡방향 속도 변화 (mm/s)')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('말기 기체 속도오차 예산 | 완료된 재사용 training 6경로\n회전 대입은 사후 민감도이며 원인 분리/가상 비행/실기 검증이 아님')
    figure=args.out/'terminal-imu-cross-budget.png';fig.savefig(figure,dpi=150);plt.close(fig)
    result={'created_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'scope':'readonly six completed spent-training paths; no new flights or live truth input',
            'measurement':'100Hz sensor update/right-end acceleration; 50Hz actual velocity/attitude interpolated only offline; along/cross fixed terminal actual heading',
            'causal_limits':['logged body force averages previous1kHz window; substituted current actual attitude is not that averaged orientation',
                             'rotation surrogate includes heading alignment; residual includes averaging/noise/dynamics/sampling',
                             'closed-loop dependent terms are not independent cause percentages; no policy efficacy claim'],
            'source_plan_sha256':sha(args.series/'plan.json'),'records':records,
            'precontact_only':args.precontact_only,
            'analyzer_sha256':sha(__file__),'figure_sha256':sha(figure),
            'qualified_successes':0,'full_trial_denominator_increment':0}
    (args.out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    shutil.copy2(__file__,args.out/Path(__file__).name)
    print(json.dumps({'completed_paths':len(records),'new_flights':0,'all_inputs_unchanged':True,
                      'maximum_relative_closure_residual_m':max(max(abs(x) for x in r['relative_budget_residual_along_cross_m']) for r in records)}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--series',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=4)
    p.add_argument('--precontact-only',action='store_true',help='최초 접촉 이전 마지막50Hz truth 시각까지만 예산 계산')
    main(p.parse_args())
