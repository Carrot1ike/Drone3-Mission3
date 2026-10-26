import json, math, csv
from pathlib import Path
from bisect import bisect_right
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
RUN=ROOT/'.local/evaluation/fresh_mission2/run22'
OUT=Path(__file__).parent
def rows(path): return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
ds=rows(RUN/'host/decisions.jsonl'); cs=rows(RUN/'host/commands.jsonl')
summary=json.loads((RUN/'host/summary.json').read_text()); start=summary['started_monotonic']
runtime=Path(json.loads((RUN/'runtime/runtime_state/state.json').read_text())['log_dir'])
gt=[r for r in rows(runtime/'evaluation/ground_truth.jsonl') if r.get('event')=='ground_truth']
times=[r['host_monotonic'] for r in gt]
def truth(t):return gt[max(0,bisect_right(times,t)-1)]
examples=[]
for index,d in enumerate(ds):
    before=d['decision']['stage_before']; stage=d['stage']
    if stage not in ('STRAIGHT','BLOCK_STRAIGHT','WALL_STRAIGHT','EXIT_STRAIGHT') or before==stage:continue
    end=next(r for r in ds[index+1:] if r['stage']!=stage)
    t0=d['frame']['received_monotonic'];t1=end['frame']['received_monotonic']
    g0=truth(t0);g1=truth(t1)
    w,x,y,z=g0['drone']['quaternion_wxyz'];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    delta=np.array(g1['drone']['position_world_m'])-np.array(g0['drone']['position_world_m'])
    forward=np.array([math.cos(yaw),math.sin(yaw),0.])
    phase=[r for r in ds[index+1:] if t0<r['frame']['received_monotonic']<t1]
    sent=[r['command'] for r in cs if t0<r['sent_monotonic']<t1 and r['command']['mode']=='hover']
    c=d['decision']['committed']
    item=dict(stage=stage,ring_index=d['ring_index'],wall_index=d['decision'].get('wall_index'),
      commit_elapsed_s=t0-start,end_elapsed_s=t1-start,
      selected_depth_m=c.get('depth'),selected_rear_depth_m=c.get('rear_depth'),
      distance_goal_m=d['straight_distance_m'],
      last_before_transition_integrated_travel_m=phase[-1]['travel_m'] if phase else None,
      observed_gt_forward_displacement_m=float(delta@forward),observed_gt_horizontal_displacement_m=float(np.linalg.norm(delta[:2])),
      displacement_minus_goal_m=float(delta@forward-d['straight_distance_m']),
      median_actual_sent_body_vx_mps=float(np.median([r['vx'] for r in sent])),
      median_actual_sent_body_vy_mps=float(np.median([r['vy'] for r in sent])),
      median_actual_sent_height_m=float(np.median([r['height_m'] for r in sent])),
      method=c.get('method'),selected_side=c.get('side'),rear_observed=c.get('rear_observed'),
      source_frame=d['frame']['image_file'],commit_policy=d['command'],
      gt_sample_age_start_s=t0-g0['host_monotonic'],gt_sample_age_end_s=t1-g1['host_monotonic'])
    examples.append(item)
payload=dict(run='run22',method='Straight phase commit to stage exit; preceding GT sample at each boundary; displacement projected on true heading at commit. Diagnostic only, never controller input. Body command median includes the boundary command; local planned velocity is .7 or .4 m/s.',examples=examples)
(OUT/'command_outcome_examples.json').write_text(json.dumps(payload,indent=2,ensure_ascii=False))
fields=[k for k in examples[0] if k!='commit_policy']
with (OUT/'command_outcome_examples.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(examples)
selected={}
for key,stage in [('ring','ALIGN'),('block','BLOCK_GAP'),('wall','WALL_1'),('exit','EXIT'),('scan','SCAN')]:
    candidates=[d for d in ds if d['stage']==stage and ((d.get('target') and d['target'].get('full')) if stage=='ALIGN' else d.get('passage') if stage in ('BLOCK_GAP','WALL_1','EXIT') else True)]
    if candidates:
      d=candidates[len(candidates)//2];selected[key]=dict(elapsed_s=d['frame']['received_monotonic']-start,decision=d)
(OUT/'selected_observations.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2))
print(json.dumps(payload,ensure_ascii=False,indent=2))
