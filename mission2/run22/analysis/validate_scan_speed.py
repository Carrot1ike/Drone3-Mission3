import json
from pathlib import Path
BASE=Path(__file__).parent;ROOT=BASE.parents[2];RUNS=ROOT/'.local/evaluation/fresh_mission2'
results={}
for name in ('run21','run22'):
 run=RUNS/name;summary=json.loads((run/'host/summary.json').read_text())
 decisions=[json.loads(l) for l in (run/'host/decisions.jsonl').read_text().splitlines()]
 commands=[json.loads(l) for l in (run/'host/commands.jsonl').read_text().splitlines()]
 scans=[]
 transitions=summary['transitions']
 for i,start in enumerate(transitions[:-1]):
  if start['to_stage'] not in ('SCAN','WALL_SCAN','EXIT_SCAN'):continue
  end=transitions[i+1];stage=start['to_stage']
  ds=[d for d in decisions if d['decision']['stage_before']==stage and start['time']<=d['frame']['received_monotonic']<=end['time']]
  cs=[c for c in commands if start['time']<=c['sent_monotonic']<end['time']]
  field={'SCAN':'ring_scan','WALL_SCAN':'wall_scan','EXIT_SCAN':'exit_scan'}[stage]
  evidence=ds[-1]['decision'][field]
  row=dict(stage=stage,next_stage=end['to_stage'],duration_s=end['time']-start['time'],decision_count=len(ds),actual_command_count=len(cs),max_abs_vx=max(abs(c['command']['vx']) for c in cs),max_abs_vy=max(abs(c['command']['vy']) for c in cs),max_abs_yawrate=max(abs(c['command']['yawrate']) for c in cs),all_ring_pass_request_zero=all(c['command'].get('ring_pass_mask',0)==0 for c in cs),all_ring_pass_ack_zero=all(c.get('ring_pass_ack',0)==0 for c in cs),visited=evidence['visited'],selected_side=evidence.get('selected_side'))
  assert row['max_abs_vx']==row['max_abs_vy']==0
  assert row['all_ring_pass_request_zero'] and row['all_ring_pass_ack_zero']
  assert [v['goal_deg'] for v in row['visited']]==[45.,-45.,0.]
  assert all(abs(v['goal_deg']-v['observed_deg'])<2. for v in row['visited'])
  scans.append(row)
 commits=[]
 for d in decisions:
  if d['stage'] not in ('STRAIGHT','WALL_STRAIGHT','BLOCK_STRAIGHT','EXIT_STRAIGHT') or d['stage']==d['decision']['stage_before']:continue
  c=d['decision']['committed'];depth=c['depth'] if c.get('kind')=='ring' else c['rear_depth']
  margin=c['distance']-depth-.03
  commits.append(dict(stage=d['stage'],kind=c['kind'],depth_m=depth,distance_m=c['distance'],inferred_clearance_margin_m=margin))
  assert abs(margin-(.2 if name=='run22' else .45))<1e-9
 results[name]=dict(scans=scans,straight_commits=commits,result=json.loads((run/'result_summary.json').read_text()))
 if name=='run22':assert len(scans)==4 and len(commits)==7
(BASE/'scan_speed_validation.json').write_text(json.dumps(results,indent=2,ensure_ascii=False))
(RUNS/'run22/scan_speed_validation.json').write_text(json.dumps(results['run22'],indent=2,ensure_ascii=False))
for name,r in results.items():
 print(name,[(s['stage'],round(s['duration_s'],3),s['selected_side']) for s in r['scans']])
 print('course',len(r['result']['ordered_sections']),'contacts',r['result']['nonfloor_contacts'],'duration',r['result']['duration_s'],'landing',r['result']['landing']['passed'])
