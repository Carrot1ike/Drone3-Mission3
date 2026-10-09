#!/usr/bin/env python3
"""Conditional failure geometry figure; no control or source logs altered."""
from pathlib import Path
import json
import hashlib
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE=Path('artifacts/mission3/acquisition/20261009-v17-baseline14-height-bearing-diagnostic-v1')
source=BASE/'episode-provenance.json'
data=json.loads(source.read_text())
rows=[r for r in data['rows'][:6] if r['conditional_points']]
assert [r['seed'] for r in rows]==[557,601,613]
fig,axes=plt.subplots(1,2,figsize=(13,5.3),constrained_layout=True)
x=np.arange(3);width=.24
for offset,key,label,color in ((-width,'relative_median_error_m','Relative marker error (offline)','#1d7f8c'),
 (0,'world_median_error_m','World error with existing own pose (offline)','#c95c37'),
 (width,'middle_sensitivity_std_m_S','Relative sensitivity std (S; not calibrated)','#8796ac')):
 values=np.array([r[key]*100 for r in rows])
 bars=axes[0].bar(x+offset,values,width,color=color,label=label)
 axes[0].bar_label(bars,labels=[f'{v:.1f}' for v in values],fontsize=9,padding=3)
axes[0].set_xticks(x,[f"seed {r['seed']}\n{r['conditional_points']} decoded points" for r in rows])
axes[0].set_ylabel('cm');axes[0].set_ylim(0,92)
axes[0].set_title('Useful relative information; unresolved world pose',fontsize=12)
axes[0].legend(fontsize=8,loc='upper left');axes[0].grid(axis='y',alpha=.2);axes[0].set_axisbelow(True)
remaining=np.array([300-r['first_eligible_after_handoff_s'] for r in rows])
duration=np.array([r['longest_gap03_episode_s'] for r in rows])
for offset,values,label,color in ((-width/2,remaining,'Remaining to original 300 s search limit','#d4a543'),
 (width/2,duration,'Longest decoded episode (gap <= 0.3 s, S)','#1d7f8c')):
 bars=axes[1].bar(x+offset,values,width,color=color,label=label)
 axes[1].bar_label(bars,labels=[f'{v:.2f}' for v in values],fontsize=9,padding=3)
axes[1].set_xticks(x,[f"seed {r['seed']}\nfirst at {r['first_eligible_after_handoff_s']:.2f} s" for r in rows])
axes[1].set_ylim(0,24);axes[1].set_ylabel('Duration (s)')
axes[1].set_title('First usable observation is late and brief',fontsize=12)
axes[1].legend(fontsize=8,loc='upper left');axes[1].grid(axis='y',alpha=.2);axes[1].set_axisbelow(True)
fig.suptitle('Saved-input height/bearing diagnostic: original failure cases',fontsize=15)
fig.supxlabel('Full baseline series: 14 flights. Other failed seeds 521 / 569 / 617 decoded 0 markers.\nNative mono320; original pose gates retained; no new flight or controller adoption.',fontsize=10)
target=BASE/'relative-geometry-and-late-observation-v2.png'
assert not target.exists();fig.savefig(target,dpi=150);plt.close(fig)
shutil.copyfile(__file__,BASE/'plot_mission3_height_bearing_diagnostic-v2.py')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
with (BASE/'figure-provenance-v2.json').open('x') as out:
 out.write(json.dumps({'source':str(source),'source_sha256':sha(source),'figure_sha256':sha(target),
 'helper_sha256':sha(__file__),'all14_denominator':14,'three_failure_cases_without_decodes':[521,569,617],
 'no_live_adoption':True,'standard_deviation_scope':'S independent pixel/height/tilt perturbations, shared calibration/world correlations excluded'},indent=2)+'\n')
print(target)
