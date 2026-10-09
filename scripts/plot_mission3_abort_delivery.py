#!/usr/bin/env python3
"""Offline failed-flight command/estimate/physics/rotor-force diagnostic."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def rows(path, start, end):
    with path.open() as f:
        for line in f:
            r=json.loads(line)
            if r['t']<start:continue
            if r['t']>end:break
            yield r


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('audit',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
    audit=json.loads(a.audit.read_text());cases=[r for r in audit['cases'] if r['original_failure'] is not None]
    fig,axes=plt.subplots(3,len(cases),figsize=(12,8),sharex='col',squeeze=False)
    evidence={str(a.audit):sha(a.audit)}
    for column,case in enumerate(cases):
        start=case['after_failure_offline_only']['first_abort_s'];lo,hi=start-1.,start+3.1
        for label,path,color,style in [('original',case['reference_run'],'#787d8b','--'),('delivery fix',case['run'],'#1c8b80','-')]:
            folder=Path(path);commands=list(rows(folder/'commands.jsonl',lo,hi));truth=list(rows(folder/'truth.jsonl',lo,hi))
            t=np.array([r['t']-start for r in commands]);estimated=[np.linalg.norm(r['own']['velocity_world_m_s']) for r in commands]
            axes[0,column].plot(t,estimated,label=label+' estimated',color=color,ls=style)
            axes[0,column].plot([r['t']-start for r in truth],[np.linalg.norm(r['aircraft']['velocity_world_m_s']) for r in truth],label=label+' actual (offline)',color=color,ls=':' if label=='original' else '-.',alpha=.9)
            axes[1,column].plot(t,[np.linalg.norm(r['velocity_world_m_s']) for r in commands],label=label+' requested velocity',color=color,ls=style)
            mt,force,pwm=[],[],[]
            for i,r in enumerate(rows(folder/'motors.jsonl',lo,hi)):
                if i%10:continue
                mt.append(r['t']-start);force.append(sum(r['motor_static_force_N']));pwm.append(sum(r['motor_output']))
            axes[2,column].plot(mt,force,label=label+' static force target',color=color,ls=style)
            axes[2,column].plot([r['t']-start for r in truth],[sum(r['motor_actual_force_N']) for r in truth],label=label+' rotor force (offline)',color=color,ls=':' if label=='original' else '-.',alpha=.9)
            for name in ['commands.jsonl','truth.jsonl','motors.jsonl','adapter-summary.json']:evidence[str(folder/name)]=sha(folder/name)
        axes[0,column].set_title(case['policy']+' / seed '+str(case['seed'])+'\n'+case['original_failure'],fontsize=10)
        axes[0,column].axhline(.05,color='#c37b12',lw=1,ls='--',label='sensor speed criterion (S)')
        for row,label in enumerate(['3D speed (m/s)','requested 3D speed (m/s)','sum motor force (N)']):
            ax=axes[row,column];ax.set_ylabel(label);ax.axvline(0,color='black',lw=1);ax.grid(alpha=.2);ax.legend(fontsize=7)
        axes[2,column].set_xlabel('seconds from original ABORT')
    fig.suptitle('Failed-flight delivery: original termination versus continued zero-velocity control\nConditional simulation only; airborne motors remain active; no landing or safe-recovery approval',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.94));image=a.out/'abort-delivery-comparison.png';fig.savefig(image,dpi=160);plt.close(fig)
    provenance={'scope':'offline diagnostic from all negative cases of the predeclared regression; no control truth input',
                'source_sha256':evidence,'script_sha256':sha(Path(__file__)),'image_sha256':sha(image),
                'cases':len(cases),'source_model_limits':'synthetic sensor quality, assembled dynamics/contact/force/timing S/U; PWM/static force/rotor lag differ; no real safety proof',
                'generated_image_path':str(image.resolve())}
    (a.out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n');print(image.resolve())


if __name__=='__main__':main()
