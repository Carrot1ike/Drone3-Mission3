#!/usr/bin/env python3
"""Offline actual/estimated terminal heights; distinct flights, no oracle fix."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(x) for x in Path(path).open()]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--before',type=Path,required=True)
    parser.add_argument('--after',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(__file__,args.out/Path(__file__).name)
    fig,axes=plt.subplots(2,1,figsize=(10,7),layout='constrained')
    records=[]
    for ax,run,title in zip(axes,(args.before,args.after),('v13 / linear 0.4 s / failed motor-off','v14 / IMU compensation 1.2 s / conditional contact pass')):
        sensor=rows(run/'sensors.jsonl');truth=rows(run/'truth.jsonl');command=rows(run/'commands.jsonl')
        summary=json.loads((run/'adapter-summary.json').read_text())
        evaluation=json.loads((run/'offline-evaluation.json').read_text())
        contact=evaluation['score']['time_s']
        end=summary['motor_off_at_s'] or next(x['t'] for x in command if x['phase']=='BLIND_RECOVER')
        chosen=[x for x in sensor if contact-3<=x['t']<=end+1e-8]
        t=np.array([x['t'] for x in chosen]);time=[x['t'] for x in truth]
        actual=np.interp(t,time,[x['aircraft']['center_world_m'][2] for x in truth])-.30
        estimated=np.array([x['estimate']['position_world_m'][2] for x in chosen])-.30
        qualified=[x['t'] for x in sensor if x['t']<=end and x['observer_packet']['range_new']
            and x['estimate']['surface_hypothesis'] in ('static_floor_candidate','moving_surface_candidate')]
        last=max(qualified)
        ax.axhspan(2.5,26.5,color='#bde4cd',alpha=.5,label='S body-height gate 2.5–26.5 mm')
        ax.axvspan(last-contact,end-contact,color='#dddddd',alpha=.35,label='After last qualified range')
        ax.plot(t-contact,estimated*1000,color='#de6b35',lw=2,label='PC estimated body height')
        ax.plot(t-contact,actual*1000,color='#244f91',lw=2,label='Actual body height (offline only)')
        ax.axvline(0,color='#555555',ls=':',label='First physical pad contact (offline)')
        off=summary['motor_off_at_s']
        ax.axvline(end-contact,color='#088474' if off else '#bb3030',ls='--',
                   label='PWM off' if off else 'Prediction expiry / recovery')
        ax.set(title=title,xlabel='Seconds relative to this flight’s first contact',ylabel='Body height above pad (mm)')
        ax.set_ylim(0,90);ax.grid(alpha=.2);ax.legend(fontsize=8,ncol=2,loc='upper right')
        records.append({'run':str(run),'contact_s_OFFLINE':contact,'last_qualified_range_s':last,
                        'motor_off_s':off,'plot_end_s':end,'original_sha256':{n:sha(run/n) for n in
                           ('sensors.jsonl','truth.jsonl','commands.jsonl','adapter-summary.json','offline-evaluation.json')}})
    fig.suptitle('Same seed 571, two separate feedback flights — conditional S/U model',fontsize=13)
    fig.savefig(args.out/'terminal-height-comparison.png',dpi=170)
    plt.close(fig)
    report={'scope':'Offline visualization only; new policy changes the trajectory and sensor stream, not the original saved failure.',
            'gate_source':'original profile .0145 +/- .012 m, unchanged software assumption',
            'cases':records,'script_sha256':sha(__file__),'figure_sha256':sha(args.out/'terminal-height-comparison.png'),
            'qualified_count':0,'formal_count':0}
    (args.out/'provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(args.out/'terminal-height-comparison.png')


if __name__=='__main__':main()
