#!/usr/bin/env python3
"""Offline-only command limits and pre-discovery movement audit."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def audit(run):
    run=Path(run)
    rows=lambda name:[json.loads(line) for line in (run/name).open()]
    commands=rows('commands.jsonl');truth=rows('truth.jsonl')
    cfg=json.loads((run/'controller-settings.json').read_text());limits=cfg['control']
    t=np.array([row['t'] for row in commands]);v=np.array([row['velocity_world_m_s'] for row in commands])
    yaw=np.array([row['yaw_rate_rad_s'] for row in commands])
    dt=np.diff(t);acc=np.linalg.norm(np.diff(v,axis=0),axis=1)/dt
    yaw_acc=np.abs(np.diff(yaw))/dt*180/math.pi
    # Abrupt ABORT/motor-off and reversal of descent on evidence loss are
    # deliberate safety overrides. List them, never silently erase samples.
    exceptions=[];normal=np.ones(len(acc),dtype=bool)
    for i in range(1,len(commands)):
        if commands[i]['phase'] in ('ABORT','MOTOR_OFF_HOLD','BLIND_RECOVER','BLIND_RISE','GEOMETRY_READY'):
            if acc[i-1]>limits['max_accel_m_s2']+1e-6 or yaw_acc[i-1]>limits.get('max_yaw_accel_deg_s2',float('inf'))+1e-6:
                normal[i-1]=False
                exceptions.append({'t':float(t[i]),'phase':commands[i]['phase'],
                                   'command_accel_m_s2':float(acc[i-1]),'command_yaw_accel_deg_s2':float(yaw_acc[i-1])})
    discoveries=[row['t'] for row in commands if row.get('vehicle') is not None]
    first=min(discoveries) if discoveries else None
    pre=[row for row in truth if first is None or row['t']<first]
    entry=next(e for e in cfg['entry']['positions'] if e['id']==json.loads((run/'run_manifest.json').read_text())['entry_id'])
    offsets=np.array([row['aircraft']['center_world_m'][:2] for row in pre])-entry['xy_m']
    actual_v=np.array([row['aircraft']['velocity_world_m_s'] for row in truth])
    phases=list(dict.fromkeys(row['phase'] for row in commands))
    result={'scope':'offline audit; actual pose is evaluation only, never controller input',
        'limits_are':'provisional software trial settings, not manufacturer maximum or hardware proof',
        'command_limits':{key:limits[key] for key in ('max_speed_m_s','max_accel_m_s2','max_yaw_rate_deg_s','max_yaw_accel_deg_s2','max_climb_speed_m_s','max_descent_speed_m_s')},
        'max_command_speed_m_s':float(np.linalg.norm(v,axis=1).max()),
        'max_command_climb_m_s':float(max(0.,v[:,2].max())),
        'max_command_descent_m_s':float(max(0.,-v[:,2].min())),
        'max_command_yaw_deg_s':float(np.abs(yaw).max()*180/math.pi),
        'max_command_accel_all_m_s2':float(acc.max()),
        'max_command_accel_normal_m_s2':float(acc[normal].max()) if normal.any() else None,
        'max_command_yaw_accel_all_deg_s2':float(yaw_acc.max()),
        'max_command_yaw_accel_normal_deg_s2':float(yaw_acc[normal].max()) if normal.any() else None,
        'explicit_safety_overrides':exceptions,
        'max_actual_speed_m_s':float(np.linalg.norm(actual_v,axis=1).max()),
        'max_actual_horizontal_speed_m_s':float(np.linalg.norm(actual_v[:,:2],axis=1).max()),
        'first_image_guidance_time_s':first,'pre_image_max_actual_entry_xy_drift_m':float(np.linalg.norm(offsets,axis=1).max()),
        'phases':phases,'unobserved_track_station_phase_present':any(p in phases for p in ('EXPLORE','WATCH','PATROL'))}
    result['command_envelope_pass']=bool(result['max_command_speed_m_s']<=limits['max_speed_m_s']+1e-6
        and result['max_command_climb_m_s']<=limits['max_climb_speed_m_s']+1e-6
        and result['max_command_descent_m_s']<=limits['max_descent_speed_m_s']+1e-6
        and result['max_command_yaw_deg_s']<=limits['max_yaw_rate_deg_s']+1e-6
        and result['max_command_accel_normal_m_s2']<=limits['max_accel_m_s2']+1e-6
        and result['max_command_yaw_accel_normal_deg_s2']<=limits['max_yaw_accel_deg_s2']+1e-6)
    target=run/'command-limits-audit.json'
    if target.exists():raise RuntimeError('existing audit is preserved')
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    audit(parser.parse_args().run)
