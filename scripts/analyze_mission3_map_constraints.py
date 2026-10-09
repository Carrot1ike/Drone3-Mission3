#!/usr/bin/env python3
"""완료된 raw ArUco 관측의 정적 지도 제약 진단. 추정/제어 변경 없음.

참값은 모든 영상/own-odometry 진단이 끝난 뒤 오프라인 association
평가만 한다. 카메라 raw 상대값과 own yaw가 공유되는 구조를 유지하며,
누락된 pose/yaw cross covariance 때문에 posterior를 계산하지 않는다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from mission3.map_constraint_geometry import circle_constraint, straight_constraint
from mission3.track import Stadium, rotation, wrap


def analyze(run):
    cfg = json.loads((run/'experiment.json').read_text())['mission']
    track = Stadium.from_config(cfg['track'])
    rows = [json.loads(s) for s in (run/'observations.jsonl').read_text().splitlines()]
    results = []
    for row in rows:
        if not row.get('valid') or 'raw_relative_pad_world_m' not in row:
            continue
        relative = np.array(row['raw_relative_pad_world_m'])
        # Undo only the estimated capture yaw. Raw image tvec, IMU roll/pitch,
        # mount and known marker-pad offset remain the original sensor fit.
        beta = -math.radians(row['yaw_deg'])
        yaw = wrap(row['raw_vehicle_heading_rad']-beta)
        pose = np.r_[np.array(row['raw_pad_world_m'])[:2]-relative[:2], yaw]
        q = (rotation(yaw).T@relative)[:2]
        pad = pose[:2]+(rotation(yaw)@np.r_[q, 0.])[:2]
        nearest, turn = track.nearest(pad)
        if turn:
            center = np.array([track.center[0]+math.copysign(track.straight/2, pad[0]-track.center[0]), track.center[1]])
            angle = math.atan2(*(pad-center)[::-1])
            direction = min((-1, 1), key=lambda d:abs(wrap(yaw+beta-angle-d*math.pi/2)))
            residual, H, null = circle_constraint(pose, q, beta, center, track.radius, direction)
            segment = 'right_curve' if center[0]>track.center[0] else 'left_curve'
        else:
            tangent = min((0., math.pi), key=lambda a:abs(wrap(yaw+beta-a)))
            residual, H, null = straight_constraint(pose, q, beta, nearest[1], tangent)
            segment = 'lower_straight' if nearest[1]<track.center[1] else 'upper_straight'
        results.append({'capture_s':row['capture_stamp_s'], 'receipt_s':row['receipt_s'],
            'raw_capture_own_pose':pose.tolist(), 'raw_relative_xy_heading_frame_m':q.tolist(),
            'raw_relative_vehicle_heading_rad':beta, 'candidate_segment':segment,
            'cross_residual_m':float(residual[0]), 'heading_residual_deg':math.degrees(residual[1]),
            'jacobian_rank':int(np.linalg.matrix_rank(H)), 'jacobian':H.tolist(),
            'unobserved_pose_direction':null.tolist(), 'null_residual_norm':float(np.linalg.norm(H@null)),
            'existing_innovation_gates_only':bool(abs(residual[0])<=cfg['observer']['max_track_cross_innovation_m']
                and abs(residual[1])<=math.radians(20.))})
    return results


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    plan={'run':str(a.run.resolve()),'scope':'reused failed training; single-frame observation geometry only, no state/covariance/control update, no truth input',
        'input_sha256':{n:hashlib.sha256((a.run/n).read_bytes()).hexdigest() for n in ('observations.jsonl','experiment.json')},
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'geometry_sha256':hashlib.sha256((Path(__file__).parent/'mission3/map_constraint_geometry.py').read_bytes()).hexdigest(),
        'measurement_noise_and_pose_cross_covariance':'unresolved, no posterior or calibrated confidence calculated'}
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    records=analyze(a.run)
    (a.out/'observation-geometry.json').write_text(json.dumps(records,indent=2)+'\n')
    # Offline evaluation starts only after every raw observation is analyzed.
    truth=[json.loads(s) for s in (a.run/'truth.jsonl').read_text().splitlines()]
    times=np.array([r['t'] for r in truth]);turn_agreement=0;assessment=[]
    for row in records:
        r=truth[min(np.searchsorted(times,row['capture_s']),len(truth)-1)]
        actual_turn=bool(r['vehicle']['in_turn'])
        predicted_turn='curve' in row['candidate_segment']
        turn_agreement+=actual_turn==predicted_turn
        assessment.append({'capture_s':row['capture_s'],'offline_actual_turn':actual_turn,
            'candidate_curve':predicted_turn,'offline_actual_heading_rad':r['vehicle']['heading_rad']})
    (a.out/'offline-association-evaluation.json').write_text(json.dumps(assessment,indent=2)+'\n')
    by_segment={}
    for row in records:
        by_segment[row['candidate_segment']]=by_segment.get(row['candidate_segment'],0)+1
    summary={'plan':plan,'valid_raw_observations':len(records),'candidate_segment_counts':by_segment,
        'innovation_gate_candidates':sum(r['existing_innovation_gates_only'] for r in records),
        'jacobian_ranks':sorted(set(r['jacobian_rank'] for r in records)),
        'maximum_null_residual_norm':max((r['null_residual_norm'] for r in records),default=None),
        'offline_curve_straight_agreement':turn_agreement,
        'decision':'No online adoption. Full own-pose covariance cross terms, delay propagation, raw pose/IMU correlations, association uncertainty and temporal gauge consistency required before EKF fusion. Instantaneous rank2 never establishes three-variable absolute pose or gyro bias.'}
    (a.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
