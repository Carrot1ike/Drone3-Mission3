#!/usr/bin/env python3
"""Independent timestamp perturbations; no plant or live input connection."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np


def audit(module_path, output):
    spec = importlib.util.spec_from_file_location('audited_vertical_fit', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ranges_t = np.arange(6) * .01
    measurements = list(zip(ranges_t, .7 - .02 * ranges_t))
    imu_t = np.arange(7) * .01
    outcomes = []
    for future_acceleration in (0., 1.):
        acceleration = np.zeros(7)
        acceleration[-1] = future_acceleration
        try:
            value = module.instantaneous_range_velocity(
                measurements, list(zip(imu_t, acceleration)), .055)
            outcomes.append({'future_acceleration_m_s2': future_acceleration,
                             'rejected': False, 'fit_m_s': value})
        except ValueError as error:
            outcomes.append({'future_acceleration_m_s2': future_acceleration,
                             'rejected': True, 'reason': str(error)})
    exact_now = module.instantaneous_range_velocity(
        measurements, list(zip(imu_t, np.zeros(7))), .06)
    result = {'scope': 'pure synthetic API chronology regression, not flight or hardware approval',
              'module': str(module_path),
              'source_sha256': hashlib.sha256(module_path.read_bytes()).hexdigest(),
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'now_s': .055, 'last_range_s': float(ranges_t[-1]),
              'last_imu_s': float(imu_t[-1]), 'outcomes': outcomes,
              'all_future_imu_rejected': all(x['rejected'] for x in outcomes),
              'exact_current_sample_velocity_m_s': exact_now,
              'does_not_validate_entire_range_fusion_or_external_adapter': True}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--module', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    audit(args.module, args.out)
