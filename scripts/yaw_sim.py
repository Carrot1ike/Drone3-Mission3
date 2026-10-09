#!/usr/bin/env python3
"""Pinned CrazySim dynamics with evaluation-only truth logging, no pose injection."""
import csv
import os
from pathlib import Path
import time

from yaw_scene import load_upstream

sim = load_upstream()
out = Path(os.environ["DRONE3_EXPERIMENT_DIR"])
truth_file = (out / "truth.csv").open("w", buffering=1)
writer = csv.writer(truth_file)
writer.writerow(["monotonic_s", "sim_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"])


class EvaluationAgent(sim.DroneAgent):
    def send_sensor_data(self):
        super().send_sensor_data()
        # This file is consumed after shutdown by evaluate_yaw.py only.
        if self.data.time >= getattr(self, "_next_evaluation", 0.):
            pos, quat = self.read_pose()
            writer.writerow([time.monotonic(), self.data.time, *pos, *quat])
            self._next_evaluation = self.data.time + .02


sim.DroneAgent = EvaluationAgent
try:
    sim.main()
finally:
    truth_file.close()
