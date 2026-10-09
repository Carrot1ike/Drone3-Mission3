#!/usr/bin/env python3
"""수평 경로의 표면 전환과 정지 지면으로 오해한 측정의 바이어스."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot(path):
    rows = list(csv.DictReader((path / "trace.csv").open()))
    value = lambda key: np.array([float(r[key]) for r in rows])
    t = value("time_s")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    axes[0].plot(t, value("world_z_m"), label="Optical world Z")
    axes[0].plot(t, value("virtual_range_m"), label="Virtual downward range")
    axes[0].plot(t, value("height_above_pad_m"), ls="--", label="Optical height above pad")
    axes[0].set_ylabel("Height [m]"); axes[0].legend()
    axes[1].plot(t, value("camera_vx_m_s"), label="Actual optical VX")
    axes[1].plot(t, value("naive_static_ground_vx_m_s"), label="Naive stationary-surface interpretation")
    axes[1].set_ylabel("VX [m/s]"); axes[1].legend()
    axes[2].step(t, [int(r["surface"] == "landing_pad") for r in rows], where="post")
    axes[2].set_yticks([0, 1], ["floor", "moving pad"])
    axes[2].set_ylabel("Virtual ray hit"); axes[2].set_xlabel("Controller elapsed time [s]")
    summary = json.loads((path / "summary.json").read_text())
    for event in summary["transitions"]:
        if event["from_surface"] is not None:
            for ax in axes: ax.axvline(event["time_s"], color="gray", alpha=.3)
    for ax in axes: ax.grid(alpha=.2)
    fig.suptitle("Mission surface replay: ground / moving pad transitions\nIdeal virtual probe, translation only; no Flow deck or firmware estimator in loop", fontsize=12)
    fig.savefig(path / "height-flow-trace.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    plot(args.path)
