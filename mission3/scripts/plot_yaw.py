#!/usr/bin/env python3
"""Post-flight figures, run with build/yaw-analysis-venv (no ROS imports)."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot(out):
    aligned = json.loads((out / "aligned.json").read_text())
    if not aligned:
        return
    report = json.loads((out / "evaluation.json").read_text())
    start = aligned[0]["monotonic_s"]
    t = np.array([r["monotonic_s"]-start for r in aligned])
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.7), sharex=True, constrained_layout=True)
    axes[0].plot(t, [r["truth_yaw_deg"] for r in aligned], color="#222222", label="Actual heading (evaluation only)", lw=2)
    axes[0].plot(t, [r["yaw_est_deg"] for r in aligned], color="#1570a6", label="Camera + gyro estimate", lw=1.4)
    axes[0].plot(t, [r["imu_only_deg"] for r in aligned], color="#bd6e15", label="Gyro-only estimate, same data", lw=1.2, ls="--")
    axes[0].axhline(report["target_visual_yaw_deg"], color="#417348", label="Initial target", ls=":")
    axes[0].set_ylabel("Yaw (deg)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title("Static ArUco yaw correction | firmware + ROS + Flow | simulation")
    axes[1].plot(t, [r["ros_rate_deg_s"] for r in aligned], color="#74439a", lw=1.3)
    axes[1].set_ylabel("ROS yaw rate\n(deg/s)")
    truth = list(csv.DictReader((out / "truth.csv").open()))
    stamps = np.array([float(r["monotonic_s"])-start for r in truth])
    mask = (stamps >= t[0]) & (stamps <= t[-1])
    for name, color in (("x_m", "#1570a6"), ("y_m", "#bd6e15")):
        values = np.array([float(r[name]) for r in truth])
        axes[2].plot(stamps[mask], 100*values[mask], label=name[:1].upper(), color=color)
    axes[2].set_ylabel("Position (cm)")
    axes[2].set_xlabel("Time since hover (s)")
    axes[2].legend(loc="upper right", fontsize=8)
    events = json.loads((out / "control-summary.json").read_text())["events"]
    for event in events:
        if event["event"] == "phase" and event["phase"] in ("SIGN_POS", "SIGN_NEG", "HOLD"):
            at = event["monotonic_s"]-start
            for ax in axes:
                ax.axvline(at, color=".65", lw=.7, ls=":")
            axes[1].text(at+.1, 8, event["phase"].replace("_", " "), fontsize=7, color=".3")
    for ax in axes:
        ax.grid(alpha=.2)
        ax.spines[["top", "right"]].set_visible(False)
    fig.savefig(out / "yaw-result.png", dpi=160)
    fig.savefig(out / "yaw-result.svg")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    plot(parser.parse_args().output)
