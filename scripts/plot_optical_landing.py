#!/usr/bin/env python3
"""Scientific plots for optical kinematics and FF comparison, evaluation only."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def rows(path):
    return list(csv.DictReader((path / "truth.csv").open()))


def plot(path, comparison=None):
    data = rows(path)
    if "monotonic_s" in data[0]:
        commands = [json.loads(line) for line in (path / "commands.jsonl").read_text().splitlines()]
        origin = np.median([r["monotonic_s"]-r["time_s"] for r in commands])
        t = np.array([float(row["monotonic_s"])-origin for row in data])
    else:
        t = np.array([float(row["time_s"]) for row in data])
    values = lambda name: np.array([float(row[name]) for row in data])
    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True, layout="constrained")
    axes[0].plot(t, values("pad_x_m"), label="Vehicle pad X")
    axes[0].plot(t, values("camera_x_m"), label="Controlled optical point X")
    axes[0].set_ylabel("Position [m]"); axes[0].legend()
    axes[1].plot(t, np.hypot(values("relative_x_m"), values("relative_y_m")), label="FF + feedback")
    axes[1].axhline(.07, color="gray", ls="--", label="Internal 7cm check")
    if comparison:
        other = rows(comparison)
        axes[1].plot([float(row["time_s"]) for row in other], [np.hypot(float(row["relative_x_m"]),float(row["relative_y_m"])) for row in other], label="Feedback only")
    axes[1].set_ylabel("Center error [m]"); axes[1].legend()
    axes[2].plot(t, values("height_above_pad_m"), label="Optical height")
    axes[2].axhline(.03, color="gray", ls="--", label="Virtual terminal height")
    axes[2].set_ylabel("Height [m]"); axes[2].legend()
    axes[3].plot(t, values("relative_speed_m_s"), label="Relative XY speed")
    yaw_axis = axes[3].twinx()
    yaw_axis.plot(t, values("heading_deg"), color="#bd6e15", label="Optical heading", alpha=.8)
    yaw_axis.set_ylabel("Heading [deg]")
    axes[3].set_ylabel("Speed [m/s]"); axes[3].set_xlabel("Elapsed time [s]")
    lines, labels = axes[3].get_legend_handles_labels()
    extra_lines, extra_labels = yaw_axis.get_legend_handles_labels()
    axes[3].legend(lines+extra_lines, labels+extra_labels)
    summary_file = path / "summary.json"
    summary = json.loads(summary_file.read_text()) if summary_file.exists() else json.loads((path / "control-summary.json").read_text())
    for event in summary["events"]:
        for ax in axes:
            ax.axvline(event["time_s"], color="gray", alpha=.25)
        if event["phase"] != "SETTLE":
            axes[0].annotate(event["phase"], xy=(event["time_s"], .98), xycoords=("data", "axes fraction"), rotation=90, va="top", fontsize=8)
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle("Image-driven optical-point pursuit and descent\nIdeal kinematics; no aircraft dynamics or physical touchdown", fontsize=12)
    fig.savefig(path / "result.png", dpi=160)
    fig.savefig(path / "result.svg")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--comparison", type=Path)
    args = parser.parse_args()
    plot(args.path, args.comparison)
