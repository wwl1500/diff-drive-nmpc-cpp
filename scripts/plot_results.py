#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def column(rows, name):
    return [float(row[name]) for row in rows]


def main():
    output_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results")
    trajectory_rows = read_csv(output_dir / "trajectory.csv")
    control_rows = read_csv(output_dir / "control.csv")
    error_rows = read_csv(output_dir / "error.csv")

    plt.figure(figsize=(7, 7))
    plt.plot(column(trajectory_rows, "x_ref"), column(trajectory_rows, "y_ref"), "--", label="reference")
    plt.plot(column(trajectory_rows, "x"), column(trajectory_rows, "y"), label="actual")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory tracking")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "trajectory_plot.png", dpi=150)

    error_time = column(error_rows, "time")
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(error_time, column(error_rows, "position_error"))
    axes[0].set_ylabel("position error [m]")
    axes[0].grid(True)
    axes[1].plot(error_time, column(error_rows, "etheta"))
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("theta error [rad]")
    axes[1].grid(True)
    fig.suptitle("Tracking error")
    fig.tight_layout()
    fig.savefig(output_dir / "error_plot.png", dpi=150)

    control_time = column(control_rows, "time")
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(control_time, column(control_rows, "v"))
    axes[0].set_ylabel("v [m/s]")
    axes[0].grid(True)
    axes[1].plot(control_time, column(control_rows, "omega"))
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("omega [rad/s]")
    axes[1].grid(True)
    fig.suptitle("Control input")
    fig.tight_layout()
    fig.savefig(output_dir / "control_plot.png", dpi=150)

    print(f"Plots saved to {output_dir}")


if __name__ == "__main__":
    main()
