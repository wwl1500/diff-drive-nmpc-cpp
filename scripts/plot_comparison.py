#!/usr/bin/env python3
"""生成 Pure Pursuit 与 NMPC 的对比图。

用法：
    python3 scripts/plot_comparison.py <pp_dir> <nmpc_dir> [output_dir]

示例：
    python3 scripts/plot_comparison.py results results_nmpc_cpp/circle
"""

import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = {k: [] for k in reader.fieldnames or []}
        for row in reader:
            for k in rows:
                rows[k].append(float(row[k]))
    return {k: np.array(v) for k, v in rows.items()}


def compute_tracking_metrics(result_dir: Path):
    err = load_csv(result_dir / "error.csv")
    position_error = err["position_error"]
    theta_error = err["etheta"]
    abs_theta_error = np.abs(theta_error)
    return {
        "mean_position_error": float(np.mean(position_error)),
        "rms_position_error": float(np.sqrt(np.mean(position_error**2))),
        "max_position_error": float(np.max(position_error)),
        "mean_abs_theta_error": float(np.mean(abs_theta_error)),
        "rms_theta_error": float(np.sqrt(np.mean(theta_error**2))),
        "max_abs_theta_error": float(np.max(abs_theta_error)),
    }


def compute_solve_time_metrics(nmpc_dir: Path):
    data = load_csv(nmpc_dir / "solve_time.csv")
    solve_times = data["solve_time_ms"]
    solve_ok = data["solve_ok"]
    failed = solve_ok < 0.5
    return {
        "solve_time_mean_ms": float(np.mean(solve_times)),
        "solve_time_median_ms": float(np.median(solve_times)),
        "solve_time_p95_ms": float(np.percentile(solve_times, 95)),
        "solve_time_p99_ms": float(np.percentile(solve_times, 99)),
        "solve_time_max_ms": float(np.max(solve_times)),
        "solve_failure_count": int(np.sum(failed)),
        "solve_failure_rate": float(np.mean(failed)),
    }


def plot_solve_time(nmpc_dir: Path, output_dir: Path) -> None:
    """图 4：NMPC 求解时间分析。"""
    data = load_csv(nmpc_dir / "solve_time.csv")
    times = data["time"]
    solve_times = data["solve_time_ms"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))

    # 上图：求解时间折线
    axes[0].plot(times, solve_times, linewidth=0.6, alpha=0.7)
    mean_val = np.mean(solve_times)
    median_val = np.median(solve_times)
    axes[0].axhline(mean_val, color="r", linestyle="--", linewidth=1,
                    label=f"mean = {mean_val:.3f} ms")
    axes[0].axhline(median_val, color="orange", linestyle=":", linewidth=1,
                    label=f"median = {median_val:.3f} ms")
    axes[0].set_ylabel("solve time [ms]")
    axes[0].set_title("NMPC Solve Time vs Time Step")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 下图：直方图
    axes[1].hist(solve_times, bins=50, edgecolor="black", alpha=0.7)
    p95 = np.percentile(solve_times, 95)
    p99 = np.percentile(solve_times, 99)
    axes[1].axvline(p95, color="r", linestyle="--", label=f"p95 = {p95:.3f} ms")
    axes[1].axvline(p99, color="orange", linestyle=":", label=f"p99 = {p99:.3f} ms")
    axes[1].axvline(mean_val, color="blue", linestyle="--", label=f"mean = {mean_val:.3f} ms")
    axes[1].set_xlabel("solve time [ms]")
    axes[1].set_ylabel("count")
    axes[1].set_title("Solve Time Distribution")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_dir / "solve_time_plot.png", dpi=150)
    plt.close(fig)
    print(f"  solve_time_plot.png: mean={mean_val:.3f}ms, p95={p95:.3f}ms, p99={p99:.3f}ms")


def plot_error_comparison(pp_dir: Path, nmpc_dir: Path, output_dir: Path) -> None:
    """图 5：误差对比图。"""
    pp_err = load_csv(pp_dir / "error.csv")
    nmpc_err = load_csv(nmpc_dir / "error.csv")

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(pp_err["time"], pp_err["position_error"], label="Pure Pursuit", linewidth=1)
    axes[0].plot(nmpc_err["time"], nmpc_err["position_error"], label="NMPC", linewidth=1)
    axes[0].set_ylabel("position error [m]")
    axes[0].set_title("Tracking Error: Pure Pursuit vs NMPC")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(pp_err["time"], pp_err["etheta"], label="Pure Pursuit", linewidth=1)
    axes[1].plot(nmpc_err["time"], nmpc_err["etheta"], label="NMPC", linewidth=1)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("theta error [rad]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_dir / "error_comparison_plot.png", dpi=150)
    plt.close(fig)

    pp_mean = np.mean(pp_err["position_error"])
    nmpc_mean = np.mean(nmpc_err["position_error"])
    print(f"  error_comparison_plot.png: PP mean={pp_mean:.4f}m, NMPC mean={nmpc_mean:.4f}m")


def plot_bar_comparison(pp_dir: Path, nmpc_dir: Path, output_dir: Path) -> None:
    """图 6：综合对比柱状图。"""
    pp_metrics = compute_tracking_metrics(pp_dir)
    nmpc_metrics = compute_tracking_metrics(nmpc_dir)
    nmpc_solve_metrics = compute_solve_time_metrics(nmpc_dir)

    metrics = {
        "Mean Pos Err [m]": (pp_metrics["mean_position_error"], nmpc_metrics["mean_position_error"]),
        "RMS Pos Err [m]": (pp_metrics["rms_position_error"], nmpc_metrics["rms_position_error"]),
        "Max Pos Err [m]": (pp_metrics["max_position_error"], nmpc_metrics["max_position_error"]),
        "Mean |Theta| Err [rad]": (pp_metrics["mean_abs_theta_error"], nmpc_metrics["mean_abs_theta_error"]),
        "Mean Solve Time [ms]": (0.0, nmpc_solve_metrics["solve_time_mean_ms"]),
    }

    labels = list(metrics.keys())
    pp_vals = [v[0] for v in metrics.values()]
    nmpc_vals = [v[1] for v in metrics.values()]

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(12, 6))
    bars1 = ax.bar(x - width / 2, pp_vals, width, label="Pure Pursuit", color="#4C72B0")
    bars2 = ax.bar(x + width / 2, nmpc_vals, width, label="NMPC (acados)", color="#DD8452")

    ax.set_ylabel("Value")
    ax.set_title("Controller Comparison: Pure Pursuit vs NMPC")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)

    # 柱状图上方显示数值
    for bar in bars1:
        height = bar.get_height()
        if height > 0:
            ax.annotate(f"{height:.4f}", xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7)
    for bar in bars2:
        height = bar.get_height()
        if height > 0:
            ax.annotate(f"{height:.4f}", xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7)

    fig.tight_layout()
    fig.savefig(output_dir / "comparison_bar_plot.png", dpi=150)
    plt.close(fig)
    print(f"  comparison_bar_plot.png")


def main() -> None:
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    pp_dir = Path(sys.argv[1])
    nmpc_dir = Path(sys.argv[2])
    output_dir = Path(sys.argv[3]) if len(sys.argv) > 3 else nmpc_dir

    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Generating comparison plots: PP={pp_dir}, NMPC={nmpc_dir}")
    plot_solve_time(nmpc_dir, output_dir)
    plot_error_comparison(pp_dir, nmpc_dir, output_dir)
    plot_bar_comparison(pp_dir, nmpc_dir, output_dir)
    print(f"Done. Plots saved to {output_dir}")


if __name__ == "__main__":
    main()
