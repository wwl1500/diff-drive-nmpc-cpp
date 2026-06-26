#!/usr/bin/env python3
"""批量运行 Pure Pursuit 与 NMPC 实验并生成统一 summary.csv。"""

import argparse
import csv
import subprocess
import sys
from pathlib import Path

from plot_comparison import compute_solve_time_metrics, compute_tracking_metrics


TRAJECTORIES = ["circle", "eight", "sine"]
CONTROLLERS = ["pp", "nmpc"]
SUMMARY_FIELDS = [
    "trajectory",
    "controller",
    "mean_position_error",
    "rms_position_error",
    "max_position_error",
    "mean_abs_theta_error",
    "rms_theta_error",
    "max_abs_theta_error",
    "solve_time_mean_ms",
    "solve_time_median_ms",
    "solve_time_p95_ms",
    "solve_time_p99_ms",
    "solve_time_max_ms",
    "solve_failure_count",
    "solve_failure_rate",
]


def run_command(command: list[str], cwd: Path) -> None:
    print("$ " + " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def result_dir(output_dir: Path, trajectory: str, controller: str) -> Path:
    return output_dir / trajectory / controller


def run_simulations(repo_dir: Path, executable: Path, output_dir: Path) -> None:
    for trajectory in TRAJECTORIES:
        for controller in CONTROLLERS:
            target_dir = result_dir(output_dir, trajectory, controller)
            target_dir.mkdir(parents=True, exist_ok=True)
            print(f"\nRunning {trajectory} / {controller} -> {target_dir}")
            run_command(
                [str(executable), trajectory, controller, str(target_dir)],
                cwd=repo_dir,
            )


def generate_plots(repo_dir: Path, output_dir: Path) -> None:
    plot_results = repo_dir / "scripts" / "plot_results.py"
    plot_comparison = repo_dir / "scripts" / "plot_comparison.py"

    for trajectory in TRAJECTORIES:
        for controller in CONTROLLERS:
            target_dir = result_dir(output_dir, trajectory, controller)
            print(f"\nGenerating plots for {trajectory} / {controller}")
            run_command([sys.executable, str(plot_results), str(target_dir)], cwd=repo_dir)

        comparison_dir = output_dir / trajectory / "comparison"
        comparison_dir.mkdir(parents=True, exist_ok=True)
        print(f"\nGenerating comparison plots for {trajectory}")
        run_command(
            [
                sys.executable,
                str(plot_comparison),
                str(result_dir(output_dir, trajectory, "pp")),
                str(result_dir(output_dir, trajectory, "nmpc")),
                str(comparison_dir),
            ],
            cwd=repo_dir,
        )


def format_metric(value) -> str:
    if value == "":
        return ""
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.6f}"


def build_summary(output_dir: Path) -> list[dict[str, str]]:
    rows = []
    for trajectory in TRAJECTORIES:
        for controller in CONTROLLERS:
            target_dir = result_dir(output_dir, trajectory, controller)
            row = {
                "trajectory": trajectory,
                "controller": controller,
            }
            row.update(compute_tracking_metrics(target_dir))
            if controller == "nmpc":
                row.update(compute_solve_time_metrics(target_dir))
            else:
                for field in SUMMARY_FIELDS:
                    if field.startswith("solve_"):
                        row[field] = ""
            rows.append({field: format_metric(row[field]) for field in SUMMARY_FIELDS})
    return rows


def write_summary(output_dir: Path) -> None:
    rows = build_summary(output_dir)
    summary_path = output_dir / "summary.csv"
    with summary_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSummary saved to {summary_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="results_experiments", help="实验输出目录")
    parser.add_argument("--skip-run", action="store_true", help="跳过仿真，只基于已有 CSV 生成图和 summary")
    parser.add_argument("--skip-plots", action="store_true", help="跳过绘图，只运行仿真并生成 summary")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    repo_dir = Path(__file__).resolve().parents[1]
    executable = repo_dir / "build" / "diff_drive_sim"
    output_dir = Path(args.output)
    if not output_dir.is_absolute():
        output_dir = repo_dir / output_dir

    if not args.skip_run and not executable.exists():
        raise FileNotFoundError(f"未找到可执行文件：{executable}，请先运行 cmake --build build")

    output_dir.mkdir(parents=True, exist_ok=True)

    if not args.skip_run:
        run_simulations(repo_dir, executable, output_dir)
    if not args.skip_plots:
        generate_plots(repo_dir, output_dir)
    write_summary(output_dir)


if __name__ == "__main__":
    main()
