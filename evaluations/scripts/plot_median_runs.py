#!/usr/bin/env python3
"""
Overlay three runs (run 1, 2, 3) for each method and side using that method's distance
(or median_of_4) vs scan index. Creates one figure per method with left/right subplots.

Run from repo root (same deps as evaluate_rosbag_distances + matplotlib):
  uv run scripts/plot_median_runs.py
"""
from __future__ import annotations

import sys
from pathlib import Path

# Allow importing from same directory (e.g. evaluate_rosbag_distances)
_scripts_dir = Path(__file__).resolve().parent
if str(_scripts_dir) not in sys.path:
    sys.path.insert(0, str(_scripts_dir))

import numpy as np

# Import evaluation helpers and distance logic
from evaluate_rosbag_distances import (
    get_per_scan_distances,
    read_scans_from_bag,
    side_from_bag_name,
    test_case_id_from_bag_name,
)

# Optional: matplotlib
try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib required. Install: uv add matplotlib", file=sys.stderr)
    sys.exit(1)


def _runner_from_bag_name(name: str) -> str:
    """e.g. straight_inward_muktha_left_1 -> muktha; straight_inward_ansis_left_1 -> ansis."""
    parts = name.split("_")
    if len(parts) >= 4:
        return parts[2]  # muktha, tissany, ansis, jeryl
    return ""


def _collect_bags_by_runner_side(rosbag_data_dir: Path):
    """Return {(runner, side): [bag_dir_run1, bag_dir_run2, bag_dir_run3]} (sorted by run)."""
    bag_dirs = sorted([
        d for d in rosbag_data_dir.iterdir()
        if d.is_dir() and d.name.startswith("straight_inward_") and (d / "metadata.yaml").exists()
    ])
    grouped: dict[tuple[str, str], list[Path]] = {}
    for d in bag_dirs:
        name = d.name
        runner = _runner_from_bag_name(name)
        side = "left" if side_from_bag_name(name) == 1 else "right"
        key = (runner, side)
        grouped.setdefault(key, []).append(d)
    for key in grouped:
        grouped[key] = sorted(grouped[key], key=lambda p: p.name)
    return grouped


def _get_tissany_left_target_len(bag_dirs: list[Path]) -> int | None:
    """Same pre-pass as evaluate_rosbag_distances for tissany_left truncation."""
    tissany_left_dirs = sorted([d for d in bag_dirs if d.name.startswith("straight_inward_tissany_left")])
    if len(tissany_left_dirs) < 3:
        return None
    counts = []
    try:
        for d in tissany_left_dirs:
            scans = read_scans_from_bag(str(d))
            counts.append(len(scans) if scans else 0)
        c2, c3 = counts[1], counts[2]
        if c2 > 0 and c3 > 0:
            return int(round((c2 + c3) / 2.0))
    except (OSError, RuntimeError):
        pass
    return None


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    rosbag_data_dir = repo_root / "rosbag_data"
    out_dir = repo_root / "evaluation_plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    if not rosbag_data_dir.is_dir():
        print("Not found:", rosbag_data_dir, file=sys.stderr)
        sys.exit(1)

    all_bag_dirs = sorted(
        d for d in rosbag_data_dir.iterdir()
        if d.is_dir() and d.name.startswith("straight_inward_") and (d / "metadata.yaml").exists()
    )
    tissany_left_target_len = _get_tissany_left_target_len(all_bag_dirs)
    if tissany_left_target_len is not None:
        print(f"tissany_left truncation: {tissany_left_target_len} scans")

    grouped = _collect_bags_by_runner_side(rosbag_data_dir)
    # (runner, side) -> list of 3 path; then for each we load and get per_scan_distances
    # method -> runner for that method (ansisg -> ansis in bag names)
    method_to_runner = {"muktha": "muktha", "tissany": "tissany", "ansisg": "ansis", "jeryl": "jeryl"}

    # Load per-scan distance series for each (runner, side, run_idx)
    # data[(runner, side)][run_idx] = dict with keys muktha, tissany, ansisg, jeryl, median_of_4 -> list of float
    data: dict[tuple[str, str], list[dict[str, list[float]]]] = {}
    for (runner, side), bags in grouped.items():
        if len(bags) != 3:
            continue
        series_list = []
        for bag_dir in bags:
            try:
                scans = read_scans_from_bag(str(bag_dir))
            except (OSError, RuntimeError):
                continue
            if not scans:
                continue
            test_case_id = test_case_id_from_bag_name(bag_dir.name)
            if test_case_id == "straight_inward_tissany_left" and tissany_left_target_len is not None:
                scans = scans[:tissany_left_target_len]
            side_int = 1 if side == "left" else -1
            per_method = get_per_scan_distances(scans, side_int)
            series_list.append(per_method)
        if len(series_list) == 3:
            data[(runner, side)] = series_list

    method_order = ["muktha", "tissany", "ansisg", "jeryl", "median_of_4"]
    sides = ["left", "right"]

    for method in method_order:
        if method == "median_of_4":
            # One figure: 4 runners x 2 sides = 8 subplots
            fig, axes = plt.subplots(4, 2, figsize=(10, 12), sharex=True)
            fig.suptitle("median_of_4 distance (3 runs overlaid)")
            runners = ["muktha", "tissany", "ansis", "jeryl"]
            for ri, runner in enumerate(runners):
                for si, side in enumerate(sides):
                    ax = axes[ri, si]
                    key = (runner, side)
                    if key not in data:
                        ax.set_title(f"{runner} {side} (no data)")
                        continue
                    for run_idx, series_list in enumerate(data[key]):
                        dists = series_list["median_of_4"]
                        x = np.arange(len(dists))
                        y = np.array(dists, dtype=float)
                        ax.plot(x, y, alpha=0.8, label=f"run {run_idx + 1}")
                    ax.axhline(0.6, color="gray", linestyle="--", alpha=0.7, label="desired 0.6 m")
                    ax.set_title(f"{runner} {side}")
                    ax.set_ylabel("distance (m)")
                    ax.legend(loc="upper right", fontsize=8)
                    ax.grid(True, alpha=0.3)
            axes[3, 0].set_xlabel("scan index")
            axes[3, 1].set_xlabel("scan index")
            plt.tight_layout()
            fig.savefig(out_dir / "median_of_4_three_runs.png", dpi=150)
            plt.close(fig)
            print("Saved", out_dir / "median_of_4_three_runs.png")
            continue

        # One figure per method: 2 subplots (left, right), 3 runs each
        runner = method_to_runner[method]
        fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
        fig.suptitle(f"Method: {method} (3 runs overlaid)")
        for si, side in enumerate(sides):
            ax = axes[si]
            key = (runner, side)
            if key not in data:
                ax.set_title(f"{side} (no data)")
                continue
            for run_idx, series_list in enumerate(data[key]):
                dists = series_list[method]
                x = np.arange(len(dists))
                y = np.array(dists, dtype=float)
                ax.plot(x, y, alpha=0.8, label=f"run {run_idx + 1}")
            ax.axhline(0.6, color="gray", linestyle="--", alpha=0.7, label="desired 0.6 m")
            ax.set_title(side)
            ax.set_ylabel("distance (m)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)
        axes[0].set_xlabel("scan index")
        axes[1].set_xlabel("scan index")
        plt.tight_layout()
        safe_name = method.replace(" ", "_")
        fig.savefig(out_dir / f"{safe_name}_three_runs.png", dpi=150)
        plt.close(fig)
        print("Saved", out_dir / f"{safe_name}_three_runs.png")

    print("Done. Plots in", out_dir)


if __name__ == "__main__":
    main()
