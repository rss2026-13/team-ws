#!/usr/bin/env python3
"""
Same overlay graphs (3 runs per plot) but evaluated by the method corresponding to the robot/runner:
- muktha's bags → muktha method distance
- tissany's bags → tissany method distance
- ansis's bags → ansisg method distance
- jeryl's bags → jeryl method distance

One figure with 4×2 subplots (runner × left/right), each subplot 3 runs overlaid.

Run from repo root:
  uv run scripts/plot_by_robot_method.py
"""
from __future__ import annotations

import sys
from pathlib import Path

_scripts_dir = Path(__file__).resolve().parent
if str(_scripts_dir) not in sys.path:
    sys.path.insert(0, str(_scripts_dir))

import numpy as np

from evaluate_rosbag_distances import (
    get_per_scan_distances,
    read_scans_from_bag,
    side_from_bag_name,
    test_case_id_from_bag_name,
)

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib required. Install: uv add matplotlib", file=sys.stderr)
    sys.exit(1)


def _runner_from_bag_name(name: str) -> str:
    """e.g. straight_inward_muktha_left_1 -> muktha; straight_inward_ansis_left_1 -> ansis."""
    parts = name.split("_")
    if len(parts) >= 4:
        return parts[2]
    return ""


def _collect_bags_by_runner_side(rosbag_data_dir: Path) -> dict[tuple[str, str], list[Path]]:
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
        print("tissany_left truncation:", tissany_left_target_len, "scans")

    grouped = _collect_bags_by_runner_side(rosbag_data_dir)
    # runner (bag name) -> method key in get_per_scan_distances output
    runner_to_method = {"muktha": "muktha", "tissany": "tissany", "ansis": "ansisg", "jeryl": "jeryl"}

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

    runners = ["muktha", "tissany", "ansis", "jeryl"]
    sides = ["left", "right"]

    fig, axes = plt.subplots(4, 2, figsize=(10, 12), sharex=True)
    fig.suptitle("Distance by robot's method (3 runs overlaid per cell)")

    for ri, runner in enumerate(runners):
        method = runner_to_method[runner]
        for si, side in enumerate(sides):
            ax = axes[ri, si]
            key = (runner, side)
            if key not in data:
                ax.set_title(f"{runner} {side} (no data)")
                continue
            for run_idx, series_list in enumerate(data[key]):
                dists = series_list[method]
                x = np.arange(len(dists))
                y = np.array(dists, dtype=float)
                ax.plot(x, y, alpha=0.8, label=f"run {run_idx + 1}")
            ax.axhline(0.6, color="gray", linestyle="--", alpha=0.7, label="desired 0.6 m")
            ax.set_title(f"{runner} {side} ({method})")
            ax.set_ylabel("distance (m)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)

    axes[3, 0].set_xlabel("scan index")
    axes[3, 1].set_xlabel("scan index")
    plt.tight_layout()
    fig.savefig(out_dir / "by_robot_method_three_runs.png", dpi=150)
    plt.close(fig)
    print("Saved", out_dir / "by_robot_method_three_runs.png")
    print("Done. Plots in", out_dir)


if __name__ == "__main__":
    main()
