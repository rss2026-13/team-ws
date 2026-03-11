#!/usr/bin/env python3
"""
Evaluate median_of_4 distance on Ansis's rosbags in rosbag_data_long_path/ and overlay all runs
by group: Left slow, Left fast, Right slow, Right fast (right: runs 1,5,6 = fast; 2,3,4 = slow).

Run from repo root:
  uv run scripts/evaluate_and_plot_rosbag_data2.py

Uses friendly names in output (e.g. "Left slow 1", "Right fast 5") instead of raw bag folder names.
"""
from __future__ import annotations

import re
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
)

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib required. Install: uv add matplotlib", file=sys.stderr)
    sys.exit(1)


# Bag name patterns for rosbag_data_long_path (Ansis only)
LEFT_SLOW_PREFIX = "left_close_slow_ansis_"
LEFT_FAST_PREFIX = "left_close_fast_ansis_"
RIGHT_PREFIX = "right_short_close_ansis_"
RIGHT_FAST_RUN_NUMBERS = {1, 5, 6}  # right runs 2, 3, 4 = slow

DESIRED_DISTANCE = 0.6  # meters


def _run_number_from_bag_name(name: str) -> int | None:
    """Parse trailing _N from bag name, e.g. right_short_close_ansis_5 -> 5."""
    match = re.match(r"^.+_(\d{1,2})$", name)
    if match and 1 <= int(match.group(1)) <= 99:
        return int(match.group(1))
    return None


def _group_key_from_bag_name(name: str) -> str | None:
    """Return one of: left_slow, left_fast, right_slow, right_fast; or None if not a known Ansis bag."""
    if name.startswith(LEFT_SLOW_PREFIX):
        return "left_slow"
    if name.startswith(LEFT_FAST_PREFIX):
        return "left_fast"
    if name.startswith(RIGHT_PREFIX):
        n = _run_number_from_bag_name(name)
        if n is not None:
            return "right_fast" if n in RIGHT_FAST_RUN_NUMBERS else "right_slow"
    return None


def _friendly_name(name: str) -> str:
    """Convert bag dir name to a short display name, e.g. left_close_fast_ansis_2 -> Left fast 2."""
    group = _group_key_from_bag_name(name)
    n = _run_number_from_bag_name(name)
    if group == "left_slow" and n is not None:
        return f"Left slow {n}"
    if group == "left_fast" and n is not None:
        return f"Left fast {n}"
    if group == "right_slow" and n is not None:
        return f"Right slow {n}"
    if group == "right_fast" and n is not None:
        return f"Right fast {n}"
    return name


def discover_long_path_bags(rosbag_data_dir: Path) -> list[Path]:
    """Return sorted list of Ansis bag dirs (left_close_*_ansis_*, right_short_close_ansis_*) with metadata.yaml."""
    prefixes = (LEFT_SLOW_PREFIX, LEFT_FAST_PREFIX, RIGHT_PREFIX)
    bag_dirs = [
        d
        for d in rosbag_data_dir.iterdir()
        if d.is_dir()
        and (d / "metadata.yaml").exists()
        and any(d.name.startswith(p) for p in prefixes)
        and _group_key_from_bag_name(d.name) is not None
    ]
    return sorted(bag_dirs)


def collect_by_group(bag_dirs: list[Path]) -> dict[str, list[Path]]:
    """Return {group: [bag_dir, ...]} for group in left_slow, left_fast, right_slow, right_fast."""
    grouped: dict[str, list[Path]] = {
        "left_slow": [],
        "left_fast": [],
        "right_slow": [],
        "right_fast": [],
    }
    for d in bag_dirs:
        key = _group_key_from_bag_name(d.name)
        if key and key in grouped:
            grouped[key].append(d)
    for key in grouped:
        grouped[key] = sorted(grouped[key], key=lambda p: (_run_number_from_bag_name(p.name) or 0, p.name))
    return grouped


def run() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    rosbag_data_dir = repo_root / "rosbag_data_long_path"
    out_dir = repo_root / "evaluation_rosbag_data_long_path"
    out_dir.mkdir(parents=True, exist_ok=True)

    if not rosbag_data_dir.is_dir():
        print("Not found:", rosbag_data_dir, file=sys.stderr)
        sys.exit(1)

    bag_dirs = discover_long_path_bags(rosbag_data_dir)
    if not bag_dirs:
        print("No Ansis rosbag dirs found under", rosbag_data_dir, file=sys.stderr)
        sys.exit(1)

    grouped = collect_by_group(bag_dirs)
    group_order = ["left_slow", "left_fast", "right_slow", "right_fast"]
    group_title = {
        "left_slow": "Left slow",
        "left_fast": "Left fast",
        "right_slow": "Right slow",
        "right_fast": "Right fast",
    }

    # Load per-scan median_of_4 for each bag, by group
    data: dict[str, list[tuple[str, list[float]]]] = {g: [] for g in group_order}
    all_rows: list[dict] = []

    for group in group_order:
        bags = grouped[group]
        for bag_dir in bags:
            name = bag_dir.name
            friendly = _friendly_name(name)
            try:
                scans = read_scans_from_bag(str(bag_dir))
            except (OSError, RuntimeError) as e:
                print("Skip", name, ":", e, file=sys.stderr)
                continue
            if not scans:
                continue
            side = 1 if "left" in name.lower() else -1
            per_method = get_per_scan_distances(scans, side)
            median_series = per_method["median_of_4"]
            data[group].append((friendly, median_series))

            # Per-bag stats for CSV/MD
            valid = [x for x in median_series if x == x]  # strip nan
            if valid:
                arr = np.array(valid)
                errs = np.abs(arr - DESIRED_DISTANCE)
                all_rows.append({
                    "bag": friendly,
                    "group": group_title[group],
                    "count": len(arr),
                    "mean": float(np.mean(arr)),
                    "std": float(np.std(arr)) if len(arr) > 1 else 0.0,
                    "min": float(np.min(arr)),
                    "max": float(np.max(arr)),
                    "mean_abs_error": float(np.mean(errs)),
                })

    # Write stats
    csv_path = out_dir / "evaluation_stats.csv"
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("bag,group,count,mean,std,min,max,mean_abs_error\n")
        for r in all_rows:
            f.write(f"{r['bag']},{r['group']},{r['count']},{r['mean']:.6f},{r['std']:.6f},{r['min']:.6f},{r['max']:.6f},{r['mean_abs_error']:.6f}\n")
    print("Wrote", csv_path)

    # Summary by group
    summary_path = out_dir / "evaluation_summary.md"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("# rosbag_data_long_path – median_of_4 summary (Ansis)\n\n")
        f.write("| Group | Runs | Count | Mean (m) | Std | Min | Max | Avg error vs 0.6 m |\n")
        f.write("|-------|------|-------|----------|-----|-----|-----|--------------------|\n")
        for group in group_order:
            rows = [r for r in all_rows if r["group"] == group_title[group]]
            if not rows:
                continue
            total_count = sum(r["count"] for r in rows)
            all_means = [r["mean"] for r in rows]
            all_errs = [r["mean_abs_error"] for r in rows]
            f.write(f"| {group_title[group]} | {len(rows)} | {total_count} | {np.mean(all_means):.4f} | — | — | — | {np.mean(all_errs):.4f} |\n")
    print("Wrote", summary_path)

    # Overlay plot: 2×2 subplots (left_slow, left_fast, right_slow, right_fast)
    fig, axes = plt.subplots(2, 2, figsize=(10, 8), sharex=True)
    axes = axes.flatten()
    for i, group in enumerate(group_order):
        ax = axes[i]
        series_list = data[group]
        for friendly, dists in series_list:
            x = np.arange(len(dists))
            y = np.array(dists, dtype=float)
            ax.plot(x, y, alpha=0.8, label=friendly)
        ax.axhline(DESIRED_DISTANCE, color="gray", linestyle="--", alpha=0.7, label="desired 0.6 m")
        ax.set_title(f"{group_title[group]} ({len(series_list)} runs)")
        ax.set_ylabel("distance (m)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[2].set_xlabel("scan index")
    axes[3].set_xlabel("scan index")
    fig.suptitle("median_of_4 distance – Ansis (rosbag_data_long_path)")
    plt.tight_layout()
    plot_path = out_dir / "median_of_4_ansis_overlay.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print("Wrote", plot_path)
    print("Done. Output in", out_dir)


if __name__ == "__main__":
    run()
