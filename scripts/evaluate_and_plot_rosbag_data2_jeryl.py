#!/usr/bin/env python3
"""
Evaluate median_of_4 distance on Jeryl's rosbags (right_slow_short_1, _2, _3) and overlay all runs.

Jeryl's bags are the plain-numbered ones only: right_slow_short_1, right_slow_short_2, right_slow_short_3
(no suffix like _muktha or _tissany). They live in the same rosbag_data_long_path/ as others.

Run from repo root:
  uv run scripts/evaluate_and_plot_rosbag_data2_jeryl.py
  uv run scripts/evaluate_and_plot_rosbag_data2_jeryl.py --rosbag-dir rosbag_data_long_path

Output uses friendly names (e.g. "Right slow 1", "Right slow 2", "Right slow 3").
"""
from __future__ import annotations

import argparse
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
)

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib required. Install: uv add matplotlib", file=sys.stderr)
    sys.exit(1)


# Jeryl's bags have no person name in them: right_slow_short_1, right_slow_short_2, right_slow_short_3.
# Exclude right_slow_short_muktha_*, right_slow_short_tissany_*, right_slow_short_jeryl_newpath_*, etc.
RIGHT_SLOW_SHORT_PREFIX = "right_slow_short_"
# Only match names that end with _N (digit(s)) and nothing else, e.g. right_slow_short_1
RIGHT_SLOW_SHORT_PLAIN_RE = re.compile(r"^right_slow_short_\d{1,2}$")
DESIRED_DISTANCE = 0.6  # meters


def _run_number_from_bag_name(name: str) -> int | None:
    """Parse trailing _N from bag name, e.g. right_slow_short_2 -> 2."""
    match = re.match(r"^.+_(\d{1,2})$", name)
    if match and 1 <= int(match.group(1)) <= 99:
        return int(match.group(1))
    return None


def _friendly_name(name: str) -> str:
    """e.g. right_slow_short_2 -> Right slow 2."""
    n = _run_number_from_bag_name(name)
    if n is not None and name.startswith(RIGHT_SLOW_SHORT_PREFIX):
        return f"Right slow {n}"
    return name


def discover_jeryl_bags(rosbag_data_dir: Path) -> list[Path]:
    """Return sorted list of Jeryl bag dirs: right_slow_short_1, right_slow_short_2, right_slow_short_3 only (no _muktha, _tissany, etc.)."""
    bag_dirs = [
        d
        for d in rosbag_data_dir.iterdir()
        if d.is_dir()
        and (d / "metadata.yaml").exists()
        and RIGHT_SLOW_SHORT_PLAIN_RE.match(d.name) is not None
    ]
    return sorted(bag_dirs, key=lambda p: (_run_number_from_bag_name(p.name) or 0, p.name))


def run(rosbag_data_dir: Path, out_dir: Path) -> None:
    if not rosbag_data_dir.is_dir():
        print("Not found:", rosbag_data_dir, file=sys.stderr)
        sys.exit(1)

    bag_dirs = discover_jeryl_bags(rosbag_data_dir)
    if not bag_dirs:
        print("No Jeryl rosbag dirs (right_slow_short_1, right_slow_short_2, right_slow_short_3) found under", rosbag_data_dir, file=sys.stderr)
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)
    all_rows: list[dict] = []
    series_list: list[tuple[str, list[float]]] = []

    for bag_dir in bag_dirs:
        name = bag_dir.name
        friendly = _friendly_name(name)
        try:
            scans = read_scans_from_bag(str(bag_dir))
        except (OSError, RuntimeError) as e:
            print("Skip", name, ":", e, file=sys.stderr)
            continue
        if not scans:
            continue
        side = -1  # right wall
        per_method = get_per_scan_distances(scans, side)
        median_series = per_method["median_of_4"]
        series_list.append((friendly, median_series))

        valid = [x for x in median_series if x == x]
        if valid:
            arr = np.array(valid)
            errs = np.abs(arr - DESIRED_DISTANCE)
            all_rows.append({
                "bag": friendly,
                "group": "Right slow",
                "count": len(arr),
                "mean": float(np.mean(arr)),
                "std": float(np.std(arr)) if len(arr) > 1 else 0.0,
                "min": float(np.min(arr)),
                "max": float(np.max(arr)),
                "mean_abs_error": float(np.mean(errs)),
            })

    # Stats CSV
    csv_path = out_dir / "evaluation_stats.csv"
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("bag,group,count,mean,std,min,max,mean_abs_error\n")
        for r in all_rows:
            f.write(f"{r['bag']},{r['group']},{r['count']},{r['mean']:.6f},{r['std']:.6f},{r['min']:.6f},{r['max']:.6f},{r['mean_abs_error']:.6f}\n")
    print("Wrote", csv_path)

    # Summary MD
    summary_path = out_dir / "evaluation_summary.md"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("# rosbag_data_long_path Jeryl – median_of_4 summary (Right slow)\n\n")
        f.write("| Run | Count | Mean (m) | Std | Min | Max | Avg error vs 0.6 m |\n")
        f.write("|-----|-------|----------|-----|-----|-----|--------------------|\n")
        for r in all_rows:
            f.write(f"| {r['bag']} | {r['count']} | {r['mean']:.4f} | {r['std']:.4f} | {r['min']:.4f} | {r['max']:.4f} | {r['mean_abs_error']:.4f} |\n")
        if all_rows:
            total_count = sum(r["count"] for r in all_rows)
            f.write(f"\n**Total scans:** {total_count}\n")
    print("Wrote", summary_path)

    # Overlay plot: single subplot, all runs
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    for friendly, dists in series_list:
        x = np.arange(len(dists))
        y = np.array(dists, dtype=float)
        ax.plot(x, y, alpha=0.8, label=friendly)
    ax.axhline(DESIRED_DISTANCE, color="gray", linestyle="--", alpha=0.7, label="desired 0.6 m")
    ax.set_title(f"Jeryl Right slow ({len(series_list)} runs)")
    ax.set_xlabel("scan index")
    ax.set_ylabel("distance (m)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.suptitle("median_of_4 distance – Jeryl (right_slow_short)")
    plt.tight_layout()
    plot_path = out_dir / "median_of_4_jeryl_overlay.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print("Wrote", plot_path)
    print("Done. Output in", out_dir)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate and plot median_of_4 for Jeryl's right_slow_short runs.")
    parser.add_argument(
        "--rosbag-dir",
        type=Path,
        default=None,
        help="Directory containing right_slow_short_* bag dirs (default: repo_root/rosbag_data_long_path)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory (default: repo_root/evaluation_rosbag_data_long_path_jeryl)",
    )
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    rosbag_data_dir = args.rosbag_dir if args.rosbag_dir is not None else repo_root / "rosbag_data_long_path"
    out_dir = args.output_dir if args.output_dir is not None else repo_root / "evaluation_rosbag_data_long_path_jeryl"
    run(rosbag_data_dir, out_dir)


if __name__ == "__main__":
    main()
