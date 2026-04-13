#!/usr/bin/env python3
"""
Analyze integrated planning + tracking runs from ROS bags.

Outputs:
- summary_metrics.csv
- xy_overview.png
- per-run cross-track plots
- summary bar charts

Works with rosbag1 or rosbag2 through rosbags.AnyReader.

Install:
    pip install rosbags numpy matplotlib pandas

Example:
    python analyze_integration_bag.py /path/to/bag_dir_or_bagfile --outdir results

"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


# =========================
# User-editable parameters
# =========================
DEFAULT_TRAJ_TOPIC = "/trajectory/current"
DEFAULT_ODOM_TOPIC = "/pf/pose/odom"   # change to /odom if needed
DEFAULT_GOAL_TOPIC = "/goal_pose"

# If final distance to goal is <= this, you could later call it a "success"
GOAL_TOLERANCE_M = 0.40

# Ignore trajectory messages with fewer than this many poses
MIN_TRAJ_POINTS = 2


@dataclass
class TimedPath:
    t: float  # seconds
    points: np.ndarray  # shape (N, 2)


@dataclass
class TimedPose:
    t: float  # seconds
    x: float
    y: float


@dataclass
class TimedGoal:
    t: float  # seconds
    x: float
    y: float


@dataclass
class RunMetrics:
    run_id: int
    start_time: float
    end_time: float
    duration_s: float
    planned_path_length_m: float
    executed_path_length_m: float
    mean_cross_track_error_m: float
    max_cross_track_error_m: float
    final_goal_error_m: float
    num_odom_samples: int
    num_path_points: int


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    diffs = np.diff(points, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def extract_xy_from_pose(pose) -> Tuple[float, float]:
    return float(pose.position.x), float(pose.position.y)


def extract_xy_from_msg(msg) -> Optional[Tuple[float, float]]:
    """
    Try several common message layouts:
    - geometry_msgs/PoseStamped
    - geometry_msgs/PoseWithCovarianceStamped
    - nav_msgs/Odometry
    - geometry_msgs/Pose
    """
    # PoseStamped: msg.pose.position
    if hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        return float(msg.pose.position.x), float(msg.pose.position.y)

    # PoseWithCovarianceStamped / Odometry: msg.pose.pose.position
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose") and hasattr(msg.pose.pose, "position"):
        return float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)

    # Raw Pose: msg.position
    if hasattr(msg, "position"):
        return float(msg.position.x), float(msg.position.y)

    return None


def extract_path_from_pose_array(msg) -> np.ndarray:
    if not hasattr(msg, "poses"):
        return np.empty((0, 2))
    pts = []
    for pose in msg.poses:
        x, y = extract_xy_from_pose(pose)
        pts.append((x, y))
    return np.asarray(pts, dtype=float)


def point_to_segment_distance(
    p: np.ndarray, a: np.ndarray, b: np.ndarray
) -> Tuple[float, np.ndarray]:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom == 0.0:
        proj = a
    else:
        t = np.dot(p - a, ab) / denom
        t = np.clip(t, 0.0, 1.0)
        proj = a + t * ab
    dist = float(np.linalg.norm(p - proj))
    return dist, proj


def point_to_polyline_distance(p: np.ndarray, polyline: np.ndarray) -> Tuple[float, np.ndarray]:
    if len(polyline) == 0:
        return float("nan"), np.array([np.nan, np.nan])
    if len(polyline) == 1:
        return float(np.linalg.norm(p - polyline[0])), polyline[0]

    best_dist = float("inf")
    best_proj = None
    for i in range(len(polyline) - 1):
        d, proj = point_to_segment_distance(p, polyline[i], polyline[i + 1])
        if d < best_dist:
            best_dist = d
            best_proj = proj
    return best_dist, best_proj


def compute_cross_track_errors(actual_xy: np.ndarray, planned_xy: np.ndarray) -> np.ndarray:
    errs = []
    for p in actual_xy:
        d, _ = point_to_polyline_distance(p, planned_xy)
        errs.append(d)
    return np.asarray(errs, dtype=float)


def find_closest_goal(goals: List[TimedGoal], t0: float, t1: float, fallback_path: np.ndarray) -> np.ndarray:
    """
    Prefer a goal_pose near the run window. If none exists, use the last point of the planned path.
    """
    if goals:
        in_window = [g for g in goals if (t0 - 1.0) <= g.t <= (t1 + 1.0)]
        if in_window:
            g = min(in_window, key=lambda g_: abs(g_.t - t0))
            return np.array([g.x, g.y], dtype=float)
    if len(fallback_path) > 0:
        return fallback_path[-1]
    return np.array([np.nan, np.nan], dtype=float)


def load_bag_data(
    bag_paths: List[Path],
    traj_topic: str,
    odom_topic: str,
    goal_topic: Optional[str],
) -> Tuple[List[TimedPath], List[TimedPose], List[TimedGoal]]:
    typestore = get_typestore(Stores.LATEST)

    trajs: List[TimedPath] = []
    odom: List[TimedPose] = []
    goals: List[TimedGoal] = []

    with AnyReader(bag_paths, default_typestore=typestore) as reader:
        wanted_topics = {traj_topic, odom_topic}
        if goal_topic:
            wanted_topics.add(goal_topic)

        connections = [c for c in reader.connections if c.topic in wanted_topics]

        if not connections:
            available = sorted(set(c.topic for c in reader.connections))
            raise RuntimeError(
                "No matching topics found.\n"
                f"Requested: {sorted(wanted_topics)}\n"
                f"Available: {available}"
            )

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            t = timestamp * 1e-9  # ns -> s
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == traj_topic:
                pts = extract_path_from_pose_array(msg)
                if len(pts) >= MIN_TRAJ_POINTS:
                    trajs.append(TimedPath(t=t, points=pts))

            elif connection.topic == odom_topic:
                xy = extract_xy_from_msg(msg)
                if xy is not None:
                    odom.append(TimedPose(t=t, x=xy[0], y=xy[1]))

            elif goal_topic and connection.topic == goal_topic:
                xy = extract_xy_from_msg(msg)
                if xy is not None:
                    goals.append(TimedGoal(t=t, x=xy[0], y=xy[1]))

    trajs.sort(key=lambda x: x.t)
    odom.sort(key=lambda x: x.t)
    goals.sort(key=lambda x: x.t)
    return trajs, odom, goals


def segment_runs(
    trajs: List[TimedPath],
    odom: List[TimedPose],
) -> List[Tuple[TimedPath, np.ndarray, np.ndarray]]:
    """
    Segment runs using trajectory updates.
    Returns tuples of:
      (trajectory_message, odom_times, odom_xy)
    """
    if not trajs or not odom:
        return []

    odom_times = np.array([o.t for o in odom], dtype=float)
    odom_xy = np.array([[o.x, o.y] for o in odom], dtype=float)

    runs = []
    for i, traj in enumerate(trajs):
        start_t = traj.t
        end_t = trajs[i + 1].t if i + 1 < len(trajs) else odom_times[-1]

        mask = (odom_times >= start_t) & (odom_times <= end_t)
        run_times = odom_times[mask]
        run_xy = odom_xy[mask]

        if len(run_xy) < 2:
            continue

        runs.append((traj, run_times, run_xy))

    return runs


def compute_run_metrics(
    runs: List[Tuple[TimedPath, np.ndarray, np.ndarray]],
    goals: List[TimedGoal],
) -> List[RunMetrics]:
    metrics: List[RunMetrics] = []

    for idx, (traj, run_times, run_xy) in enumerate(runs, start=1):
        cte = compute_cross_track_errors(run_xy, traj.points)
        goal_xy = find_closest_goal(goals, run_times[0], run_times[-1], traj.points)
        final_err = float(np.linalg.norm(run_xy[-1] - goal_xy))

        metrics.append(
            RunMetrics(
                run_id=idx,
                start_time=float(run_times[0]),
                end_time=float(run_times[-1]),
                duration_s=float(run_times[-1] - run_times[0]),
                planned_path_length_m=path_length(traj.points),
                executed_path_length_m=path_length(run_xy),
                mean_cross_track_error_m=float(np.nanmean(cte)),
                max_cross_track_error_m=float(np.nanmax(cte)),
                final_goal_error_m=final_err,
                num_odom_samples=int(len(run_xy)),
                num_path_points=int(len(traj.points)),
            )
        )

    return metrics


def make_xy_overview_plot(
    runs: List[Tuple[TimedPath, np.ndarray, np.ndarray]],
    outdir: Path,
) -> None:
    plt.figure(figsize=(8, 8))
    for idx, (traj, _, run_xy) in enumerate(runs, start=1):
        plt.plot(traj.points[:, 0], traj.points[:, 1], linestyle="--", label=f"planned {idx}")
        plt.plot(run_xy[:, 0], run_xy[:, 1], label=f"actual {idx}")

        plt.scatter(traj.points[0, 0], traj.points[0, 1], marker="o")
        plt.scatter(traj.points[-1, 0], traj.points[-1, 1], marker="x")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Planned vs actual trajectories")
    plt.axis("equal")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(outdir / "xy_overview.png", dpi=200)
    plt.close()


def make_cross_track_plots(
    runs: List[Tuple[TimedPath, np.ndarray, np.ndarray]],
    outdir: Path,
) -> None:
    for idx, (traj, run_times, run_xy) in enumerate(runs, start=1):
        cte = compute_cross_track_errors(run_xy, traj.points)
        t_rel = run_times - run_times[0]

        plt.figure(figsize=(8, 4))
        plt.plot(t_rel, cte)
        plt.xlabel("time since run start [s]")
        plt.ylabel("cross-track error [m]")
        plt.title(f"Run {idx}: cross-track error vs time")
        plt.tight_layout()
        plt.savefig(outdir / f"run_{idx:02d}_cross_track.png", dpi=200)
        plt.close()


def make_summary_barplots(df: pd.DataFrame, outdir: Path) -> None:
    plot_specs = [
        ("mean_cross_track_error_m", "Mean cross-track error [m]", "mean_cross_track_error.png"),
        ("max_cross_track_error_m", "Max cross-track error [m]", "max_cross_track_error.png"),
        ("final_goal_error_m", "Final goal error [m]", "final_goal_error.png"),
        ("duration_s", "Run duration [s]", "run_duration.png"),
        ("planned_path_length_m", "Planned path length [m]", "planned_path_length.png"),
        ("executed_path_length_m", "Executed path length [m]", "executed_path_length.png"),
    ]

    for col, ylabel, filename in plot_specs:
        plt.figure(figsize=(8, 4))
        plt.bar(df["run_id"].astype(str), df[col])
        plt.xlabel("run id")
        plt.ylabel(ylabel)
        plt.title(ylabel + " by run")
        plt.tight_layout()
        plt.savefig(outdir / filename, dpi=200)
        plt.close()


def print_topic_summary(trajs: List[TimedPath], odom: List[TimedPose], goals: List[TimedGoal]) -> None:
    print(f"Loaded {len(trajs)} trajectory messages")
    print(f"Loaded {len(odom)} odometry/localization samples")
    print(f"Loaded {len(goals)} goal messages")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", nargs="+", help="Path(s) to rosbag file or rosbag2 directory")
    parser.add_argument("--traj-topic", default=DEFAULT_TRAJ_TOPIC)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--goal-topic", default=DEFAULT_GOAL_TOPIC)
    parser.add_argument("--outdir", default="bag_analysis_results")
    args = parser.parse_args()

    bag_paths = [Path(p) for p in args.bag]
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    trajs, odom, goals = load_bag_data(
        bag_paths=bag_paths,
        traj_topic=args.traj_topic,
        odom_topic=args.odom_topic,
        goal_topic=args.goal_topic,
    )
    print_topic_summary(trajs, odom, goals)

    runs = segment_runs(trajs, odom)
    if not runs:
        raise RuntimeError(
            "No valid runs were detected.\n"
            "Check that your trajectory topic updates once per run and that odom samples exist in the same time window."
        )

    metrics = compute_run_metrics(runs, goals)
    df = pd.DataFrame([m.__dict__ for m in metrics])
    df.to_csv(outdir / "summary_metrics.csv", index=False)

    make_xy_overview_plot(runs, outdir)
    make_cross_track_plots(runs, outdir)
    make_summary_barplots(df, outdir)

    print("\nSaved outputs to:", outdir.resolve())
    print(df.to_string(index=False))

    # Optional quick textual summary
    print("\nAggregate summary:")
    print(f"  Mean of mean CTE:   {df['mean_cross_track_error_m'].mean():.3f} m")
    print(f"  Mean of max CTE:    {df['max_cross_track_error_m'].mean():.3f} m")
    print(f"  Mean final error:   {df['final_goal_error_m'].mean():.3f} m")
    print(f"  Mean run duration:  {df['duration_s'].mean():.3f} s")


if __name__ == "__main__":
    main()