#!/usr/bin/env python3
"""
Replay and analyze particle-filter performance on multiple ROS2 bag directories.

What this script does:
1) Optionally re-run PF on each input bag multiple times by orchestrating:
   - map server + particle filter launch
   - optional TF->Odometry relay (for bags that do not contain /vesc/odom)
   - ros2 bag play with configurable start/end window
   - ros2 bag record of /pf topics
2) Analyze the recorded PF outputs and compute quantitative metrics:
   - convergence rate (time-to-confidence)
   - average confidence after convergence
   - PF pose publishing rate
3) Save plots and tabular summaries.

Typical usage (inside the team-ws ROS container):
  python3 evaluations/scripts/analyze_pf_rosbags.py --run-pf --num-runs 3

Analyze previously recorded PF run bags only:
  python3 evaluations/scripts/analyze_pf_rosbags.py --analyze-only
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

PF_ODOM_TOPIC = "/pf/pose/odom"
PF_PARTICLES_TOPIC = "/pf/particles"
TF_TOPIC = "/tf"
DEFAULT_BAGS = ["corridor1", "corridor2", "corridor3", "corridor4"]
BASE_NUM_PARTICLES = 100
BASE_NUM_RAYS = 99
BASE_SIGMA_X = 0.03
BASE_SIGMA_Y = 0.01
BASE_SIGMA_THETA = 0.04


@dataclass
class RunMetrics:
    bag_name: str
    run_id: int
    confidence_source: str
    published_msgs: int
    publish_rate_hz: float
    particle_msgs: int
    converged: bool
    convergence_time_s: float
    post_convergence_confidence: float
    min_confidence: float


def _team_ws_root() -> Path:
    # evaluations/scripts/<this file> -> team-ws root is parents[2]
    return Path(__file__).resolve().parents[2]


def _default_bag_dirs(root: Path) -> list[Path]:
    out: list[Path] = []
    for name in DEFAULT_BAGS:
        p = root / name
        if p.is_dir() and (p / "metadata.yaml").exists():
            out.append(p)
    return out


def _read_topic_messages(bag_dir: Path, topic: str) -> list[tuple[float, Any]]:
    """Return list of (timestamp_sec, deserialized_message)."""
    out: list[tuple[float, Any]] = []
    with AnyReader([bag_dir], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            return out
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.typestore.deserialize_cdr(raw, conn.msgtype)
            out.append((float(ts) * 1e-9, msg))
    return out


def _calc_publish_rate_hz(timestamps_sec: list[float]) -> float:
    if len(timestamps_sec) < 2:
        return float("nan")
    dt = timestamps_sec[-1] - timestamps_sec[0]
    if dt <= 0:
        return float("nan")
    return float((len(timestamps_sec) - 1) / dt)


def _particle_spread_from_pose_array(msg: Any) -> float:
    if not hasattr(msg, "poses") or len(msg.poses) == 0:
        return float("nan")
    xs = np.array([p.position.x for p in msg.poses], dtype=float)
    ys = np.array([p.position.y for p in msg.poses], dtype=float)
    if xs.size < 2:
        return float("nan")
    var_xy = float(np.var(xs) + np.var(ys))
    return math.sqrt(max(var_xy, 0.0))


def _detect_convergence(
    t_sec: np.ndarray,
    spread: np.ndarray,
    threshold: float,
    hold_s: float,
    min_start_s: float = 0.0,
) -> tuple[bool, float]:
    if t_sec.size == 0 or spread.size == 0:
        return False, float("nan")
    valid = np.isfinite(spread)
    t = t_sec[valid]
    s = spread[valid]
    if t.size == 0:
        return False, float("nan")

    for i in range(len(t)):
        if t[i] < min_start_s:
            continue
        if s[i] > threshold:
            continue
        end_t = t[i] + hold_s
        mask = (t >= t[i]) & (t <= end_t)
        if np.any(mask) and np.all(s[mask] <= threshold):
            return True, float(t[i] - t[0])
    return False, float("nan")


def _rolling_jitter_from_odom(
    t_sec: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    window_s: float = 1.0,
) -> np.ndarray:
    """
    Compute rolling positional jitter from odom trajectory:
    jitter(t) = sqrt(var(x_window) + var(y_window)) over trailing window.
    """
    n = len(t_sec)
    out = np.full(n, np.nan, dtype=float)
    if n == 0:
        return out
    start = 0
    for i in range(n):
        while start < i and (t_sec[i] - t_sec[start]) > window_s:
            start += 1
        if i - start + 1 < 3:
            continue
        xw = xs[start : i + 1]
        yw = ys[start : i + 1]
        out[i] = math.sqrt(max(float(np.var(xw) + np.var(yw)), 0.0))
    return out


def _safe_float(v: float) -> str:
    return "nan" if (not isinstance(v, float) or not math.isfinite(v)) else f"{v:.6f}"


def _start_process(
    cmd: list[str],
    cwd: Path,
    log_path: Path,
    env: dict[str, str] | None = None,
) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = open(log_path, "w", encoding="utf-8")
    return subprocess.Popen(
        cmd,
        cwd=str(cwd),
        env=env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _stop_process(proc: subprocess.Popen, grace_s: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    t0 = time.time()
    while proc.poll() is None and (time.time() - t0) < grace_s:
        time.sleep(0.1)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            return


def _publish_initial_pose_once(cwd: Path, x: float, y: float, yaw: float) -> None:
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    msg = (
        "{header: {frame_id: 'map'}, "
        f"pose: {{pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}, "
        "covariance: [0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, "
        "0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0.2]}}"
    )
    subprocess.run(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/initialpose",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            msg,
        ],
        cwd=str(cwd),
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _wait_for_map(cwd: Path, timeout_s: float = 12.0) -> bool:
    """Block until one /map message is observed, or timeout."""
    try:
        result = subprocess.run(
            [
                "ros2",
                "topic",
                "echo",
                "--once",
                "/map",
                "--qos-durability",
                "transient_local",
                "--qos-reliability",
                "reliable",
            ],
            cwd=str(cwd),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout_s,
            check=False,
        )
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def _preflight_map_check(map_yaml: str, params_file: str) -> None:
    """Verify /map can be observed before starting a long sweep."""
    logs_dir = _team_ws_root() / "evaluations" / "pf_quantitative_output" / "logs"
    proc = _start_process(
        [
            "ros2",
            "launch",
            "localization",
            "localize.launch.xml",
            f"map:={map_yaml}",
            f"params_file:={params_file}",
        ],
        cwd=_team_ws_root(),
        log_path=logs_dir / "preflight_map_check_localize.log",
    )
    try:
        time.sleep(2.5)
        if not _wait_for_map(_team_ws_root(), timeout_s=20.0):
            raise RuntimeError("Preflight failed: /map not observed before sweep start")
    finally:
        _stop_process(proc)


def run_pf_replays(
    bag_dirs: list[Path],
    output_root: Path,
    num_runs: int,
    map_yaml: str,
    params_file: str,
    start_sec: float,
    end_sec: float,
    initial_x: float,
    initial_y: float,
    initial_yaw: float,
    initialize_pose: bool = True,
    record_particles: bool = True,
    per_bag_windows: dict[str, tuple[float, float]] | None = None,
    per_bag_known_starts: dict[str, tuple[float, float, float]] | None = None,
    enforce_map_before_play: bool = False,
) -> list[Path]:
    run_bag_dirs: list[Path] = []
    logs_dir = output_root / "logs"
    replay_dir = output_root / "replay_runs"
    replay_dir.mkdir(parents=True, exist_ok=True)
    for bag in bag_dirs:
        bag_start = max(start_sec, 0.0)
        bag_end = end_sec
        if per_bag_windows is not None and bag.name in per_bag_windows:
            bag_start, bag_end = per_bag_windows[bag.name]
            bag_start = max(0.0, bag_start)
        duration = None
        if bag_end > bag_start:
            duration = bag_end - bag_start

        for run_idx in range(1, num_runs + 1):
            run_name = f"{bag.name}_run{run_idx}"
            out_bag_base = replay_dir / run_name

            localize = _start_process(
                [
                    "ros2",
                    "launch",
                    "localization",
                    "localize.launch.xml",
                    f"map:={map_yaml}",
                    f"params_file:={params_file}",
                ],
                cwd=_team_ws_root(),
                log_path=logs_dir / f"{run_name}_localize.log",
            )
            time.sleep(3.0)
            if enforce_map_before_play and not _wait_for_map(_team_ws_root(), timeout_s=20.0):
                _stop_process(localize)
                raise RuntimeError(
                    f"/map not observed for {run_name}; refusing to start replay"
                )

            relay = _start_process(
                [
                    "python3",
                    str(Path(__file__).resolve()),
                    "--mode",
                    "tf-relay",
                ],
                cwd=_team_ws_root(),
                log_path=logs_dir / f"{run_name}_relay.log",
            )
            time.sleep(1.5)

            record_topics = [PF_ODOM_TOPIC]
            if record_particles:
                record_topics.append(PF_PARTICLES_TOPIC)
            record = _start_process(
                ["ros2", "bag", "record", *record_topics, "-o", str(out_bag_base)],
                cwd=_team_ws_root(),
                log_path=logs_dir / f"{run_name}_record.log",
            )
            time.sleep(1.0)

            play_cmd = [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--clock",
                "--rate",
                "1.0",
                "--start-offset",
                str(bag_start),
            ]
            play = _start_process(
                play_cmd,
                cwd=_team_ws_root(),
                log_path=logs_dir / f"{run_name}_play.log",
            )
            time.sleep(1.0)

            if initialize_pose:
                init_pose = (initial_x, initial_y, initial_yaw)
                if per_bag_known_starts is not None and bag.name in per_bag_known_starts:
                    init_pose = per_bag_known_starts[bag.name]
                _publish_initial_pose_once(
                    _team_ws_root(), init_pose[0], init_pose[1], init_pose[2]
                )

            if duration is not None:
                t0 = time.time()
                while play.poll() is None and (time.time() - t0) < duration:
                    time.sleep(0.2)
                if play.poll() is None:
                    _stop_process(play)
            else:
                play.wait(timeout=None)

            # Let callback queue flush before stopping record.
            time.sleep(1.0)
            _stop_process(record)
            _stop_process(relay)
            _stop_process(localize)

            # ros2 bag record writes a directory with suffix _0 in many configs.
            # We keep whichever path contains metadata.yaml.
            candidates = sorted(replay_dir.glob(f"{run_name}*"))
            selected = None
            for c in candidates:
                if c.is_dir() and (c / "metadata.yaml").exists():
                    selected = c
                    break
            if selected is not None:
                run_bag_dirs.append(selected)
            else:
                print(
                    f"[WARN] No recorded output bag found for {run_name}",
                    file=sys.stderr,
                )
    return run_bag_dirs


def _write_params_file(
    path: Path,
    num_particles: int,
    num_rays: int,
    sigma_x: float,
    sigma_y: float,
    sigma_theta: float,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = f"""particle_filter:
  ros__parameters:
    num_particles: {num_particles}
    num_beams_per_particle: {num_rays}
    scan_field_of_view: 4.71
    scan_theta_discretization: 500.0
    map_topic: "/map"
    scan_topic: "/scan"
    odom_topic: "/vesc/odom"
    particle_filter_frame: "pf/base_link"
    deterministic: false
    debug: false
    lidar_scale_to_map_scale: 1.0
    motion_model:
      sigma_x: {sigma_x}
      sigma_y: {sigma_y}
      sigma_theta: {sigma_theta}
"""
    path.write_text(content, encoding="utf-8")


def _load_time_windows_csv(path: Path) -> dict[str, tuple[float, float]]:
    windows: dict[str, tuple[float, float]] = {}
    if not path.exists():
        return windows
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            corridor = (row.get("corridor") or "").strip()
            if not corridor:
                continue
            s_raw = (row.get("selected_start_sec") or "").strip()
            e_raw = (row.get("selected_end_sec") or "").strip()
            if not s_raw or not e_raw:
                continue
            try:
                s = float(s_raw)
                e = float(e_raw)
            except ValueError:
                continue
            if e > s >= 0.0:
                windows[corridor] = (s, e)
    return windows


def _load_known_starts_csv(path: Path) -> dict[str, tuple[float, float, float]]:
    starts: dict[str, tuple[float, float, float]] = {}
    if not path.exists():
        return starts
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            corridor = (row.get("corridor") or "").strip()
            if not corridor:
                continue
            x_raw = (row.get("known_start_x") or "").strip()
            y_raw = (row.get("known_start_y") or "").strip()
            yaw_raw = (row.get("known_start_yaw") or "").strip()
            if not x_raw or not y_raw:
                continue
            try:
                x = float(x_raw)
                y = float(y_raw)
                yaw = float(yaw_raw) if yaw_raw else 0.0
            except ValueError:
                continue
            starts[corridor] = (x, y, yaw)
    return starts


def _setting_name(factor: str, value: str) -> str:
    return f"{factor}_{value}".replace(".", "p")


def _aggregate_publish_rate(
    metrics: list[RunMetrics], factor: str, setting_value: str
) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    by_bag: dict[str, list[float]] = {}
    for m in metrics:
        if math.isfinite(m.publish_rate_hz):
            by_bag.setdefault(m.bag_name, []).append(m.publish_rate_hz)
    for bag_name, vals in sorted(by_bag.items()):
        arr = np.array(vals, dtype=float)
        rows.append(
            {
                "factor": factor,
                "setting": setting_value,
                "bag": bag_name,
                "runs": str(len(vals)),
                "mean_publish_rate_hz": f"{float(np.mean(arr)):.6f}",
                "std_publish_rate_hz": f"{float(np.std(arr)):.6f}",
                "min_publish_rate_hz": f"{float(np.min(arr)):.6f}",
                "max_publish_rate_hz": f"{float(np.max(arr)):.6f}",
            }
        )
    if by_bag:
        all_vals = np.array([v for vals in by_bag.values() for v in vals], dtype=float)
        rows.append(
            {
                "factor": factor,
                "setting": setting_value,
                "bag": "ALL",
                "runs": str(len(all_vals)),
                "mean_publish_rate_hz": f"{float(np.mean(all_vals)):.6f}",
                "std_publish_rate_hz": f"{float(np.std(all_vals)):.6f}",
                "min_publish_rate_hz": f"{float(np.min(all_vals)):.6f}",
                "max_publish_rate_hz": f"{float(np.max(all_vals)):.6f}",
            }
        )
    return rows


def run_publish_rate_sweep(
    bag_dirs: list[Path],
    output_root: Path,
    map_yaml: str,
    start_sec: float,
    end_sec: float,
    initial_x: float,
    initial_y: float,
    initial_yaw: float,
    num_runs: int,
    variance_scales: list[float],
    particle_values: list[int],
    ray_values: list[int],
    convergence_threshold: float,
    convergence_hold_s: float,
    convergence_min_start_s: float,
    min_allowed_publish_rate_hz: float,
    sweep_initialize_pose: bool,
    per_bag_windows: dict[str, tuple[float, float]] | None = None,
    per_bag_known_starts: dict[str, tuple[float, float, float]] | None = None,
) -> None:
    sweep_root = output_root / "publish_rate_sweep"
    sweep_root.mkdir(parents=True, exist_ok=True)

    # One-factor-at-a-time sweep around baseline params.
    settings: list[tuple[str, str, int, int, float, float, float]] = []
    for s in variance_scales:
        settings.append(
            (
                "variance_scale",
                f"{s:.3f}",
                BASE_NUM_PARTICLES,
                BASE_NUM_RAYS,
                BASE_SIGMA_X * s,
                BASE_SIGMA_Y * s,
                BASE_SIGMA_THETA * s,
            )
        )
    for p in particle_values:
        settings.append(
            (
                "num_particles",
                str(p),
                p,
                BASE_NUM_RAYS,
                BASE_SIGMA_X,
                BASE_SIGMA_Y,
                BASE_SIGMA_THETA,
            )
        )
    for r in ray_values:
        settings.append(
            (
                "num_rays",
                str(r),
                BASE_NUM_PARTICLES,
                r,
                BASE_SIGMA_X,
                BASE_SIGMA_Y,
                BASE_SIGMA_THETA,
            )
        )
    # Stress test: high particles with low ray count to keep runtime practical.
    settings.append(
        (
            "stress",
            "2000p10r",
            2000,
            10,
            BASE_SIGMA_X,
            BASE_SIGMA_Y,
            BASE_SIGMA_THETA,
        )
    )

    aggregate_rows: list[dict[str, str]] = []
    plot_labels: list[str] = []
    plot_means: list[float] = []
    plot_errs: list[float] = []

    for idx, (factor, value_label, npart, nrays, sx, sy, st) in enumerate(settings, 1):
        setting_id = _setting_name(factor, value_label)
        setting_root = sweep_root / setting_id
        params_path = setting_root / "params.yaml"
        _write_params_file(params_path, npart, nrays, sx, sy, st)
        print(
            f"[sweep {idx}/{len(settings)}] {factor}={value_label} "
            f"(particles={npart}, rays={nrays}, sigmas=({sx:.4f},{sy:.4f},{st:.4f}))"
        )

        run_bags = run_pf_replays(
            bag_dirs=bag_dirs,
            output_root=setting_root,
            num_runs=num_runs,
            map_yaml=map_yaml,
            params_file=str(params_path),
            start_sec=start_sec,
            end_sec=end_sec,
            initial_x=initial_x,
            initial_y=initial_y,
            initial_yaw=initial_yaw,
            initialize_pose=sweep_initialize_pose,
            record_particles=True,
            per_bag_windows=per_bag_windows,
            per_bag_known_starts=per_bag_known_starts,
            enforce_map_before_play=False,
        )
        metrics = analyze_pf_runs(
            run_bag_dirs=run_bags,
            output_root=setting_root,
            convergence_threshold=convergence_threshold,
            convergence_hold_s=convergence_hold_s,
            convergence_min_start_s=convergence_min_start_s,
        )
        bad_rates = [
            m
            for m in metrics
            if (not math.isfinite(m.publish_rate_hz))
            or (m.publish_rate_hz < min_allowed_publish_rate_hz)
        ]
        if bad_rates:
            details = ", ".join(
                [
                    f"{m.bag_name}_run{m.run_id}:{m.publish_rate_hz:.3f}Hz"
                    if math.isfinite(m.publish_rate_hz)
                    else f"{m.bag_name}_run{m.run_id}:nanHz"
                    for m in bad_rates
                ]
            )
            print(
                "[WARN] Low/invalid publish rates observed "
                f"(threshold {min_allowed_publish_rate_hz:.2f} Hz) in {setting_id}: {details}",
                file=sys.stderr,
            )

        rows = _aggregate_publish_rate(metrics, factor=factor, setting_value=value_label)
        aggregate_rows.extend(rows)
        all_row = next((r for r in rows if r["bag"] == "ALL"), None)
        if all_row is not None:
            plot_labels.append(f"{factor}:{value_label}")
            plot_means.append(float(all_row["mean_publish_rate_hz"]))
            plot_errs.append(float(all_row["std_publish_rate_hz"]))

    agg_csv = sweep_root / "publish_rate_sweep_summary.csv"
    with open(agg_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "factor",
                "setting",
                "bag",
                "runs",
                "mean_publish_rate_hz",
                "std_publish_rate_hz",
                "min_publish_rate_hz",
                "max_publish_rate_hz",
            ],
        )
        writer.writeheader()
        for r in aggregate_rows:
            writer.writerow(r)

    md_path = sweep_root / "publish_rate_sweep_summary.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# PF publish-rate sweep summary\n\n")
        f.write("- Topic measured: `/pf/pose/odom`\n")
        f.write(f"- Runs per setting: {num_runs}\n")
        f.write(f"- Bags per setting: {len(bag_dirs)}\n")
        f.write(f"- Time window: [{start_sec:.2f}, ")
        f.write("end]\n\n" if end_sec < 0 else f"{end_sec:.2f}]\n\n")
        f.write("| factor | setting | bag | runs | mean_hz | std_hz | min_hz | max_hz |\n")
        f.write("|--------|---------|-----|------|---------|--------|--------|--------|\n")
        for r in aggregate_rows:
            f.write(
                f"| {r['factor']} | {r['setting']} | {r['bag']} | {r['runs']} | "
                f"{r['mean_publish_rate_hz']} | {r['std_publish_rate_hz']} | "
                f"{r['min_publish_rate_hz']} | {r['max_publish_rate_hz']} |\n"
            )

    if plot_labels:
        fig, ax = plt.subplots(figsize=(13, 6))
        xs = np.arange(len(plot_labels))
        ax.bar(xs, plot_means, yerr=plot_errs, capsize=4)
        ax.set_xticks(xs)
        ax.set_xticklabels(plot_labels, rotation=45, ha="right")
        ax.set_ylabel("Publish rate [Hz]")
        ax.set_title("PF publish rate sweep (`/pf/pose/odom`)")
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(sweep_root / "publish_rate_sweep.png", dpi=150)
        plt.close(fig)


def analyze_pf_runs(
    run_bag_dirs: list[Path],
    output_root: Path,
    convergence_threshold: float,
    convergence_hold_s: float,
    convergence_min_start_s: float,
) -> list[RunMetrics]:
    metrics: list[RunMetrics] = []
    plots_dir = output_root / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    fig_curve, ax_curve = plt.subplots(figsize=(12, 6))
    fig_rate, ax_rate = plt.subplots(figsize=(12, 6))
    fig_conf, ax_conf = plt.subplots(figsize=(12, 6))

    labels: list[str] = []
    rates: list[float] = []
    confs: list[float] = []

    for run_bag in sorted(run_bag_dirs):
        run_name = run_bag.name
        if "_run" in run_name:
            bag_name, run_tag = run_name.rsplit("_run", 1)
            try:
                run_id = int(run_tag)
            except ValueError:
                run_id = 1
        else:
            bag_name = run_name
            run_id = 1

        odom_msgs = _read_topic_messages(run_bag, PF_ODOM_TOPIC)
        particle_msgs = _read_topic_messages(run_bag, PF_PARTICLES_TOPIC)

        odom_ts = [t for t, _ in odom_msgs]
        publish_rate_hz = _calc_publish_rate_hz(odom_ts)

        part_ts = np.array([t for t, _ in particle_msgs], dtype=float)
        spreads = np.array(
            [_particle_spread_from_pose_array(m) for _, m in particle_msgs], dtype=float
        )
        odom_ts_np = np.array([t for t, _ in odom_msgs], dtype=float)
        odom_x = np.array(
            [m.pose.pose.position.x for _, m in odom_msgs], dtype=float
        )
        odom_y = np.array(
            [m.pose.pose.position.y for _, m in odom_msgs], dtype=float
        )

        if part_ts.size > 0:
            part_ts = part_ts - part_ts[0]
        if odom_ts_np.size > 0:
            odom_ts_np = odom_ts_np - odom_ts_np[0]

        # Prefer particle spread when available; fallback to odom jitter proxy.
        confidence_source = "particles"
        confidence_series_t = part_ts
        confidence_series = spreads
        if not np.any(np.isfinite(spreads)) and odom_ts_np.size > 0:
            confidence_source = "odom_jitter"
            confidence_series_t = odom_ts_np
            confidence_series = _rolling_jitter_from_odom(odom_ts_np, odom_x, odom_y)

        converged, conv_t = _detect_convergence(
            confidence_series_t,
            confidence_series,
            convergence_threshold,
            convergence_hold_s,
            convergence_min_start_s,
        )
        post_conf = float("nan")
        min_conf = float("nan")
        if np.any(np.isfinite(confidence_series)):
            min_conf = float(np.nanmin(confidence_series))
        if converged and np.isfinite(conv_t):
            idx = np.where(confidence_series_t >= conv_t)[0]
            if idx.size > 0:
                post_conf = float(np.nanmean(confidence_series[idx[0] :]))

        metric = RunMetrics(
            bag_name=bag_name,
            run_id=run_id,
            confidence_source=confidence_source,
            published_msgs=len(odom_msgs),
            publish_rate_hz=publish_rate_hz,
            particle_msgs=len(particle_msgs),
            converged=converged,
            convergence_time_s=conv_t,
            post_convergence_confidence=post_conf,
            min_confidence=min_conf,
        )
        metrics.append(metric)

        label = f"{bag_name} r{run_id}"
        if confidence_series_t.size > 0 and confidence_series.size > 0:
            src_suffix = " (p)" if confidence_source == "particles" else " (o)"
            ax_curve.plot(
                confidence_series_t,
                confidence_series,
                alpha=0.7,
                label=f"{label}{src_suffix}",
            )
        labels.append(label)
        rates.append(publish_rate_hz)
        confs.append(post_conf)

    ax_curve.axhline(convergence_threshold, color="black", linestyle="--", alpha=0.7)
    ax_curve.set_title("Particle spread over time")
    ax_curve.set_xlabel("Time from first PF particle msg [s]")
    ax_curve.set_ylabel("Spread sqrt(var(x)+var(y)) [m]")
    ax_curve.grid(True, alpha=0.3)
    if len(metrics) <= 20:
        ax_curve.legend(fontsize=8, ncol=2)
    fig_curve.tight_layout()
    fig_curve.savefig(plots_dir / "spread_vs_time.png", dpi=150)
    plt.close(fig_curve)

    x = np.arange(len(labels))
    ax_rate.bar(x, [0.0 if not math.isfinite(r) else r for r in rates])
    ax_rate.set_xticks(x)
    ax_rate.set_xticklabels(labels, rotation=45, ha="right")
    ax_rate.set_title("PF publishing rate (/pf/pose/odom)")
    ax_rate.set_ylabel("Hz")
    ax_rate.grid(True, axis="y", alpha=0.3)
    fig_rate.tight_layout()
    fig_rate.savefig(plots_dir / "publish_rate.png", dpi=150)
    plt.close(fig_rate)

    ax_conf.bar(x, [0.0 if not math.isfinite(c) else c for c in confs])
    ax_conf.set_xticks(x)
    ax_conf.set_xticklabels(labels, rotation=45, ha="right")
    ax_conf.set_title("Average spread after convergence (lower is better)")
    ax_conf.set_ylabel("Spread [m]")
    ax_conf.grid(True, axis="y", alpha=0.3)
    fig_conf.tight_layout()
    fig_conf.savefig(plots_dir / "post_convergence_confidence.png", dpi=150)
    plt.close(fig_conf)

    csv_path = output_root / "pf_quantitative_metrics.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "bag",
                "run_id",
                "confidence_source",
                "published_msgs",
                "publish_rate_hz",
                "particle_msgs",
                "converged",
                "convergence_time_s",
                "post_convergence_confidence",
                "min_confidence",
            ]
        )
        for m in metrics:
            writer.writerow(
                [
                    m.bag_name,
                    m.run_id,
                    m.confidence_source,
                    m.published_msgs,
                    _safe_float(m.publish_rate_hz),
                    m.particle_msgs,
                    str(m.converged),
                    _safe_float(m.convergence_time_s),
                    _safe_float(m.post_convergence_confidence),
                    _safe_float(m.min_confidence),
                ]
            )

    md_path = output_root / "pf_quantitative_summary.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# PF quantitative testing summary\n\n")
        f.write(
            "- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback\n"
        )
        f.write(
            f"- Converged when spread <= {convergence_threshold:.3f} for at least {convergence_hold_s:.2f}s (starting search after {convergence_min_start_s:.2f}s)\n"
        )
        f.write("- Publishing rate from `/pf/pose/odom` timestamps\n\n")
        f.write("| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |\n")
        f.write("|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|\n")
        for m in metrics:
            f.write(
                f"| {m.bag_name} | {m.run_id} | {m.confidence_source} | {m.converged} | "
                f"{_safe_float(m.convergence_time_s)} | {_safe_float(m.post_convergence_confidence)} | "
                f"{_safe_float(m.min_confidence)} | {_safe_float(m.publish_rate_hz)} | "
                f"{m.published_msgs} | {m.particle_msgs} |\n"
            )

        # Aggregate per bag across runs.
        f.write("\n## Per-bag averages across runs\n\n")
        f.write("| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |\n")
        f.write("|-----|------|----------------|-----------------|--------------------|---------------------|\n")
        by_bag: dict[str, list[RunMetrics]] = {}
        for m in metrics:
            by_bag.setdefault(m.bag_name, []).append(m)
        for bag_name, rows in sorted(by_bag.items()):
            conv_rows = [r for r in rows if r.converged and math.isfinite(r.convergence_time_s)]
            avg_conv = float(np.mean([r.convergence_time_s for r in conv_rows])) if conv_rows else float("nan")
            conf_rows = [r.post_convergence_confidence for r in rows if math.isfinite(r.post_convergence_confidence)]
            rate_rows = [r.publish_rate_hz for r in rows if math.isfinite(r.publish_rate_hz)]
            avg_conf = float(np.mean(conf_rows)) if conf_rows else float("nan")
            avg_rate = float(np.mean(rate_rows)) if rate_rows else float("nan")
            f.write(
                f"| {bag_name} | {len(rows)} | {len(conv_rows)} | "
                f"{_safe_float(avg_conv)} | {_safe_float(avg_conf)} | {_safe_float(avg_rate)} |\n"
            )
    return metrics


def run_tf_to_odom_relay() -> None:
    """ROS mode: relay TF odom->base_link transform to /vesc/odom Odometry."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage

    class TfRelay(Node):
        def __init__(self) -> None:
            super().__init__("tf_to_odom_relay")
            self.pub = self.create_publisher(Odometry, "/vesc/odom", 10)
            self.sub = self.create_subscription(TFMessage, TF_TOPIC, self.cb, 50)
            self.last_t: float | None = None
            self.last_x: float | None = None
            self.last_y: float | None = None
            self.last_yaw: float | None = None

        @staticmethod
        def _yaw_from_quat(z: float, w: float) -> float:
            return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

        def cb(self, msg: TFMessage) -> None:
            for tf in msg.transforms:
                parent = tf.header.frame_id
                child = tf.child_frame_id
                if parent != "odom":
                    continue
                if not child.endswith("base_link"):
                    continue

                t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                yaw = self._yaw_from_quat(
                    tf.transform.rotation.z, tf.transform.rotation.w
                )

                odom = Odometry()
                odom.header = tf.header
                odom.child_frame_id = child
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation = tf.transform.rotation

                if (
                    self.last_t is not None
                    and t > self.last_t
                    and self.last_x is not None
                    and self.last_y is not None
                    and self.last_yaw is not None
                ):
                    dt = t - self.last_t
                    odom.twist.twist.linear.x = (x - self.last_x) / dt
                    odom.twist.twist.linear.y = (y - self.last_y) / dt
                    dyaw = math.atan2(
                        math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw)
                    )
                    odom.twist.twist.angular.z = dyaw / dt

                self.last_t = t
                self.last_x = x
                self.last_y = y
                self.last_yaw = yaw
                self.pub.publish(odom)

    rclpy.init()
    node = TfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run PF repeatedly on corridor bags and analyze convergence/confidence/rate."
    )
    parser.add_argument(
        "--mode",
        default="analyze",
        choices=["analyze", "tf-relay"],
        help="Internal mode. Use default for normal analysis; tf-relay is started automatically during replay.",
    )
    parser.add_argument(
        "--run-pf",
        action="store_true",
        help="Replay input bags through PF and record /pf output before analysis.",
    )
    parser.add_argument(
        "--record-only",
        action="store_true",
        help="Generate PF replay run bags only (skip analysis/plots).",
    )
    parser.add_argument(
        "--run-sweep",
        action="store_true",
        help="Run one-factor-at-a-time sweep for publish rate: variances, particles, rays.",
    )
    parser.add_argument(
        "--sweep-init-mode",
        choices=["known", "unknown"],
        default="known",
        help="Initialization mode for sweep runs.",
    )
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="Skip replay and only analyze existing run output bags.",
    )
    parser.add_argument(
        "--bag-dirs",
        nargs="*",
        default=None,
        help="Input bag directories (default: corridor1 corridor2 corridor3 corridor4 in team-ws root).",
    )
    parser.add_argument(
        "--num-runs",
        type=int,
        default=3,
        help="Number of PF replays per input bag (default: 3).",
    )
    parser.add_argument(
        "--start-sec",
        type=float,
        default=0.0,
        help="Playback start offset in seconds from bag start.",
    )
    parser.add_argument(
        "--end-sec",
        type=float,
        default=-1.0,
        help="Playback end time in seconds from bag start. Negative means play to bag end.",
    )
    parser.add_argument(
        "--map-yaml",
        default="src/localization/maps/stata_basement.yaml",
        help="Map yaml path passed to localization launch.",
    )
    parser.add_argument(
        "--params-file",
        default="src/localization/localization/real_params.yaml",
        help="Base params file for non-sweep --run-pf mode.",
    )
    parser.add_argument(
        "--init-x",
        type=float,
        default=0.0,
        help="Initial pose x for /initialpose.",
    )
    parser.add_argument(
        "--init-y",
        type=float,
        default=0.0,
        help="Initial pose y for /initialpose.",
    )
    parser.add_argument(
        "--init-yaw",
        type=float,
        default=0.0,
        help="Initial pose yaw (radians) for /initialpose.",
    )
    parser.add_argument(
        "--init-mode",
        choices=["known", "unknown", "both"],
        default="known",
        help="known: publish /initialpose; unknown: do not initialize PF; both: run both modes with equal run counts.",
    )
    parser.add_argument(
        "--convergence-threshold",
        type=float,
        default=0.35,
        help="Convergence threshold on spread sqrt(var(x)+var(y)).",
    )
    parser.add_argument(
        "--convergence-hold-sec",
        type=float,
        default=1.5,
        help="How long spread must stay below threshold to count as converged.",
    )
    parser.add_argument(
        "--convergence-min-start-sec",
        type=float,
        default=2.0,
        help="Ignore early transients: start searching for convergence after this time.",
    )
    parser.add_argument(
        "--output-dir",
        default="evaluations/pf_quantitative_output",
        help="Output directory under team-ws root.",
    )
    parser.add_argument(
        "--variance-scales",
        nargs="*",
        type=float,
        default=[0.5, 1.0, 2.0],
        help="Sweep values for common multiplier on motion sigmas (sigma_x/y/theta).",
    )
    parser.add_argument(
        "--particle-values",
        nargs="*",
        type=int,
        default=[50, 100, 200],
        help="Sweep values for num_particles.",
    )
    parser.add_argument(
        "--ray-values",
        nargs="*",
        type=int,
        default=[50, 99, 150],
        help="Sweep values for num_beams_per_particle.",
    )
    parser.add_argument(
        "--min-allowed-publish-rate-hz",
        type=float,
        default=6.0,
        help="Publish-rate warning threshold per run; low values are reported but do not stop the sweep.",
    )
    parser.add_argument(
        "--time-windows-csv",
        default="evaluations/corridor_time_windows.csv",
        help="CSV containing selected_start_sec/selected_end_sec per corridor.",
    )
    parser.add_argument(
        "--use-time-windows-csv",
        action="store_true",
        help="Use per-corridor start/end from --time-windows-csv when available.",
    )
    parser.add_argument(
        "--use-known-starts-csv",
        action="store_true",
        help="Use known-start x/y/yaw per corridor from --time-windows-csv.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.mode == "tf-relay":
        run_tf_to_odom_relay()
        return

    root = _team_ws_root()
    output_root = (root / args.output_dir).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    if args.bag_dirs:
        bag_dirs = [Path(p).resolve() for p in args.bag_dirs]
    else:
        bag_dirs = _default_bag_dirs(root)
    bag_dirs = [b for b in bag_dirs if b.is_dir() and (b / "metadata.yaml").exists()]
    if not bag_dirs:
        print("No valid input bag directories found.", file=sys.stderr)
        sys.exit(1)

    run_bag_dirs: list[Path] = []
    replay_root = output_root / "replay_runs"
    per_bag_windows = (
        _load_time_windows_csv((root / args.time_windows_csv).resolve())
        if args.use_time_windows_csv
        else None
    )
    per_bag_known_starts = (
        _load_known_starts_csv((root / args.time_windows_csv).resolve())
        if args.use_known_starts_csv
        else None
    )

    if args.run_sweep:
        run_publish_rate_sweep(
            bag_dirs=bag_dirs,
            output_root=output_root,
            map_yaml=args.map_yaml,
            start_sec=max(0.0, args.start_sec),
            end_sec=args.end_sec,
            initial_x=args.init_x,
            initial_y=args.init_y,
            initial_yaw=args.init_yaw,
            num_runs=max(1, args.num_runs),
            variance_scales=args.variance_scales,
            particle_values=args.particle_values,
            ray_values=args.ray_values,
            convergence_threshold=args.convergence_threshold,
            convergence_hold_s=args.convergence_hold_sec,
            convergence_min_start_s=max(0.0, args.convergence_min_start_sec),
            min_allowed_publish_rate_hz=args.min_allowed_publish_rate_hz,
            sweep_initialize_pose=(args.sweep_init_mode == "known"),
            per_bag_windows=per_bag_windows,
            per_bag_known_starts=per_bag_known_starts,
        )
        print(f"Done. Sweep outputs written to: {output_root / 'publish_rate_sweep'}")
        return

    if args.run_pf and not args.analyze_only:
        n_runs = max(1, args.num_runs)
        start_sec = max(0.0, args.start_sec)

        if args.init_mode == "both":
            known_root = output_root / "known_start"
            unknown_root = output_root / "unknown_start"
            known_runs = run_pf_replays(
                bag_dirs=bag_dirs,
                output_root=known_root,
                num_runs=n_runs,
                map_yaml=args.map_yaml,
                params_file=args.params_file,
                start_sec=start_sec,
                end_sec=args.end_sec,
                initial_x=args.init_x,
                initial_y=args.init_y,
                initial_yaw=args.init_yaw,
                initialize_pose=True,
                per_bag_windows=per_bag_windows,
                per_bag_known_starts=per_bag_known_starts,
            )
            unknown_runs = run_pf_replays(
                bag_dirs=bag_dirs,
                output_root=unknown_root,
                num_runs=n_runs,
                map_yaml=args.map_yaml,
                params_file=args.params_file,
                start_sec=start_sec,
                end_sec=args.end_sec,
                initial_x=args.init_x,
                initial_y=args.init_y,
                initial_yaw=args.init_yaw,
                initialize_pose=False,
                per_bag_windows=per_bag_windows,
                per_bag_known_starts=per_bag_known_starts,
            )
            run_bag_dirs = known_runs + unknown_runs
        else:
            run_bag_dirs = run_pf_replays(
                bag_dirs=bag_dirs,
                output_root=output_root,
                num_runs=n_runs,
                map_yaml=args.map_yaml,
                params_file=args.params_file,
                start_sec=start_sec,
                end_sec=args.end_sec,
                initial_x=args.init_x,
                initial_y=args.init_y,
                initial_yaw=args.init_yaw,
                initialize_pose=(args.init_mode == "known"),
                per_bag_windows=per_bag_windows,
                per_bag_known_starts=per_bag_known_starts,
            )
    else:
        for p in sorted(replay_root.glob("*")):
            if p.is_dir() and (p / "metadata.yaml").exists():
                run_bag_dirs.append(p)

    if not run_bag_dirs:
        print(
            "No PF run output bags found to analyze. Use --run-pf or point --output-dir to existing replay outputs.",
            file=sys.stderr,
        )
        sys.exit(1)

    if args.record_only:
        print(f"Done. Replay run bags written to: {output_root}")
        return

    analyze_pf_runs(
        run_bag_dirs=run_bag_dirs,
        output_root=output_root,
        convergence_threshold=args.convergence_threshold,
        convergence_hold_s=args.convergence_hold_sec,
        convergence_min_start_s=max(0.0, args.convergence_min_start_sec),
    )
    print(f"Done. Outputs written to: {output_root}")


if __name__ == "__main__":
    main()
