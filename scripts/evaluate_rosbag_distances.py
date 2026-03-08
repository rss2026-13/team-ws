#!/usr/bin/env python3
"""
Evaluate wall distance from lidar using each of the 4 branch methods (muktha, tissany, ansisg, jeryl)
over all rosbags in rosbag_data/, and write aggregated stats to evaluation_stats.csv and evaluation_stats.md.

Run from repo root (no ROS required on macOS):
  uv run scripts/evaluate_rosbag_distances.py

On Linux with ROS 2: source /opt/ros/humble/setup.bash then run the same.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# Prefer ROS 2 reader when available; else use rosbags (works without ROS, e.g. on macOS)
_USE_ROS = False
LaserScan = None  # type: ignore
rclpy = None

try:
    import rclpy as _rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan as _LaserScan
    import rosbag2_py
    _USE_ROS = True
    LaserScan = _LaserScan
    rclpy = _rclpy
except ImportError:
    pass

if not _USE_ROS:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
        _ROS2_TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
    except ImportError:
        print(
            "Neither ROS 2 (rclpy, rosbag2_py) nor rosbags available. "
            "Install: uv sync (adds rosbags), or source ROS and use its Python.",
            file=sys.stderr,
        )
        raise SystemExit(1) from None

# ---------------------------------------------------------------------------
# Branch-specific distance-from-scan logic (pure functions; scan = LaserScan-like with .ranges, .angle_min, .angle_max, .angle_increment)
# side: 1 = left wall, -1 = right wall
# ---------------------------------------------------------------------------


def distance_muktha(ranges: np.ndarray, angle_min: float, angle_max: float, angle_increment: float, side: int) -> float | None:  # pylint: disable=unused-argument
    """Muktha: filter by angle (margin pi*0.1, window pi*0.7), 16th percentile closest points, least-squares line, dist = |b|/sqrt(1+m^2)."""
    margin = math.pi * 0.1
    window = math.pi * 0.7
    n = len(ranges)
    if n == 0 or angle_increment <= 0:
        return None
    angles = angle_min + np.arange(n) * angle_increment
    # Left wall (side > 0): use left side of scan (angles near angle_max, positive in REP 105).
    # Right wall (side < 0): use right side of scan (angles near angle_min, negative in REP 105).
    if side > 0:
        end_angle = min(angle_max - margin, angle_max)
        start_angle = max(end_angle - window, angle_min)
    else:
        start_angle = max(angle_min + margin, angle_min)
        end_angle = min(start_angle + window, angle_max)
    mask = (angles >= start_angle) & (angles <= end_angle)
    r = np.array(ranges, dtype=float)[mask]
    a = angles[mask]
    valid = np.isfinite(r)
    if not np.any(valid):
        return None
    r, a = r[valid], a[valid]
    p30 = np.percentile(r, 16)
    used = r <= p30
    if np.sum(used) < 2:
        used = np.ones(len(r), dtype=bool)
    x = r[used] * np.cos(a[used])
    y = r[used] * np.sin(a[used])
    den = len(x) * np.sum(x**2) - (np.sum(x)**2)
    if abs(den) < 1e-6:
        return None
    m = (len(x) * np.sum(x * y) - np.sum(x) * np.sum(y)) / den
    b = (np.sum(y) - m * np.sum(x)) / len(x)
    return float(abs(b) / math.sqrt(1 + m * m))


def distance_tissany(ranges: np.ndarray, angle_min: float, angle_max: float, angle_increment: float, side: int) -> float | None:  # pylint: disable=unused-argument
    """Tissany: half-scan (left/right by index), linregress, dist = |intercept|/sqrt(slope^2+1)."""
    try:
        from scipy import stats
    except ImportError:
        return None
    n = len(ranges)
    if n < 3:
        return None
    angles = np.linspace(angle_min, angle_max, n)
    half = n // 2
    if side == 1:
        r = np.array(ranges[half + 1 : -1], dtype=float)
        a = angles[half + 1 : -1]
    else:
        r = np.array(ranges[0:half], dtype=float)
        a = angles[0:half]
    valid = np.isfinite(r)
    if np.sum(valid) < 2:
        return None
    x = r[valid] * np.cos(a[valid])
    y = r[valid] * np.sin(a[valid])
    slope, intercept, _, _, _ = stats.linregress(x, y)
    return float(abs(intercept) / np.sqrt(slope**2 + 1))


def _angles_points_from_scan(ranges: np.ndarray, angle_min: float, angle_max: float) -> tuple[np.ndarray, np.ndarray]:
    n = len(ranges)
    angles = np.linspace(angle_min, angle_max, n)
    points = np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])
    return angles, points


def _iepf(points: np.ndarray, D_t: float) -> list[np.ndarray]:
    if len(points) < 2:
        return [points]
    start, end = 0, len(points) - 1
    line_vec = points[end] - points[start]
    norm = np.linalg.norm(line_vec)
    if norm < 1e-9:
        return [points]
    line_vec /= norm
    point_vecs = points - points[start]
    projections = point_vecs @ line_vec
    closest = np.outer(projections, line_vec) + points[start]
    distances = np.linalg.norm(points - closest, axis=1)
    max_i = np.argmax(distances)
    if distances[max_i] > D_t:
        left = _iepf(points[: max_i + 1], D_t)
        right = _iepf(points[max_i:], D_t)
        return left + right
    return [points]


def _fit_segment(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mean = points.mean(axis=0)
    centered = points - mean
    _, _, vh = np.linalg.svd(centered)
    direction = vh[0]
    t = centered @ direction
    t_min, t_max = t.min(), t.max()
    p1 = mean + t_min * direction
    p2 = mean + t_max * direction
    return p1, p2


def _point_dir(wall: tuple[np.ndarray, np.ndarray], dir_xy: tuple[float, float], margin: float = 0.3) -> np.ndarray | None:
    p1, p2 = wall
    line_vec = p2 - p1
    line_len = np.linalg.norm(line_vec)
    if line_len < 1e-9:
        return None
    line_vec /= line_len
    dir_vec = np.array(dir_xy, dtype=float)
    dir_vec /= np.linalg.norm(dir_vec)
    det = line_vec[0] * dir_vec[1] - line_vec[1] * dir_vec[0]
    if abs(det) < 1e-9:
        return None
    t = (dir_vec[0] * (p1[1] - 0) - dir_vec[1] * (p1[0] - 0)) / det
    u = (line_vec[0] * (p1[1] - 0) - line_vec[1] * (p1[0] - 0)) / det
    if t < -margin or t > line_len + margin or u < 0:
        return None
    return p1 + t * line_vec


def distance_ansisg(ranges: np.ndarray, angle_min: float, angle_max: float, angle_increment: float, side: int) -> float | None:  # pylint: disable=unused-argument
    """Ansisg: IEPF wall segments, then raycast side rays; distance = min distance to wall along those rays."""
    angles, points = _angles_points_from_scan(ranges, angle_min, angle_max)
    mask = np.isfinite(ranges) & (np.abs(angles) < np.pi * 0.5)
    points = points[mask]
    if len(points) < 2:
        return None
    clusters = _iepf(points, D_t=0.1)
    walls = []
    for cluster in clusters:
        if len(cluster) < 10:
            continue
        walls.append(_fit_segment(cluster))
    if not walls:
        return None
    side_spread = 45.0 * math.pi / 180.0
    side_samples = 11
    if side == 1:
        ray_angles = np.linspace(math.pi / 2 - side_spread, math.pi / 2, side_samples)
    else:
        ray_angles = np.linspace(-math.pi / 2, -math.pi / 2 + side_spread, side_samples)
    closest = np.inf
    for angle in ray_angles:
        d = (math.cos(angle), math.sin(angle))
        for wall in walls:
            pt = _point_dir(wall, d, 0.1)
            if pt is not None:
                closest = min(closest, np.linalg.norm(pt))
    return float(closest) if closest != np.inf else None


def distance_jeryl(ranges: np.ndarray, angle_min: float, angle_max: float, angle_increment: float, side: int) -> float | None:  # pylint: disable=unused-argument
    """Jeryl: filter (SIDE*y > -1, x in 0..5), polyfit(x,y,1), dist = |b|/sqrt(m^2+1)."""
    n = len(ranges)
    if n == 0:
        return None
    angles = np.linspace(angle_min, angle_max, n)
    x = np.array(ranges, dtype=float) * np.cos(angles)
    y = np.array(ranges, dtype=float) * np.sin(angles)
    mask = (side * y > -1) & (x > 0.0) & (x < 5.0) & np.isfinite(ranges)
    x, y = x[mask], y[mask]
    if len(x) < 2:
        return None
    coeffs = np.polyfit(x, y, 1)
    m, b = coeffs[0], coeffs[1]
    return float(abs(b) / math.sqrt(m * m + 1))


METHODS = {
    "muktha": distance_muktha,
    "tissany": distance_tissany,
    "ansisg": distance_ansisg,
    "jeryl": distance_jeryl,
}


def side_from_bag_name(name: str) -> int:
    """Return 1 for left, -1 for right from bag dir name like straight_inward_muktha_left_1."""
    return 1 if "left" in name.lower() else -1


def test_case_id_from_bag_name(name: str) -> str:
    """Strip trailing _1, _2, _3 to get test case (3 runs per test case). E.g. straight_inward_muktha_left_1 -> straight_inward_muktha_left."""
    if name.endswith("_1") or name.endswith("_2") or name.endswith("_3"):
        return name.rsplit("_", 1)[0]
    return name


def _read_scans_rosbags(bag_path: str, topic: str = "/scan") -> list[tuple[float, SimpleNamespace]]:
    """Read /scan from a ROS2 bag using rosbags (no ROS required). Returns (timestamp_ns, scan_like)."""
    out = []
    bag_dir = Path(bag_path)
    if not bag_dir.is_dir():
        return out
    try:
        with AnyReader([bag_dir], default_typestore=_ROS2_TYPESTORE) as reader:
            connections = [c for c in reader.connections if c.topic == topic]
            if not connections:
                return out
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.typestore.deserialize_cdr(rawdata, connection.msgtype)
                scan = SimpleNamespace(
                    ranges=list(getattr(msg, "ranges", [])),
                    angle_min=float(getattr(msg, "angle_min", 0.0)),
                    angle_max=float(getattr(msg, "angle_max", 0.0)),
                    angle_increment=float(getattr(msg, "angle_increment", 0.0)),
                )
                out.append((float(timestamp), scan))
    except Exception:  # anyreader can raise on open or deserialize
        pass
    return out


def get_per_scan_distances(
    scans: list[tuple[float, SimpleNamespace]],
    side: int,
) -> dict[str, list[float]]:
    """For a list of (timestamp, scan) and side (1=left, -1=right), return per-method lists of distances.
    Keys: 'muktha', 'tissany', 'ansisg', 'jeryl', 'median_of_4'. Each value has len(scans) elements (nan if no value)."""
    method_keys = list(METHODS.keys())
    n = len(scans)
    out: dict[str, list[float]] = {k: [] for k in method_keys}
    out["median_of_4"] = []
    ranges_list = [np.array(s.ranges, dtype=float) for _, s in scans]
    for (_, msg), ranges in zip(scans, ranges_list):
        inc = msg.angle_increment
        if inc <= 0 and len(msg.ranges) > 1:
            inc = (msg.angle_max - msg.angle_min) / (len(msg.ranges) - 1)
        four = [fn(ranges, msg.angle_min, msg.angle_max, inc, side) for fn in METHODS.values()]
        valid = [d for d in four if d is not None]
        med = float(np.median(valid)) if valid else float("nan")
        for k, d in zip(method_keys, four):
            out[k].append(d if d is not None else float("nan"))
        out["median_of_4"].append(med)
    return out


def read_scans_from_bag(bag_path: str, topic: str = "/scan") -> list[tuple[float, SimpleNamespace]]:
    """(timestamp_ns, scan) for each /scan message. scan has .ranges, .angle_min, .angle_max, .angle_increment."""
    if _USE_ROS:
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        storage_filter = rosbag2_py.StorageFilter(topics=[topic])
        reader.set_filter(storage_filter)
        out = []
        while reader.has_next():
            entry = reader.read_next()
            if len(entry) == 3:
                _topic_name, data, timestamp = entry
            else:
                _topic_name, data = entry
                timestamp = 0
            msg = deserialize_message(data, LaserScan())
            out.append((timestamp, msg))
        return out
    return _read_scans_rosbags(bag_path, topic)


def _metrics_description() -> str:
    return """## Metrics (per bag, per method)

- **count**: Number of lidar scans in the bag for which this method produced a valid distance (meters).
- **mean**: Mean of those distances (average wall distance over the run).
- **std**: Standard deviation of the distances (spread; higher = more variation/oscillation).
- **min** / **max**: Minimum and maximum distance observed.
- **mean_abs_error**: Average absolute error (meters) vs the desired wall distance of **0.6 m**; i.e. mean of |distance − 0.6| over scans. Lower = closer to target.

Each **method** is a different way of estimating "distance to the wall" from the same lidar scan:
- **muktha**, **tissany**, **ansisg**, **jeryl**: The four branch algorithms (angle-filtered line fit, half-scan linregress, IEPF segments + raycast, polyfit on side filter).
- **median_of_4**: For each scan, the median of the four methods' distances; then we report count/mean/std/min/max over those per-scan medians. Use this to compare a single aggregate estimate vs individual methods.

**Test case structure:** Each test case has 3 runs (e.g. straight_inward_muktha_left_1, _2, _3). There are 3 runs per (runner, side): 3 left + 3 right per runner. The **per test case (3 runs)** tables pool all scans from the 3 bags in that test case and report one row per (test_case, method).

**tissany_left:** Bags in this test case are truncated to the average scan count of the other two left runs (_2 and _3) so the long run (_1) does not dominate the pooled stats.
"""


def run_evaluation(rosbag_data_dir: str, output_dir: str) -> None:
    rosbag_data_dir = Path(rosbag_data_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    bag_dirs = sorted([d for d in rosbag_data_dir.iterdir() if d.is_dir() and d.name.startswith("straight_inward_") and (d / "metadata.yaml").exists()])
    if not bag_dirs:
        print("No rosbag directories found under", rosbag_data_dir, file=sys.stderr)
        return

    # Pre-pass: compute tissany_left target length from the other two left runs (_2 and _3)
    tissany_left_target_len: int | None = None
    tissany_left_dirs = sorted([d for d in bag_dirs if d.name.startswith("straight_inward_tissany_left")])
    if len(tissany_left_dirs) >= 3:
        counts: list[int] = []
        try:
            for d in tissany_left_dirs:
                scans_pre = read_scans_from_bag(str(d))
                counts.append(len(scans_pre) if scans_pre else 0)
            # target = average of _2 and _3 (indices 1 and 2 after sort: ..._left_1, _2, _3)
            c2, c3 = counts[1], counts[2]
            if c2 > 0 and c3 > 0:
                tissany_left_target_len = int(round((c2 + c3) / 2.0))
                print(f"[1/3] tissany_left target length (avg of _2 and _3): {tissany_left_target_len} scans")
        except (OSError, RuntimeError):
            pass  # skip truncation if pre-pass fails

    n_bags = len(bag_dirs)
    print(f"[1/3] Found {n_bags} rosbag(s) (3 runs per test case: left + right per runner). Starting evaluation...")
    all_rows = []
    # Pool distances and per-scan errors per (test_case_id, method) for 3-run aggregation
    pooled: dict[tuple[str, str], list[float]] = {}
    pooled_errors: dict[tuple[str, str], list[float]] = {}
    method_order = list(METHODS.keys()) + ["median_of_4"]
    DESIRED_DISTANCE = 0.6  # meters; reference for mean_abs_error

    for idx, bag_dir in enumerate(bag_dirs):
        name = bag_dir.name
        test_case_id = test_case_id_from_bag_name(name)
        print(f"  [{idx + 1}/{n_bags}] Processing: {name} (test case: {test_case_id})")
        side = side_from_bag_name(name)
        try:
            scans = read_scans_from_bag(str(bag_dir))
        except (OSError, RuntimeError) as e:
            print(f"    Skip: {e}")
            continue
        if not scans:
            print("    No /scan messages in bag, skipping.")
            continue
        # Truncate tissany_left bags to average length of the other two left runs
        if test_case_id == "straight_inward_tissany_left" and tissany_left_target_len is not None:
            n_before = len(scans)
            scans = scans[:tissany_left_target_len]
            if n_before > len(scans):
                print(f"    Truncated to {len(scans)} scans (tissany_left target)")
        print(f"    Scans: {len(scans)}")
        ranges_list = [np.array(s.ranges, dtype=float) for _, s in scans]
        method_keys = list(METHODS.keys())
        # One pass: per-scan median of 4 methods + per-method distances and absolute errors vs desired distance (0.6 m)
        median_of_four_dists = []
        dists_per_method: dict[str, list[float]] = {k: [] for k in method_keys}
        errors_per_method: dict[str, list[float]] = {k: [] for k in method_keys}
        for (_, msg), ranges in zip(scans, ranges_list):
            inc = msg.angle_increment
            if inc <= 0 and len(msg.ranges) > 1:
                inc = (msg.angle_max - msg.angle_min) / (len(msg.ranges) - 1)
            four = [fn(ranges, msg.angle_min, msg.angle_max, inc, side) for fn in METHODS.values()]
            valid = [d for d in four if d is not None]
            med = float(np.median(valid)) if valid else None
            if med is not None:
                median_of_four_dists.append(med)
            for k, d in zip(method_keys, four):
                if d is not None:
                    dists_per_method[k].append(d)
                    errors_per_method[k].append(abs(d - DESIRED_DISTANCE))
        # Individual methods
        for method_key in method_keys:
            dists = dists_per_method[method_key]
            errors = errors_per_method[method_key]
            key = (test_case_id, method_key)
            pooled.setdefault(key, []).extend(dists)
            pooled_errors.setdefault(key, []).extend(errors)
            mean_abs_err = float(np.mean(errors)) if errors else float("nan")
            if dists:
                arr = np.array(dists)
                all_rows.append({
                    "bag": name,
                    "method": method_key,
                    "side": "left" if side == 1 else "right",
                    "count": len(arr),
                    "mean": float(np.mean(arr)),
                    "std": float(np.std(arr)) if len(arr) > 1 else 0.0,
                    "min": float(np.min(arr)),
                    "max": float(np.max(arr)),
                    "mean_abs_error": mean_abs_err,
                    "aggregation": "per_bag",
                })
            else:
                all_rows.append({"bag": name, "method": method_key, "side": "left" if side == 1 else "right", "count": 0, "mean": float("nan"), "std": float("nan"), "min": float("nan"), "max": float("nan"), "mean_abs_error": float("nan"), "aggregation": "per_bag"})
        # Median-of-4 (separate method; error = |median - desired|)
        key_med = (test_case_id, "median_of_4")
        pooled.setdefault(key_med, []).extend(median_of_four_dists)
        errors_med = [abs(m - DESIRED_DISTANCE) for m in median_of_four_dists]
        pooled_errors.setdefault(key_med, []).extend(errors_med)
        if median_of_four_dists:
            arr = np.array(median_of_four_dists)
            mean_abs_err_med = float(np.mean(errors_med))
            all_rows.append({
                "bag": name,
                "method": "median_of_4",
                "side": "left" if side == 1 else "right",
                "count": len(arr),
                "mean": float(np.mean(arr)),
                "std": float(np.std(arr)) if len(arr) > 1 else 0.0,
                "min": float(np.min(arr)),
                "max": float(np.max(arr)),
                "mean_abs_error": mean_abs_err_med,
                "aggregation": "per_bag",
            })
        else:
            all_rows.append({"bag": name, "method": "median_of_4", "side": "left" if side == 1 else "right", "count": 0, "mean": float("nan"), "std": float("nan"), "min": float("nan"), "max": float("nan"), "mean_abs_error": float("nan"), "aggregation": "per_bag"})

    # Aggregated rows: one per (test_case_id, method) pooling all 3 runs
    for (tc_id, method_key), dists in pooled.items():
        if not dists:
            continue
        arr = np.array(dists)
        errs = pooled_errors.get((tc_id, method_key), [])
        mean_abs_err = float(np.mean(errs)) if errs else float("nan")
        side = "left" if "left" in tc_id else "right"
        all_rows.append({
            "bag": f"{tc_id} (3 runs)",
            "method": method_key,
            "side": side,
            "count": len(arr),
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)) if len(arr) > 1 else 0.0,
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "mean_abs_error": mean_abs_err,
            "aggregation": "3_runs",
        })

    # Order: by method, then by bag (per_bag first, then 3_runs)
    def row_sort_key(r: dict) -> tuple:
        m = method_order.index(r["method"])
        agg = 0 if r.get("aggregation") == "per_bag" else 1
        return (m, agg, r["bag"])
    all_rows.sort(key=row_sort_key)

    print(f"[2/3] Writing evaluation_stats.csv and evaluation_stats.md (per bag + per test case 3 runs)...")
    csv_path = output_dir / "evaluation_stats.csv"
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("bag,method,side,aggregation,count,mean,std,min,max,mean_abs_error\n")
        for r in all_rows:
            agg = r.get("aggregation", "per_bag")
            err = r.get("mean_abs_error", float("nan"))
            err_str = f"{err:.6f}" if (err == err) else "nan"
            f.write(f"{r['bag']},{r['method']},{r['side']},{agg},{r['count']},{r['mean']:.6f},{r['std']:.6f},{r['min']:.6f},{r['max']:.6f},{err_str}\n")
    print(f"  Wrote {csv_path}")

    md_path = output_dir / "evaluation_stats.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Distance evaluation stats\n\n")
        f.write(_metrics_description())
        f.write("\n---\n\n")
        for method in method_order:
            rows = [r for r in all_rows if r["method"] == method]
            if not rows:
                continue
            f.write(f"## Method: `{method}`\n\n")
            per_bag = [r for r in rows if r.get("aggregation") == "per_bag"]
            if per_bag:
                f.write("### Per bag (individual runs)\n\n")
                f.write("| bag | side | count | mean | std | min | max | avg error |\n")
                f.write("|-----|------|-------|------|-----|-----|-----|----------|\n")
                for r in per_bag:
                    err = r.get("mean_abs_error", float("nan"))
                    err_s = f"{err:.4f}" if err == err else "—"
                    f.write(f"| {r['bag']} | {r['side']} | {r['count']} | {r['mean']:.4f} | {r['std']:.4f} | {r['min']:.4f} | {r['max']:.4f} | {err_s} |\n")
                f.write("\n")
            three_runs = [r for r in rows if r.get("aggregation") == "3_runs"]
            if three_runs:
                f.write("### Per test case (3 runs pooled)\n\n")
                f.write("| test case | side | count | mean | std | min | max | avg error |\n")
                f.write("|------------|------|-------|------|-----|-----|-----|----------|\n")
                for r in three_runs:
                    err = r.get("mean_abs_error", float("nan"))
                    err_s = f"{err:.4f}" if err == err else "—"
                    f.write(f"| {r['bag']} | {r['side']} | {r['count']} | {r['mean']:.4f} | {r['std']:.4f} | {r['min']:.4f} | {r['max']:.4f} | {err_s} |\n")
                f.write("\n")
    print(f"  Wrote {md_path}")
    print("[3/3] Done.")


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    rosbag_data_dir = repo_root / "rosbag_data"
    output_dir = repo_root
    if not rosbag_data_dir.is_dir():
        print("Not found:", rosbag_data_dir, file=sys.stderr)
        sys.exit(1)
    if _USE_ROS and rclpy is not None:
        rclpy.init()
    try:
        run_evaluation(str(rosbag_data_dir), str(output_dir))
    finally:
        if _USE_ROS and rclpy is not None:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
