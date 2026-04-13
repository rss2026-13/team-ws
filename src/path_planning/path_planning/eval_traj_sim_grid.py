import math
import os
import time
from datetime import datetime
 
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
 
# ================= CONFIGURATION =================
# Grid planner publishes to /trajectory/current (traj_pub),
# but only after uncommenting self.traj_pub.publish(...) in plan_path().
# The publish_viz() call goes to RViz only — this evaluator needs the PoseArray.
PATH_TOPIC = "/trajectory/current"
# =================================================
 
 
def path_length(poses) -> float:
    if len(poses) < 2:
        return 0.0
    return sum(
        math.hypot(
            poses[i+1].position.x - poses[i].position.x,
            poses[i+1].position.y - poses[i].position.y,
        )
        for i in range(len(poses) - 1)
    )
 
 
def path_smoothness(poses) -> float:
    """Average absolute heading change between consecutive segments (rad).
    Lower = smoother. A perfectly straight path scores 0."""
    if len(poses) < 3:
        return 0.0
    angles = []
    for i in range(1, len(poses) - 1):
        dx1 = poses[i].position.x   - poses[i-1].position.x
        dy1 = poses[i].position.y   - poses[i-1].position.y
        dx2 = poses[i+1].position.x - poses[i].position.x
        dy2 = poses[i+1].position.y - poses[i].position.y
        a = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
        a = (a + math.pi) % (2 * math.pi) - math.pi
        angles.append(abs(a))
    return float(np.mean(angles))
 
 
def path_clearance(poses) -> float:
    """
    Average straight-line segment length (m).
    For a visibility-graph smoothed path, longer segments = fewer unnecessary
    turns = better use of the smoothing pass. Higher is better.
    """
    if len(poses) < 2:
        return 0.0
    segs = [
        math.hypot(
            poses[i+1].position.x - poses[i].position.x,
            poses[i+1].position.y - poses[i].position.y,
        )
        for i in range(len(poses) - 1)
    ]
    return float(np.mean(segs))
 
 
def smoothing_ratio(raw_wps: int, smoothed_wps: int) -> float:
    """Fraction of waypoints removed by the smoothing pass. Higher = more compact."""
    if raw_wps == 0:
        return 0.0
    return 1.0 - smoothed_wps / raw_wps
 
 
class GridEvaluator(Node):
    def __init__(self):
        super().__init__("grid_evaluator")
        self.get_logger().info(f"Grid evaluator | Listening on: {PATH_TOPIC}")
        self.get_logger().info(
            "Uncomment self.traj_pub.publish(...) in the grid planner's plan_path().\n"
            "Then set a goal in RViz. Ctrl-C to print results."
        )
 
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.create_subscription(PoseArray,   PATH_TOPIC,  self.path_cb, 10)
 
        self._goal_time: float | None = None
        self._goal_count = 0
        self._pending_goal = False
 
        # raw waypoint count comes from the planner log; we approximate it here
        # as the pose count before smoothing. Since we only receive the smoothed
        # path over the topic, we track smoothed count and note the limitation.
        self.runs: list[dict] = []
 
    # ------------------------------------------------------------------
 
    def goal_cb(self, msg: PoseStamped):
        self._goal_time = time.perf_counter()
        self._pending_goal = True
        self._goal_count += 1
        self.get_logger().info(
            f"Goal #{self._goal_count} received at "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
 
    def path_cb(self, msg: PoseArray):
        if self._goal_time is None:
            return
 
        elapsed   = time.perf_counter() - self._goal_time
        poses     = msg.poses
        length    = path_length(poses)
        smooth    = path_smoothness(poses)
        clearance = path_clearance(poses)
        n_wp      = len(poses)
 
        # Efficiency: path length per waypoint — higher means fewer, longer hops
        wp_efficiency = length / n_wp if n_wp > 0 else 0.0
 
        if self._pending_goal:
            self.runs.append({
                "goal":          self._goal_count,
                "time_s":        elapsed,
                "length_m":      length,
                "smoothness":    smooth,
                "waypoints":     n_wp,
                "seg_length_m":  clearance,
                "wp_efficiency": wp_efficiency,
            })
            self._pending_goal = False
            self.get_logger().info(
                f"  Path #{self._goal_count} — "
                f"time={elapsed:.3f}s  len={length:.2f}m  "
                f"wps={n_wp}  smoothness={smooth:.4f}rad  "
                f"avg_seg={clearance:.2f}m  wp_eff={wp_efficiency:.3f}m/wp"
            )
        else:
            # Grid planner re-plans on every odom tick — update in place
            self.runs[-1].update({
                "time_s":        elapsed,
                "length_m":      length,
                "smoothness":    smooth,
                "waypoints":     n_wp,
                "seg_length_m":  clearance,
                "wp_efficiency": wp_efficiency,
            })
 
    # ------------------------------------------------------------------
 
    def print_summary(self):
        if not self.runs:
            print("\n[eval] No complete goal→path pairs recorded.")
            print("[eval] Check that self.traj_pub.publish(self.trajectory.toPoseArray())")
            print("       is NOT commented out in the grid planner's plan_path().")
            return
 
        SEP = "=" * 76
        print(f"\n{SEP}")
        print(f"  Grid Planner Evaluation  ({len(self.runs)} goal(s))")
        print(SEP)
        hdr = (f"  {'Goal':<5} {'Time(s)':>8} {'Len(m)':>8} {'WPs':>5} "
               f"{'Smooth':>8} {'AvgSeg(m)':>10} {'WP Eff':>8}")
        print(hdr)
        print(f"  {'-'*5} {'-'*8} {'-'*8} {'-'*5} {'-'*8} {'-'*10} {'-'*8}")
        for r in self.runs:
            print(
                f"  {r['goal']:<5} {r['time_s']:>8.3f} {r['length_m']:>8.3f} "
                f"{r['waypoints']:>5} {r['smoothness']:>8.4f} "
                f"{r['seg_length_m']:>10.3f} {r['wp_efficiency']:>8.3f}"
            )
 
        if len(self.runs) > 1:
            print(f"  {'-'*5} {'-'*8} {'-'*8} {'-'*5} {'-'*8} {'-'*10} {'-'*8}")
            keys = ("time_s", "length_m", "waypoints", "smoothness",
                    "seg_length_m", "wp_efficiency")
            for label, fn in [("MEAN", np.mean), ("STD", np.std)]:
                vals = [fn([r[k] for r in self.runs]) for k in keys]
                print(
                    f"  {label:<5} {vals[0]:>8.3f} {vals[1]:>8.3f} "
                    f"{vals[2]:>5.1f} {vals[3]:>8.4f} "
                    f"{vals[4]:>10.3f} {vals[5]:>8.3f}"
                )
        print(SEP + "\n")
 
 
# ======================================================================
# Plotting
# ======================================================================
 
def plot_summary(runs: list[dict], out_dir: str = "."):
    if not runs:
        return
 
    os.makedirs(out_dir, exist_ok=True)
 
    x = np.arange(1, len(runs) + 1)
 
    metrics = [
        ("time_s",        "Planning Time (s)",       "Planning Time",                   "#4C72B0"),
        ("length_m",      "Path Length (m)",          "Path Length",                     "#DD8452"),
        ("smoothness",    "Avg Turn Angle (rad)",     "Smoothness  (lower = smoother)",  "#C44E52"),
        ("waypoints",     "Waypoint Count",           "Waypoints After Smoothing",       "#55A868"),
        ("seg_length_m",  "Avg Segment Length (m)",   "Avg Segment Length\n(higher = fewer unnecessary turns)", "#8172B2"),
        ("wp_efficiency", "Path Length / Waypoint",   "Waypoint Efficiency\n(higher = longer hops)", "#937860"),
    ]
 
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    fig.suptitle("Grid Path Planner Evaluation (A* + Visibility Smoothing)",
                 fontsize=13, fontweight="bold")
 
    for ax, (key, ylabel, title, color) in zip(axes.flatten(), metrics):
        vals = [r[key] for r in runs]
 
        ax.scatter(x, vals, color=color, s=80, edgecolors="black",
                   linewidth=0.7, zorder=3)
 
        if len(vals) > 1:
            z = np.polyfit(x, vals, 1)
            ax.plot(x, np.poly1d(z)(x), color="black", linestyle="--",
                    linewidth=1.2, label="trend", zorder=2)
 
        mean_val = float(np.mean(vals))
        ax.axhline(mean_val, color=color, linestyle=":", linewidth=1.5,
                   label=f"mean={mean_val:.3f}", zorder=2)
        ax.legend(fontsize=7)
 
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Goal #")
        ax.set_xticks(x)
        ax.grid(axis="y", linestyle="--", alpha=0.4, zorder=1)
 
    plt.tight_layout()
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"eval_GRID_{ts}.png")
    plt.savefig(path, dpi=150)
    print(f"[eval] Plot saved → {path}")
    plt.show()
 
 
# ======================================================================
# Entry point
# ======================================================================
 
def main(args=None):
    rclpy.init(args=args)
    node = GridEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        plot_summary(node.runs)
        node.destroy_node()
        rclpy.shutdown()
 
