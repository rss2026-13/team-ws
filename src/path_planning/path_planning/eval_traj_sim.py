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
# Sampling planner (RRT*) publishes to /trajectory/current
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
    """Average absolute heading change between segments (rad). Lower = smoother."""
    if len(poses) < 3:
        return 0.0
    angles = []
    for i in range(1, len(poses) - 1):
        dx1 = poses[i].position.x   - poses[i-1].position.x
        dy1 = poses[i].position.y   - poses[i-1].position.y
        dx2 = poses[i+1].position.x - poses[i].position.x
        dy2 = poses[i+1].position.y - poses[i].position.y
        a = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
        a = (a + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]
        angles.append(abs(a))
    return float(np.mean(angles))
 
 
class LiveEvaluator(Node):
    def __init__(self):
        super().__init__("live_evaluator")
 
        self.get_logger().info(f"Sampling (RRT*) evaluator | Listening on: {PATH_TOPIC}")
        self.get_logger().info("Set a goal in RViz to begin. Ctrl-C to print results.")
 
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.create_subscription(PoseArray,   PATH_TOPIC,  self.path_cb, 10)
 
        self._goal_time: float | None = None
        self._goal_count = 0
 
        # One entry per goal→path pair
        self.runs: list[dict] = []
 
        # Track the latest path so we can report it even if no new one arrives
        self._pending_goal = False
 
    def goal_cb(self, msg: PoseStamped):
        self._goal_time = time.perf_counter()
        self._pending_goal = True
        self._goal_count += 1
        self.get_logger().info(
            f"Goal #{self._goal_count} received at "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
 
    def path_cb(self, msg: PoseArray):
        # Ignore path updates that arrive before any goal was set
        if self._goal_time is None:
            return
 
        elapsed = time.perf_counter() - self._goal_time
        poses   = msg.poses
        length  = path_length(poses)
        smooth  = path_smoothness(poses)
 
        if self._pending_goal:
            # First path after this goal — record a new run
            self.runs.append({
                "goal":       self._goal_count,
                "time_s":     elapsed,
                "length_m":   length,
                "smoothness": smooth,
                "waypoints":  len(poses),
            })
            self._pending_goal = False
            self.get_logger().info(
                f"  Path received — time={elapsed:.2f}s  "
                f"len={length:.2f}m  waypoints={len(poses)}  "
                f"smoothness={smooth:.4f}rad"
            )
        else:
            # Subsequent optimised path for the same goal — update in place
            self.runs[-1].update({
                "time_s":     elapsed,
                "length_m":   length,
                "smoothness": smooth,
                "waypoints":  len(poses),
            })
 
    def print_summary(self):
        if not self.runs:
            print("\n[eval] No complete goal→path pairs recorded.")
            return
 
        SEP = "=" * 58
        print(f"\n{SEP}")
        print(f"  Evaluation Summary — SAMPLING  ({len(self.runs)} goal(s))")
        print(SEP)
        print(f"  {'Goal':<6} {'Time (s)':>10} {'Length (m)':>12} {'Waypoints':>10} {'Smoothness':>12}")
        print(f"  {'-'*6} {'-'*10} {'-'*12} {'-'*10} {'-'*12}")
        for r in self.runs:
            print(
                f"  {r['goal']:<6} {r['time_s']:>10.3f} {r['length_m']:>12.3f} "
                f"{r['waypoints']:>10} {r['smoothness']:>12.4f}"
            )
 
        if len(self.runs) > 1:
            print(f"  {'-'*6} {'-'*10} {'-'*12} {'-'*10} {'-'*12}")
            for label, key in [("mean", None), ("std", None)]:
                vals = {k: [r[k] for r in self.runs]
                        for k in ("time_s", "length_m", "waypoints", "smoothness")}
                fn = np.mean if label == "mean" else np.std
                print(
                    f"  {label.upper():<6} "
                    f"{fn(vals['time_s']):>10.3f} "
                    f"{fn(vals['length_m']):>12.3f} "
                    f"{fn(vals['waypoints']):>10.1f} "
                    f"{fn(vals['smoothness']):>12.4f}"
                )
 
        print(SEP + "\n")
 
 
def plot_summary(runs: list[dict], out_dir: str = "."):
    """Save a 2x2 figure with per-goal bars and trend lines for each metric."""
    if not runs:
        return
 
    os.makedirs(out_dir, exist_ok=True)
    goals      = [r["goal"]       for r in runs]
    times      = [r["time_s"]     for r in runs]
    lengths    = [r["length_m"]   for r in runs]
    waypoints  = [r["waypoints"]  for r in runs]
    smoothness = [r["smoothness"] for r in runs]
 
    metrics = [
        (times,      "Planning Time (s)",     "Planning Time",              "#4C72B0"),
        (lengths,    "Path Length (m)",        "Path Length",                "#DD8452"),
        (waypoints,  "Waypoints",              "Waypoint Count",             "#55A868"),
        (smoothness, "Avg Turn Angle (rad)",   "Smoothness\n(lower=smoother)", "#C44E52"),
    ]
 
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    fig.suptitle("Sampling Planner Evaluation (RRT*)", fontsize=14, fontweight="bold")
 
    x = np.arange(1, len(goals) + 1)
 
    for ax, (vals, ylabel, title, color) in zip(axes.flatten(), metrics):
        # Bar chart
        ax.scatter(x, vals, color=color, s=80, edgecolors="black",
                   linewidth=0.7, zorder=3)
 
        # Trend line (only meaningful with >1 run)
        if len(vals) > 1:
            z = np.polyfit(x, vals, 1)
            ax.plot(x, np.poly1d(z)(x), color="black", linestyle="--",
                    linewidth=1.2, label="trend", zorder=2)
            ax.legend(fontsize=8)
 
        # Mean line
        mean_val = float(np.mean(vals))
        ax.axhline(mean_val, color=color, linestyle=":", linewidth=1.5,
                   label=f"mean={mean_val:.3f}", zorder=2)
 
        ax.set_title(title, fontsize=11)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Goal #")
        ax.set_xticks(x)
        ax.grid(axis="y", linestyle="--", alpha=0.4, zorder=1)
 
    plt.tight_layout()
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"eval_SAMPLING_{ts}.png")
    plt.savefig(path, dpi=150)
    print(f"[eval] Plot saved → {path}")
    plt.show()
 
 
def main(args=None):
    rclpy.init(args=args)
    node = LiveEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        plot_summary(node.runs)
        node.destroy_node()
        rclpy.shutdown()
