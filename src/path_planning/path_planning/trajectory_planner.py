import math
import random
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from path_planning.utils import LineTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class PathPlan(Node):
    """Listens for goal pose published by RViz and plans a path."""

    def __init__(self):
        super().__init__("trajectory_planner")
        def p(name): return self.declare_parameter(name, None).value  # noqa: used below

        self.declare_parameter("odom_topic", "default")
        self.declare_parameter("map_topic", "default")
        self.declare_parameter("wall_buffer_m", 0.4)
        self.declare_parameter("rrt_phase_b_max_seconds", 30.0)
        self.declare_parameter("rrt_phase_b_corridor_radius_m", 0.75)
        self.declare_parameter("rrt_phase_b_wall_bias_radius_m", 0.75)

        gp = lambda name: self.get_parameter(name).value
        self.odom_topic = gp("odom_topic")
        self.map_topic = gp("map_topic")
        self.wall_buffer_m = gp("wall_buffer_m")
        self.phase_b_max_seconds = gp("rrt_phase_b_max_seconds")
        self.phase_b_corridor_radius_m = gp("rrt_phase_b_corridor_radius_m")

        self.goal_bias = 0.05
        self.goal_bias_radius_m = 3.0

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.pose_cb, 10)
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory/current", 10)
        self.tree_pub = self.create_publisher(Marker, "/planned_trajectory/tree", 1)
        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

        self.pose = None
        self.goal = None
        self.map_grid = None
        self.map_width = self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = self.map_origin_y = 0.0
        self.cos_yaw = 1.0
        self.sin_yaw = 0.0
        self.obstacle_cells = []

        self._needs_replan = False
        self._tree_nodes = None
        self._tree_parents = None
        self._tree_costs = None
        self._tree_goal = None
        self._tree_start = None
        self._tree_goal_idx = None
        self._phase = None
        self._phase_start_time = None
        self._current_path = []
        self._t_last_viz = 0.0
        self._step_size = self._neighbor_radius = self._goal_radius = self._corridor_radius_px = 1

        self.create_timer(0.01, self._planning_tick)

    # ── Map ──────────────────────────────────────────────────────────────────

    def map_cb(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        occupancy = np.logical_or(grid == -1, grid > 50)
        buf = max(0, int(math.ceil(self.wall_buffer_m / msg.info.resolution)))
        self.map_grid = self._inflate(occupancy, buf)
        self.map_width, self.map_height = msg.info.width, msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.cos_yaw, self.sin_yaw = math.cos(yaw), math.sin(yaw)
        ys, xs = np.where(self.map_grid)
        self.obstacle_cells = list(zip(xs.tolist(), ys.tolist()))
        self.get_logger().info(f"Received map ({self.map_width}x{self.map_height}) for RRT*.")

    def _inflate(self, occ, r):
        if r <= 0:
            return occ
        h, w = occ.shape
        out = occ.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx*dx + dy*dy > r*r:
                    continue
                xs0, xs1 = max(0, -dx), min(w, w - dx)
                ys0, ys1 = max(0, -dy), min(h, h - dy)
                xd0, xd1 = max(0, dx), min(w, w + dx)
                yd0, yd1 = max(0, dy), min(h, h + dy)
                out[yd0:yd1, xd0:xd1] |= occ[ys0:ys1, xs0:xs1]
        return out

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def pose_cb(self, msg):
        self.pose = msg.pose.pose
        if self._tree_start is not None:
            cur = self.w2m(self.pose.position.x, self.pose.position.y)
            d = math.hypot((cur[0]-self._tree_start[0])*self.map_resolution,
                           (cur[1]-self._tree_start[1])*self.map_resolution)
            if d > 0.5:
                self._needs_replan = True

    def goal_cb(self, msg):
        self.goal = msg.pose
        self._needs_replan = True

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def w2m(self, wx, wy):
        dx, dy = wx - self.map_origin_x, wy - self.map_origin_y
        lx = self.cos_yaw*dx + self.sin_yaw*dy
        ly = -self.sin_yaw*dx + self.cos_yaw*dy
        return int(round(lx/self.map_resolution)), int(round(ly/self.map_resolution))

    def m2w(self, mx, my):
        lx, ly = mx*self.map_resolution, my*self.map_resolution
        return (self.cos_yaw*lx - self.sin_yaw*ly + self.map_origin_x,
                self.sin_yaw*lx + self.cos_yaw*ly + self.map_origin_y)

    def in_bounds(self, mx, my):
        return 0 <= mx < self.map_width and 0 <= my < self.map_height

    def is_free(self, mx, my):
        return self.in_bounds(mx, my) and not self.map_grid[my, mx]

    def line_is_free(self, p0, p1):
        x0, y0, x1, y1 = p0[0], p0[1], p1[0], p1[1]
        steps = max(1, int(math.hypot(x1-x0, y1-y0)))
        for i in range(steps + 1):
            t = i / steps
            if not self.is_free(int(round(x0+t*(x1-x0))), int(round(y0+t*(y1-y0)))):
                return False
        return True

    def steer(self, frm, to, step):
        dx, dy = to[0]-frm[0], to[1]-frm[1]
        d = math.hypot(dx, dy)
        if d <= step:
            return to
        return int(round(frm[0] + step*dx/d)), int(round(frm[1] + step*dy/d))

    def nearest_index(self, nodes, target):
        tx, ty = target
        return min(range(len(nodes)), key=lambda i: (nodes[i][0]-tx)**2 + (nodes[i][1]-ty)**2)

    def near_indices(self, nodes, target, radius):
        tx, ty, r2 = target[0], target[1], radius*radius
        return [i for i, (nx, ny) in enumerate(nodes) if (nx-tx)**2+(ny-ty)**2 <= r2]

    # ── RRT* core ─────────────────────────────────────────────────────────────

    def _rrt_iter(self, nodes, parents, costs, goal, step, nr, gr, goal_idx,
                  optimize_goal, forced_sample=None):
        if forced_sample is not None:
            sample = forced_sample
        elif random.random() < self.goal_bias:
            r_px = int(self.goal_bias_radius_m / self.map_resolution)
            s = (goal[0]+random.randint(-r_px, r_px), goal[1]+random.randint(-r_px, r_px))
            sample = s if (self.in_bounds(*s) and self.is_free(*s)) else goal
        else:
            sample = (random.randint(0, self.map_width-1), random.randint(0, self.map_height-1))
            if not self.is_free(*sample):
                return goal_idx

        ni = self.nearest_index(nodes, sample)
        new = self.steer(nodes[ni], sample, step)
        if not self.is_free(*new) or not self.line_is_free(nodes[ni], new):
            return goal_idx

        near = self.near_indices(nodes, new, nr)
        best_p, best_c = ni, costs[ni] + math.hypot(new[0]-nodes[ni][0], new[1]-nodes[ni][1])
        for idx in near:
            c = costs[idx] + math.hypot(new[0]-nodes[idx][0], new[1]-nodes[idx][1])
            if c < best_c and self.line_is_free(nodes[idx], new):
                best_p, best_c = idx, c

        nodes.append(new); parents.append(best_p); costs.append(best_c)
        new_idx = len(nodes) - 1

        for idx in near:
            rc = best_c + math.hypot(nodes[idx][0]-new[0], nodes[idx][1]-new[1])
            if rc < costs[idx] and self.line_is_free(new, nodes[idx]):
                parents[idx], costs[idx] = new_idx, rc

        if math.hypot(new[0]-goal[0], new[1]-goal[1]) <= gr and self.line_is_free(new, goal):
            ec = costs[new_idx] + math.hypot(goal[0]-new[0], goal[1]-new[1])
            if goal_idx is None:
                nodes.append(goal); parents.append(new_idx); costs.append(ec)
                goal_idx = len(nodes) - 1
            elif optimize_goal and ec < costs[goal_idx]:
                parents[goal_idx], costs[goal_idx] = new_idx, ec

        return goal_idx

    def _recompute_costs(self, nodes, parents, costs):
        n = len(nodes)
        children = [[] for _ in range(n)]
        for i in range(1, n): children[parents[i]].append(i)
        costs[0] = 0.0
        q = [0]; head = 0
        while head < len(q):
            u = q[head]; head += 1
            for v in children[u]:
                costs[v] = costs[u] + math.hypot(nodes[v][0]-nodes[u][0], nodes[v][1]-nodes[u][1])
                q.append(v)

    def _extract_path(self, goal_idx):
        path, idx = [], goal_idx
        while True:
            path.append(self._tree_nodes[idx])
            if idx == 0: break
            idx = self._tree_parents[idx]
        path.reverse()
        return path

    def _obstacle_sample(self, step):
        ox, oy = random.choice(self.obstacle_cells)
        angle = random.uniform(0, 2*math.pi)
        sx = int(round(ox + step*math.cos(angle)))
        sy = int(round(oy + step*math.sin(angle)))
        return (sx, sy) if self.in_bounds(sx, sy) and self.is_free(sx, sy) else None

    # ── Planning tick ─────────────────────────────────────────────────────────

    def _reset_tree(self):
        if self.pose is None or self.goal is None or self.map_grid is None:
            return
        start = self.w2m(self.pose.position.x, self.pose.position.y)
        goal = self.w2m(self.goal.position.x, self.goal.position.y)
        if not self.is_free(*start):
            self.get_logger().warn("Start point is not free."); return
        if not self.is_free(*goal):
            self.get_logger().warn("Goal point is not free."); return
        self._tree_start, self._tree_goal = start, goal
        self._tree_nodes, self._tree_parents, self._tree_costs = [start], [0], [0.0]
        self._tree_goal_idx = None
        self._phase = 'A'
        self._phase_start_time = time.perf_counter()
        self._current_path = []
        self._t_last_viz = time.perf_counter()
        res = self.map_resolution
        self._step_size = max(2, int(0.6/res))
        self._neighbor_radius = max(self._step_size+1, int(1.2/res))
        self._goal_radius = max(2, int(0.7/res))
        self._corridor_radius_px = int(self.phase_b_corridor_radius_m/res)
        self.get_logger().info("Starting Phase A.")

    def _planning_tick(self):
        if self._needs_replan:
            self._needs_replan = False
            self._reset_tree()
            return
        if self._tree_nodes is None:
            return

        nodes, parents, costs = self._tree_nodes, self._tree_parents, self._tree_costs
        goal = self._tree_goal
        step, nr, gr = self._step_size, self._neighbor_radius, self._goal_radius
        goal_idx = self._tree_goal_idx

        for _ in range(50):
            if self._phase == 'A':
                fs = self._obstacle_sample(step) if self.obstacle_cells and random.random() < 0.5 else None
                goal_idx = self._rrt_iter(nodes, parents, costs, goal, step, nr, gr,
                                          goal_idx, optimize_goal=False, forced_sample=fs)
                if goal_idx is not None:
                    self._tree_goal_idx = goal_idx
                    self._current_path = self._extract_path(goal_idx)
                    self._publish_path()
                    elapsed = time.perf_counter() - self._phase_start_time
                    self.get_logger().info(
                        f"Phase A done in {elapsed:.1f}s: {len(self._current_path)} points. Starting Phase B.")
                    self._phase = 'B'
                    self._phase_start_time = time.perf_counter()
                    break
                elif time.perf_counter() - self._phase_start_time > 30.0:
                    self.get_logger().warn("Phase A timed out after 30s.")
                    self._tree_nodes = None
                    return

            elif self._phase == 'B':
                if time.perf_counter() - self._phase_start_time > self.phase_b_max_seconds:
                    self.get_logger().info("Phase B complete.")
                    self._tree_nodes = None
                    return
                if random.random() < 0.3 and len(self._current_path) > 1:
                    anchor = self._current_path[random.randint(0, len(self._current_path)-1)]
                    c = int(round(anchor[0]+random.randint(-self._corridor_radius_px, self._corridor_radius_px))), \
                        int(round(anchor[1]+random.randint(-self._corridor_radius_px, self._corridor_radius_px)))
                    fs = c if self.in_bounds(*c) and self.is_free(*c) else None
                else:
                    fs = self._obstacle_sample(step) if self.obstacle_cells else None

                prev = costs[goal_idx]
                goal_idx = self._rrt_iter(nodes, parents, costs, goal, step, nr, gr,
                                          goal_idx, optimize_goal=True, forced_sample=fs)
                self._tree_goal_idx = goal_idx
                if costs[goal_idx] < prev:
                    self._recompute_costs(nodes, parents, costs)
                    self._current_path = self._extract_path(goal_idx)
                    self._publish_path()

        now = time.perf_counter()
        if now - self._t_last_viz >= 0.1:
            self._t_last_viz = now
            self._publish_tree(nodes, parents)

    def _publish_path(self):
        self.trajectory.points = [self.m2w(px, py) for px, py in self._current_path]
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

    def _publish_tree(self, nodes, parents):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "planned_trajectory/tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.2; marker.color.g = 0.7; marker.color.b = 1.0; marker.color.a = 0.45
        for i in range(max(1, len(nodes)-5000), len(nodes)):
            x0, y0 = self.m2w(nodes[i][0], nodes[i][1])
            x1, y1 = self.m2w(nodes[parents[i]][0], nodes[parents[i]][1])
            p0 = Point(); p0.x = x0; p0.y = y0; p0.z = 0.02
            p1 = Point(); p1.x = x1; p1.y = y1; p1.z = 0.02
            marker.points += [p0, p1]
        self.tree_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPlan())
    rclpy.shutdown()
