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
        self.declare_parameter("odom_topic", "default")
        self.declare_parameter("map_topic", "default")
        self.declare_parameter("wall_buffer_m", 0.12)
        self.declare_parameter("rrt_phase_a_iterations", 10000)
        self.declare_parameter("rrt_phase_b_iterations", 4000)
        # Wall-clock cap for phase B (optimization). <= 0 means no time limit (only iteration cap).
        self.declare_parameter("rrt_phase_b_max_seconds", 7.0)
        self.declare_parameter("rrt_phase_b_corridor_radius_m", 0.75)

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.map_topic = (
            self.get_parameter("map_topic").get_parameter_value().string_value
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            map_qos,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10,
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10,
        )
        self.tree_pub = self.create_publisher(
            Marker,
            "/planned_trajectory/tree",
            1,
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.pose_cb,
            10,
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

        self.pose = None
        self.goal = None
        self.map_grid = None
        self.map_info = None
        self.map_height = 0
        self.map_width = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.cos_yaw = 1.0
        self.sin_yaw = 0.0
        self.ready_logged = False

        self.phase_a_iterations = (
            self.get_parameter("rrt_phase_a_iterations")
            .get_parameter_value()
            .integer_value
        )
        self.phase_b_iterations = (
            self.get_parameter("rrt_phase_b_iterations")
            .get_parameter_value()
            .integer_value
        )
        self.phase_b_max_seconds = (
            self.get_parameter("rrt_phase_b_max_seconds")
            .get_parameter_value()
            .double_value
        )
        self.phase_b_corridor_radius_m = (
            self.get_parameter("rrt_phase_b_corridor_radius_m")
            .get_parameter_value()
            .double_value
        )
        self.goal_bias = 0.05
        self.goal_bias_radius_m = 3.0
        self.wall_buffer_m = (
            self.get_parameter("wall_buffer_m").get_parameter_value().double_value
        )
        self.last_start_point = None
        self.last_end_point = None
        self.obstacle_cells = []

    def map_cb(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        # Unknown is treated as occupied for safer paths.
        occupancy = np.logical_or(grid == -1, grid > 50)
        buffer_pixels = max(0, int(math.ceil(self.wall_buffer_m / msg.info.resolution)))
        self.map_grid = self.inflate_obstacles(occupancy, buffer_pixels)
        self.map_info = msg.info
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        q = msg.info.origin.orientation
        self.map_origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.cos_yaw = math.cos(self.map_origin_yaw)
        self.sin_yaw = math.sin(self.map_origin_yaw)

        ys, xs = np.where(self.map_grid)
        self.obstacle_cells = list(zip(xs.tolist(), ys.tolist()))

        if not self.ready_logged:
            self.get_logger().info(
                f"Received map ({self.map_width}x{self.map_height}) for RRT*."
            )
            self.ready_logged = True
        self.plan_path()

    def inflate_obstacles(self, occupancy_grid, radius_pixels):
        """Inflate occupied cells by a circular radius in pixels."""
        if radius_pixels <= 0:
            return occupancy_grid

        h, w = occupancy_grid.shape
        inflated = occupancy_grid.copy()

        for dy in range(-radius_pixels, radius_pixels + 1):
            for dx in range(-radius_pixels, radius_pixels + 1):
                if dx * dx + dy * dy > radius_pixels * radius_pixels:
                    continue

                x_src_start = max(0, -dx)
                x_src_end = min(w, w - dx)
                y_src_start = max(0, -dy)
                y_src_end = min(h, h - dy)

                x_dst_start = max(0, dx)
                x_dst_end = min(w, w + dx)
                y_dst_start = max(0, dy)
                y_dst_end = min(h, h + dy)

                inflated[y_dst_start:y_dst_end, x_dst_start:x_dst_end] |= occupancy_grid[
                    y_src_start:y_src_end, x_src_start:x_src_end
                ]

        return inflated

    def pose_cb(self, pose):
        self.pose = pose.pose.pose

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.last_start_point = None
        self.last_end_point = None
        self.plan_path()

    def world_to_map(self, x_world, y_world):
        dx = x_world - self.map_origin_x
        dy = y_world - self.map_origin_y
        local_x = self.cos_yaw * dx + self.sin_yaw * dy
        local_y = -self.sin_yaw * dx + self.cos_yaw * dy
        mx = int(round(local_x / self.map_resolution))
        my = int(round(local_y / self.map_resolution))
        return mx, my

    def map_to_world(self, mx, my):
        local_x = mx * self.map_resolution
        local_y = my * self.map_resolution
        wx = self.cos_yaw * local_x - self.sin_yaw * local_y + self.map_origin_x
        wy = self.sin_yaw * local_x + self.cos_yaw * local_y + self.map_origin_y
        return wx, wy

    def in_bounds(self, mx, my):
        return 0 <= mx < self.map_width and 0 <= my < self.map_height

    def is_free(self, mx, my):
        return self.in_bounds(mx, my) and not self.map_grid[my, mx]

    def line_is_free(self, p0, p1):
        x0, y0 = p0
        x1, y1 = p1
        distance = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(distance))
        for i in range(steps + 1):
            t = i / steps
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))
            if not self.is_free(x, y):
                return False
        return True

    def steer(self, from_node, to_node, step_size):
        fx, fy = from_node
        tx, ty = to_node
        dx = tx - fx
        dy = ty - fy
        distance = math.hypot(dx, dy)
        if distance <= step_size:
            return tx, ty
        ratio = step_size / distance
        return int(round(fx + ratio * dx)), int(round(fy + ratio * dy))

    def nearest_index(self, nodes, target):
        tx, ty = target
        best_idx = 0
        best_dist = float("inf")
        for i, (nx, ny) in enumerate(nodes):
            dist_sq = (nx - tx) ** 2 + (ny - ty) ** 2
            if dist_sq < best_dist:
                best_dist = dist_sq
                best_idx = i
        return best_idx

    def near_indices(self, nodes, target, radius):
        tx, ty = target
        radius_sq = radius * radius
        near = []
        for i, (nx, ny) in enumerate(nodes):
            dist_sq = (nx - tx) ** 2 + (ny - ty) ** 2
            if dist_sq <= radius_sq:
                near.append(i)
        return near

    def _rrt_star_one_iteration(
        self,
        nodes,
        parents,
        costs,
        goal,
        step_size,
        neighbor_radius,
        goal_radius,
        goal_idx,
        optimize_goal,
        forced_sample=None,
    ):
        """One RRT* grow step. If optimize_goal and goal_idx set, improve goal parent when cheaper."""
        if forced_sample is not None:
            sample = forced_sample
        elif random.random() < self.goal_bias:
            r_px = int(self.goal_bias_radius_m / self.map_resolution)
            sample = (
                goal[0] + random.randint(-r_px, r_px),
                goal[1] + random.randint(-r_px, r_px),
            )
            if not (self.in_bounds(sample[0], sample[1]) and self.is_free(sample[0], sample[1])):
                sample = goal
        else:
            sample = (
                random.randint(0, self.map_width - 1),
                random.randint(0, self.map_height - 1),
            )
            if not self.is_free(sample[0], sample[1]):
                return goal_idx

        nearest_idx = self.nearest_index(nodes, sample)
        nearest_node = nodes[nearest_idx]
        new_node = self.steer(nearest_node, sample, step_size)

        if not self.is_free(new_node[0], new_node[1]):
            return goal_idx
        if not self.line_is_free(nearest_node, new_node):
            return goal_idx

        near = self.near_indices(nodes, new_node, neighbor_radius)
        best_parent = nearest_idx
        best_cost = costs[nearest_idx] + math.hypot(
            new_node[0] - nearest_node[0],
            new_node[1] - nearest_node[1],
        )

        for idx in near:
            candidate = nodes[idx]
            if not self.line_is_free(candidate, new_node):
                continue
            candidate_cost = costs[idx] + math.hypot(
                new_node[0] - candidate[0],
                new_node[1] - candidate[1],
            )
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_parent = idx

        nodes.append(new_node)
        parents.append(best_parent)
        costs.append(best_cost)
        new_idx = len(nodes) - 1

        for idx in near:
            candidate = nodes[idx]
            if not self.line_is_free(new_node, candidate):
                continue
            rewired_cost = best_cost + math.hypot(
                candidate[0] - new_node[0],
                candidate[1] - new_node[1],
            )
            if rewired_cost < costs[idx]:
                parents[idx] = new_idx
                costs[idx] = rewired_cost

        if (
            math.hypot(new_node[0] - goal[0], new_node[1] - goal[1]) <= goal_radius
            and self.line_is_free(new_node, goal)
        ):
            edge_to_goal = math.hypot(goal[0] - new_node[0], goal[1] - new_node[1])
            conn_cost = costs[new_idx] + edge_to_goal
            if goal_idx is None:
                nodes.append(goal)
                parents.append(new_idx)
                costs.append(conn_cost)
                goal_idx = len(nodes) - 1
            elif optimize_goal and conn_cost < costs[goal_idx]:
                parents[goal_idx] = new_idx
                costs[goal_idx] = conn_cost

        return goal_idx

    def _rrt_recompute_costs_from_root(self, nodes, parents, costs):
        """Propagate costs along the tree after rewiring (root = index 0)."""
        n = len(nodes)
        if n == 0:
            return
        children = [[] for _ in range(n)]
        for i in range(1, n):
            children[parents[i]].append(i)
        costs[0] = 0.0
        queue = [0]
        head = 0
        while head < len(queue):
            u = queue[head]
            head += 1
            for v in children[u]:
                costs[v] = costs[u] + math.hypot(
                    nodes[v][0] - nodes[u][0],
                    nodes[v][1] - nodes[u][1],
                )
                queue.append(v)

    def rrt_star(self, start, goal):
        step_size = max(2, int(0.6 / self.map_resolution))
        neighbor_radius = max(step_size + 1, int(1.2 / self.map_resolution))
        goal_radius = max(2, int(0.7 / self.map_resolution))

        nodes = [start]
        parents = [0]
        costs = [0.0]
        goal_idx = None

        # Phase A: find first feasible path, up to 30 seconds.
        t_phase_a = time.perf_counter()
        t_last_viz_a = t_phase_a
        viz_interval_a = 0.1
        while time.perf_counter() - t_phase_a < 30.0:
            # 50% obstacle-based sampling: pick a random obstacle cell and offset outward.
            forced_sample = None
            if self.obstacle_cells and random.random() < 0.5:
                ox, oy = random.choice(self.obstacle_cells)
                angle = random.uniform(0, 2 * math.pi)
                sx = int(round(ox + step_size * math.cos(angle)))
                sy = int(round(oy + step_size * math.sin(angle)))
                if self.in_bounds(sx, sy) and self.is_free(sx, sy):
                    forced_sample = (sx, sy)

            goal_idx = self._rrt_star_one_iteration(
                nodes,
                parents,
                costs,
                goal,
                step_size,
                neighbor_radius,
                goal_radius,
                goal_idx,
                optimize_goal=False,
                forced_sample=forced_sample,
            )
            if goal_idx is not None:
                break
            now = time.perf_counter()
            if now - t_last_viz_a >= viz_interval_a:
                t_last_viz_a = now
                yield [], nodes, parents

        if goal_idx is None:
            near_goal_idx = self.nearest_index(nodes, goal)
            if self.line_is_free(nodes[near_goal_idx], goal):
                nodes.append(goal)
                parents.append(near_goal_idx)
                goal_idx = len(nodes) - 1
            else:
                yield [], nodes, parents
                return

        def extract_path(goal_idx):
            path = []
            idx = goal_idx
            while True:
                path.append(nodes[idx])
                if idx == 0:
                    break
                idx = parents[idx]
            path.reverse()
            return path

        # Publish after Phase A.
        yield extract_path(goal_idx), nodes, parents

        # Phase B: optimize until wall-clock max, publish tree continuously and path on improvement.
        corridor_radius_px = int(self.phase_b_corridor_radius_m / self.map_resolution)
        t_phase_b = time.perf_counter()
        t_last_viz = t_phase_b
        viz_interval = 0.1  # publish tree every 100ms
        current_path = extract_path(goal_idx)
        while True:
            now = time.perf_counter()
            if self.phase_b_max_seconds > 0.0 and (
                now - t_phase_b >= self.phase_b_max_seconds
            ):
                break

            forced_sample = None
            r = random.random()
            if r < 0.5 and len(current_path) > 1:
                # 50% sample near current best path.
                anchor = current_path[random.randint(0, len(current_path) - 1)]
                candidate = (
                    anchor[0] + random.randint(-corridor_radius_px, corridor_radius_px),
                    anchor[1] + random.randint(-corridor_radius_px, corridor_radius_px),
                )
                if self.in_bounds(candidate[0], candidate[1]) and self.is_free(candidate[0], candidate[1]):
                    forced_sample = candidate
            elif self.obstacle_cells:
                # 50% obstacle-based sampling.
                ox, oy = random.choice(self.obstacle_cells)
                angle = random.uniform(0, 2 * math.pi)
                sx = int(round(ox + step_size * math.cos(angle)))
                sy = int(round(oy + step_size * math.sin(angle)))
                if self.in_bounds(sx, sy) and self.is_free(sx, sy):
                    forced_sample = (sx, sy)

            prev_cost = costs[goal_idx]
            goal_idx = self._rrt_star_one_iteration(
                nodes,
                parents,
                costs,
                goal,
                step_size,
                neighbor_radius,
                goal_radius,
                goal_idx,
                optimize_goal=True,
                forced_sample=forced_sample,
            )

            improved = costs[goal_idx] < prev_cost
            if improved:
                self._rrt_recompute_costs_from_root(nodes, parents, costs)
                current_path = extract_path(goal_idx)
            now = time.perf_counter()
            if improved or (now - t_last_viz >= viz_interval):
                t_last_viz = now
                yield extract_path(goal_idx), nodes, parents

    def publish_tree_marker(self, nodes, parents):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "planned_trajectory/tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.2
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.45

        for i in range(1, len(nodes)):
            parent_i = parents[i]
            x0, y0 = self.map_to_world(nodes[i][0], nodes[i][1])
            x1, y1 = self.map_to_world(nodes[parent_i][0], nodes[parent_i][1])

            p0 = Point()
            p0.x = x0
            p0.y = y0
            p0.z = 0.02
            p1 = Point()
            p1.x = x1
            p1.y = y1
            p1.z = 0.02
            marker.points.append(p0)
            marker.points.append(p1)

        self.tree_pub.publish(marker)

    def plan_path(self):
        if self.pose is None or self.goal is None or self.map_grid is None:
            return

        start_point = self.world_to_map(self.pose.position.x, self.pose.position.y)
        end_point = self.world_to_map(self.goal.position.x, self.goal.position.y)

        endpoints_changed = (
            self.last_start_point != start_point or self.last_end_point != end_point
        )
        if not endpoints_changed:
            return

        if not self.is_free(start_point[0], start_point[1]):
            self.get_logger().warn("Start point is not free.")
            return
        if not self.is_free(end_point[0], end_point[1]):
            self.get_logger().warn("Goal point is not free.")
            return

        first = True
        last_path = None
        found_path = False
        for path_pixels, tree_nodes, tree_parents in self.rrt_star(start_point, end_point):
            self.publish_tree_marker(tree_nodes, tree_parents)
            if not path_pixels:
                # Phase A still searching — just visualizing the growing tree.
                continue
            found_path = True
            if path_pixels != last_path:
                self.trajectory.points = [
                    self.map_to_world(px, py) for (px, py) in path_pixels
                ]
                self.traj_pub.publish(self.trajectory.toPoseArray())
                self.trajectory.publish_viz()
                if first:
                    self.get_logger().info(
                        f"Phase A: published initial path with {len(self.trajectory.points)} points."
                    )
                    first = False
                else:
                    self.get_logger().info(
                        f"Phase B: improved path to {len(self.trajectory.points)} points."
                    )
                last_path = path_pixels
        if not found_path:
            self.get_logger().warn("RRT* failed to find a path in 10000 iterations.")
            return
        self.last_start_point = start_point
        self.last_end_point = end_point


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()