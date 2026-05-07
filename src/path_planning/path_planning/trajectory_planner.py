import heapq
import time
from queue import PriorityQueue

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt
from scipy.signal import convolve2d
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String

from path_planning.utils import *


class PathPlan(Node):
    """Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter("odom_topic", "default")
        self.declare_parameter("map_topic", "default")

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.map_topic = (
            self.get_parameter("map_topic").get_parameter_value().string_value
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, 1
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_cb, 10
        )

        self.traj_pub = self.create_publisher(PoseArray, "/trajectory/current", 10)

        self.pose_sub = self.create_subscription(
            Odometry, self.odom_topic, self.pose_cb, 10
        )

        self.robot_state_sub = self.create_subscription(
            String, "/robot_state", self.robot_state_cb, 10
        )

        self.robot_state = "MCL_INITIALIZATION"
        self.pose = None
        self.goal = None
        self.map = None

        self.height = None
        self.width = None

        # 8-connected neighbors
        self.NEIGHBORS = [
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ]

        self.blur_radius = 6
        self.downsample_factor = 3
        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def map_cb(self, msg):
        map_data = np.array(msg.data, np.double)
        width, height = msg.info.width, msg.info.height
        map_data = map_data.reshape((height, width))

        # # self.get_logger().info(f"Map reshaped to: {map_data.shape}")
        # map_data[map_data != 0] = 1.0
        # radius = self.blur_radius
        # y, x = np.ogrid[-radius : radius + 1, -radius : radius + 1]
        # mask = x**2 + y**2 <= radius**2
        # map_data = convolve2d(
        #     map_data, mask, mode="same", boundary="fill", fillvalue=1.0
        # )
        # # self.get_logger().info(f"Map convolved to: {map_data.shape}")
        # self.map = map_data[:: self.downsample_factor, :: self.downsample_factor]
        # self.map = self.map != 0
        map_data = np.where(map_data != 0, 1, 0)
        self.dist = distance_transform_edt(map_data == 0)
        self.map = map_data[:: self.downsample_factor, :: self.downsample_factor]
        self.dist = self.dist[:: self.downsample_factor, :: self.downsample_factor]

        self.map = np.where(
            self.dist <= self.blur_radius,
            np.inf,
            5.0 / (self.dist - self.blur_radius + 1) ** 1.5,
        )
        self.height, self.width = self.map.shape
        self.map_info = msg.info
        origin_p = msg.info.origin.position
        origin_o = msg.info.origin.orientation
        quat = [origin_o.x, origin_o.y, origin_o.z, origin_o.w]
        yaw = R.from_quat(quat).as_euler("xyz")[2]
        self.origin = (origin_p.x, origin_p.y, yaw)
        self.rot_matrix = np.array(
            [
                [
                    np.cos(self.origin[2]),
                    -np.sin(self.origin[2]),
                ],
                [
                    np.sin(self.origin[2]),
                    np.cos(self.origin[2]),
                ],
            ]
        )
        self.get_logger().info(
            f"Path planner map received and processed, shape: {self.map.shape}"
        )

    def pixel_to_world(self, x_pixel, y_pixel):
        if self.map is None:
            raise ValueError(
                "Map info is not set. Cannot convert pixel to world coordinates."
            )
        if isinstance(x_pixel, int):
            x_pixel = np.array([x_pixel])
        if isinstance(y_pixel, int):
            y_pixel = np.array([y_pixel])
        xy = (
            np.stack((x_pixel, y_pixel), axis=1)
            * self.map_info.resolution
            * self.downsample_factor
            @ self.rot_matrix.T
        )
        return (
            xy[:, 0] + self.origin[0],
            xy[:, 1] + self.origin[1],
        )

    def world_to_pixel(self, x_world, y_world):
        if self.map is None:
            raise ValueError(
                "Map info is not set. Cannot convert world to pixel coordinates."
            )
        if isinstance(x_world, (int, float)):
            x_world = np.array([x_world])
        if isinstance(y_world, (int, float)):
            y_world = np.array([y_world])
        xy = np.stack((x_world, y_world), axis=1) - np.array(self.origin[:2])
        xy = xy @ self.rot_matrix
        return (
            np.round(
                xy[:, 0] / self.map_info.resolution / self.downsample_factor
            ).astype(int),
            np.round(
                xy[:, 1] / self.map_info.resolution / self.downsample_factor
            ).astype(int),
        )

    def pose_cb(self, pose):
        self.pose = pose.pose.pose
        self.plan_path()

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.plan_path()

    def robot_state_cb(self, msg: String):
        old_state = self.robot_state
        self.robot_state = msg.data

        # Check if we just transitioned INTO a navigating state
        if "NAVIGATING" in self.robot_state and "NAVIGATING" not in old_state:
            if self.goal is not None:
                self.get_logger().info(
                    "State changed to NAVIGATING with goal present. Planning path..."
                )
                self.plan_path()

    def heuristic(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return (dx * dx + dy * dy) ** 0.5

    def line_of_sight(self, a, b):
        # bresenham's line algorithm
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if self.map[y0, x0]:
                return False
            if x0 == x1 and y0 == y1:
                break
            err2 = err * 2
            if err2 > -dy:
                err -= dy
                x0 += sx
            if err2 < dx:
                err += dx
                y0 += sy
        return True

    def iter_neighbors(self, x, y):
        for dx, dy in self.NEIGHBORS:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # if not self.map[ny, nx]:
                yield nx, ny

    def reconstruct_path(self, parent, current_idx):
        path = []
        while True:
            x = current_idx % self.width
            y = current_idx // self.width
            path.append((x, y))
            if parent[current_idx] == current_idx:
                break
            current_idx = parent[current_idx]
        path.reverse()
        return path

    def to_idx(self, x, y):
        return y * self.width + x

    def from_idx(self, idx):
        return (idx % self.width, idx // self.width)

    def smooth_path(self, path):
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev = smoothed[-1]
            next = path[i + 1]
            if self.line_of_sight(prev, next):
                continue
            smoothed.append(path[i])
        smoothed.append(path[-1])
        return smoothed

    def A_star(self, start, goal):
        sx, sy = int(start[0]), int(start[1])
        gx, gy = int(goal[0]), int(goal[1])

        start_idx = self.to_idx(sx, sy)
        goal_idx = self.to_idx(gx, gy)

        size = self.width * self.height

        gscore = np.full(size, np.inf, dtype=np.float32)
        parent = np.full(size, -1, dtype=np.int32)
        closed = np.zeros(size, dtype=bool)

        gscore[start_idx] = 0.0
        parent[start_idx] = start_idx

        open_set = []
        heapq.heappush(open_set, (self.heuristic((sx, sy), (gx, gy)), start_idx))

        while open_set:
            _, current_idx = heapq.heappop(open_set)

            if closed[current_idx]:
                continue

            if current_idx == goal_idx:
                return self.reconstruct_path(parent, current_idx)

            closed[current_idx] = True

            cx, cy = self.from_idx(current_idx)

            for nx, ny in self.iter_neighbors(cx, cy):
                neighbor_idx = self.to_idx(nx, ny)

                if closed[neighbor_idx]:
                    continue

                dx = cx - nx
                dy = cy - ny
                tentative = gscore[current_idx] + (dx * dx + dy * dy) ** 0.5 * (
                    1.0 + self.map[ny, nx]
                )

                if tentative < gscore[neighbor_idx]:
                    gscore[neighbor_idx] = tentative
                    parent[neighbor_idx] = current_idx
                    f = tentative + self.heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_set, (f, neighbor_idx))

        return []

    def IEPF(self, points, D_t):
        start = 0
        end = len(points) - 1
        line_vec = points[end] - points[start]
        line_vec /= np.linalg.norm(line_vec)
        point_vecs = points - points[start]
        projections = point_vecs @ line_vec
        closest_points = np.outer(projections, line_vec) + points[start]
        distances = np.linalg.norm(points - closest_points, axis=1)
        max_index = np.argmax(distances)
        if distances[max_index] > D_t:
            left_segments = self.IEPF(points[: max_index + 1], D_t)
            right_segments = self.IEPF(points[max_index:], D_t)
            return left_segments + right_segments[1:]
        else:
            return [points[0], points[-1]]

    def plan_path(self):
        if (
            self.pose is None
            or self.goal is None
            or self.map is None
            or ("NAVIGATING" not in self.robot_state)
        ):
            return
        # self.get_logger().info("Planning path...")
        # time_start = time.time()
        start_pixel = self.world_to_pixel(self.pose.position.x, self.pose.position.y)
        goal_pixel = self.world_to_pixel(self.goal.position.x, self.goal.position.y)
        path_pixels = self.A_star(start_pixel, goal_pixel)
        # self.get_logger().info(f"Path found with {len(path_pixels)} points")
        path_pixels = self.IEPF(np.array(path_pixels, dtype=np.float32), D_t=1.0)
        path_world = self.pixel_to_world(
            [p[0] for p in path_pixels], [p[1] for p in path_pixels]
        )
        # time_end = time.time()
        # self.get_logger().info(
        #     f"Path planning took {time_end - time_start:.2f} seconds"
        # )
        # self.get_logger().info(f"Path planned with {len(path_world[0])} points")

        self.trajectory.points = list(zip(path_world[0], path_world[1]))
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
