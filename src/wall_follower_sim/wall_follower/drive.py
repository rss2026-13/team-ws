#!/usr/bin/env python3
import multiprocessing
from collections import deque

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32


def closest_point(wall):
    p1, p2 = wall
    line_vec = p2 - p1
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return np.linalg.norm(p1)
    line_vec /= line_len
    point_vec = -p1
    proj = point_vec @ line_vec
    if proj < 0:
        closest_point = p1
    elif proj > line_len:
        closest_point = p2
    else:
        closest_point = proj * line_vec + p1
    return closest_point


def _min_ray_dist(walls, angles):
    """Vectorized minimum ray-cast distance across all angles and walls.

    For each ray direction (given by an angle) and each wall segment, compute
    the intersection point (if any) and return the minimum distance from the
    origin across all valid intersections.  Returns np.inf when no ray hits
    any wall.

    The ray–segment test solves:
        p1 + t * line_unit = u * dir
    for scalars t (position along segment) and u (distance along ray).
    A hit is valid when 0 <= t <= line_len and u >= 0.
    """
    if len(walls) == 0 or len(angles) == 0:
        return np.inf

    # --- precompute wall geometry (W walls) ---
    # p1: (W, 2), p2: (W, 2)
    p1 = np.array([w[0] for w in walls], dtype=np.float64)  # (W, 2)
    p2 = np.array([w[1] for w in walls], dtype=np.float64)  # (W, 2)
    line_vec = p2 - p1  # (W, 2)
    line_len = np.linalg.norm(line_vec, axis=1)  # (W,)

    # Mask out degenerate (zero-length) walls
    valid_wall = line_len > 0
    if not np.any(valid_wall):
        return np.inf

    p1 = p1[valid_wall]  # (W', 2)
    line_vec = line_vec[valid_wall]  # (W', 2)
    line_len = line_len[valid_wall]  # (W',)
    line_unit = line_vec / line_len[:, np.newaxis]  # (W', 2)

    # --- precompute ray directions (A angles) ---
    angles = np.asarray(angles, dtype=np.float64)
    dir_x = np.cos(angles)  # (A,)
    dir_y = np.sin(angles)  # (A,)

    # --- broadcast intersection math over (A, W') ---
    # line_unit components: lx, ly  shape (W',)
    lx = line_unit[:, 0]  # (W',)
    ly = line_unit[:, 1]  # (W',)

    # det = lx * dy - ly * dx   shape (A, W')
    det = (
        lx[np.newaxis, :] * dir_y[:, np.newaxis]
        - ly[np.newaxis, :] * dir_x[:, np.newaxis]
    )

    # Avoid division by zero; we'll mask these out later
    safe_det = np.where(det != 0, det, 1.0)

    # p1 components
    p1x = p1[:, 0]  # (W',)
    p1y = p1[:, 1]  # (W',)

    # t = (dx * p1y - dy * p1x) / det   -- position along segment
    t = (
        dir_x[:, np.newaxis] * p1y[np.newaxis, :]
        - dir_y[:, np.newaxis] * p1x[np.newaxis, :]
    ) / safe_det

    # u = (lx * p1y - ly * p1x) / det   -- distance along ray
    u = (
        lx[np.newaxis, :] * p1y[np.newaxis, :] - ly[np.newaxis, :] * p1x[np.newaxis, :]
    ) / safe_det

    # Valid hits: det != 0, 0 <= t <= line_len, u >= 0
    valid = (det != 0) & (t >= 0) & (t <= line_len[np.newaxis, :]) & (u >= 0)

    # Compute hit-point distances (u is the distance along a unit direction)
    # Replace invalid entries with inf so they don't affect the min
    dists = np.where(valid, u, np.inf)

    return float(np.min(dists))


class PIDController:
    def __init__(self, kp, ki, kd, max_i, max_d):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_i = max_i
        self.max_d = max_d

        self.previous_error = 0
        self.integral = 0

        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0

    def update(self, setpoint, pv, dt):
        error = setpoint - pv
        self.integral *= 0.9
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_i), -self.max_i)
        derivative = (error - self.previous_error) / dt
        derivative = max(min(derivative, self.max_d), -self.max_d)

        self.last_p_term = self.kp * error
        self.last_i_term = self.ki * self.integral
        self.last_d_term = self.kd * derivative

        control = self.last_p_term + self.last_i_term + self.last_d_term
        self.previous_error = error
        return control


class DriveController:
    def __init__(
        self,
        kp,
        ki,
        kd,
        max_i,
        max_d,
        side,
        side_spread,
        side_samples,
        front_spread,
        front_samples,
        velocity,
        desired_distance,
        drive_publisher,
        front_treshold,
        front_error_ratio,
        distance_publisher=None,
        angle_publisher=None,
    ):
        self.pid_controller = PIDController(kp, ki, kd, max_i, max_d)
        self.side = side
        self.side_spread = side_spread
        self.side_samples = side_samples
        self.front_spread = front_spread
        self.front_samples = front_samples
        self.velocity = velocity
        self.desired_distance = desired_distance
        self.drive_publisher = drive_publisher
        self.previous_error = 0
        self.integral = 0
        self.last_time = None
        self.front_treshold = front_treshold
        self.front_error_ratio = front_error_ratio

        self.prev_front_error = 0
        self.prev_closest_dist = 0
        self.distance_publisher = distance_publisher
        self.angle_publisher = angle_publisher

        # Precompute angle arrays (they never change between calls)
        self._side_angles = np.linspace(
            -self.side_spread + self.side * np.pi / 2,
            self.side_spread + self.side * np.pi / 2,
            self.side_samples,
        )
        self._front_angles = np.linspace(
            -self.front_spread,
            self.front_spread,
            self.front_samples,
        )

    def update(self, walls):
        if len(walls) == 0:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.velocity
            self.drive_publisher.publish(drive_msg)
            return

        # --- side distance (closest point in the side angle range) ---
        closest_dist = _min_ray_dist(walls, self._side_angles)
        if closest_dist == np.inf:
            closest_dist = self.prev_closest_dist
        self.prev_closest_dist = closest_dist
        side_error = closest_dist - self.desired_distance

        # --- front distance (closest point in the front angle range) ---
        forward_dist = _min_ray_dist(walls, self._front_angles)
        if forward_dist == np.inf:
            front_error = self.prev_front_error
        else:
            front_error = (
                max(0, self.front_treshold - forward_dist) / self.front_treshold
            ) ** 2
        self.prev_front_error = front_error

        error = side_error - self.front_error_ratio * front_error

        current_time = rclpy.clock.Clock().now()
        if self.last_time is None:
            dt = 0.05
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        control = self.pid_controller.update(0, error, dt)
        steering_angle = control * (-self.side)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.velocity
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)
