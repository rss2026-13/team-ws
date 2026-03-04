#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped


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


def point_dir(wall, dir, margin=0.3):
    p1, p2 = wall
    line_vec = p2 - p1
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return None
    line_vec /= line_len
    dir_vec = np.array(dir)
    dir_vec /= np.linalg.norm(dir_vec)
    det = line_vec[0] * dir_vec[1] - line_vec[1] * dir_vec[0]
    if det == 0:
        return None
    t = (dir_vec[0] * (p1[1] - 0) - dir_vec[1] * (p1[0] - 0)) / det
    u = (line_vec[0] * (p1[1] - 0) - line_vec[1] * (p1[0] - 0)) / det
    if t < -margin or t > line_len + margin or u < 0:
        return None
    return p1 + t * line_vec


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

    def update(self, walls):
        if len(walls) == 0:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.velocity
            self.drive_publisher.publish(drive_msg)
            return

        closest_dist = np.inf
        if self.side == 1:
            angles = np.linspace(
                -self.side_spread + np.pi / 2,
                0 + np.pi / 2,
                self.side_samples,
            )
        else:
            angles = np.linspace(
                0 - np.pi / 2,
                self.side_spread - np.pi / 2,
                self.side_samples,
            )
        for angle in angles:
            for wall in walls:
                dist = point_dir(wall, (np.cos(angle), np.sin(angle)), 0.1)
                if dist is not None:
                    closest_dist = min(closest_dist, np.linalg.norm(dist))
        if closest_dist == np.inf:
            closest_dist = self.prev_closest_dist
        self.prev_closest_dist = closest_dist
        side_error = closest_dist - self.desired_distance
        forward_dist = np.inf
        angles = np.linspace(
            -self.front_spread,
            self.front_spread,
            self.front_samples,
        )
        for angle in angles:
            for wall in walls:
                dist = point_dir(wall, (np.cos(angle), np.sin(angle)), 0.3)
                if dist is not None:
                    forward_dist = min(forward_dist, np.linalg.norm(dist))
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
