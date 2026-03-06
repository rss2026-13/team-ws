#!/usr/bin/env python3
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped

from wall_follower.geometry_utilities import point_dir
from wall_follower.pid_controller import PIDController


class DriveController:
    def __init__(
        self,
        pid_params,
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
        clock,
    ):
        self.pid_controller = PIDController(
            **pid_params,
        )
        self.side = side
        self.side_spread = side_spread / 180.0 * np.pi
        self.side_samples = side_samples
        self.front_spread = front_spread / 180.0 * np.pi
        self.front_samples = front_samples
        self.velocity = velocity
        self.desired_distance = desired_distance
        self.front_treshold = front_treshold
        self.front_error_ratio = front_error_ratio
        self.drive_publisher = drive_publisher
        self.clock = clock
        self.previous_error = 0
        self.last_time = None
        self.prev_front_error = 0
        self.prev_closest_dist = 0

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
        current_time = self.clock.now()
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
