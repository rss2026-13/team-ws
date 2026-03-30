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

    def update(self, walls, laser_scan):
        if len(walls) == 0:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.velocity
            self.drive_publisher.publish(drive_msg)
            return

        ranges = np.array(laser_scan.ranges, dtype="float32")

        angles = np.linspace(
            laser_scan.angle_min, laser_scan.angle_max, num=ranges.shape[0]
        )

        # Convert the ranges to Cartesian coordinates.
        # Consider the robot to be facing in the positive x direction.
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Filter out values that are out of range
        # and values on the wrong side
        valid_points = self.side * y > 0
        valid_points = np.logical_and(valid_points, x < 1.5)
        valid_points = np.logical_and(valid_points, x > 0.0)

        # Compute the average distance
        dists = np.abs(y[valid_points])
        dist = np.sum(dists) / dists.shape[0]
        side_error = dist - self.desired_distance
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
