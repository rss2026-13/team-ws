#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")

        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("output_topic", "/drive")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("margin", 0.5)
        self.declare_parameter("max_deceleration", 10.0)
        self.declare_parameter("cone_angle", 10.0)  # In degrees
        self.declare_parameter("car_width", 0.25)
        self.declare_parameter(
            "lidar_offset", 0.05
        )  # Distance from lidar to front bumper
        self.DRIVE_TOPIC = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )
        self.OUTPUT_TOPIC = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.SCAN_TOPIC = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.MARGIN = self.get_parameter("margin").get_parameter_value().double_value
        self.MAX_DECELERATION = (
            self.get_parameter("max_deceleration").get_parameter_value().double_value
        )
        self.CONE_ANGLE = (
            self.get_parameter("cone_angle").get_parameter_value().double_value
        )
        self.CAR_WIDTH = (
            self.get_parameter("car_width").get_parameter_value().double_value
        )
        self.LIDAR_OFFSET = (
            self.get_parameter("lidar_offset").get_parameter_value().double_value
        )
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.OUTPUT_TOPIC, 10
        )
        self.debug_publisher = self.create_publisher(String, "/xd", 10)
        self.stop = False
        self.scan_data = None
        self.drive_command = None
        self.distance_pub = self.create_publisher(Float32, "/sc_wall_dist", 10)

    def drive_callback(self, msg):
        self.drive_command = msg
        self.get_logger().debug("Received new drive command")
        self.evaluate_safety()

    def scan_callback(self, msg):
        self.scan_data = msg
        self.get_logger().debug("Received new scan data")
        self.evaluate_safety()

    def evaluate_safety(self):
        if self.scan_data is None or self.drive_command is None:
            return
        if self.drive_command.drive.speed < 0.001:
            return
        if not self.stop:
            speed = self.drive_command.drive.speed
            front_treshold = self.MARGIN + (speed**2) / (2 * self.MAX_DECELERATION)
            self.debug_publisher.publish(
                String(data=f"Front threshold: {front_treshold:.2f}")
            )
            self.get_logger().debug(
                f"Evaluating safety: speed={speed:.2f}, front_threshold={front_treshold:.2f}"
            )
            angle_min = self.scan_data.angle_min
            angle_max = self.scan_data.angle_max
            ranges = np.array(self.scan_data.ranges, dtype=np.float32)
            angles = np.linspace(
                angle_min, angle_max, num=ranges.shape[0], dtype=np.float32
            )

            # Prefilter: only keep points that are in front of the car (angle
            # roughly forward) and within plausible range.  The furthest a
            # danger-zone point can be is front_treshold from a front corner
            # plus the corner's own offset from the lidar origin.
            half_width = self.CAR_WIDTH / 2.0
            max_relevant_range = front_treshold + np.sqrt(
                self.LIDAR_OFFSET**2 + half_width**2
            )
            cone_rad = np.radians(self.CONE_ANGLE)
            # The widest angle (from the LIDAR origin) that a danger-zone
            # point could have.  The worst case is a point at distance
            # front_treshold from a front corner, at the edge of the cone.
            # The corner is at (LIDAR_OFFSET, ±half_width), and the cone
            # opens ±cone_rad from the forward axis of that corner.  The
            # furthest-out y a point can reach is:
            #   corner_y + front_treshold * sin(cone_rad)
            # and the smallest x it can have is:
            #   corner_x - front_treshold  (point directly behind the corner)
            # but realistically x >= 0 for anything in front, so use 0 as the
            # conservative minimum x to get the widest possible angle.
            worst_y = half_width + front_treshold * np.sin(cone_rad)
            worst_x = max(self.LIDAR_OFFSET - front_treshold, 0.0)
            max_angle = np.arctan2(worst_y, worst_x) if worst_x > 0.0 else np.pi / 2.0
            # Clamp to pi/2 – nothing behind the car matters
            max_angle = min(max_angle, np.pi / 2.0)

            angle_mask = np.abs(angles) <= max_angle
            range_mask = (ranges > 0.0) & (ranges <= max_relevant_range)
            mask = angle_mask & range_mask
            if not np.any(mask):
                return

            angles_f = angles[mask]
            ranges_f = ranges[mask]

            # Convert to cartesian once
            cos_a = np.cos(angles_f)
            sin_a = np.sin(angles_f)
            px = ranges_f * cos_a
            py = ranges_f * sin_a

            dist_x = px - self.LIDAR_OFFSET
            mask = (np.abs(py) < half_width) & (dist_x > 0)
            if np.any(mask):
                distance = Float32()
                distance.data = float(np.mean(dist_x[mask])) 
                self.distance_pub.publish(distance)

            # --- Rectangle check (vectorised) ---
            rect_hit = np.any(
                (px > 0.0)
                & (px < self.LIDAR_OFFSET + front_treshold)
                & (np.abs(py) < half_width)
            )

            if rect_hit:
                self.stop = True
                self.get_logger().warn(
                    "Frontal object detected! Stopping the robot. Rectangle hit"
                )
                self.get_logger().warn(f"Front treshold: {front_treshold:.2f}")

            if not self.stop:
                # --- Cone check from both front corners (vectorised) ---
                threshold_sq = front_treshold * front_treshold
                for cy in (-half_width, half_width):
                    dx = px - self.LIDAR_OFFSET
                    dy = py - cy
                    dist_sq = dx * dx + dy * dy
                    close_mask = dist_sq < threshold_sq
                    if not np.any(close_mask):
                        continue
                    # Only compute arctan for the nearby subset
                    dx_c = dx[close_mask]
                    dy_c = dy[close_mask]
                    ang = np.abs(np.arctan2(dy_c, dx_c))
                    if np.any(ang < cone_rad):
                        self.stop = True
                        self.get_logger().warn(
                            "Frontal object detected! Stopping the robot. Cone hit."
                        )
                        self.get_logger().warn(f"Front treshold: {front_treshold:.2f}")
                        break
        if self.stop:
            safe_command = AckermannDriveStamped()
            safe_command.header.stamp = self.get_clock().now().to_msg()
            safe_command.drive.speed = 0.0
            safe_command.drive.steering_angle = 0.0
            self.drive_publisher.publish(safe_command)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()