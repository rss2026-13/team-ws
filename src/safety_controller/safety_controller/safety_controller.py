#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


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
        # if not self.stop:
        speed = self.drive_command.drive.speed
        front_treshold = (
            self.MARGIN + self.LIDAR_OFFSET + (speed**2) / (2 * self.MAX_DECELERATION)
        )
        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        ranges = np.array(self.scan_data.ranges)
        angles = np.linspace(angle_min, angle_max, num=ranges.shape[0])
        cone_mask = np.abs(angles) < np.radians(self.CONE_ANGLE)
        # width_mask = np.abs(np.sin(angles)) * ranges < (self.CAR_WIDTH / 2 + 0.05)
        danger_mask = cone_mask & (ranges < front_treshold)
        closest_obstacle = np.min(ranges[cone_mask]) if np.any(cone_mask) else np.inf
        if np.any(danger_mask):
            self.get_logger().warn("Obstacle detected! Stopping the car.")
            self.stop = True
        self.debug_publisher.publish(
            String(
                data=f"Front threshold: {front_treshold:.2f}, Danger: {np.any(danger_mask)}, Closest: {closest_obstacle:.2f}"
            )
        )

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
