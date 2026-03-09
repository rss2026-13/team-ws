#!/usr/bin/env python3
import numpy as np
import rclpy
import shapely
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")

        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("output_topic", "/drive")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("margin", 0.5)
        self.declare_parameter("max_deceleration", 10.0)
        self.declare_parameter("cone_angle", 10.0)
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

        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_callback, 10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.OUTPUT_TOPIC, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
        )
        self.stop = False
        self.scan_data = None
        self.drive_command = None

        self.robot_polygon = shapely.box(-0.275, -0.125, 0.05, 0.125).buffer(0.15)

    def drive_callback(self, msg):
        self.drive_command = msg
        # self.get_logger().info("Received new drive command")
        self.evaluate_safety()

    def scan_callback(self, msg):
        self.scan_data = msg
        # self.get_logger().info("Received new scan data")
        self.evaluate_safety()

    def evaluate_safety(self):
        if self.scan_data is None or self.drive_command is None:
            return
        if self.drive_command.drive.speed < 0.001:
            return
        if not self.stop:
            speed = self.drive_command.drive.speed
            front_treshold = self.MARGIN + (speed**2) / (2.0 * self.MAX_DECELERATION)
            self.get_logger().info(
                f"Evaluating safety: speed={speed:.2f}, front_threshold={front_treshold:.2f}"
            )
            angle_min = self.scan_data.angle_min
            angle_max = self.scan_data.angle_max
            ranges = np.array(self.scan_data.ranges)
            angles = np.linspace(angle_min, angle_max, num=ranges.shape[0])
            points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T
            front_corners = [
                [0.05, -0.125],
                [0.05, 0.125],
            ]
            for corner in front_corners:
                for point in points:
                    if (
                        np.linalg.norm(point - corner) < front_treshold
                        and abs(
                            np.arctan2(point[1] - corner[1], point[0] - corner[0])
                            * 180
                            / np.pi
                        )
                        < self.CONE_ANGLE
                    ):
                        self.stop = True
                        self.get_logger().warn(
                            "Frontal object detected! Stopping the robot."
                        )
                        break
            # Check rectangle between the corners as well
            for point in points:
                if 0.05 < point[0] < 0.05 + self.MARGIN and -0.125 < point[1] < 0.125:
                    self.stop = True
                    self.get_logger().warn(
                        "Frontal object detected in the safety margin! Stopping the robot."
                    )
                    break
            shapely_points = [shapely.geometry.Point(p) for p in points]
            for point in shapely_points:
                if self.robot_polygon.contains(point):
                    self.stop = True
                    self.get_logger().warn(
                        "Obstacle too close to robot footprint detected! Stopping the robot."
                    )
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
