#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")
        # /vesc/low_level/ackermann_cmd
        # /vesc/low_level/input/safety
        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("output_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("margin", 0.2)
        self.declare_parameter("max_deceleration", 2.0)
        self.declare_parameter("car_width", 0.25)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter(
            "lidar_offset", 0.15
        )  # Distance from lidar to front bumper
        self.declare_parameter("visualize", False)

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
        self.CAR_WIDTH = (
            self.get_parameter("car_width").get_parameter_value().double_value
        )
        self.WHEELBASE = (
            self.get_parameter("wheelbase").get_parameter_value().double_value
        )
        self.LIDAR_OFFSET = (
            self.get_parameter("lidar_offset").get_parameter_value().double_value
        )
        self.VISUALIZE = (
            self.get_parameter("visualize").get_parameter_value().bool_value
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
        # self.distance_pub = self.create_publisher(Float32, "/sc_wall_dist", 10)
        self.front_threshold_pub = self.create_publisher(Float32, "/sc_front_threshold", 10)

        # self.marker_pub = self.create_publisher(Marker, "/safety_marker", 1)
        self.is_collision = False
        self.scan_data = None
        self.drive_command = None
        self.scan_cos_angles = None
        self.scan_sin_angles = None

        self.clear_scan_count = 0
        self.CLEAR_SCANS_REQUIRED = 3

        self.LIDAR_X = 0.275 #lidar x from base link
        self.front_mask = None

        self.declare_parameter("min_obstacle_points", 5)
        self.MIN_OBSTACLE_POINTS = (
            self.get_parameter("min_obstacle_points").get_parameter_value().integer_value
        )

        self.safety_stop_pub = self.create_publisher(Bool, "/safety_stop", 10)

    def drive_callback(self, msg):
        self.drive_command = msg
        self.get_logger().debug("Received new drive command")
        self.evaluate_safety()

    def scan_callback(self, msg):
        if self.scan_cos_angles is None or self.scan_sin_angles is None:
            angles = np.linspace(msg.angle_min, msg.angle_max, num=len(msg.ranges))
            front_mask = np.abs(angles) < (np.pi / 2)
            self.front_mask = front_mask
            self.scan_cos_angles = np.cos(angles[front_mask])
            self.scan_sin_angles = np.sin(angles[front_mask]) 

        self.scan_data = msg
        self.get_logger().debug("Received new scan data")
        self.evaluate_safety()

    def evaluate_safety(self):
        if self.scan_data is None or self.drive_command is None:
            return

        velocity = self.drive_command.drive.speed
        if velocity < 0.001:
            return

        # Kinematic Threshold
        front_threshold = self.MARGIN + (velocity**2) / (2 * self.MAX_DECELERATION)
        self.front_threshold_pub.publish(Float32(data = front_threshold))

        delta = self.drive_command.drive.steering_angle
        if self.VISUALIZE:
            self.publish_safety_marker(delta, front_threshold)

        # Get Cartesian points
        ranges = np.array(self.scan_data.ranges)[self.front_mask]
        px = ranges * self.scan_cos_angles
        py = ranges * self.scan_sin_angles

        # Publish distance from front bumper
        # dist_x = px - self.LIDAR_OFFSET
        # mask = (np.abs(py) < self.CAR_WIDTH / 2) & (dist_x > 0)
        # if np.any(mask):
        #     distance = Float32()
        #     distance.data = float(np.percentile(dist_x[mask], 5))
        #     self.distance_pub.publish(distance)

        # Adjust for LIDAR offset (Move points to car's front bumper frame)
        # We check if points are within 'front_threshold' OF THE BUMPER
        collision_zone_start = self.LIDAR_OFFSET
        collision_zone_end = self.LIDAR_OFFSET + front_threshold


        if abs(delta) < 0.01:  # Straight Path
            # Points must be ahead of bumper AND within threshold
            in_path = (
                (px > collision_zone_start)
                & (px < collision_zone_end)
                & (np.abs(py) < self.CAR_WIDTH / 2)
            )
        else:  # Curved Path (Bicycle Model)
            R = self.WHEELBASE / np.tan(
                delta
            )  # Radius of car rotation, max and min account for front and back corner

            bumper_to_cor_x = self.LIDAR_X + self.LIDAR_OFFSET  # 0.275 + 0.15 = 0.425m

            # R_max: CoR → front far corner (outer swept edge)
            R_max = np.sqrt(bumper_to_cor_x**2 + (abs(R) + self.CAR_WIDTH / 2)**2)
            # R_min: CoR → front near corner (inner swept edge)
            R_min = np.sqrt(bumper_to_cor_x**2 + (abs(R) - self.CAR_WIDTH / 2)**2)


            # Points in car center of rotation (cor) frame
            px_cor = px + self.LIDAR_X
            py_cor = py - R
 
            pr = np.sqrt(px_cor**2 + py_cor**2)
            pangle = np.mod(np.arctan2(px_cor, py_cor * -np.sign(R)), 2 * np.pi)

             # Angle to the front bumper arc in the CoR frame
            bumper_angle = np.arcsin(np.clip(bumper_to_cor_x / pr, -1.0, 1.0))
            p_bumper_ahead_dist = (pangle - bumper_angle) * pr
 
            in_path = (
                (pr > R_min)
                & (pr < R_max)
                & (p_bumper_ahead_dist > 0)
                & (p_bumper_ahead_dist < front_threshold)
            )


        self.is_collision = np.sum(in_path) >= self.MIN_OBSTACLE_POINTS
 
        if self.is_collision:
            self.clear_scan_count = 0
        else:
            self.clear_scan_count += 1
 
        if self.is_collision or self.clear_scan_count < self.CLEAR_SCANS_REQUIRED:
            self.get_logger().info("Safety controller pubbing drive")
            safe_command = AckermannDriveStamped()
            safe_command.header.stamp = self.get_clock().now().to_msg()
            safe_command.drive.speed = 0.0
            safe_command.drive.steering_angle = 0.0
            self.drive_publisher.publish(safe_command)
            self.safety_stop_pub.publish(Bool(data = True))
            self.get_logger().info("Frontal object detected! Stopping the robot.")
        else:
            self.safety_stop_pub.publish(Bool(data = False))


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
