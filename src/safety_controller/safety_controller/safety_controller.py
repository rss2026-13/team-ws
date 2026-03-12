#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")
        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("output_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("margin", 0.2)
        self.declare_parameter("max_deceleration", 2.5)
        self.declare_parameter("car_width", 0.25)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("lidar_offset", 0.12)          # Lidar to front bumper
        self.declare_parameter("lidar_front_axle_distance", 0.05)  # Lidar to front axle
        self.declare_parameter("collision_point_threshold", 5) # Min points to trigger stop
        self.declare_parameter("visualize", False)

        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.OUTPUT_TOPIC = self.get_parameter("output_topic").get_parameter_value().string_value
        self.SCAN_TOPIC = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.MARGIN = self.get_parameter("margin").get_parameter_value().double_value
        self.MAX_DECELERATION = self.get_parameter("max_deceleration").get_parameter_value().double_value
        self.CAR_WIDTH = self.get_parameter("car_width").get_parameter_value().double_value
        self.WHEELBASE = self.get_parameter("wheelbase").get_parameter_value().double_value
        self.LIDAR_OFFSET = self.get_parameter("lidar_offset").get_parameter_value().double_value
        self.LIDAR_FRONT_AXLE_DIST = self.get_parameter("lidar_front_axle_distance").get_parameter_value().double_value
        self.COLLISION_POINT_THRESHOLD = self.get_parameter("collision_point_threshold").get_parameter_value().integer_value
        self.VISUALIZE = self.get_parameter("visualize").get_parameter_value().bool_value

        # Precomputed constants
        self.rear_axle_dist = self.WHEELBASE - self.LIDAR_FRONT_AXLE_DIST
        self.rear_to_bumper = self.rear_axle_dist + self.LIDAR_OFFSET

        self.drive_subscription = self.create_subscription(AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.OUTPUT_TOPIC, 10)
        self.distance_pub = self.create_publisher(Float32, "/sc_wall_dist", 10)
        self.marker_pub = self.create_publisher(Marker, "/safety_marker", 1)

        self.is_collision = False
        self.scan_data = None
        self.drive_command = None
        self.scan_cos_angles = None
        self.scan_sin_angles = None
        self.laser_frame = "laser"  # Update to match your actual laser frame_id

    def drive_callback(self, msg):
        self.drive_command = msg

    def scan_callback(self, msg):
        if self.scan_cos_angles is None or self.scan_sin_angles is None:
            angles = np.linspace(msg.angle_min, msg.angle_max, num=len(msg.ranges))
            self.scan_cos_angles = np.cos(angles)
            self.scan_sin_angles = np.sin(angles)

        self.scan_data = msg
        self.evaluate_safety()

    def evaluate_safety(self):
        if self.scan_data is None or self.drive_command is None:
            return

        velocity = self.drive_command.drive.speed
        delta = self.drive_command.drive.steering_angle

        if velocity < 0.001:
            return

        # Kinematic stopping distance threshold
        front_threshold = self.MARGIN + (velocity ** 2) / (2 * self.MAX_DECELERATION)

        if self.VISUALIZE:
            self.publish_safety_marker(delta, front_threshold)

        # Get Cartesian points, filtering out invalid lidar returns
        ranges_raw = np.array(self.scan_data.ranges)
        valid = np.isfinite(ranges_raw)
        ranges = ranges_raw[valid]
        cos_angles = self.scan_cos_angles[valid]
        sin_angles = self.scan_sin_angles[valid]
        px = ranges * cos_angles
        py = ranges * sin_angles

        # Publish 5th-percentile distance to nearest obstacle ahead of bumper
        # (5th percentile is robust to noise while still catching close obstacles)
        dist_x = px - self.LIDAR_OFFSET
        mask = (np.abs(py) < self.CAR_WIDTH / 2) & (dist_x > 0)
        if np.any(mask):
            distance = Float32()
            distance.data = float(np.percentile(dist_x[mask], 5))
            self.distance_pub.publish(distance)

        in_path = np.zeros(len(px), dtype=bool)

        if abs(delta) < 0.01:  # Straight path
            collision_zone_start = self.LIDAR_OFFSET
            collision_zone_end = self.LIDAR_OFFSET + front_threshold
            in_path = (px > collision_zone_start) & \
                      (px < collision_zone_end) & \
                      (np.abs(py) < self.CAR_WIDTH / 2)
        else:  # Curved path (bicycle model)
            R = self.WHEELBASE / np.tan(delta)
            R_max = np.sqrt(self.rear_to_bumper**2 + (abs(R) + self.CAR_WIDTH / 2)**2)
            R_min = abs(R) - self.CAR_WIDTH / 2

            # Transform points into center of rotation (CoR) frame
            px_cor = px + self.rear_axle_dist
            py_cor = py - R
            pr = np.sqrt(px_cor**2 + py_cor**2)
            pangle = np.mod(np.arctan2(px_cor, py_cor * -np.sign(R)), 2 * np.pi)

            # Arc distance from bumper to each point
            bumper_angle = np.arcsin(np.clip(self.rear_to_bumper / pr, -1.0, 1.0))
            p_bumper_ahead_dist = (pangle - bumper_angle) * pr

            in_path = (pr > R_min) & (pr < R_max) & \
                      (p_bumper_ahead_dist > 0) & (p_bumper_ahead_dist < front_threshold)

        # Require multiple points to filter out noise/spurious returns
        self.is_collision = np.sum(in_path) > self.COLLISION_POINT_THRESHOLD

        if self.is_collision:
            safe_command = AckermannDriveStamped()
            safe_command.header.stamp = self.get_clock().now().to_msg()
            safe_command.drive.speed = 0.0
            safe_command.drive.steering_angle = delta
            self.drive_publisher.publish(safe_command)
            self.get_logger().warn("Frontal object detected! Stopping the robot.")

    def create_point(self, x, y):
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p

    def publish_safety_marker(self, delta, stop_dist):
        marker = Marker()
        marker.header.frame_id = self.laser_frame
        marker.ns = "safety_zone"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        num_steps = 20

        if abs(delta) < 0.01:  # Straight path
            y_inner = -self.CAR_WIDTH / 2
            y_outer = self.CAR_WIDTH / 2
            x_start = self.LIDAR_OFFSET
            x_end = x_start + stop_dist

            marker.points.append(self.create_point(x_start, y_outer))
            marker.points.append(self.create_point(x_end, y_outer))
            marker.points.append(self.create_point(x_start, y_inner))
            marker.points.append(self.create_point(x_end, y_inner))
            marker.points.append(self.create_point(x_start, y_inner))
            marker.points.append(self.create_point(x_start, y_outer))

        else:  # Curved path
            R = self.WHEELBASE / np.tan(delta)
            R_min = abs(R) - self.CAR_WIDTH / 2
            R_max = np.sqrt(self.rear_to_bumper**2 + (abs(R) + self.CAR_WIDTH / 2)**2)

            def get_arc_point(radius, current_arc_dist, theta_start):
                theta = theta_start + (current_arc_dist / radius)
                # CoR is at (-rear_axle_dist, R) in laser frame
                x = radius * np.sin(theta) - self.rear_axle_dist
                if R > 0:  # Left turn
                    y = R - radius * np.cos(theta)
                else:      # Right turn
                    y = R + radius * np.cos(theta)
                return self.create_point(x, y)

            theta_start_min = np.arcsin(np.clip(self.rear_to_bumper / R_min, -1.0, 1.0))
            theta_start_max = np.arcsin(np.clip(self.rear_to_bumper / R_max, -1.0, 1.0))

            for i in range(num_steps):
                d1 = (i / num_steps) * stop_dist
                d2 = ((i + 1) / num_steps) * stop_dist

                marker.points.append(get_arc_point(R_max, d1, theta_start_max))
                marker.points.append(get_arc_point(R_max, d2, theta_start_max))
                marker.points.append(get_arc_point(R_min, d1, theta_start_min))
                marker.points.append(get_arc_point(R_min, d2, theta_start_min))

            # Front bumper crossbar
            marker.points.append(get_arc_point(R_max, 0.0, theta_start_max))
            marker.points.append(get_arc_point(R_min, 0.0, theta_start_min))

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()