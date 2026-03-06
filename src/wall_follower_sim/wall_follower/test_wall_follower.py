#!/usr/bin/env python3
import math
import numpy as np
import time as pythontime
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from wall_follower.np_encrypt import encode
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class WallTest(Node):
    def __init__(self):
        super().__init__("test_wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("pose_topic", "/pose")

        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)
        self.declare_parameter("start_x", -4.0)
        self.declare_parameter("start_y", -5.4)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("end_x", 5.0)
        self.declare_parameter("end_y", -5.0)
        self.declare_parameter("name", "default")

        # Fetch constants from the ROS parameter server
        self.TEST_NAME = self.get_parameter("name").get_parameter_value().string_value
        self.SCAN_TOPIC = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.POSE_TOPIC = (
            self.get_parameter("pose_topic").get_parameter_value().string_value
        )
        self.DRIVE_TOPIC = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )

        self.SIDE = self.get_parameter("side").get_parameter_value().integer_value
        self.VELOCITY = (
            self.get_parameter("velocity").get_parameter_value().double_value
        )
        self.DESIRED_DISTANCE = (
            self.get_parameter("desired_distance").get_parameter_value().double_value
        )
        self.START_x = self.get_parameter("start_x").get_parameter_value().double_value
        self.START_y = self.get_parameter("start_y").get_parameter_value().double_value
        self.START_z = self.get_parameter("start_z").get_parameter_value().double_value
        self.END_x = self.get_parameter("end_x").get_parameter_value().double_value
        self.END_y = self.get_parameter("end_y").get_parameter_value().double_value
        self.NAME = self.get_parameter("name").get_parameter_value().string_value

        self.get_logger().info("Test Name %s" % (self.TEST_NAME))

        self.max_time_per_test = 120
        self.end_threshold = 1.0

        self.positions = []
        self.dist_to_end = np.infty
        self.saves = {}

        # Buffers for wall metrics: from wall_follower's distance/angle topics
        self.past_distances = []
        self.past_angles = []
        self.metric_max_buffer = 5000

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()

        # A publisher for navigation commands
        self.pose_pub = self.create_publisher(Pose, "pose", 1)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.DRIVE_TOPIC, 1
        )

        # A publisher for the end position marker
        self.marker_pub = self.create_publisher(Marker, "/end_position_marker", 1)

        # A subscriber to laser scans
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 1)

        # Subscribers to wall_follower's distance/angle (compute metrics ourselves)
        self.create_subscription(Float32, "/distance", self._cb_distance, 10)
        self.create_subscription(Float32, "/angle", self._cb_angle, 10)

        self.START_POSE = [self.START_x, self.START_y, self.START_z]
        self.END_POSE = [self.END_x, self.END_y]

        self.buffer_count = 0
        self.place_car(self.START_POSE)

        self.moved = False

    def _cb_distance(self, msg):
        self.past_distances.append(float(msg.data))
        if len(self.past_distances) > self.metric_max_buffer:
            self.past_distances.pop(0)

    def _cb_angle(self, msg):
        self.past_angles.append(float(msg.data))
        if len(self.past_angles) > self.metric_max_buffer:
            self.past_angles.pop(0)

    def _log_wall_metrics(self):
        """Log wall metrics as error from desired: |actual - desired|, then average or average of squared."""
        self.get_logger().info("--- Wall metrics (error from desired) for test '%s' ---" % self.TEST_NAME)
        desired_d = self.DESIRED_DISTANCE
        desired_a = 0.0  # desired angle = parallel to wall

        # Distance error metrics
        if self.past_distances:
            arr = np.array(self.past_distances)
            n = len(arr)
            err_d = np.abs(arr - desired_d)
            err_d_sq = (arr - desired_d) ** 2
            self.get_logger().info(
                "  average_distance_error (mean |d-desired|): %.4f (n=%d)" % (float(np.mean(err_d)), n)
            )
            self.get_logger().info(
                "  squared_distance_error (mean (d-desired)^2): %.4f" % float(np.mean(err_d_sq))
            )
            self.get_logger().info(
                "  std_distance_error:  %.4f" % (float(np.std(arr - desired_d)) if n > 1 else 0.0)
            )
            self.get_logger().info(
                "  ste_distance_error:  %.4f"
                % (float(np.std(arr - desired_d) / math.sqrt(n)) if n > 1 else 0.0)
            )
        else:
            self.get_logger().info("  average_distance_error: (no data)")
            self.get_logger().info("  squared_distance_error: (no data)")
            self.get_logger().info("  std_distance_error: (no data)")
            self.get_logger().info("  ste_distance_error: (no data)")

        # Angle error metrics (desired angle = 0): average and std only
        if self.past_angles:
            arr = np.array(self.past_angles)
            n = len(arr)
            err_a = np.abs(arr - desired_a)
            self.get_logger().info(
                "  average_angle_error (mean |angle|): %.4f (n=%d)" % (float(np.mean(err_a)), n)
            )
            self.get_logger().info(
                "  std_angle_error: %.4f" % (float(np.std(arr - desired_a)) if n > 1 else 0.0)
            )
        else:
            self.get_logger().info("  average_angle_error: (no data)")
            self.get_logger().info("  std_angle_error: (no data)")

        self.get_logger().info("------------------------------------")

    def place_car(self, pose):
        p = Pose()

        p.position.x = pose[0]
        p.position.y = pose[1]

        # Convert theta to a quaternion
        quaternion = R.from_euler('xyz', [0, 0, pose[2]]).as_quat()
        p.orientation.y = quaternion[1]
        p.orientation.z = quaternion[2]
        p.orientation.w = quaternion[3]

        self.pose_pub.publish(p)
        pythontime.sleep(0.05)

    def publish_end_position_marker(self):
        """Visualize the end position of the test"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "end_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.END_x
        marker.pose.position.y = self.END_y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue

        self.marker_pub.publish(marker)

    def laser_callback(self, laser_scan):
        self.publish_end_position_marker()

        # Give buffer time for controller to begin working before letting the car go
        if self.buffer_count < 100:
            self.place_car(self.START_POSE)
            self.buffer_count += 1
            if self.buffer_count == 30:
                self.get_logger().info(
                    f"Placed Car: {self.START_POSE[0]}, {self.START_POSE[1]}"
                )
            return

        from_frame_rel = "base_link"
        to_frame_rel = "map"

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return

        if not self.moved:
            diff = np.linalg.norm(
                np.array([self.START_x, self.START_y])
                - np.array([t.transform.translation.x, t.transform.translation.y])
            )
            if 0.3 < (diff):
                self.place_car(self.START_POSE)
                self.get_logger().info(
                    f"Not at start {self.START_x-t.transform.translation.x}, {self.START_y-t.transform.translation.y}, diff {diff}"
                )
                return
            else:
                self.moved = True
                # self.get_logger().info('Moved: %s' % (self.moved))
                self.start_time = self.get_clock().now()

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
        valid_points = self.SIDE * y > 0
        valid_points = np.logical_and(valid_points, x < 1.5)
        valid_points = np.logical_and(valid_points, x > 0.0)

        # Compute the average distance
        dists = np.abs(y[valid_points])
        dist = np.sum(dists) / dists.shape[0]
        # self.get_logger().info('Avg dist: %f' % (dist))

        pos = [t.transform.translation.x, t.transform.translation.y]

        time = self.get_clock().now() - self.start_time
        time_d = time.nanoseconds * 1e-9
        self.positions.append([time_d] + pos + [dist])
        self.dist_to_end = np.linalg.norm(np.array(pos) - np.array(self.END_POSE))
        # self.get_logger().info(
        #             f'Time: {time_d}, Max time: {self.max_time_per_test}')

        if time_d > self.max_time_per_test:
            self.get_logger().error(
                "\n\n\n\n\nERROR: Test timed out! Your car was not able to reach the target end position.\n\n\n\n\n"
            )
            self._log_wall_metrics()
            # Send a message of zero
            stop = AckermannDriveStamped()
            stop.drive.speed = 0.0
            stop.drive.steering_angle = 0.0
            self.drive_pub.publish(stop)
            self.saves[self.TEST_NAME] = encode(np.array(self.positions))
            np.savez_compressed(self.TEST_NAME + "_log", **self.saves)
            raise SystemExit
        if self.dist_to_end < self.end_threshold:
            self.get_logger().info(
                "\n\n\n\n\nReached end of the test w/ Avg dist from wall = %f!\n\n\n\n\n"
                % (dist)
            )
            self._log_wall_metrics()
            stop = AckermannDriveStamped()
            stop.drive.speed = 0.0
            stop.drive.steering_angle = 0.0
            self.drive_pub.publish(stop)
            self.saves[self.TEST_NAME] = encode(np.array(self.positions))
            np.savez_compressed(self.TEST_NAME + "_log", **self.saves)
            raise SystemExit


def main():
    rclpy.init()
    wall_follower_test = WallTest()
    try:
        rclpy.spin(wall_follower_test)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")
    wall_follower_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
