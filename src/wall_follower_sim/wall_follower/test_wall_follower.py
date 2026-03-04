#!/usr/bin/env python3
import numpy as np
import math
import time as pythontime
import matplotlib.pyplot as plt
import rclpy
import os

from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from wall_follower.np_encrypt import encode
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class WallTest(Node):
    def __init__(self):
        super().__init__("test_wall_follower")

        # ===============================
        # ORIGINAL PARAMETERS (UNCHANGED)
        # ===============================
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("name", "default")

        self.declare_parameter("start_x", -4.0)
        self.declare_parameter("start_y", -5.4)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("end_x", 5.0)
        self.declare_parameter("end_y", -5.0)

        self.TEST_NAME = self.get_parameter("name").value
        self.DRIVE_TOPIC = self.get_parameter("drive_topic").value
        self.POSE_TOPIC = self.get_parameter("pose_topic").value

        self.START_x = self.get_parameter("start_x").value
        self.START_y = self.get_parameter("start_y").value
        self.START_z = self.get_parameter("start_z").value
        self.END_x = self.get_parameter("end_x").value
        self.END_y = self.get_parameter("end_y").value

        self.START_POSE = [self.START_x, self.START_y, self.START_z]
        self.END_POSE = [self.END_x, self.END_y]

        self.get_logger().info(f"Test Name {self.TEST_NAME}")

        # ===============================
        # ORIGINAL TEST VARIABLES
        # ===============================
        self.max_time_per_test = 120
        self.end_threshold = 1.0
        self.buffer_count = 0
        self.moved = False
        self.positions = []
        self.dist_to_end = np.inf

        # ===============================
        # TF (UNCHANGED)
        # ===============================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===============================
        # ORIGINAL PUBLISHERS
        # ===============================
        self.pose_pub = self.create_publisher(Pose, self.POSE_TOPIC, 1)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.DRIVE_TOPIC, 1
        )
        self.marker_pub = self.create_publisher(Marker, "/end_position_marker", 1)

        # ===============================
        # METRIC PUBLISHERS
        # ===============================
        self.avg_dist_pub = self.create_publisher(Float32, 'average_distance', 10)
        self.sq_dist_pub = self.create_publisher(Float32, 'squared_distance', 10)
        self.std_dist_pub = self.create_publisher(Float32, 'std_distance', 10)
        self.ste_dist_pub = self.create_publisher(Float32, 'ste_distance', 10)
        self.avg_angle_pub = self.create_publisher(Float32, 'average_angle', 10)
        self.sq_angle_pub = self.create_publisher(Float32, 'squared_angle', 10)

        # ===============================
        # Subscribe to controller outputs
        # ===============================
        
        self.create_subscription(Float32, "/wall_follower_ns/distance", self.distance_callback, 10)
        self.create_subscription(Float32, "/wall_follower_ns/angle", self.angle_callback, 10)

        self.past_distances = []
        self.past_angles = []
        self.latest_distance = None

        # ===============================
        # LIVE PLOT
        # ===============================
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Distance")

        # ===============================
        # ORIGINAL START
        # ===============================
        self.start_time = self.get_clock().now()
        self.place_car(self.START_POSE)

        # IMPORTANT:
        # Replace LaserScan callback loop with timer
        self.timer = self.create_timer(0.05, self.test_loop)

    # ==========================================================
    # ORIGINAL PLACE CAR (UNCHANGED)
    # ==========================================================
    def place_car(self, pose):
        p = Pose()
        p.position.x = pose[0]
        p.position.y = pose[1]

        quaternion = R.from_euler('xyz', [0, 0, pose[2]]).as_quat()
        p.orientation.y = quaternion[1]
        p.orientation.z = quaternion[2]
        p.orientation.w = quaternion[3]

        self.pose_pub.publish(p)
        pythontime.sleep(0.05)

    # ==========================================================
    # CONTROLLER DATA CALLBACKS
    # ==========================================================
    def distance_callback(self, msg):
        self.latest_distance = msg.data
        self.past_distances.append(msg.data)

    def angle_callback(self, msg):
        self.past_angles.append(msg.data)

    # ==========================================================
    # THIS IS YOUR ORIGINAL LASER CALLBACK LOGIC
    # Now moved into a timer loop (so lifecycle is preserved)
    # ==========================================================
    def test_loop(self):

        self.publish_end_position_marker()

        # Give controller time before letting go
        if self.buffer_count < 100:
            self.place_car(self.START_POSE)
            self.buffer_count += 1
            return

        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except TransformException:
            return

        pos = [t.transform.translation.x, t.transform.translation.y]

        if not self.moved:
            diff = np.linalg.norm(
                np.array([self.START_x, self.START_y]) - np.array(pos)
            )
            if diff > 0.3:
                self.place_car(self.START_POSE)
                return
            else:
                self.moved = True
                self.start_time = self.get_clock().now()

        time_d = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # LOG POSITION + DISTANCE
        if self.latest_distance is not None:
            self.positions.append([time_d] + pos + [self.latest_distance])
            self.update_plot()

        self.dist_to_end = np.linalg.norm(np.array(pos) - np.array(self.END_POSE))

        if time_d > self.max_time_per_test:
            self.get_logger().error("Test timed out!")
            self.finish_test()

        if self.dist_to_end < self.end_threshold:
            self.get_logger().info("Reached end of test!")
            self.finish_test()

    # ==========================================================
    def publish_end_position_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.pose.position.x = self.END_x
        marker.pose.position.y = self.END_y
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.marker_pub.publish(marker)

    # ==========================================================
    def update_plot(self):
        self.line.set_xdata(np.arange(len(self.past_distances)))
        self.line.set_ydata(self.past_distances)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # ==========================================================
    def finish_test(self):

        distances = np.array(self.past_distances)
        angles = np.array(self.past_angles)
    
        if len(distances) == 0 or len(angles) == 0:
            self.get_logger().error("No metric data!")
            raise SystemExit
    
        # ===============================
        # METRIC COMPUTATION
        # ===============================
        avg_dist = float(np.mean(distances))
        sq_dist = float(np.mean(distances ** 2))
        std_dist = float(np.std(distances))
        ste_dist = float(std_dist / math.sqrt(len(distances)))
    
        avg_angle = float(np.mean(angles))
        sq_angle = float(np.mean(angles ** 2))
    
        # ===============================
        # PUBLISH METRICS
        # ===============================
        self.publish_metric(self.avg_dist_pub, avg_dist)
        self.publish_metric(self.sq_dist_pub, sq_dist)
        self.publish_metric(self.std_dist_pub, std_dist)
        self.publish_metric(self.ste_dist_pub, ste_dist)
        self.publish_metric(self.avg_angle_pub, avg_angle)
        self.publish_metric(self.sq_angle_pub, sq_angle)
    
        # ===============================
        # LOG METRICS
        # ===============================
        self.get_logger().info("========== TEST METRICS ==========")
        self.get_logger().info(f"Test Name: {self.TEST_NAME}")
        self.get_logger().info(f"Average Distance: {avg_dist:.4f}")
        self.get_logger().info(f"Mean Squared Distance: {sq_dist:.4f}")
        self.get_logger().info(f"Std Distance: {std_dist:.4f}")
        self.get_logger().info(f"Std Error Distance: {ste_dist:.4f}")
        self.get_logger().info(f"Average Angle: {avg_angle:.4f}")
        self.get_logger().info(f"Mean Squared Angle: {sq_angle:.4f}")
        self.get_logger().info("==================================")
    
        # ===============================
        # SAVE FINAL PLOTS
        # ===============================
        output_dir = "test_results"
        os.makedirs(output_dir, exist_ok=True)
    
        # ---- 1) Final Distance Plot ----
        plt.ioff()
        plt.figure()
        plt.plot(distances)
        plt.xlabel("Timestep")
        plt.ylabel("Distance")
        plt.title(f"Distance Error - {self.TEST_NAME}")
        plt.grid()
        plt.savefig(f"{output_dir}/{self.TEST_NAME}_distance.png")
        plt.close()
    
        self.get_logger().info(f"Plots saved to {output_dir}/")
    
        # ===============================
        # STOP VEHICLE
        # ===============================
        stop = AckermannDriveStamped()
        stop.drive.speed = 0.0
        stop.drive.steering_angle = 0.0
        self.drive_pub.publish(stop)
    
        # ===============================
        # SAVE LOG FILE
        # ===============================
        self.saves = {}
        self.saves[self.TEST_NAME] = encode(np.array(self.positions))
        np.savez_compressed(self.TEST_NAME + "_log", **self.saves)
    
        
        raise SystemExit

    # def finish_test(self):

    #     distances = np.array(self.past_distances)
    #     angles = np.array(self.past_angles)

    #     if len(distances) == 0 or len(angles) == 0:
    #         self.get_logger().error("No metric data!")
    #         raise SystemExit

    #     avg_dist = float(np.mean(distances))
    #     sq_dist = float(np.mean(distances ** 2))
    #     std_dist = float(np.std(distances))
    #     ste_dist = float(std_dist / math.sqrt(len(distances)))

    #     avg_angle = float(np.mean(angles))
    #     sq_angle = float(np.mean(angles ** 2))

    #     self.publish_metric(self.avg_dist_pub, avg_dist)
    #     self.publish_metric(self.sq_dist_pub, sq_dist)
    #     self.publish_metric(self.std_dist_pub, std_dist)
    #     self.publish_metric(self.ste_dist_pub, ste_dist)
    #     self.publish_metric(self.avg_angle_pub, avg_angle)
    #     self.publish_metric(self.sq_angle_pub, sq_angle)
        
    #     self.get_logger().info("========== TEST METRICS ==========")
    #     self.get_logger().info(f"Test Name: {self.TEST_NAME}")
    #     self.get_logger().info(f"Average Distance: {avg_dist:.4f}")
    #     self.get_logger().info(f"Mean Squared Distance: {sq_dist:.4f}")
    #     self.get_logger().info(f"Std Distance: {std_dist:.4f}")
    #     self.get_logger().info(f"Std Error Distance: {ste_dist:.4f}")
    #     self.get_logger().info(f"Average Angle: {avg_angle:.4f}")
    #     self.get_logger().info(f"Mean Squared Angle: {sq_angle:.4f}")
    #     self.get_logger().info("==================================")


    #     stop = AckermannDriveStamped()
    #     stop.drive.speed = 0.0
    #     stop.drive.steering_angle = 0.0
    #     self.drive_pub.publish(stop)

    #     self.saves = {}
    #     self.saves[self.TEST_NAME] = encode(np.array(self.positions))
    #     np.savez_compressed(self.TEST_NAME + "_log", **self.saves)

    #     raise SystemExit

    def publish_metric(self, publisher, value):
        msg = Float32()
        msg.data = value
        publisher.publish(msg)


def main():
    rclpy.init()
    node = WallTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# import numpy as np
# import time as pythontime
# import rclpy

# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Pose
# from ackermann_msgs.msg import AckermannDriveStamped
# from visualization_msgs.msg import Marker
# from wall_follower.np_encrypt import encode
# from scipy.spatial.transform import Rotation as R
# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener


# class WallTest(Node):
#     def __init__(self):
#         super().__init__("test_wall_follower")
#         # Declare parameters to make them available for use
#         self.declare_parameter("scan_topic", "/scan")
#         self.declare_parameter("drive_topic", "/drive")
#         self.declare_parameter("pose_topic", "/pose")

#         self.declare_parameter("side", 1)
#         self.declare_parameter("velocity", 1.0)
#         self.declare_parameter("desired_distance", 1.0)
#         self.declare_parameter("start_x", -4.0)
#         self.declare_parameter("start_y", -5.4)
#         self.declare_parameter("start_z", 0.0)
#         self.declare_parameter("end_x", 5.0)
#         self.declare_parameter("end_y", -5.0)
#         self.declare_parameter("name", "default")

#         # Fetch constants from the ROS parameter server
#         self.TEST_NAME = self.get_parameter("name").get_parameter_value().string_value
#         self.SCAN_TOPIC = (
#             self.get_parameter("scan_topic").get_parameter_value().string_value
#         )
#         self.POSE_TOPIC = (
#             self.get_parameter("pose_topic").get_parameter_value().string_value
#         )
#         self.DRIVE_TOPIC = (
#             self.get_parameter("drive_topic").get_parameter_value().string_value
#         )

#         self.SIDE = self.get_parameter("side").get_parameter_value().integer_value
#         self.VELOCITY = (
#             self.get_parameter("velocity").get_parameter_value().double_value
#         )
#         self.DESIRED_DISTANCE = (
#             self.get_parameter("desired_distance").get_parameter_value().double_value
#         )
#         self.START_x = self.get_parameter("start_x").get_parameter_value().double_value
#         self.START_y = self.get_parameter("start_y").get_parameter_value().double_value
#         self.START_z = self.get_parameter("start_z").get_parameter_value().double_value
#         self.END_x = self.get_parameter("end_x").get_parameter_value().double_value
#         self.END_y = self.get_parameter("end_y").get_parameter_value().double_value
#         self.NAME = self.get_parameter("name").get_parameter_value().string_value

#         self.get_logger().info("Test Name %s" % (self.TEST_NAME))

#         self.max_time_per_test = 120
#         self.end_threshold = 1.0

#         self.positions = []
#         self.dist_to_end = np.infty
#         self.saves = {}

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.start_time = self.get_clock().now()

#         # A publisher for navigation commands
#         self.pose_pub = self.create_publisher(Pose, "pose", 1)
#         self.drive_pub = self.create_publisher(
#             AckermannDriveStamped, self.DRIVE_TOPIC, 1
#         )

#         # A publisher for the end position marker
#         self.marker_pub = self.create_publisher(Marker, "/end_position_marker", 1)

#         # A subscriber to laser scans
#         self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 1)

#         self.START_POSE = [self.START_x, self.START_y, self.START_z]
#         self.END_POSE = [self.END_x, self.END_y]

#         self.buffer_count = 0
#         self.place_car(self.START_POSE)

#         self.moved = False

#     def place_car(self, pose):
#         p = Pose()

#         p.position.x = pose[0]
#         p.position.y = pose[1]

#         # Convert theta to a quaternion
#         quaternion = R.from_euler('xyz', [0, 0, pose[2]]).as_quat()
#         p.orientation.y = quaternion[1]
#         p.orientation.z = quaternion[2]
#         p.orientation.w = quaternion[3]

#         self.pose_pub.publish(p)
#         pythontime.sleep(0.05)

#     def publish_end_position_marker(self):
#         """Visualize the end position of the test"""
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "end_position"
#         marker.id = 0
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = self.END_x
#         marker.pose.position.y = self.END_y
#         marker.pose.position.z = 0.0
#         marker.scale.x = 0.5
#         marker.scale.y = 0.5
#         marker.scale.z = 0.5
#         marker.color.a = 1.0  # Alpha
#         marker.color.r = 0.0  # Red
#         marker.color.g = 1.0  # Green
#         marker.color.b = 0.0  # Blue

#         self.marker_pub.publish(marker)

#     def laser_callback(self, laser_scan):
#         self.publish_end_position_marker()

#         # Give buffer time for controller to begin working before letting the car go
#         if self.buffer_count < 100:
#             self.place_car(self.START_POSE)
#             self.buffer_count += 1
#             if self.buffer_count == 30:
#                 self.get_logger().info(
#                     f"Placed Car: {self.START_POSE[0]}, {self.START_POSE[1]}"
#                 )
#             return

#         from_frame_rel = "base_link"
#         to_frame_rel = "map"

#         try:
#             t = self.tf_buffer.lookup_transform(
#                 to_frame_rel, from_frame_rel, rclpy.time.Time()
#             )
#         except TransformException as ex:
#             self.get_logger().info(
#                 f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
#             )
#             return

#         if not self.moved:
#             diff = np.linalg.norm(
#                 np.array([self.START_x, self.START_y])
#                 - np.array([t.transform.translation.x, t.transform.translation.y])
#             )
#             if 0.3 < (diff):
#                 self.place_car(self.START_POSE)
#                 self.get_logger().info(
#                     f"Not at start {self.START_x-t.transform.translation.x}, {self.START_y-t.transform.translation.y}, diff {diff}"
#                 )
#                 return
#             else:
#                 self.moved = True
#                 # self.get_logger().info('Moved: %s' % (self.moved))
#                 self.start_time = self.get_clock().now()

#         ranges = np.array(laser_scan.ranges, dtype="float32")

#         angles = np.linspace(
#             laser_scan.angle_min, laser_scan.angle_max, num=ranges.shape[0]
#         )

#         # Convert the ranges to Cartesian coordinates.
#         # Consider the robot to be facing in the positive x direction.
#         x = ranges * np.cos(angles)
#         y = ranges * np.sin(angles)

#         # Filter out values that are out of range
#         # and values on the wrong side
#         valid_points = self.SIDE * y > 0
#         valid_points = np.logical_and(valid_points, x < 1.5)
#         valid_points = np.logical_and(valid_points, x > 0.0)

#         # Compute the average distance
#         dists = np.abs(y[valid_points])
#         dist = np.sum(dists) / dists.shape[0]
#         # self.get_logger().info('Avg dist: %f' % (dist))

#         pos = [t.transform.translation.x, t.transform.translation.y]

#         time = self.get_clock().now() - self.start_time
#         time_d = time.nanoseconds * 1e-9
#         self.positions.append([time_d] + pos + [dist])
#         self.dist_to_end = np.linalg.norm(np.array(pos) - np.array(self.END_POSE))
#         # self.get_logger().info(
#         #             f'Time: {time_d}, Max time: {self.max_time_per_test}')

#         if time_d > self.max_time_per_test:
#             self.get_logger().error(
#                 "\n\n\n\n\nERROR: Test timed out! Your car was not able to reach the target end position.\n\n\n\n\n"
#             )
#             # Send a message of zero
#             stop = AckermannDriveStamped()
#             stop.drive.speed = 0.0
#             stop.drive.steering_angle = 0.0
#             self.drive_pub.publish(stop)
#             self.saves[self.TEST_NAME] = encode(np.array(self.positions))
#             np.savez_compressed(self.TEST_NAME + "_log", **self.saves)
#             raise SystemExit
#         if self.dist_to_end < self.end_threshold:
#             self.get_logger().info(
#                 "\n\n\n\n\nReached end of the test w/ Avg dist from wall = %f!\n\n\n\n\n"
#                 % (dist)
#             )
#             stop = AckermannDriveStamped()
#             stop.drive.speed = 0.0
#             stop.drive.steering_angle = 0.0
#             self.drive_pub.publish(stop)
#             self.saves[self.TEST_NAME] = encode(np.array(self.positions))
#             np.savez_compressed(self.TEST_NAME + "_log", **self.saves)
#             raise SystemExit


# def main():
#     rclpy.init()
#     wall_follower_test = WallTest()
#     try:
#         rclpy.spin(wall_follower_test)
#     except SystemExit:
#         rclpy.logging.get_logger("Quitting").info("Done")
#     wall_follower_test.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
