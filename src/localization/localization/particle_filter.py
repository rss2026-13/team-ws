import numpy as np
import rclpy
from geometry_msgs.msg import (
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from localization.motion_model import MotionModel
from localization.sensor_model import SensorModel

assert rclpy


class ParticleFilter(Node):
    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter("particle_filter_frame", "default")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("num_particles", 100)
        self.declare_parameter("debug", False)
        self.particle_filter_frame = (
            self.get_parameter("particle_filter_frame")
            .get_parameter_value()
            .string_value
        )
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.num_particles = (
            self.get_parameter("num_particles").get_parameter_value().integer_value
        )
        self.laser_sub = self.create_subscription(
            LaserScan, scan_topic, self.laser_callback, 1
        )

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 1
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 1
        )

        self.click_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.pose_callback_global, 1
        )

        self.global_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.pose_callback, 1
        )
        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)
        self.particle_pub = self.create_publisher(PoseArray, "/pf/particles", 1)
        if self.debug:
            self.scan_pub = self.create_publisher(LaserScan, "/pf/scan_sim", 1)
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self, self.num_particles)
        self.clock = rclpy.clock.Clock()
        self.last_odom_time = self.clock.now()
        self.last_laser_time = self.clock.now()
        self.particles = np.zeros((self.num_particles, 3), dtype=np.float32)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.failure_time = 0
        self.softening_factor_min = 2.5
        self.softening_factor_max = 40.0
        self.softening_factor = self.softening_factor_min
        self.softening_factor_steps = 50
        self.softening_factor_current_step = 0

        self.get_logger().info("=============+READY+=============")

    def pose_callback(self, msg):
        pose = None
        if isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
        if isinstance(msg, PoseStamped):
            pose = msg.pose
        if pose is None:
            self.get_logger().error(
                "Received unsupported message type in pose_callback"
            )
            return
        x = pose.position.x
        z = pose.orientation.z
        y = pose.position.y
        w = pose.orientation.w
        theta = np.arctan2(2 * (w * z), 1 - 2 * (z**2))
        self.initialize_particles(x, y, theta, True)
        self.publish_pose()
        self.softening_factor = self.softening_factor_min
        self.softening_factor_current_step = 0
        self.get_logger().info(
            "Initialized particles at pose: %f, %f, %f" % (x, y, theta)
        )

    def pose_callback_global(self, msg):
        self.initialize_particles_global()
        self.publish_pose()
        self.get_logger().info("Initialized particles globally based on map")

    def initialize_particles_global(self):
        if not self.sensor_model.map_set:
            pass
        map = 1 - self.sensor_model.map
        indices = np.argwhere(map > 0.5)[:, 0]
        chosen_indices = indices[np.random.choice(len(indices), self.num_particles)]
        x = (
            chosen_indices
            % self.sensor_model.map_info.width
            * self.sensor_model.resolution
        )
        y = (
            chosen_indices
            // self.sensor_model.map_info.width
            * self.sensor_model.resolution
        )
        rot_matrix = np.array(
            [
                [
                    np.cos(self.sensor_model.origin[2]),
                    -np.sin(self.sensor_model.origin[2]),
                ],
                [
                    np.sin(self.sensor_model.origin[2]),
                    np.cos(self.sensor_model.origin[2]),
                ],
            ]
        )
        xy = np.stack((x, y), axis=1) @ rot_matrix.T
        x, y = (
            xy[:, 0] + self.sensor_model.origin[0],
            xy[:, 1] + self.sensor_model.origin[1],
        )
        theta = np.random.uniform(-np.pi, np.pi, size=self.num_particles)
        self.particles = np.stack((x, y, theta), axis=1)
        self.softening_factor_current_step = self.softening_factor_steps
        self.softening_factor = self.softening_factor_max

    def initialize_particles(self, x, y, theta, noisy=True):
        self.particles = np.array(
            [
                [
                    x + np.random.normal(0, 2) if noisy else x,
                    y + np.random.normal(0, 2) if noisy else y,
                    theta + np.random.uniform(-np.pi, np.pi) if noisy else theta,
                ]
                for _ in range(self.num_particles)
            ],
            dtype=np.float32,
        )
        self.get_logger().info("Sample particle: %s" % (self.particles[0],))

    def publish_pose(self):
        if not self.sensor_model.map_set:
            return
        if self.particles.shape[0] == 0:
            self.get_logger().warn("No particles to publish pose from!")
            return
        avg_x = np.mean(self.particles[:, 0])
        if not isinstance(avg_x, float):
            self.get_logger().error(
                f"avg_x is not a float! This is unexpected. avg_x: {avg_x}, type: {type(avg_x)}"
            )
        avg_y = np.mean(self.particles[:, 1])
        avg_theta = np.arctan2(
            np.mean(np.sin(self.particles[:, 2])), np.mean(np.cos(self.particles[:, 2]))
        )
        self.logger.info(f"Publishing pose: {avg_x}, {avg_y}, {avg_theta}")
        t = TransformStamped()
        t.header.stamp = self.clock.now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.particle_filter_frame
        t.transform.translation.x = float(avg_x)
        t.transform.translation.y = float(avg_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(avg_theta / 2)
        t.transform.rotation.w = np.cos(avg_theta / 2)
        self.tf_broadcaster.sendTransform(t)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.clock.now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose.position.x = float(avg_x)
        odom_msg.pose.pose.position.y = float(avg_y)
        odom_msg.pose.pose.orientation.z = np.sin(avg_theta / 2)
        odom_msg.pose.pose.orientation.w = np.cos(avg_theta / 2)
        self.odom_pub.publish(odom_msg)

        # if self.debug and self.sensor_model.map_set:
        #     scan = self.sensor_model.scan_sim.scan(
        #         np.array([[avg_x, avg_y, avg_theta]])
        #     )
        #     msg = LaserScan()
        #     msg.header.stamp = self.clock.now().to_msg()
        #     msg.header.frame_id = self.particle_filter_frame
        #     msg.angle_min = -self.sensor_model.scan_field_of_view / 2
        #     msg.angle_max = self.sensor_model.scan_field_of_view / 2
        #     msg.angle_increment = self.sensor_model.scan_field_of_view / (
        #         self.sensor_model.num_beams_per_particle - 1
        #     )
        #     msg.range_min = 0.0
        #     msg.range_max = 100.0
        #     msg.ranges = scan[0].tolist()
        #     msg.intensities = scan[0].tolist()
        #     self.scan_pub.publish(msg)

    def publish_particles(self):
        return
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.clock.now().to_msg()
        pose_array_msg.header.frame_id = "map"
        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle[0]
            pose.position.y = particle[1]
            theta = particle[2]
            pose.orientation.z = np.sin(theta / 2)
            pose.orientation.w = np.cos(theta / 2)
            pose_array_msg.poses.append(pose)
        self.particle_pub.publish(pose_array_msg)

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        dt = (self.clock.now() - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = self.clock.now()
        odometry = [linear.x * dt, linear.y * dt, angular.z * dt]
        self.particles = self.motion_model.evaluate(
            self.particles, odometry, desperation=0
        )
        self.publish_pose()
        self.publish_particles()

    def update_softening_factor(self):
        if self.softening_factor_current_step > 0:
            self.softening_factor_current_step -= 1
            self.softening_factor = self.softening_factor_min + (
                self.softening_factor_max - self.softening_factor_min
            ) * np.cos(
                (1 - self.softening_factor_current_step / self.softening_factor_steps)
                * np.pi
                / 2
            )
        else:
            self.softening_factor = self.softening_factor_min

    def laser_callback(self, msg):
        ranges = msg.ranges
        num_beams = self.sensor_model.num_beams_per_particle
        if not isinstance(self.sensor_model.laser_angles, np.ndarray):
            self.sensor_model.laser_angles = np.linspace(
                msg.angle_min, msg.angle_max, len(ranges)
            )
            self.sensor_model.downsampled_angles = np.array(
                [
                    self.sensor_model.laser_angles[
                        int((i + 0.5) * len(ranges) / num_beams)
                    ]
                    for i in range(num_beams)
                ],
                dtype=np.float32,
            )
        ranges = np.array(
            [
                ranges[int((i + 0.5) * len(ranges) / num_beams)]
                for i in range(num_beams)
            ],
            dtype=np.float32,
        )
        self.sensor_model.evaluate(self.particles, ranges)
        probabilities = self.sensor_model.weights ** (1 / self.softening_factor)
        self.particles = self.particles[
            np.random.choice(
                self.num_particles,
                self.num_particles,
                p=probabilities / np.sum(probabilities),
            )
        ]
        self.publish_pose()
        self.publish_particles()


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
