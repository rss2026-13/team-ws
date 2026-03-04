#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterEvent
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

from wall_follower.drive import DriveController
from wall_follower.visualization_tools import VisualizationTools
from wall_follower.wall_detection import detect_walls


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("wall_topic", "/wall")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)
        self.declare_parameter("pid_controller_kp", 0.45)
        self.declare_parameter("pid_controller_ki", 0.125)
        self.declare_parameter("pid_controller_kd", 0.07)
        self.declare_parameter("pid_controller_maxi", 3.0)
        self.declare_parameter("pid_controller_maxd", 4.0)
        # self.add_on_set_parameters_callback(self.parameters_callback)
        self._param_event_sub = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameters_callback,
            10,
        )

        self.SCAN_TOPIC = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.DRIVE_TOPIC = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )
        self.WALL_TOPIC = (
            self.get_parameter("wall_topic").get_parameter_value().string_value
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
        )
        self.marker_publisher = self.create_publisher(Marker, self.WALL_TOPIC, 10)
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.DRIVE_TOPIC, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.visualization_tools = VisualizationTools(
            self.marker_publisher, "laser_model", self.tf_buffer
        )
        self.distance_publisher = self.create_publisher(Float32, "/distance", 10)
        self.angle_publisher = self.create_publisher(Float32, "/angle", 10)
        # Leave parameters that can be changed at runtime to post_init
        self.post_init()

    def post_init(self):
        self.SIDE = self.get_parameter("side").get_parameter_value().integer_value
        self.VELOCITY = (
            self.get_parameter("velocity").get_parameter_value().double_value
        )
        self.DESIRED_DISTANCE = (
            self.get_parameter("desired_distance").get_parameter_value().double_value
        )
        self.PID_CONTROLLER_KP = (
            self.get_parameter("pid_controller_kp").get_parameter_value().double_value
        )
        self.PID_CONTROLLER_KI = (
            self.get_parameter("pid_controller_ki").get_parameter_value().double_value
        )
        self.PID_CONTROLLER_KD = (
            self.get_parameter("pid_controller_kd").get_parameter_value().double_value
        )
        self.PID_CONTROLLER_MAXI = (
            self.get_parameter("pid_controller_maxi").get_parameter_value().double_value
        )
        self.PID_CONTROLLER_MAXD = (
            self.get_parameter("pid_controller_maxd").get_parameter_value().double_value
        )
        self.drive_controller = DriveController(
            kp=self.PID_CONTROLLER_KP,
            ki=self.PID_CONTROLLER_KI,
            kd=self.PID_CONTROLLER_KD,
            max_i=self.PID_CONTROLLER_MAXI,
            max_d=self.PID_CONTROLLER_MAXD,
            side=self.SIDE,
            side_spread=np.pi / 4,
            side_samples=11,
            front_spread=0.12,
            front_samples=5,
            velocity=self.VELOCITY,
            desired_distance=self.DESIRED_DISTANCE,
            drive_publisher=self.drive_publisher,
            front_treshold=self.DESIRED_DISTANCE * 3.0,
            front_error_ratio=2 + self.VELOCITY * 0.5,
            distance_publisher=self.distance_publisher,
            angle_publisher=self.angle_publisher,
        )

    def parameters_callback(self, params):
        if not any(
            param.name
            in [
                "side",
                "velocity",
                "desired_distance",
                "pid_controller_kp",
                "pid_controller_ki",
                "pid_controller_kd",
                "pid_controller_maxi",
                "pid_controller_maxd",
            ]
            for param in params.changed_parameters
        ):
            return
        self.get_logger().info("Parameters updated, updating controller parameters")
        self.post_init()

    def scan_callback(self, laser_scan):
        walls = detect_walls(laser_scan, self.SIDE, min_points=3, D_t=0.5)
        self.visualization_tools.plot_walls(walls, laser_scan.header.stamp)
        self.drive_controller.update(walls)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
