#!/usr/bin/env python3
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterEvent
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

from wall_follower.drive_controller import DriveController
from wall_follower.visualization_tools import VisualizationTools
from wall_follower.wall_detection import detect_walls


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.declare_parameter("topics.scan", "/scan")
        self.declare_parameter("topics.drive", "/drive")
        self.declare_parameter("topics.wall", "/wall")
        self.declare_parameter("controller.side", 1)
        self.declare_parameter("controller.velocity", 1.0)
        self.declare_parameter("controller.desired_distance", 1.0)
        self.declare_parameter("controller.pid.kp", 0.45)
        self.declare_parameter("controller.pid.ki", 0.125)
        self.declare_parameter("controller.pid.kd", 0.07)
        self.declare_parameter("controller.pid.max_i", 3.0)
        self.declare_parameter("controller.pid.max_d", 4.0)
        self.declare_parameter("controller.pid.decay", 0.9)

        self._param_event_sub = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameters_callback,
            10,
        )

        self.SCAN_TOPIC = self.get_parameter("topics.scan").value
        self.DRIVE_TOPIC = self.get_parameter("topics.drive").value
        self.WALL_TOPIC = self.get_parameter("topics.wall").value
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
            self.marker_publisher, "laser", self.tf_buffer
        )
        self.post_init()

    def post_init(self):
        self.side = self.get_parameter("controller.side").value
        self.velocity = self.get_parameter("controller.velocity").value
        self.desired_distance = self.get_parameter("controller.desired_distance").value
        self.pid_params = {
            k: v.value
            for k, v in self.get_parameters_by_prefix("controller.pid").items()
        }
        self.drive_controller = DriveController(
            pid_params=self.pid_params,
            side=self.side,
            side_spread=45.0,
            side_samples=11,
            front_spread=15.0,
            front_samples=11,
            velocity=self.velocity,
            desired_distance=self.desired_distance,
            drive_publisher=self.drive_publisher,
            front_treshold=self.desired_distance * 3.0,
            front_error_ratio=3,
            clock=rclpy.clock.Clock(),
        )

    def parameters_callback(self, params):
        if not any(
            param.name.startswith("controller.") for param in params.changed_parameters
        ):
            return
        self.get_logger().info(
            "Controller parameter updated, rebuilding drive controller"
        )
        self.post_init()

    def scan_callback(self, laser_scan):
        walls = detect_walls(laser_scan, min_points=10, D_t=0.1)
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
