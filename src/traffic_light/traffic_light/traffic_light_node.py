#!/usr/bin/env python3
"""
traffic_light_node.py

ROS2 node that:
  - Subscribes to the camera feed
  - Detects red traffic lights via color segmentation
  - Knows the traffic light's world position (provided at launch)
  - Uses localization to compute distance to the light
  - Publishes drive commands: stop if red AND within stopping range,
    otherwise pass through commands from the planner/teleop

Subscriptions:
    /zed/zed_node/rgb/image_rect_color  (sensor_msgs/Image)
    /odom or /pf/pose/odom              (nav_msgs/Odometry)  -- your localization output
    /drive_input                        (ackermann_msgs/AckermannDriveStamped) -- upstream planner

Publications:
    /drive                              (ackermann_msgs/AckermannDriveStamped)
    /traffic_light_debug_img            (sensor_msgs/Image)
"""

import math

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from traffic_light_detection import is_red_light_on


class TrafficLightNode(Node):
    """
    Controls the racecar in response to a traffic light.

    Parameters (set via ROS2 params or defaults below):
        traffic_light_x  (float): world x-coordinate of the traffic light
        traffic_light_y  (float): world y-coordinate of the traffic light
        stop_distance    (float): distance in metres at which to start stopping (default 1.0 m)
        min_red_pixels   (int):   pixel threshold for red detection (default 100)
    """

    def __init__(self):
        super().__init__("traffic_light_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("traffic_light_x", 0.0)
        self.declare_parameter("traffic_light_y", 0.0)
        self.declare_parameter("stop_distance", 1.0)   # metres

        self.tl_x = self.get_parameter("traffic_light_x").value
        self.tl_y = self.get_parameter("traffic_light_y").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.min_red_pixels = self.get_parameter("min_red_pixels").value

        # ── State ─────────────────────────────────────────────────────────────
        self.red_light_detected = False
        self.car_x = 0.0
        self.car_y = 0.0
        self.bridge = CvBridge()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/image_rect_color",
            self.image_callback,
            5,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )
        self.drive_input_sub = self.create_subscription(
            AckermannDriveStamped,
            "/drive_input",
            self.drive_input_callback,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.debug_pub = self.create_publisher(Image, "/traffic_light_debug_img", 10)

        self.get_logger().info(
            f"TrafficLightNode initialized. "
            f"Light at ({self.tl_x}, {self.tl_y}), "
            f"stop_distance={self.stop_distance} m"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """Update car position from localization."""
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

    def image_callback(self, image_msg: Image):
        """Run red-light detection on every incoming camera frame."""
        try:
            img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        self.red_light_detected, bbox = is_red_light_on(
            img
        )

       # Annotate and publish debug image
        debug_img = img.copy()
        if self.red_light_detected and bbox is not None: # Draw the box around the detection
            cv2.rectangle(debug_img, bbox[0], bbox[1], (0, 0, 255), 3)
            label = "RED LIGHT DETECTED"
        else:
            label = "NO RED LIGHT"

        color = (0, 0, 255) if self.red_light_detected else (0, 255, 0)
        cv2.putText(debug_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))


    def drive_input_callback(self, drive_msg: AckermannDriveStamped):
        """
        Intercept upstream drive commands.
        Stop the car if:
          1. A red light is detected, AND
          2. The car is within `stop_distance` metres of the traffic light.
        Otherwise, forward the command unchanged.
        """
        distance_to_light = self._distance_to_light()
        should_stop = self.red_light_detected and distance_to_light <= self.stop_distance

        if should_stop:
            self.get_logger().info(
                f"Red light detected at {distance_to_light:.2f} m — STOPPING.",
                throttle_duration_sec=1.0,
            )
            stop_cmd = AckermannDriveStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.steering_angle = drive_msg.drive.steering_angle  # keep steering
            self.drive_pub.publish(stop_cmd)
        else:
            self.drive_pub.publish(drive_msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _distance_to_light(self) -> float:
        """Euclidean distance from current car position to the traffic light."""
        dx = self.car_x - self.tl_x
        dy = self.car_y - self.tl_y
        return math.sqrt(dx * dx + dy * dy)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
