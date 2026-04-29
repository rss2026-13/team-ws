#!/usr/bin/env python3
"""
traffic_light_node.py

ROS2 node that:
  - Subscribes to the camera feed
  - Detects red traffic lights via color segmentation
  - Knows the traffic light's world position (provided at launch, default 1m ahead)
  - Uses localization to compute distance to the light
  - Publishes drive commands: stop if red AND within stopping range,
    otherwise pass through commands from the path planner

  NOTE: Traffic light position is currently a fixed parameter. In the future this
  will be replaced by a subscription to homography output from a YOLO annotator.

Subscriptions:
    /zed/zed_node/rgb/image_rect_color          (sensor_msgs/Image)
    /pf/pose/odom                               (nav_msgs/Odometry)  -- particle filter localization
    /vesc/low_level/input/navigation            (ackermann_msgs/AckermannDriveStamped) -- path planner

Publications:
    /vesc/low_level/input/navigation_filtered   (ackermann_msgs/AckermannDriveStamped) -- to VESC
    /traffic_light_debug_img                    (sensor_msgs/Image)
"""

import math

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from traffic_light.traffic_light_detection import is_red_light_on
from std_msgs.msg import Bool


class TrafficLightNode(Node):
    """
    Intercepts path planner drive commands and stops the car at a red light.

    Parameters (set via ROS2 params at launch, or use defaults):
        traffic_light_x  (float): world x-coordinate of the traffic light (default: car_x + 1.0)
        traffic_light_y  (float): world y-coordinate of the traffic light (default: car_y)
        stop_distance    (float): distance in metres at which to stop (default: 1.0 m)
        min_red_pixels   (int):   pixel count threshold for red detection (default: 100)

    TODO: Replace traffic_light_x/y params with a subscription to homography output
          from the YOLO annotator once that pipeline is ready.
    """

    # Sentinel value meaning "use the car's position at startup + 1m ahead"
    _DEFAULT_LIGHT_OFFSET = 1.0

    def __init__(self):
        super().__init__("traffic_light_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("traffic_light_x", float("nan"))  # nan = use default offset
        self.declare_parameter("traffic_light_y", float("nan"))
        self.declare_parameter("stop_distance", 1.0)

        self.tl_x = self.get_parameter("traffic_light_x").value
        self.tl_y = self.get_parameter("traffic_light_y").value
        self.stop_distance = self.get_parameter("stop_distance").value

        # ── State ─────────────────────────────────────────────────────────────
        self.red_light_detected = False
        self.car_x = 0.0
        self.car_y = 0.0
        self._light_position_set = False  # becomes True once we fix the default position
        self.bridge = CvBridge()
        self.traffic_light_stop = False

        # ── Subscribers ───────────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/image_rect_color",
            self.image_callback,
            5,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/pf/pose/odom",
            self.odom_callback,
            10,
        )
        # Path planner output → we intercept this and gate it through stop logic
        self.drive_input_sub = self.create_subscription(
            AckermannDriveStamped,
            "/vesc/low_level/input/navigation",
            self.drive_input_callback,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        # Publishes filtered commands so the VESC receives our gated output.
        # In your launch file, make sure the VESC is listening to this topic,
        # or remap /vesc/low_level/input/navigation_filtered → wherever your
        # VESC driver expects drive commands.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/vesc/low_level/input/navigation",
            10,
        )
        self.debug_pub = self.create_publisher(Image, "/traffic_light_debug_img", 10)

        self.traffic_light_stop_pub = self.create_publisher(
            Bool,
            "/traffic_light_stop",
            10
        )

        self.get_logger().info(
            "TrafficLightNode initialized — waiting for first odom to set light position."
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """Update car position from particle filter localization."""
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

        # On the very first odom message, fix the default light position to
        # 1 m ahead of wherever the car starts, if no explicit param was given.
        if not self._light_position_set:
            if math.isnan(self.tl_x) or math.isnan(self.tl_y):
                # TODO: replace this block with homography/YOLO subscription output
                self.tl_x = self.car_x + self._DEFAULT_LIGHT_OFFSET
                self.tl_y = self.car_y
                self.get_logger().info(
                    f"No traffic_light_x/y params given — defaulting to 1 m ahead: "
                    f"({self.tl_x:.2f}, {self.tl_y:.2f})"
                )
            else:
                self.get_logger().info(
                    f"Traffic light position set from params: ({self.tl_x:.2f}, {self.tl_y:.2f})"
                )
            self._light_position_set = True

    def image_callback(self, image_msg: Image):
        """Run red-light detection on every incoming camera frame."""
        try:
            img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        self.red_light_detected, bbox = is_red_light_on(img)

        # Annotate and publish debug image
        debug_img = img.copy()
        if self.red_light_detected and bbox is not None:
            cv2.rectangle(debug_img, bbox[0], bbox[1], (0, 0, 255), 3)
            label = "RED LIGHT DETECTED"
        else:
            label = "NO RED LIGHT"

        color = (0, 0, 255) if self.red_light_detected else (0, 255, 0)
        cv2.putText(debug_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

    def drive_input_callback(self, drive_msg: AckermannDriveStamped):
        """
        Intercept path planner commands from /vesc/low_level/input/navigation.
        Stop the car if:
          1. A red light is detected, AND
          2. The car is within `stop_distance` metres of the traffic light.
        Otherwise, forward the command unchanged.
        """
        if not self._light_position_set:
            # Haven't received odom yet — pass commands through so the car isn't
            # frozen at startup waiting for the first pose estimate.
            self.drive_pub.publish(drive_msg)
            return

        distance_to_light = self._distance_to_light()
        #should_stop = self.red_light_detected and distance_to_light <= self.stop_distance Add this back later
        should_stop=self.red_light_detected

        if should_stop:
            self.get_logger().info(
                f"Red light detected at {distance_to_light:.2f} m — STOPPING.",
                throttle_duration_sec=1.0,
            )
            stop_cmd = AckermannDriveStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.steering_angle = drive_msg.drive.steering_angle  # preserve steering
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
