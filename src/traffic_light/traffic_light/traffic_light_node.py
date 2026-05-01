#!/usr/bin/env python3
"""
traffic_light_node.py

ROS2 node that stops the racecar at a red traffic light.

Detection pipeline:
  1. YOLO annotator detects whether a traffic light is present in the image
     and publishes its pixel location → /yolo/traffic_light_location_px
  2. TrafficLightHomography converts pixel → car-frame (x, y) metres
     → /traffic_light_relative_location
  3. This node subscribes to both, and:
       - Uses the YOLO Bool  (/yolo/traffic_light_detected) to decide whether
         to stop (combined with color segmentation for red vs. not-red)
       - Uses the homography ConeLocation to know how far away the light is

Subscriptions:
    /zed/zed_node/rgb/image_rect_color          (sensor_msgs/Image)
    /pf/pose/odom                               (nav_msgs/Odometry)  -- particle filter localization
    /vesc/low_level/input/navigation            (ackermann_msgs/AckermannDriveStamped) -- path planner

    /yolo/traffic_light_detected                (std_msgs/Bool)
    /traffic_light_relative_location            (vs_msgs/ConeLocation)
    
Publications:
    /vesc/low_level/input/navigation   (ackermann_msgs/AckermannDriveStamped) -- to VESC
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
from vs_msgs.msg import ConeLocation


class TrafficLightNode(Node):
    """
    Intercepts path planner drive commands and stops the car at a red light.

        Stop condition:
        YOLO sees a traffic light  (yolo_light_detected = True)
        AND color segmentation confirms it is red  (red_light_detected = True)
        AND the light is within stop_distance metres  (from homography)
        
    Parameters: stop_distance (float): metres at which to stop
    """


    def __init__(self):
        super().__init__("traffic_light_node")

        # ── Parameters ────────────────────────────────────────────────────────

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value  # set in launch file; different for simulator vs racecar

        self.stop_distance = self.get_parameter("stop_distance").value

        # ── State ─────────────────────────────────────────────────────────────
        self.red_light_detected = False       # from color segmentation
        self.yolo_light_detected = False      # from YOLO (traffic light class present)
        self.light_x = None                   # car-frame x from homography (metres ahead)
        self.light_y = None                   # car-frame y from homography
        self._last_steering_angle = 0.0
      
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
        self.drive_input_sub = self.create_subscription(
            AckermannDriveStamped,
            "/vesc/low_level/input/navigation",
            self.drive_input_callback,
            10,
        )
              # YOLO: tells us whether a traffic light object is visible at all
        self.yolo_sub = self.create_subscription(
            Bool,
            "/yolo/traffic_light_detected",
            self.yolo_callback,
            10,
        )
        # Homography: gives us the car-frame position of the light
        self.homography_sub = self.create_subscription(
            ConeLocation,
            "/traffic_light_relative_location",
            self.homography_callback,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            DRIVE_TOPIC,
            10,
        )
        self.debug_pub = self.create_publisher(Image, "/traffic_light_debug_img", 10)

        self.stop_timer=self.create_timer(0.05, self.stop_timer_callback)
      
        # This publisher will be required for integration in the state machine, publish "should_stop"
        self.traffic_light_stop_pub = self.create_publisher(
            Bool,
            "/traffic_light_stop",
            10
        )

        self.get_logger().info(
            f"TrafficLightNode initialized. stop_distance={self.stop_distance} m"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
         """Track car position (used for future world-frame logic if needed)."""
        # Currently unused since homography gives us car-frame distance directly.
        pass

    def yolo_callback(self, msg: Bool):
        """Update whether YOLO sees a traffic light in the current frame."""
        self.yolo_light_detected = msg.data

        # If YOLO no longer sees a light, clear the cached position so we
        # don't stop based on a stale homography reading.
        if not self.yolo_light_detected:
            self.light_x = None
            self.light_y = None

    def homography_callback(self, msg: ConeLocation):
        """
        Receive the car-frame position of the traffic light from homography.
        x_pos is distance ahead of the car (positive = in front).
        y_pos is lateral offset (positive = left).
        """
        self.light_x = msg.x_pos
        self.light_y = msg.y_pos

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
            # Build a status label showing all three conditions
        dist_str = f"{self._distance_to_light():.1f}m" if self._distance_to_light() is not None else "?m"
        label = (
            f"YOLO={'Y' if self.yolo_light_detected else 'N'} | "
            f"RED={'Y' if self.red_light_detected else 'N'} | "
            f"DIST={dist_str}"
        )
        color = (0, 0, 255) if self._should_stop() else (0, 255, 0)
        cv2.putText(debug_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        if self._should_stop():
            cv2.putText(debug_img, "STOPPED", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
          
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

    def drive_input_callback(self, drive_msg: AckermannDriveStamped):
        """
        Intercept path planner commands from /vesc/low_level/input/navigation.
        Stop the car if:
          1. A red light is detected, AND
          2. The car is within `stop_distance` metres of the traffic light.
        Otherwise, forward the command unchanged.
        """
        self._last_steering_angle = drive_msg.drive.steering_angle

        if self.should_stop:
            self.get_logger().info(
                f"Stopping — red={self.red_light_detected}, "
                f"yolo={self.yolo_light_detected}, "
                f"dist={self._distance_to_light()}",
                throttle_duration_sec=1.0,
            )
        else:
          self.drive_pub.publish(drive_msg)

    def stop_timer_callback(self):
        """Actively publish speed=0 at 20 Hz while stopped."""
        if self._should_stop():
            stop_cmd = AckermannDriveStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.steering_angle = self._last_steering_angle
            self.drive_pub.publish(stop_cmd)

    def _should_stop(self) -> bool:
        """
        Stop if ALL of:
          1. YOLO sees a traffic light in the frame
          2. Color segmentation confirms it is red
          3. The light is within stop_distance metres

        If homography hasn't given us a position yet but both detectors agree
        it's red, we stop anyway to be safe (distance unknown → assume close).
        """
        if not (self.yolo_light_detected and self.red_light_detected):
            return False

        dist = self._distance_to_light()
        if dist is None:
            # No homography reading yet — both detectors say red, stop to be safe
            return True

        return dist <= self.stop_distance

    def _distance_to_light(self):
        """
        Distance to the traffic light in the car frame (metres).
        Returns None if no homography reading has been received.
        x_pos from homography is forward distance, y_pos is lateral.
        """
        if self.light_x is None or self.light_y is None:
            return None
        return math.sqrt(self.light_x ** 2 + self.light_y ** 2)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
