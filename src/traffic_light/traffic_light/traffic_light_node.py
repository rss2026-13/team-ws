#!/usr/bin/env python3
"""
traffic_light_node.py

ROS2 node that stops the racecar at a red traffic light.

Detection pipeline:
  1. YOLO annotator detects whether a traffic light is present in the image,
     publishes its pixel location → /yolo/traffic_light_location_px
     and its bounding box       → /yolo/traffic_light_bbox
  2. TrafficLightHomography converts pixel → car-frame (x, y) metres
     → /traffic_light_relative_location
  3. This node:
       - Crops the camera image to the YOLO bounding box
       - Runs color segmentation ONLY inside that crop to confirm red
       - Slows down when approaching the light
       - Stops if YOLO sees a light AND crop is red AND within stop_distance

Subscriptions:
    /zed/zed_node/rgb/image_rect_color          (sensor_msgs/Image)
    /pf/pose/odom                               (nav_msgs/Odometry)
    /vesc/low_level/input/navigation            (ackermann_msgs/AckermannDriveStamped)
    /yolo/traffic_light_detected                (std_msgs/Bool)
    /yolo/traffic_light_bbox                    (sensor_msgs/RegionOfInterest)
    /traffic_light_relative_location            (vs_msgs/ConeLocation)

Publications:
    DRIVE_TOPIC                                 (ackermann_msgs/AckermannDriveStamped)
    /traffic_light_debug_img                    (sensor_msgs/Image)
    /traffic_light_stop                         (std_msgs/Bool)
"""

import math

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, RegionOfInterest
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from vs_msgs.msg import ConeLocation

from traffic_light.traffic_light_detection import is_red_light_on


class TrafficLightNode(Node):
    """
    Stops the racecar when a red traffic light is detected.

    Slowdown behavior:
        - Beyond slowdown_distance: full planner speed
        - Between slowdown_distance and stop_distance: speed scaled linearly down
          to min_approach_speed
        - Within stop_distance (and red): full stop

    Stop condition (all of these must be true):
        1. YOLO sees a traffic light in the frame
        2. Color segmentation inside the YOLO bounding box confirms it is red
        3. The light is within stop_distance metres (from homography)
           OR homography hasn't reported yet (fail-safe: assume close)
    """

    def __init__(self):
        super().__init__("traffic_light_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation_filtered")
        self.declare_parameter("stop_distance", 1.5)      # metres — full stop inside this
        self.declare_parameter("slowdown_distance", 1.6)  # metres — start slowing here
        self.declare_parameter("min_approach_speed", 0.1) # m/s — slowest before full stop

        DRIVE_TOPIC = self.get_parameter("drive_topic").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.slowdown_distance = self.get_parameter("slowdown_distance").value
        self.min_approach_speed = self.get_parameter("min_approach_speed").value

        # ── State ─────────────────────────────────────────────────────────────
        self.red_light_detected = False
        self.yolo_light_detected = False
        self.light_x = None
        self.light_y = None
        self.yolo_bbox = None
        self._last_steering_angle = 0.0
        self._planner_speed = 0.0   # cached from trajectory follower for slowdown scaling
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
        self.yolo_sub = self.create_subscription(
            Bool,
            "/yolo/traffic_light_detected",
            self.yolo_callback,
            10,
        )
        self.bbox_sub = self.create_subscription(
            RegionOfInterest,
            "/yolo/traffic_light_bbox",
            self.bbox_callback,
            10,
        )
        self.homography_sub = self.create_subscription(
            ConeLocation,
            "/traffic_light_relative_location",
            self.homography_callback,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, "/traffic_light_debug_img", 10)
        self.traffic_light_stop_pub = self.create_publisher(Bool, "/traffic_light_stop", 10)

        # Active stop timer at 20 Hz — keeps publishing speed=0 while stopped
        # so the car stays stopped even if the planner goes quiet
        self.stop_timer = self.create_timer(0.05, self.stop_timer_callback)

        self.get_logger().info(
            f"TrafficLightNode initialized. "
            f"drive_topic={DRIVE_TOPIC}, "
            f"stop_distance={self.stop_distance} m, "
            f"slowdown_distance={self.slowdown_distance} m"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """Currently unused — homography gives car-frame distance directly."""
        pass

    def yolo_callback(self, msg: Bool):
        """Update whether YOLO sees a traffic light in the current frame."""
        self.yolo_light_detected = msg.data

        # Clear stale state when light leaves the frame
        if not self.yolo_light_detected:
            self.light_x = None
            self.light_y = None
            self.yolo_bbox = None
            self.red_light_detected = False

    def bbox_callback(self, msg: RegionOfInterest):
        """Cache the latest YOLO bounding box for use in image_callback."""
        self.yolo_bbox = (
            msg.x_offset,
            msg.y_offset,
            msg.x_offset + msg.width,
            msg.y_offset + msg.height,
        )  # stored as (x1, y1, x2, y2)

    def homography_callback(self, msg: ConeLocation):
        """Receive car-frame position of the traffic light from homography."""
        self.light_x = msg.x_pos
        self.light_y = msg.y_pos

    def image_callback(self, image_msg: Image):
        """
        Crop image to YOLO bounding box and run color segmentation on the crop.
        """
        try:
            img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        debug_img = img.copy()

        if self.yolo_light_detected and self.yolo_bbox is not None:
            x1, y1, x2, y2 = self.yolo_bbox

            # Clamp bbox to image bounds
            h, w = img.shape[:2]
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(0, min(x2, w))
            y2 = max(0, min(y2, h))

            # Draw YOLO bounding box in blue
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

            crop = img[y1:y2, x1:x2]
            if crop.size > 0:
                self.red_light_detected, bbox_in_crop = is_red_light_on(crop)

                # Draw color segmentation box translated to full image coords
                if self.red_light_detected and bbox_in_crop is not None:
                    (cx1, cy1), (cx2, cy2) = bbox_in_crop
                    cv2.rectangle(
                        debug_img,
                        (x1 + cx1, y1 + cy1),
                        (x1 + cx2, y1 + cy2),
                        (0, 0, 255),
                        3,
                    )
            else:
                self.red_light_detected = False
        else:
            self.red_light_detected = False

        # Status label
        dist = self._distance_to_light()
        dist_str = f"{dist:.1f}m" if dist is not None else "?m"
        label = (
            f"YOLO={'Y' if self.yolo_light_detected else 'N'} | "
            f"RED={'Y' if self.red_light_detected else 'N'} | "
            f"DIST={dist_str}"
        )
        color = (0, 0, 255) if self._should_stop() else (0, 255, 0)
        cv2.putText(debug_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        if self._should_stop():
            cv2.putText(
                debug_img, "STOPPED", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3
            )
        elif self._is_slowing_down():
            cv2.putText(
                debug_img, f"SLOWING ({dist_str})", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2  # orange
            )

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

    def drive_input_callback(self, drive_msg: AckermannDriveStamped):
        """
        Read the trajectory follower's commands so the slowdown ramp can
        scale them. We do NOT forward commands here — the mux handles arbitration:
          - nav_0 (this node, highest priority): only publishes when stopping/slowing
          - nav_2 (trajectory follower, lowest priority): publishes always
        When this node is silent on nav_0, the mux falls through to the
        trajectory follower on nav_2 automatically.
        """
        # Cache speed and steering so the stop/slowdown timer can reference them
        self._last_steering_angle = drive_msg.drive.steering_angle
        self._planner_speed = drive_msg.drive.speed

    def stop_timer_callback(self):
        """
        Runs at 20 Hz. Publishes on nav_0 (highest mux priority) only when
        actively stopping or slowing. Silence = mux falls through to trajectory
        follower on nav_2.
        """
        should = self._should_stop()
        self.traffic_light_stop_pub.publish(Bool(data=should))

        if should:
            # Full stop
            stop_cmd = AckermannDriveStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.steering_angle = self._last_steering_angle
            self.drive_pub.publish(stop_cmd)
            self.get_logger().info(
                f"Stopping — red={self.red_light_detected}, "
                f"yolo={self.yolo_light_detected}, "
                f"dist={self._distance_to_light()}",
                throttle_duration_sec=1.0,
            )
        elif self._is_slowing_down():
            # Publish a scaled-down speed on nav_0 to override the trajectory
            # follower's full speed on nav_2
            scaled_speed = self._scaled_approach_speed(self._planner_speed)
            slow_cmd = AckermannDriveStamped()
            slow_cmd.header.stamp = self.get_clock().now().to_msg()
            slow_cmd.drive.speed = scaled_speed
            slow_cmd.drive.steering_angle = self._last_steering_angle
            self.drive_pub.publish(slow_cmd)
            self.get_logger().info(
                f"Slowing — dist={self._distance_to_light():.2f}m, "
                f"speed={scaled_speed:.2f}m/s",
                throttle_duration_sec=1.0,
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _should_stop(self) -> bool:
        """Full stop: red detected AND within stop_distance (or distance unknown)."""
        if not (self.yolo_light_detected and self.red_light_detected):
            return False
        dist = self._distance_to_light()
        if dist is None:
            return True  # no homography yet — stop to be safe
        return dist <= self.stop_distance

    def _is_slowing_down(self) -> bool:
        """
        True when red is detected and car is between slowdown_distance
        and stop_distance — i.e. in the approach zone.
        """
        if not (self.yolo_light_detected and self.red_light_detected):
            return False
        dist = self._distance_to_light()
        if dist is None:
            return False
        return self.stop_distance < dist <= self.slowdown_distance

    def _scaled_approach_speed(self, planner_speed: float) -> float:
        """
        Linearly scale speed between min_approach_speed (at stop_distance)
        and the planner's commanded speed (at slowdown_distance).

                planner_speed ────────────┐
                                           \
              min_approach_speed ───────────┤────── 0
                                 ^          ^
                          slowdown_dist  stop_dist
        """
        dist = self._distance_to_light()
        if dist is None:
            return self.min_approach_speed

        # How far through the slowdown zone are we? 0.0 = just entered, 1.0 = at stop line
        t = 1.0 - (dist - self.stop_distance) / (self.slowdown_distance - self.stop_distance)
        t = max(0.0, min(1.0, t))  # clamp to [0, 1]

        # Interpolate between planner_speed and min_approach_speed
        scaled = planner_speed * (1.0 - t) + self.min_approach_speed * t
        return max(scaled, self.min_approach_speed)

    def _distance_to_light(self):
        """Car-frame distance to the light in metres. None if homography not yet received."""
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
