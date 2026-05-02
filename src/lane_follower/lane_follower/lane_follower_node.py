#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# Minimum number of Hough segment endpoints to consider a detection confident
MIN_PTS_CONFIDENT = 2

# EMA smoothing factor: 0 = always use previous, 1 = always use current
SMOOTH_ALPHA = 0.4

# Only keep edges where the source pixel is bright (white tape, not blue/dark)
WHITE_THRESH = 180


class LaneFollowerNode(Node):
    """
    Detects left and right lane boundaries from the ZED camera feed.

    Subscribes:
        /zed/zed_node/rgb/image_rect_color (sensor_msgs/Image, bgra8, 640x360)

    Publishes:
        /lane_lines     (std_msgs/Float32MultiArray)
                        8 floats: [lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2]
                        A missing side is encoded as [-1, -1, -1, -1].
        /lane_debug_img (sensor_msgs/Image)
                        Camera frame with detected lines drawn on it.
    """

    def __init__(self):
        super().__init__("lane_follower_node")

        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("hough_threshold", 30)
        self.declare_parameter("hough_min_line_length", 20)
        self.declare_parameter("hough_max_line_gap", 80)

        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5
        )
        self.lane_pub = self.create_publisher(Float32MultiArray, "/lane_lines", 10)
        self.debug_pub = self.create_publisher(Image, "/lane_debug_img", 10)

        self.bridge = CvBridge()

        # Smoothed line state: stored as (m, b) where x = m*y + b
        self._left_coeffs = None
        self._right_coeffs = None

        self.get_logger().info("Lane Follower Node Initialized")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgra8")

        canny_low = self.get_parameter("canny_low").get_parameter_value().integer_value
        canny_high = self.get_parameter("canny_high").get_parameter_value().integer_value
        hough_thresh = self.get_parameter("hough_threshold").get_parameter_value().integer_value
        min_len = self.get_parameter("hough_min_line_length").get_parameter_value().integer_value
        max_gap = self.get_parameter("hough_max_line_gap").get_parameter_value().integer_value

        left_line, right_line, debug_img = self.detect_lanes(
            img, canny_low, canny_high, hough_thresh, min_len, max_gap
        )

        data = []
        data += list(left_line) if left_line is not None else [-1.0, -1.0, -1.0, -1.0]
        data += list(right_line) if right_line is not None else [-1.0, -1.0, -1.0, -1.0]

        lane_msg = Float32MultiArray()
        lane_msg.data = [float(v) for v in data]
        self.lane_pub.publish(lane_msg)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

    def detect_lanes(self, img, canny_low, canny_high, hough_thresh, min_len, max_gap):
        h, w = img.shape[:2]
        y_top = int(h * 0.55)
        y_bottom = int(h * 1)

        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, canny_low, canny_high)

        # Only keep edges on bright (white) pixels — filters out blue lines and dark artifacts
        bright_mask = (gray > WHITE_THRESH).astype(np.uint8) * 255
        edges = cv2.bitwise_and(edges, bright_mask)

        # Trapezoid mask: keep lower road region
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0,             y_bottom),
            (w,             y_bottom),
            (int(w * 0.95), y_top),
            (int(w * 0.05), y_top),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(
            masked, 1, np.pi / 180,
            threshold=hough_thresh,
            minLineLength=min_len,
            maxLineGap=max_gap,
        )

        left_pts, right_pts = [], []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                # Require steeper slope to reject near-horizontal lines
                if slope < -0.4:
                    left_pts += [(x1, y1), (x2, y2)]
                elif slope > 0.4:
                    right_pts += [(x1, y1), (x2, y2)]

        self._left_coeffs = self._smooth_fit(left_pts, self._left_coeffs)
        self._right_coeffs = self._smooth_fit(right_pts, self._right_coeffs)

        left_line = self._coeffs_to_line(self._left_coeffs, y_top, y_bottom)
        right_line = self._coeffs_to_line(self._right_coeffs, y_top, y_bottom)

        debug = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        cv2.polylines(debug, [polygon], True, (0, 255, 255), 1)
        if left_line:
            cv2.line(debug, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 0, 255), 3)
        if right_line:
            cv2.line(debug, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 255, 0), 3)

        return left_line, right_line, debug

    def _smooth_fit(self, pts, prev_coeffs):
        if len(pts) < MIN_PTS_CONFIDENT:
            return None
        try:
            coeffs = np.polyfit([p[1] for p in pts], [p[0] for p in pts], 1)
        except np.linalg.LinAlgError:
            return None
        if prev_coeffs is not None:
            coeffs = SMOOTH_ALPHA * coeffs + (1 - SMOOTH_ALPHA) * prev_coeffs
        return coeffs

    def _coeffs_to_line(self, coeffs, y_top, y_bottom):
        if coeffs is None:
            return None
        x_bottom = int(np.polyval(coeffs, y_bottom))
        x_top = int(np.polyval(coeffs, y_top))
        return (x_bottom, y_bottom, x_top, y_top)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
