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

# HSV thresholds for white tape: low saturation, high value
# In OpenCV HSV: H 0-180, S 0-255, V 0-255
WHITE_S_MAX = 20   # S < ~8% of 255  — keeps blue out (blue S starts at ~18)
WHITE_V_MIN = 171  # V > ~69% of 255 — blue tops out at ~65% (~166), so ~10pt margin

# When a prior estimate exists, only consider segments within this many pixels
# of the predicted position at y_bottom
LINE_GATE_PX = 80

# Segments within this many pixels of each other (at y_bottom) form one cluster
LINE_CLUSTER_PX = 60


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

        self.declare_parameter("canny_low", 80)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("hough_threshold", 12)
        self.declare_parameter("hough_min_line_length", 20)
        self.declare_parameter("hough_max_line_gap", 80)

        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5
        )
        self.lane_pub = self.create_publisher(Float32MultiArray, "/lane_lines", 10)
        self.debug_pub = self.create_publisher(Image, "/lane_debug_img", 10)
        self.mask_pub = self.create_publisher(Image, "/lane_white_mask", 10)

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

        left_line, right_line, debug_img, mask_img = self.detect_lanes(
            img, canny_low, canny_high, hough_thresh, min_len, max_gap
        )

        data = []
        data += list(left_line) if left_line is not None else [-1.0, -1.0, -1.0, -1.0]
        data += list(right_line) if right_line is not None else [-1.0, -1.0, -1.0, -1.0]

        lane_msg = Float32MultiArray()
        lane_msg.data = [float(v) for v in data]
        self.lane_pub.publish(lane_msg)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_img, "bgr8"))

    def detect_lanes(self, img, canny_low, canny_high, hough_thresh, min_len, max_gap):
        h, w = img.shape[:2]
        y_top = int(h * 0.42)
        y_bottom = int(h * 0.8)

        bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv,
                                 (0,           0,           WHITE_V_MIN),
                                 (180, WHITE_S_MAX,         255))

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, canny_low, canny_high)
        edges = cv2.bitwise_and(edges, white_mask)

        # Trapezoid mask: keep lower road region
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0,             y_bottom),
            (w,             y_bottom),
            (int(w * 0.98), y_top),
            (int(w * 0.02), y_top),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(
            masked, 1, np.pi / 180,
            threshold=hough_thresh,
            minLineLength=min_len,
            maxLineGap=max_gap,
        )

        left_segs, right_segs = [], []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if slope < -0.25:
                    left_segs.append((x1, y1, x2, y2))
                elif slope > 0.25:
                    right_segs.append((x1, y1, x2, y2))

        left_pts  = self._select_pts(left_segs,  y_bottom, self._left_coeffs)
        right_pts = self._select_pts(right_segs, y_bottom, self._right_coeffs)

        new_left  = self._smooth_fit(left_pts,  self._left_coeffs)
        new_right = self._smooth_fit(right_pts, self._right_coeffs)

        # If lines cross, reject only the one that moved furthest from its prior
        lx = int(np.polyval(new_left,  y_bottom)) if new_left  is not None else None
        rx = int(np.polyval(new_right, y_bottom)) if new_right is not None else None
        if lx is not None and rx is not None and lx >= rx:
            prev_lx = int(np.polyval(self._left_coeffs,  y_bottom)) if self._left_coeffs  is not None else None
            prev_rx = int(np.polyval(self._right_coeffs, y_bottom)) if self._right_coeffs is not None else None
            left_drift  = abs(lx - prev_lx) if prev_lx is not None else 0
            right_drift = abs(rx - prev_rx) if prev_rx is not None else 0
            if left_drift >= right_drift:
                new_left  = self._left_coeffs
            else:
                new_right = self._right_coeffs

        self._left_coeffs  = new_left
        self._right_coeffs = new_right

        left_line  = self._coeffs_to_line(self._left_coeffs,  y_top, y_bottom)
        right_line = self._coeffs_to_line(self._right_coeffs, y_top, y_bottom)

        mask_img = cv2.bitwise_and(bgr, bgr, mask=white_mask)

        debug = bgr.copy()
        cv2.polylines(debug, [polygon], True, (0, 255, 255), 1)
        if left_line:
            cv2.line(debug, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 0, 255), 3)
        if right_line:
            cv2.line(debug, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 255, 0), 3)

        return left_line, right_line, debug, mask_img

    # ------------------------------------------------------------------

    @staticmethod
    def _seg_x_at_y(x1, y1, x2, y2, y):
        """Extrapolate a segment to find its x at a given y."""
        if y2 == y1:
            return (x1 + x2) / 2.0
        return x1 + (x2 - x1) * (y - y1) / float(y2 - y1)

    def _select_pts(self, segs, y_ref, prev_coeffs):
        """
        From a list of Hough segments, pick the best cluster and return its points.

        If we have a prior estimate, gate to segments within LINE_GATE_PX of it.
        If nothing falls in the gate (or no prior), fall back to the densest cluster.
        """
        if not segs:
            return []

        xs = [self._seg_x_at_y(*s, y_ref) for s in segs]

        if prev_coeffs is not None:
            ref_x = float(np.polyval(prev_coeffs, y_ref))
            gated = [s for s, x in zip(segs, xs) if abs(x - ref_x) < LINE_GATE_PX]
            if gated:
                return [(s[0], s[1]) for s in gated] + [(s[2], s[3]) for s in gated]

        # No prior or gate was empty — find the densest cluster by sorting x-intercepts
        # and picking the longest run within LINE_CLUSTER_PX
        order = sorted(range(len(segs)), key=lambda i: xs[i])
        best_start, best_len = 0, 1
        cur_start, cur_len = 0, 1
        for i in range(1, len(order)):
            if xs[order[i]] - xs[order[cur_start]] <= LINE_CLUSTER_PX:
                cur_len += 1
                if cur_len > best_len:
                    best_start, best_len = cur_start, cur_len
            else:
                cur_start = i
                cur_len = 1
        chosen = [segs[order[i]] for i in range(best_start, best_start + best_len)]
        return [(s[0], s[1]) for s in chosen] + [(s[2], s[3]) for s in chosen]

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
