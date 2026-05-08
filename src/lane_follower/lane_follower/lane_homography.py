#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray

# PTS_IMAGE_PLANE units are in pixels; top-left origin, u right, v down
PTS_IMAGE_PLANE = [[430, 343],
                   [179, 268],
                   [362, 342],
                   [395, 250]]

# PTS_GROUND_PLANE units are in inches; x=forward, y=left from camera
PTS_GROUND_PLANE = [[22.25, -2.25],
                    [29.0, 9.5],
                    [22.0, 0.25],
                    [30.0, -3.0]]

METERS_PER_INCH = 0.0254


class LaneHomography(Node):
    """
    Subscribes to /lane_lines (pixel coords), transforms each endpoint to
    ground-plane metres via homography, publishes to /lane_lines_transformed.

    Message format (both topics): Float32MultiArray with 8 floats
        [lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2]
    Invalid lines are encoded as -1 sentinels and passed through unchanged.
    """

    def __init__(self):
        super().__init__('lane_homography')

        np_pts_ground = np.float32(
            np.array(PTS_GROUND_PLANE) * METERS_PER_INCH
        )[:, np.newaxis, :]
        np_pts_image = np.float32(
            np.array(PTS_IMAGE_PLANE, dtype=float)
        )[:, np.newaxis, :]

        self.h, _ = cv2.findHomography(np_pts_image, np_pts_ground)
        if self.h is None:
            self.get_logger().error(
                'Homography could not be computed — fill in PTS_IMAGE_PLANE '
                'and PTS_GROUND_PLANE in lane_homography.py')

        self.sub = self.create_subscription(
            Float32MultiArray, '/lane_lines', self.callback, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/lane_lines_transformed', 10)

        self.get_logger().info('Lane Homography node started')

    def callback(self, msg):
        if self.h is None:
            self.get_logger().warn('Homography not set — skipping. Fill in PTS_IMAGE_PLANE and PTS_GROUND_PLANE.',
                                   throttle_duration_sec=5.0)
            return
        data = msg.data
        if len(data) < 8:
            return

        out = []
        for i in range(0, 8, 4):
            u1, v1, u2, v2 = data[i], data[i+1], data[i+2], data[i+3]
            if u1 == -1.0:
                out.extend([-1.0, -1.0, -1.0, -1.0])
            else:
                x1, y1 = self._transform(u1, v1)
                x2, y2 = self._transform(u2, v2)
                out.extend([x1, y1, x2, y2])

        result = Float32MultiArray()
        result.data = out
        self.pub.publish(result)

    def _transform(self, u, v):
        pt = np.dot(self.h, np.array([[u], [v], [1.0]]))
        s = 1.0 / pt[2, 0]
        return float(pt[0, 0] * s), float(pt[1, 0] * s)


def main(args=None):
    rclpy.init(args=args)
    node = LaneHomography()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
