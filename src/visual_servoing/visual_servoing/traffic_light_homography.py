#!/usr/bin/env python3
"""
traffic_light_homography.py

Adapts the existing HomographyTransformer for traffic lights.
Subscribes to the YOLO traffic light pixel location and publishes
the real-world (x, y) position of the light in the car frame.

Subscriptions:
    /yolo/traffic_light_location_px   (vs_msgs/ConeLocationPixel)

Publications:
    /traffic_light_relative_location  (vs_msgs/ConeLocation)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from vs_msgs.msg import ConeLocation, ConeLocationPixel


######################################################
# COPY YOUR CALIBRATED POINTS FROM homography_transformer.py
# PTS_IMAGE_PLANE units are in pixels
PTS_IMAGE_PLANE = [[299, 213],
                   [562, 323],
                   [206, 168],
                   [571, 169]]

# PTS_GROUND_PLANE units are in inches
PTS_GROUND_PLANE = [[42, 5],
                    [23, -7],
                    [82.5, 28],
                    [77, -43]]
######################################################

METERS_PER_INCH = 0.0254


class TrafficLightHomography(Node):
    def __init__(self):
        super().__init__("traffic_light_homography")

        # Build homography matrix from calibration points
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, _ = cv2.findHomography(np_pts_image, np_pts_ground)

        self.traffic_light_sub = self.create_subscription(
            ConeLocationPixel,
            "/yolo/traffic_light_location_px",
            lambda msg: self.pixel_callback(msg, self.traffic_light_pub, "Traffic Light"),
            10,
        )
        self.traffic_light_pub = self.create_publisher(
            ConeLocation,
            "/traffic_light_relative_location",
            10,
        )
        self.parking_meter_sub = self.create_subscription(
            ConeLocationPixel,
            "/yolo/parking_meter_location_px",
            lambda msg: self.pixel_callback(msg, self.parking_meter_pub, "Parking Meter"),
            10,
        )
        self.parking_meter_pub = self.create_publisher(
            ConeLocation,
            "/parking_meter_relative_location",
            10,
        )
        self.get_logger().info("TrafficLightHomography initialized.")

    def pixel_callback(self, msg: ConeLocationPixel, publisher, object_label: str):
        x, y = self.transform_uv_to_xy(msg.u, msg.v)

        out = ConeLocation()
        out.x_pos = x
        out.y_pos = y
        publisher.publish(out)

        #self.get_logger().info(
            #f"{object_label} pixel ({msg.u:.0f}, {msg.v:.0f}) → "
            #f"car frame ({x:.2f} m, {y:.2f} m)",
            #throttle_duration_sec=1.0,
        #)

    def transform_uv_to_xy(self, u, v):
        """Convert pixel (u,v) to car-frame (x,y) in metres using homography."""
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        return float(homogeneous_xy[0, 0]), float(homogeneous_xy[1, 0])


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightHomography()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
