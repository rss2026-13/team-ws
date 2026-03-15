#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file
from rclpy.node import Node
from sensor_msgs.msg import Image

# import your color segmentation algorithm; call this function in ros_image_callback!
from visual_servoing.computer_vision.color_segmentation import cd_color_segmentation
from vs_msgs.msg import ConeLocationPixel


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """

    def __init__(self):
        super().__init__("cone_detector")
        self.declare_parameter("debug", False)
        self.declare_parameter("topics.image", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("topics.cone_pos", "/relative_cone_px")
        self.declare_parameter("topics.cone_debug", "/cone_debug_img")
        self.declare_parameter("color_segmentation.delta", 20)
        self.declare_parameter("color_segmentation.bounds", [5, 210, 110, 30, 255, 255])
        self.declare_parameter("line_follower.active", False)
        self.declare_parameter("line_follower.roi.xmin", 0)
        self.declare_parameter("line_follower.roi.xmax", 600)
        self.declare_parameter("line_follower.roi.ymin", 100)
        self.declare_parameter("line_follower.roi.ymax", 200)

        self.debug = self.get_parameter("debug").value
        self.IMAGE_TOPIC = self.get_parameter("topics.image").value
        self.CONE_POS_TOPIC = self.get_parameter("topics.cone_pos").value
        self.CONE_DEBUG_TOPIC = self.get_parameter("topics.cone_debug").value
        self.delta = self.get_parameter("color_segmentation.delta").value
        self.bounds = self.get_parameter("color_segmentation.bounds").value
        self.bounds = (
            self.bounds[0:3],
            self.bounds[3:6],
        )
        self.line_follower_active = self.get_parameter("line_follower.active").value
        self.roi_xmin = self.get_parameter("line_follower.roi.xmin").value
        self.roi_xmax = self.get_parameter("line_follower.roi.xmax").value
        self.roi_ymin = self.get_parameter("line_follower.roi.ymin").value
        self.roi_ymax = self.get_parameter("line_follower.roi.ymax").value
        self.cone_pub = self.create_publisher(
            ConeLocationPixel, self.CONE_POS_TOPIC, 10
        )
        self.debug_pub = self.create_publisher(Image, self.CONE_DEBUG_TOPIC, 10)
        self.image_sub = self.create_subscription(
            Image, self.IMAGE_TOPIC, self.image_callback, 5
        )
        self.bridge = CvBridge()  # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

        self.prev_pos = None
        self.delta = 20

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        if self.line_follower_active:
            image = image[self.roi_ymin : self.roi_ymax, self.roi_xmin : self.roi_xmax]

        bbox = cd_color_segmentation(
            image,
            None,
            debug=False,
            prev_pos=self.prev_pos,
            delta=self.delta,
            bounds=self.bounds,
        )
        if bbox is not None:
            (x1, y1), (x2, y2) = bbox
            cone_location = ConeLocationPixel()
            cone_location.u = float((x1 + x2) / 2)
            cone_location.v = float(y2)
            self.prev_pos = (cone_location.u, cone_location.v)
            self.cone_pub.publish(cone_location)
        else:
            self.get_logger().warn("No cone detected in current frame.")
            self.prev_pos = None
        if self.debug:
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
