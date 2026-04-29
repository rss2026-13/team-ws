#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import torch

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dataclasses import dataclass
from rclpy.node import Node
from typing import List
from ultralytics import YOLO
from vs_msgs.msg import ParkingMeterLocation

from vs_msgs.msg import ConeLocationPixel #added this so that we can use existing cone homog code for the traffic light

from std_msgs.msg import Bool


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    # Bounding box coordinates in the original image:
    x1: int
    y1: int
    x2: int
    y2: int


class YoloAnnotatorNode(Node):
    def __init__(self) -> None:
        super().__init__("final_detector")

        # Declare and get ROS parameters
        self.model_name = (
            self.declare_parameter("model", "yolo11n.pt")
            .get_parameter_value()
            .string_value
        )
        self.conf_threshold = (
            self.declare_parameter("conf_threshold", 0.85)
            .get_parameter_value()
            .double_value
        )
        self.iou_threshold = (
            self.declare_parameter("iou_threshold", 0.7)
            .get_parameter_value()
            .double_value
        )

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(self.model_name)
        self.model.to(self.device)

        self.class_color_map = self.get_class_color_map()
        self.allowed_cls = [
            i for i, name in self.model.names.items()
            if name in self.class_color_map
        ]

        self.get_logger().info(f"Running {self.model_name} on device {self.device}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}")
        if self.allowed_cls:
            self.get_logger().info(f"You've chosen to keep these class IDs: {self.allowed_cls}")
        else:
            self.get_logger().warn("No allowed classes matched the model's class list.")

        # Create publisher and subscribers
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.on_image, 10)
        self.pub = self.create_publisher(
            Image, "/yolo/annotated_image", 10)
        
        #need following publishers for integration

        self.meter_detected_pub = self.create_publisher(
            Bool, "yolo/parking_meter_detected", 10)
        self.light_detected_pub = self.create_publisher(
            Bool, "yolo/traffic_light_detected", 10)
        
        self.meter_loc_pub = self.create_publisher(
            ParkingMeterLocation, "/yolo/parking_meter_location", 10)

        self.light_loc_pub = self.create_publisher(
            ConeLocationPixel, "/yolo/traffic_light_location_px", 10) #add traffic light location publishing



    def get_class_color_map(self) -> dict[str, tuple[int, int, int]]:
        """
        Return a dictionary mapping a list of COCO class names you want to keep
        to the detection BGR colors in the annotated image. COCO class names include
        "chair", "couch", "tv", "laptop", "dining table", and many more. The list
        of available classes can be found in `self.model.names`.
        """
        # Set mapping to detect parking meter or traffic light
        return {
            "parking meter": (0, 255, 0),
            "traffic light": (255, 0, 0),
        }

    def on_image(self, msg: Image) -> None:
        # Convert ROS -> OpenCV (BGR)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO inference
        try:
            results = self.model(
                bgr,
                classes=self.allowed_cls,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        if not results:
            return

        # Convert results to Detection List
        dets = self.results_to_detections(results[0])

        # Draw detections on BGR image
        annotated = self.draw_detections(bgr, dets)

        # Publish annotated BGR image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)

    def results_to_detections(self, result) -> List[Detection]:
        """
        Convert an Ultralytics result into a Detection list.

        YOLOv11 outputs:
          result.boxes.xyxy: (N, 4) tensor
          result.boxes.conf: (N,) tensor
          result.boxes.cls:  (N,) tensor
        """
        detections = []
        meter_detect = 0
        light_detect = 0

        if result.boxes is None:
            return detections

        xyxy = result.boxes.xyxy
        conf = result.boxes.conf
        cls = result.boxes.cls

        # Convert Torch tensors -> CPU numpy
        xyxy_np = xyxy.detach().cpu().numpy() if hasattr(xyxy, "detach") else np.asarray(xyxy)
        conf_np = conf.detach().cpu().numpy() if hasattr(conf, "detach") else np.asarray(conf)
        cls_np = cls.detach().cpu().numpy() if hasattr(cls, "detach") else np.asarray(cls)

        # TODO: Store YOLO outputs as Detections. Iterate through xyxy_np, conf_np, and cls_np
        #       to append a Detection with all its instance variables filled in to the
        #       detections List.
        #
        # Hint: use Python's zip keyword to iterate through the three arrays in a single for loop.
        for box, confidence, class_id in zip(xyxy_np, conf_np, cls_np):
            x1, y1, x2, y2 = box
            class_id = int(class_id)
            class_name = self.model.names[class_id]

            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(confidence),
                    x1=int(x1),
                    y1=int(y1),
                    x2=int(x2),
                    y2=int(y2),
                )
            )

            if class_name == "parking meter":
                meter_location = ParkingMeterLocation()
                meter_location.u = float((x1 + x2) / 2)
                meter_location.v = float(y2)
                self.meter_loc_pub.publish(meter_location)
                meter_detect = 1
            elif class_name == "traffic light":
                light_location = ConeLocationPixel()
                light_location.u = float((x1 + x2) / 2)   # horizontal center
                light_location.v = float(y2)               # bottom of bounding box
                self.light_loc_pub.publish(light_location)
                light_detect = 1

        if meter_detect == 1:
            self.meter_detected_pub.publish(Bool(data=True))
        else:
            self.meter_detected_pub.publish(Bool(data=False))

        if light_detect == 1:
            self.light_detected_pub.publish(Bool(data=True))
        else:
            self.light_detected_pub.publish(Bool(data=False))
            
        return detections

    def draw_detections(
        self,
        bgr_image: np.ndarray,
        detections: List[Detection],
    ) -> np.ndarray:

        out_image = bgr_image.copy()

        for det in detections:
            # TODO: Get the bounding box for the detection
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            # TODO: Draw the bounding box around the detection to the output image.
            #       Use the colors you specified per class in `get_class_color_map`
            #       by accessing the self.class_color_map dictionary.
            #
            # Hint: Use cv2's `rectangle` function to draw a rectangle on the annotated image.
            color = self.class_color_map.get(det.class_name, (255, 255, 255))
            cv2.rectangle(out_image, (x1, y1), (x2, y2), color, 2)
            # TODO: Label the box with the class name and confidence.
            #
            # Hint: Use cv2's `putText` function to put text on the annotated image.
            label = f"{det.class_name} {det.confidence:.2f}"
            text_y = max(y1 - 10, 20)

            cv2.putText(
                out_image,
                label,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        return out_image


def main() -> None:
    rclpy.init()
    node = YoloAnnotatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

