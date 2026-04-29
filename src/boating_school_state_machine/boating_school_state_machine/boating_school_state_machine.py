#!/usr/bin/env python3
"""
"""
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Bool


class BoatingSchoolNode(Node):
    """
    """

    def __init__(self):
        super().__init__("boating_school_state_machine")

        # ── Parameters ────────────────────────────────────────────────────────
        self.robot_state = "MCL_INITIALIZATION"
        self.previous_robot_state = None
        self.initial_pose = None
        self.parking_spots = []

        self.start_time = self.get_clock().now()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            Odometry,
            "/pf/pose/odom",
            self.pose_cb,
            10
        )

        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_cb,
            10
        )

        self.traffic_light_stop_sub = self.create_subscription(
            Bool,
            "/traffic_light_stop",
            self.traffic_light_stop_cb,
            10
        )

        self.parking_meter_detection_sub = self.create_subscription(
            Bool,
            "yolo/parking_meter_detected",
            self.parking_meter_detection_cb,
            10
        )

        self.parking_status_sub = self.create_subscription(
            Bool,
            "/parked",
            self.parking_status_cb,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.robot_state_pub = self.create_publisher(String, '/robot_state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info("State Controller Started. Waiting 5 seconds for MCL...") 

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def pose_cb(self, msg: Odometry):
        """
        Contains Logic for MCL Initialization
        """
        if self.robot_state == "MCL_INITIALIZATION":
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

            if elapsed_time >= 5.0:
                self.initial_pose = PoseStamped()

                self.initial_pose.header = msg.header
                self.initial_pose.pose = msg.pose.pose

                self.robot_state = "WAITING_FOR_PATH"
                self.robot_state_pub.publish(String(data = self.robot_state))

                self.get_logger().info(f"Initial X: {self.initial_pose.pose.position.x}, Initial Y: {self.initial_pose.pose.position.y}")

    def clicked_point_cb(self, msg: PointStamped):
        """
        Reads in Parking Spots via TA Clicked Points
        """
        if self.robot_state != "WAITING_FOR_PATH":
            return

        parking_spot = PoseStamped()
        parking_spot.header = msg.header

        parking_spot.pose.position.x = msg.point.x
        parking_spot.pose.position.y = msg.point.y
        parking_spot.pose.position.z = msg.point.z

        parking_spot.pose.orientation.x = 0.0
        parking_spot.pose.orientation.y = 0.0
        parking_spot.pose.orientation.z = 0.0
        parking_spot.pose.orientation.w = 1.0

        self.parking_spots.append(parking_spot)

        if len(self.parking_spots) == 2:
            for spot in self.parking_spots:
                self.get_logger().info(f"Parking Spot at X: {spot.pose.position.x}, Y: {spot.pose.position.y}")
            
            self.robot_state = "NAVIGATING_TO_SPOT_1"
            self.robot_state_pub.publish(String(data = self.robot_state))
            self.goal_pub.publish(self.parking_spots[0])
    
    def traffic_light_stop_cb(self, msg: Bool):
        """
        Logic Controlling Behavior when Stopping at Traffic Light
        """
        if msg.data and self.robot_state != "TRAFFIC_STOP":
            if "NAVIGATING" in self.robot_state:
                self.get_logger().info("At Red Traffic Light: Suspending Navigation")
                self.previous_robot_state = self.robot_state
                self.robot_state = "TRAFFIC_STOP"
                self.robot_state_pub.publish(String(data = self.robot_state))
        if not msg.data and self.robot_state == "TRAFFIC_STOP":
            self.get_logger().info("Traffic Light Turned Green: Resuming Navigation")
            self.robot_state = self.previous_robot_state
            self.robot_state_pub.publish(String(data = self.robot_state))
        return

    def parking_meter_detection_cb(self, msg: Bool):
        """
        Logic for Transitioning into Parking Phase
        """
        if "NAVIGATING" not in self.robot_state:
            return

        if msg.data:    #If parking meter detected
            if self.robot_state == "NAVIGATING_TO_SPOT_1":
                self.robot_state = "PARKING_AT_SPOT_1"
            elif self.robot_state == "NAVIGATING_TO_SPOT_2":
                self.robot_state = "PARKING_AT_SPOT_2"

            self.robot_state_pub.publish(String(data = self.robot_state))
            self.get_logger().info(f"Parking Meter Detected: {self.robot_state}");

    def parking_status_cb(self, msg: Bool):
        """
        Logic for Transiting back to Navigation after Parking
        """
        if msg.data:    #If successfully parked
            if self.robot_state == "PARKING_AT_SPOT_1":
                self.get_logger().info("Successfully Parked at Spot 1")

                self.robot_state = "NAVIGATING_TO_SPOT_2"
                self.robot_state_pub.publish(String(data = self.robot_state))

                self.goal_pub.publish(self.parking_spots[1])
            elif self.robot_state == "PARKING_AT_SPOT_2":
                self.get_logger().info("Successfully Parked at Spot 2")

                self.robot_state = "NAVIGATING_TO_START"
                self.robot_state_pub.publish(String(data = self.robot_state))

                self.goal_pub.publish(self.initial_pose)
        
# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BoatingSchoolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
