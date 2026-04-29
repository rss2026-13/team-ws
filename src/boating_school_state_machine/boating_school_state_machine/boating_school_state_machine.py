#!/usr/bin/env python3
"""
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String


class BoatingSchoolNode(Node):
    """
    Controls the racecar in response to a traffic light.

    Parameters (set via ROS2 params or defaults below):
    """

    def __init__(self):
        super().__init__("boating_school_state_machine")

        # ── Parameters ────────────────────────────────────────────────────────
        self.robot_state = "MCL_INITIALIZATION"
        self.previous_robot_state = None
        self.initial_pose = None
        self.goal_poses = []

        self.start_time = self.get_clock().now()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            Odometry,
            "/pf/pose/odom",
            self.pose_cb,
            10
        )

        self.clicked_point_subg = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_cb,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.robot_state_pub = self.create_publisher(String, '/robot_state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info("State Controller Started. Waiting 5 seconds for MCL...") 
    # ── Callbacks ─────────────────────────────────────────────────────────────

    def pose_cb(self, msg: Odometry):
        if self.robot_state == "MCL_INITIALIZATION":
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

            if elapsed_time >= 5.0:
                self.initial_pose = msg.pose.pose

                self.robot_state = "WAITING_FOR_PATH"
                self.robot_state_pub.publish(String(data = self.robot_state))

                self.get_logger().info(f"Initial X: {self.initial_pose.position.x}, Initial Y: {self.initial_pose.position.y}")

    def clicked_point_cb(self, msg: PointStamped):
        if self.state != "WAITING_FOR_PATH":
            return

        goal_pose = PoseStamped()
        goal_pose.header = msg.header

        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.pose.position.z = msg.point.z

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.goal_poses.append(msg)

        if len(self.goal_poses) == 2:
            for spot in self.goal_poses:
                self.get_logger().info(f"Parking Spot at X: {spot.pose.position.x}, Y: {spot.pose.position.y}")
            
            self.robot_state = "NAVIGATING_TO_SPOT_1"
            self.robot_state_pub.publish(String(data = self.robot_state))
            self.goal_pub.publish(self.goal_poses[0])

# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BoatingSchoolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
