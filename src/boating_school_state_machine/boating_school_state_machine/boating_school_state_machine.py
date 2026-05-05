#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from ackermann_msgs.msg import AckermannDriveStamped

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

        self.declare_parameter("mcl_initialization_time", 5.0)
        self.mcl_initialization_time = (
            self.get_parameter("mcl_initialization_time").get_parameter_value().double_value
        )
        self.latest_pf_estimate = None
        self.pf_sub = None
        self.mcl_timer = None

        self.sweep_phase = 0
        self.sweep_count = 0
        self.sweep_timer = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
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

        self.goal_reached_sub = self.create_subscription(
            Bool,
            "/goal_reached",
            self.goal_reached_cb,
            10
        )

        self.safety_stop_sub = self.create_subscription(
            Bool,
            "/safety_stop",  
            self.safety_stop_cb,
            10
        )
    
        # ── Publishers ────────────────────────────────────────────────────────
        self.robot_state_pub = self.create_publisher(String, '/robot_state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sweep_pub = self.create_publisher(
            AckermannDriveStamped,
            "/vesc/low_level/input/navigation",
            10
        )

        self.get_logger().info("State Controller Started. Waiting 5 seconds for MCL...") 

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.robot_state != "MCL_INITIALIZATION" or self.mcl_timer is not None:
            return

        self.get_logger().info(f"Initial pose received. Waiting {self.mcl_initialization_time}S for MCL to converge...")

        self.pf_sub = self.create_subscription(
            Odometry,
            "/pf/pose/odom",
            lambda odom_msg: setattr(self, 'latest_pf_estimate', odom_msg),
            10
        )

        def finalize_localization():
            if self.latest_pf_estimate is None:
                self.get_logger().warn("PF estimate not yet available, retrying...")
                return

            self.initial_pose = PoseStamped()
            self.initial_pose.header = self.latest_pf_estimate.header
            self.initial_pose.pose = self.latest_pf_estimate.pose.pose

            if self.pf_sub is not None:
                self.destroy_subscription(self.pf_sub)
                self.pf_sub = None

            self.mcl_timer.cancel()
            self.mcl_timer = None

            self.robot_state = "WAITING_FOR_PATH"
            self.robot_state_pub.publish(String(data=self.robot_state))
            self.get_logger().info(
                f"MCL converged. Initial pose - X: {self.initial_pose.pose.position.x:.3f}, Y: {self.initial_pose.pose.position.y:.3f}"
            )

        self.mcl_timer = self.create_timer(self.mcl_initialization_time, finalize_localization)


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
        if ("NAVIGATING" not in self.robot_state) and ("SEARCHING" not in self.robot_state):
            return

        if msg.data:    #If parking meter detected
            if self.sweep_timer is not None:
                self.sweep_timer.cancel()
                self.sweep_timer = None
            
            if "SPOT_1" in self.robot_state: 
                self.robot_state = "PARKING_AT_SPOT_1"
            elif "SPOT_2" in self.robot_state:
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
        

    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return
        if self.robot_state == "NAVIGATING_TO_SPOT_1":
            self.robot_state = "SEARCHING_FOR_SPOT_1"
            self.robot_state_pub.publish(String(data = self.robot_state))
            self.get_logger().info("Starting sweep to find parking spot 1...")
            self.start_sweep()
        elif self.robot_state == "NAVIGATING_TO_SPOT_2":
            self.robot_state = "SEARCHING_FOR_SPOT_2"
            self.robot_state_pub.publish(String(data = self.robot_state))
            self.get_logger().info("Starting sweep to find parking spot 2...")
            self.start_sweep()
        elif self.robot_state == "NAVIGATING_TO_START":
            self.robot_state = "COMPLETE"
            self.robot_state_pub.publish(String(data = self.robot_state))


    def start_sweep(self):
        self.sweep_phase = 0
        self.sweep_count = 0
        if self.sweep_timer is not None:
            self.sweep_timer.cancel()
        self.sweep_timer = self.create_timer(0.3, self.sweep_step)


    def sweep_step(self):
        if self.sweep_count > 16:
            self.get_logger().warn("Full sweep complete, parking meter not found.")
            self.sweep_timer.cancel()
            self.sweep_timer = None
            return

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()

        if self.sweep_phase == 0:
            drive_cmd.drive.speed = 0.25
            drive_cmd.drive.steering_angle = 0.34
            self.sweep_phase = 1
        else:
            drive_cmd.drive.speed = -0.25
            drive_cmd.drive.steering_angle = -0.34  
            self.sweep_phase = 0

        self.sweep_pub.publish(drive_cmd)
        self.sweep_count += 1


    def safety_stop_cb(self, msg: Bool):
        if msg.data and self.robot_state != "SAFETY_STOP":
            if "NAVIGATING" in self.robot_state or "SEARCHING" in self.robot_state:
                self.previous_robot_state = self.robot_state
                self.robot_state = "SAFETY_STOP"
                self.robot_state_pub.publish(String(data = self.robot_state))
        if not msg.data and self.robot_state == "SAFETY_STOP":
            self.get_logger().info(f"previous state before safety {self.robot_state}")
            self.robot_state = self.previous_robot_state
            self.robot_state_pub.publish(String(data = self.robot_state))
            if "SEARCHING" in self.previous_robot_state:
                self.start_sweep()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BoatingSchoolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
