#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker # Added for visualization
from geometry_msgs.msg import Point # Added for circle points
from rcl_interfaces.msg import SetParametersResult
import math


class ParkingState:
    CAPTURED = 0
    DOCKING = 1
    PARKED = 2


class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value  # set in launch file; different for simulator vs racecar

        # Publishers and Subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)
        self.capture_radius_pub = self.create_publisher(Float32, "/capture_radius", 10)
        self.parking_state_pub = self.create_publisher(Int32, "/parking_state", 10)
        self.cone_angle_pub = self.create_publisher(Float32, "/cone_angle", 10)
        self.create_subscription(ConeLocation, "/relative_cone", self.relative_cone_callback, 1)
        self.viz_pub = self.create_publisher(Marker, "/parking_visualization", 10)

        # Physical Car Constants
        self.WHEELBASE = 0.325
        self.CAR_WIDTH = 0.25
        self.MAX_STEERING_ANGLE = 0.34
        self.TURN_RADIUS = self.WHEELBASE / np.tan(self.MAX_STEERING_ANGLE)
        
        # Physical Car Variables
        self.declare_parameter("velocity", 0.5)
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value

        # Controller Variables
        self.declare_parameter("parking_distance", 2.0)
        self.PARKING_DISTANCE = self.get_parameter("parking_distance").get_parameter_value().double_value
        self.BASE_LOOKAHEAD_DIST = 0.75 * self.VELOCITY

        # Thresholds and State Trackers
        self.declare_parameter("visualize_parking", False)
        self.VISUALIZE = self.get_parameter("visualize_parking").get_parameter_value().bool_value
        self.DISTANCE_THRESHOLD = 0.05
        self.ANGLE_THRESHOLD = 0.10
        self.parking_state = ParkingState.DOCKING
        self.escape_direction = 0
        self.relative_x = 0
        self.relative_y = 0

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("Parking Controller Initialized")


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        cone_dist = math.sqrt((self.relative_x * self.relative_x) + (self.relative_y * self.relative_y))
        cone_angle = np.arctan2(self.relative_y, self.relative_x)

        # Radius of "capture circle", the boundary after which the car has enough space to face the cone at the parking distance
        r_capture = self.PARKING_DISTANCE + (2 * self.TURN_RADIUS * (abs(cone_angle) / np.pi))
        
        # Visualization
        #################################
        if self.VISUALIZE:
            # Publish Capture Circle
            blue_marker = self.make_circle_marker(r_capture, [0, 0, 1], 0, "base_link")
            self.viz_pub.publish(blue_marker)

            # Publish Black Parking Circle
            black_marker = self.make_circle_marker(self.PARKING_DISTANCE, [0, 0, 0], 1, "base_link")
            self.viz_pub.publish(black_marker)
        #################################

        # Check if the car is in the proper parking state (within a threshold of distance and orientation angle)
        is_parked = (abs(cone_dist - self.PARKING_DISTANCE) <= self.DISTANCE_THRESHOLD) and (abs(cone_angle) <= self.ANGLE_THRESHOLD) 
        if is_parked:
            self.parking_state = ParkingState.PARKED
            self.drive_publisher(0.0, 0.0, self.get_clock().now().to_msg()) # Tell car to stop
            self.error_publisher()
            self.capture_radius_publisher(r_capture)
            self.parking_state_publisher()
            self.cone_angle_publisher(cone_angle)
            return

        # Determine whether the car is captured or ready to dock
        if cone_dist <= r_capture:
            self.parking_state = ParkingState.CAPTURED
        elif cone_dist > r_capture * 1.1:
            self.parking_state = ParkingState.DOCKING

        if self.parking_state == ParkingState.CAPTURED:    # CAPTURED: Escape from the capture circle
            # Determine whether to turn left or right relative to cone to escape
            if self.escape_direction == 0:
                self.escape_direction = np.sign(cone_angle)

            if abs(cone_angle) < np.pi / 2:   # Car facing cone but too close -> Backup straight to keep cone in view
                self.drive_publisher(-self.VELOCITY, 0.0, self.get_clock().now().to_msg())
                # Old Code that doesn't keep cone in cam FOV but is more space efficient (spiral at 45 deg rel to cone)
                # delta = -abs(cone_angle) * self.escape_direction * np.radians(135) 
                # drive_cmd.drive.steering_angle = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE) 
            else:   # Car facing away and too close. Forward spiral at 45 degrees relative to cone (or max angle)
                delta = abs(cone_angle) * self.escape_direction * np.radians(135) 
                delta = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE) 
                self.drive_publisher(self.VELOCITY, delta, self.get_clock().now().to_msg())
        else:   # DOCKING: Outside of capture circle, starting converging to parking spot
            self.escape_direction = 0 # Reset escape direction
            
            L = self.BASE_LOOKAHEAD_DIST
            kappa = (2 * np.sin(cone_angle)) / L
            delta = np.arctan(kappa * self.WHEELBASE)
            delta = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE) 
            self.drive_publisher(self.VELOCITY, delta, self.get_clock().now().to_msg()) 

        self.error_publisher()
        self.capture_radius_publisher(r_capture)
        self.parking_state_publisher()
        self.cone_angle_publisher(cone_angle)


    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt((self.relative_x * self.relative_x) + (self.relative_y * self.relative_y))
        self.error_pub.publish(error_msg)
    

    def drive_publisher(self, speed, steering_angle, time_stamp):
        """
        Publish the driving message to be sent to the car by the parking controller
        """
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.header.stamp = time_stamp
        self.drive_pub.publish(drive_cmd)


    def capture_radius_publisher(self, capture_radius):
        """
        Publish the current capture radius calculated by the controller
        """
        capture_msg = Float32()
        capture_msg.data = capture_radius
        self.capture_radius_pub.publish(capture_msg)


    def parking_state_publisher(self):
        """
        Publish the current parking state of the controller as an int32
        """
        parking_state_msg = Int32()
        parking_state_msg.data = self.parking_state
        self.parking_state_pub.publish(parking_state_msg)


    def cone_angle_publisher(self, cone_angle):
        """
        Publish the current cone angle relative to the car
        """
        cone_msg = Float32()
        cone_msg.data = cone_angle 
        self.cone_angle_pub.publish(cone_msg)


    def make_circle_marker(self, radius, color, marker_id, frame_id):
        """Helper to create a LINE_STRIP circle marker centered at the cone."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "parking_bounds"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Position the circle's center at the cone
        marker.pose.position.x = self.relative_x
        marker.pose.position.y = self.relative_y
        marker.pose.orientation.w = 1.0
        
        # Line thickness
        marker.scale.x = 0.05 
        
        # Color (RGBA)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0 # Opacity
        
        # Create points for a 360-degree circle
        points = []
        for angle in np.linspace(0, 2 * np.pi, 50):
            p = Point()
            p.x = radius * math.cos(angle)
            p.y = radius * math.sin(angle)
            p.z = 0.0
            marker.points.append(p)
            
        return marker


    def parameters_callback(self, params):
        for param in params:
            if param.name == 'parking_distance':
                self.PARKING_DISTANCE = param.value
                self.get_logger().info(f"Updated parking_distance to {self.PARKING_DISTANCE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.BASE_LOOKAHEAD_DIST = 0.75 * self.VELOCITY
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
                self.get_logger().info(f"Updated base_lookahead_dist to {self.BASE_LOOKAHEAD_DIST}")
            elif param.name == 'visualize_parking':
                self.VISUALIZE = param.value
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
