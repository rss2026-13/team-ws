#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
import math


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

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(
            ConeLocation, "/relative_cone", self.relative_cone_callback, 1)

        self.declare_parameter("parking_distance", 2.0)
        self.PARKING_DISTANCE = self.get_parameter("parking_distance").get_parameter_value().double_value
        self.relative_x = 0
        self.relative_y = 0
        self.WHEELBASE = 0.325
        self.CAR_WIDTH = 0.25
        self.MAX_STEERING_ANGLE = 0.34
        self.TURN_RADIUS = self.WHEELBASE / np.tan(self.MAX_STEERING_ANGLE)
        self.declare_parameter("velocity", 0.5)
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.BASE_LOOKAHEAD_DIST = 0.75 * self.VELOCITY
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.captured = False
        self.DISTANCE_THRESHOLD = 0.05
        self.ANGLE_THRESHOLD = 0.10
        self.escape_direction = 0

        self.get_logger().info("Parking Controller Initialized")


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        #################################
        cone_dist = math.sqrt((self.relative_x * self.relative_x) + (self.relative_y * self.relative_y))
        cone_angle = np.arctan2(self.relative_y, self.relative_x)

        is_parked = (abs(cone_dist - self.PARKING_DISTANCE) <= self.DISTANCE_THRESHOLD) and (abs(cone_angle) <= self.ANGLE_THRESHOLD) 
        if is_parked:
            self.get_logger().info("Is Parked")
            drive_cmd.drive.speed = 0.0
            drive_cmd.drive.steering_angle = 0.0
            drive_cmd.header.stamp = self.get_clock().now().to_msg()
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()
            return

        # Radius of "capture circle", the boundary after which the car has enough space to face the cone at the parking distance
        r_capture = self.PARKING_DISTANCE + (2 * self.TURN_RADIUS * (abs(cone_angle) / np.pi))

        if cone_dist <= r_capture:
            self.captured = True
        elif cone_dist > r_capture * 1.1:
            self.captured = False 

        if self.captured:
            # Escape from the capture circle
            self.get_logger().info(f"Is captured: alpha = {cone_angle}")
            
            if self.escape_direction == 0:
                self.escape_direction = np.sign(cone_angle)

            if abs(cone_angle) < np.pi / 2:
                # Facing cone, too close. Backward spiral at 45 degrees relative to cone
                drive_cmd.drive.speed = -self.VELOCITY
                # delta = -abs(cone_angle) * self.escape_direction * np.radians(135) 
                # drive_cmd.drive.steering_angle = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE) 
                drive_cmd.drive.steering_angle = 0.0
            else:
                # Facing away, too close. Forward spiral at 45 degrees relative to cone
                drive_cmd.drive.speed = self.VELOCITY
                delta = abs(cone_angle) * self.escape_direction * np.radians(135) 
                drive_cmd.drive.steering_angle = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE) 
        else:
            # Dock (Outside of capture circle)
            self.escape_direction = 0
            self.get_logger().info(f"Not captured, docking: alpha = {cone_angle}")
            
            drive_cmd.drive.speed = self.VELOCITY
            
            L = self.BASE_LOOKAHEAD_DIST
            kappa = (2 * np.sin(cone_angle)) / L
            delta = np.arctan(kappa * self.WHEELBASE)
            drive_cmd.drive.steering_angle = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
       #################################
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()


    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        #################################
        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt((self.relative_x * self.relative_x) + (self.relative_y * self.relative_y))
        #################################

        self.error_pub.publish(error_msg)


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
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
