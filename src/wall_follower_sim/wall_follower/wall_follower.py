#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Float32
from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        # TODO: Initialize your publishers and subscribers here
        self.declare_parameter("base_lookahead_dist", 0.75 * self.VELOCITY)
        self.declare_parameter("car_length", 0.325) 
        self.declare_parameter("max_steering_angle", 0.34)
        self.declare_parameter("wall_visual", "/wall_visual")
        self.declare_parameter("path_visual", "/path_visual")
        self.declare_parameter("delta_visual", "/delta_visual")
        self.declare_parameter("intersection_visual", "/intersection_visual")

        self.BASE_LOOKAHEAD_DIST = self.get_parameter('base_lookahead_dist').get_parameter_value().double_value
        self.CAR_LENGTH = self.get_parameter('car_length').get_parameter_value().double_value
        self.MAX_STEERING_ANGLE = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.WALL_VISUAL = self.get_parameter('wall_visual').get_parameter_value().string_value
        self.PATH_VISUAL = self.get_parameter('path_visual').get_parameter_value().string_value
        self.INTERSECTION_VISUAL = self.get_parameter('intersection_visual').get_parameter_value().string_value
        self.DELTA_VISUAL = self.get_parameter('delta_visual').get_parameter_value().string_value

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.scan_listener,
            10
        )

        self.drive_pub_ = self.create_publisher(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            10
        )

        self.distance_pub_ = self.create_publisher(
            Float32,
            "/distance",
            10 
        )

        self.angle_pub_ = self.create_publisher(
            Float32,
            "/angle",
            10 
        )
        self.wall_pub = self.create_publisher(Marker, self.WALL_VISUAL, 1)
        self.path_pub = self.create_publisher(Marker, self.PATH_VISUAL, 1)
        self.intersection_pub = self.create_publisher(Marker, self.INTERSECTION_VISUAL, 1)
        self.delta_pub = self.create_publisher(Marker, self.DELTA_VISUAL, 1)

        self.past_error = 0
        self.past_time = 0
        self.kd = 0.05 # error derivative gain value empirically found


    # TODO: Write your callback functions here    
    def scan_listener(self, scan):
        """Control loop triggered by new LIDAR data from scan_sub_"""
        x, y = self.filter_scan(scan)
        wall_params = self.estimate_wall(x, y) # (m, b) line and slope of wall in base_link frame
        desired_path_params = (wall_params[0], wall_params[1] - self.SIDE * self.DESIRED_DISTANCE) # The desired path is offset by the desired distance from the wall (m, b)
        lookahead_dist = max(self.BASE_LOOKAHEAD_DIST, 1.1 * abs(desired_path_params[1])) # Lookahead distance should at least be as big as the desired distance from the wall
        ref_angle, ref_x, ref_y = self.find_ref_angle(lookahead_dist, desired_path_params)

        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = current_time - self.past_time
        error = desired_path_params[1]
        error_derivative = (error - self.past_error) / dt

        delta_pp = 0
        delta_kd = self.kd * error_derivative

        # Check if desired path is within lookahead_dist and give command based on pure pursuit/simple bicycle model to reach
        if ref_angle is not None:
            VisualizationTools.plot_line([ref_x, ref_x + 0.1], [ref_y, ref_y + 0.1], self.intersection_pub, color=(0.0, 1.0, 0.0), frame="/base_link")
            delta_pp = np.arctan2(2 * self.CAR_LENGTH * np.sin(ref_angle), lookahead_dist)
        else:
            self.get_logger().info(f"NO REF ANGLE FOUND")

        delta = delta_pp + delta_kd # Steering command is based on pure pursuit + derivative control
        delta = np.clip(delta, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE) # Make sure steering angle command is possible

        self.publish_drive_message(delta, self.VELOCITY)

        self.past_time = current_time
        self.past_error = error

        # Visualtization and graphing
        wall_angle = np.arctan2(wall_params[0], 1) # Angle is determined by the slope of the wall in the base_link frame
        perp_distance = abs(wall_params[1]) / np.sqrt((wall_params[0] ** 2) + 1) # Perpendicular distance given by point to line formula

        distance_float = Float32()
        angle_float = Float32()
        distance_float.data = perp_distance
        angle_float.data = wall_angle
        self.distance_pub_.publish(distance_float)
        self.angle_pub_.publish(angle_float)

        visual_x = np.linspace(-2.0, 2.0, num=2)
        wall_y = wall_params[0] * visual_x + wall_params[1]
        VisualizationTools.plot_line(visual_x, wall_y, self.wall_pub, color=(1.0, 0.0, 1.0), frame="/base_link")

        desired_path_y = desired_path_params[0] * visual_x + desired_path_params[1]
        VisualizationTools.plot_line(visual_x, desired_path_y, self.path_pub, color=(1.0, 0.0, 1.0), frame="/base_link")

        delta_y = visual_x * np.tan(delta)
        VisualizationTools.plot_line(visual_x, delta_y, self.delta_pub, color=(0.0, 0.0, 1.0), frame="/base_link")
        

    def publish_drive_message(self, steer_angle, speed):
        """Helper function to publish drive commands"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = steer_angle
        drive_msg.drive.speed = speed 
        self.drive_pub_.publish(drive_msg)


    def find_ref_angle(self, lookahead_dist, path_params):
        """
        Calculates the intersection point between the lookahead circle and the desired path.

        This function solves the quadratic equation formed by the intersection of a 
        circle of radius `lookahead_dist` and a line defined by `y = mx + b`.

        Args:
            lookahead_dist (float): The radius of the lookahead circle in meters.
            path_params (tuple): A tuple containing (slope, intercept) of the target path 
                relative to the car's frame.

        Returns:
            tuple: (ref_angle, x, y) if a valid intersection is found in front of the car:
                - ref_angle (float): Angle to the target point in radians.
                - x (float): X-coordinate of the target point in meters.
                - y (float): Y-coordinate of the target point in meters.
            None: If no real intersection exists or the intersection is behind the car.
        """
        m, b = path_params
        intersect_poly = np.polynomial.polynomial.Polynomial((b**2 - lookahead_dist**2, 2*m*b, 1 + m**2))

        x = np.max(intersect_poly.roots())
        if np.iscomplex(x) or (x < 0):
            return None
        y = m*x + b
        ref_angle = np.arctan2(y, x)
        return ref_angle, x, y


    def estimate_wall(self, x, y):
        """
        Estimates the wall line using a first-order polynomial fit.

        Args:
            x (np.ndarray): Array of x-coordinates of filtered LIDAR points.
            y (np.ndarray): Array of y-coordinates of filtered LIDAR points.

        Returns:
            tuple: (m, b) representing the line equation y = mx + b:
                - m (float): The slope of the estimated wall.
                - b (float): The y-intercept of the estimated wall.
        """
        coeffs = np.polyfit(x, y, 1)
        m, b = coeffs[0], coeffs[1]
        return m, b


    def filter_scan(self, scan):
        """
        Converts raw LaserScan data to Cartesian coordinates and filters for the wall.

        Filters the points to keep only those that are within a 5-meter range of the
        car, and mainly on the side specified by the `side` parameter (with points
        within a meter of the opposite side being allowed).

        Args:
            scan (sensor_msgs.msg.LaserScan): The incoming raw message from the LIDAR.

        Returns:
            tuple: (x, y) coordinates of the filtered points:
                - x (np.ndarray): Filtered x-coordinates in meters.
                - y (np.ndarray): Filtered y-coordinates in meters.
        """
        ranges = np.array(scan.ranges, dtype="float32") 
        angles = np.linspace(scan.angle_min, scan.angle_max, num=ranges.shape[0])

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        mask = (self.SIDE * y > -1) & (x > 0.0) & (x < 5)
        x = x[mask]
        y = y[mask]

        return x, y


    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    