#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult

from scipy import stats

from std_msgs.msg import Float32


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
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

        # subscriber to the lidar scan topic
        self.scan_sub = self.create_subscription(
            LaserScan, 
            self.SCAN_TOPIC, 
            self.scan_callback, 
            10
        )
        
        # publisher to send drive commands to motor
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC, 
            10
        )
        
        self.kp=2.5
        self.kd=5
        
        self.dist_pub = self.create_publisher(Float32, 'distance', 10)
        self.angle_pub = self.create_publisher(Float32, 'angle', 10)
        self.used_points_pub = self.create_publisher(Marker, "used_points_marker", 10)
        self.wall_line_pub = self.create_publisher(Marker, "wall_line_marker", 10)

        # TODO: Write your callback functions here   
    def scan_callback(self, msg):
        
        #LIDAR info
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        filtered_ranges=ranges
        filtered_angles=angles
        
        #hypothesis:
        #keep negative angle values (first half of ranges array) if following right wall
        #keep positive angle values (second half of ranges array) if following left wall

        # divide_by=len(ranges)//4 #divides the field of view into 4 quadrants
        
        # right_wall_vals=ranges[0:divide_by]
        # right_wall_angles=angles[0:divide_by]
        
        # left_wall_vals=ranges[3*divide_by:-1] #look at area from 3rd quadrant to angle_max
        # left_wall_angles=angles[3*divide_by:-1]
        
        divide_by=len(filtered_ranges)//2 #divides the field of view into 2 
        
        right_wall_vals=filtered_ranges[0:divide_by]
        right_wall_angles=filtered_angles[0:divide_by]
        
        left_wall_vals=filtered_ranges[divide_by+1:-1]
        left_wall_angles=filtered_angles[divide_by+1:-1]
        
        if self.SIDE==1:
            usable_range=left_wall_vals
            usable_angles=left_wall_angles
        else:
            usable_range=right_wall_vals
            usable_angles=right_wall_angles

        valid_mask = (
            np.isfinite(usable_range)
            & (usable_range >= msg.range_min)
            & (usable_range <= msg.range_max)
        )
        usable_range = usable_range[valid_mask]
        usable_angles = usable_angles[valid_mask]

        if usable_range.size < 2:
            self.get_logger().warn("Not enough valid scan points for wall fit.")
            return

        # Convert Polar to Cartesian (base_link frame)
        x = usable_range * np.cos(usable_angles)
        y = usable_range * np.sin(usable_angles)
        self.publish_used_points_marker(x, y)

        # Perform Linear Regression: y = mx + c
        # slope = m, intercept = c
        slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
        self.publish_wall_line_marker(x, slope, intercept)

        # Calculate distance from origin (the car) to the line
        # For a line y = mx + c, the distance to (0,0) is:
        dist = abs(intercept) / np.sqrt(slope**2 + 1)
        
        # Calculate error (angle of the car relative to the wall)
        wall_angle = np.arctan(slope)
        
        # 2. Corrected PD Logic
        # Calculate distance error
        dist_error = self.DESIRED_DISTANCE - dist
        dist_error=dist_error/4 #quarter error term
        wall_angle=wall_angle/4
        
        # Correct the steering direction based on which side we follow
        if self.SIDE == 1: # Left wall
            # Too close (dist_error > 0) -> Need to turn Right (negative)
            # Too far (dist_error < 0) -> Need to turn Left (positive)
            # The wall_angle (slope) also needs to be factored correctly
            steer_angle = (-self.kp * dist_error) + (self.kd * wall_angle)
        else: # Right wall
            # Too close (dist_error > 0) -> Need to turn Left (positive)
            # Too far (dist_error < 0) -> Need to turn Right (negative)
            steer_angle = (self.kp * dist_error) + (self.kd * wall_angle)
        
        # Publish metrics for evaluation
        dist_msg = Float32()
        dist_msg.data = float(dist)
        self.dist_pub.publish(dist_msg)
        self.get_logger().info(f'The distance: {dist_msg}')

        angle_msg = Float32()
        angle_msg.data = float(wall_angle)
        self.angle_pub.publish(angle_msg)
            
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        
        # # Use the parameters fetched in __init__
        drive_msg.drive.speed = self.get_parameter('velocity').value
        drive_msg.drive.steering_angle =  steer_angle
        
        self.drive_pub.publish(drive_msg)
        
        
    
    def publish_used_points_marker(self, x_points, y_points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wall_follower"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1

        marker.points = []
        for x_val, y_val in zip(x_points, y_points):
            point = Point()
            point.x = float(x_val)
            point.y = float(y_val)
            point.z = 0.0
            marker.points.append(point)

        self.used_points_pub.publish(marker)

    def publish_wall_line_marker(self, x_points, slope, intercept):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wall_follower"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2

        x_min = float(np.min(x_points))
        x_max = float(np.max(x_points))
        if np.isclose(x_min, x_max):
            x_max = x_min + 0.01

        line_x = np.array([x_min, x_max], dtype=float)
        line_y = slope * line_x + intercept

        marker.points = []
        for x_val, y_val in zip(line_x, line_y):
            point = Point()
            point.x = float(x_val)
            point.y = float(y_val)
            point.z = 0.0
            marker.points.append(point)

        self.wall_line_pub.publish(marker)

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
    
    
