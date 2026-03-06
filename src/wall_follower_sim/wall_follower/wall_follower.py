#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


from wall_follower.visualization_tools import VisualizationTools
import math




class WallFollower(Node):


   def __init__(self):
       super().__init__("wall_follower")
       # Declare parameters to make them available for use
       # DO NOT MODIFY THIS!
       self.declare_parameter("scan_topic", "/scan")
       self.declare_parameter("drive_topic", "/drive")
       self.declare_parameter("side", -1)
       self.declare_parameter("velocity", 0.5)
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
       self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/high_level/input/nav_1', 10)
       self.publisherVision = self.create_publisher(LaserScan, '/vision', 10)
       self.subscriber = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
       # True distance (perpendicular to wall) and angle (wall relative to robot heading)
       self.distance_pub = self.create_publisher(Float32, '/distance', 10)
       self.angle_pub = self.create_publisher(Float32, '/angle', 10)
  
   def listener_callback(self, msg):
       drive_command = AckermannDriveStamped()


       #desired qualities
       self.SIDE
       self.VELOCITY
       d = self.DESIRED_DISTANCE


       #filter vision (if right side??)
       filteredmsg = LaserScan()
       filteredmsg.header.stamp = msg.header.stamp
       filteredmsg.header.frame_id = 'base_link'
       filteredmsg.range_min = msg.range_min
       filteredmsg.range_max = msg.range_max
       filteredmsg.scan_time = msg.scan_time
       margin = math.pi * 0.1
       window = math.pi * 0.7
       angle_increment = msg.angle_increment
       if angle_increment <= 0.0:
           return

       total_points = len(msg.ranges)
       if total_points == 0:
           return

       def angle_to_index(target_angle: float) -> int:
           idx = int(round((target_angle - msg.angle_min) / angle_increment))
           return max(0, min(total_points - 1, idx))

       if self.SIDE > 0:
           start_angle = min(msg.angle_min + margin, msg.angle_max)
           end_angle = min(start_angle + window, msg.angle_max)
           start_idx = angle_to_index(start_angle)
           end_idx = angle_to_index(end_angle)
           if end_idx < start_idx:
               return

           filteredmsg.angle_min = msg.angle_min + start_idx * angle_increment
           filteredmsg.angle_max = msg.angle_min + end_idx * angle_increment
           filteredmsg.angle_increment = angle_increment
           filteredmsg.ranges = list(msg.ranges[start_idx : end_idx + 1])
       else:
           end_angle = max(msg.angle_max - margin, msg.angle_min)
           start_angle = max(end_angle - window, msg.angle_min)
           start_idx = angle_to_index(start_angle)
           end_idx = angle_to_index(end_angle)
           if end_idx < start_idx:
               return

           filteredmsg.angle_min = msg.angle_min + start_idx * angle_increment
           filteredmsg.angle_max = msg.angle_min + end_idx * angle_increment
           filteredmsg.angle_increment = angle_increment
           filteredmsg.ranges = list(msg.ranges[start_idx : end_idx + 1])


       #limit by distance
       # minDistance = min(filteredmsg.ranges)
       # minDistanceAngleIndex = filteredmsg.ranges.index(minDistance)
       # minDistanceAngle = filteredmsg.angle_min + filteredmsg.angle_increment * minDistanceAngleIndex
       # filteredmsg.angle_min = max(filteredmsg.angle_min, minDistanceAngle - 0.5)
       # filteredmsg.angle_max = min(filteredmsg.angle_max, minDistanceAngle + 0.5)
       # filteredmsg.angle_increment = msg.angle_increment
       # count = int((filteredmsg.angle_max - minDistanceAngle) / filteredmsg.angle_increment)
       # filteredmsg.ranges = filteredmsg.ranges[minDistanceAngleIndex - count:minDistanceAngleIndex + count + 1]


       # --- add distance info without removing points ---
       ranges = np.array(filteredmsg.ranges)
       angles = filteredmsg.angle_min + np.arange(len(ranges)) * filteredmsg.angle_increment


       # filter out invalid points for calculation only
       valid_mask = np.isfinite(ranges)
       if np.any(valid_mask):
           valid_ranges = ranges[valid_mask]
           valid_angles = angles[valid_mask]
          
           # compute 25th percentile distance
           p25_distance = np.percentile(valid_ranges, 30)
          
           # mark points in the lowest 25%
           closest_mask = valid_ranges <= p25_distance
          
           # keep original ranges, but store mask in intensities
           filteredmsg.ranges = ranges.tolist()
           filteredmsg.intensities = closest_mask.astype(float).tolist()








       self.publisherVision.publish(filteredmsg)
       # self.get_logger().info('Publishing: "%s"' % count)


       #find the wall with least squares (x is forward, y is left?)
       pointsX = np.array([math.cos(filteredmsg.angle_min + i * filteredmsg.angle_increment) * x for i, x in enumerate(filteredmsg.ranges)])
       pointsY = np.array([math.sin(filteredmsg.angle_min + i * filteredmsg.angle_increment) * x for i, x in enumerate(filteredmsg.ranges)])
       m = (len(pointsX) * np.sum(pointsX * pointsY) - np.sum(pointsX) * np.sum(pointsY)) / (len(pointsX) * sum(pointsX ** 2) - sum(pointsX)**2)
       b = (np.sum(pointsY) - m * np.sum(pointsX)) / len(pointsX)
      
       den = len(pointsX) * np.sum(pointsX**2) - (np.sum(pointsX))**2
       if abs(den) < 1e-6:
           return  # or skip frame


       # True perpendicular distance to wall: |b| / sqrt(1 + m^2)
       true_distance = abs(b) / math.sqrt(1 + m * m)
       # True angle of wall relative to robot heading (radians)
       true_angle = math.atan(m) if self.SIDE < 0 else -math.atan(m)


       dist_msg = Float32()
       dist_msg.data = float(true_distance)
       self.distance_pub.publish(dist_msg)
       angle_msg = Float32()
       angle_msg.data = float(true_angle)
       self.angle_pub.publish(angle_msg)


       #figure out how to get to the right distance from the wall


       # make sure its parallel too




       #set drive
       drive_command.header.stamp = self.get_clock().now().to_msg()
       drive_command.header.frame_id = 'base_link'


       #pos yaw = left
       #zero steering angle velocity = change steering as quickly as possible
       #positive means absolute rate of change either left or right (controller tries not to exceed this)
       drive_command.drive.steering_angle = 0.5 # desired virtual angle (radians)
       drive_command.drive.steering_angle_velocity = 0.75 # desired rate of change (radians/s)


       # All are measured at the vehicle's
       # center of rotation, typically the center of the rear axle.
       # Direction is forward unless the sign is negative, indicating reverse.


       # Zero acceleration means change speed as quickly as
       # possible. Positive acceleration indicates a desired absolute
       # magnitude; that includes deceleration.
       theta = math.atan(m) if self.SIDE < 0 else -1 * math.atan(m)
       error = -2 * theta - (abs(b) - abs(d) * math.sqrt(1 + m*m))
       error = 0.5 * error
       gain = 2.5
       if self.SIDE < 0:
           if 0 > error:
               drive_command.drive.steering_angle = -1 * gain * abs(error)
           else:
               drive_command.drive.steering_angle = gain * abs(error)
       else:
           if 0 > error:
               drive_command.drive.steering_angle = gain * abs(error)
           else:
               drive_command.drive.steering_angle = -1 * gain * abs(error)
       
       #just for robot
       drive_command.drive.steering_angle *= -1

       drive_command.drive.speed = self.VELOCITY #m/s
       drive_command.drive.acceleration = 1.0
       drive_command.drive.jerk = 0.0 #


       self.publisher.publish(drive_command)
       self.get_logger().info('Publishing Desired: "%s"' % d)
       self.get_logger().info('Publishing Current: "%s"' % b)


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