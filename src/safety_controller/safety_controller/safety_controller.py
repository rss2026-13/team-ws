#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker




class SafetyController(Node):
   def __init__(self):
       super().__init__("safety_controller")


       self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
       self.declare_parameter("output_topic", "/drive")
       self.declare_parameter("scan_topic", "/scan")
       self.declare_parameter("margin", 0.5)
       self.declare_parameter("max_deceleration", 10.0)
       self.declare_parameter("cone_angle", 10.0) # In degrees
       self.declare_parameter("car_width", 0.25)
       self.declare_parameter(
           "lidar_offset", 0.05
       )  # Distance from lidar to front bumper
       self.DRIVE_TOPIC = (
           self.get_parameter("drive_topic").get_parameter_value().string_value
       )
       self.OUTPUT_TOPIC = (
           self.get_parameter("output_topic").get_parameter_value().string_value
       )
       self.SCAN_TOPIC = (
           self.get_parameter("scan_topic").get_parameter_value().string_value
       )
       self.MARGIN = self.get_parameter("margin").get_parameter_value().double_value
       self.MAX_DECELERATION = (
           self.get_parameter("max_deceleration").get_parameter_value().double_value
       )
       self.CONE_ANGLE = (
           self.get_parameter("cone_angle").get_parameter_value().double_value
       )
       self.CAR_WIDTH = (
           self.get_parameter("car_width").get_parameter_value().double_value
       )
       self.LIDAR_OFFSET = (
           self.get_parameter("lidar_offset").get_parameter_value().double_value
       )
       self.drive_subscription = self.create_subscription(
           AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_callback, 10
       )
       self.scan_subscription = self.create_subscription(
           LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
       )
       self.drive_publisher = self.create_publisher(
           AckermannDriveStamped, self.OUTPUT_TOPIC, 10
       )
       self.safety_lidar_publisher = self.create_publisher(
           LaserScan, "/safety_controller_lidar", 10
       )
       self.bbox_publisher = self.create_publisher(Marker, "/safety_controller_bbox", 10)
       self.stop = False
       self.scan_data = None
       self.drive_command = None


   def drive_callback(self, msg):
       self.drive_command = msg
       self.get_logger().debug("Received new drive command")
       self.evaluate_safety()


   def scan_callback(self, msg):
       self.scan_data = msg
       self.get_logger().debug("Received new scan data")
       self.evaluate_safety()


   def evaluate_safety(self):
       if self.scan_data is None or self.drive_command is None:
           return
       if self.drive_command.drive.speed < 0.001:
           return
       front_corners = [
           [self.LIDAR_OFFSET, -self.CAR_WIDTH / 2],
           [self.LIDAR_OFFSET, self.CAR_WIDTH / 2],
       ]
       corner_stop_flags = [self.stop for _ in front_corners]
       rectangle_stop = False
       if not self.stop:
           speed = self.drive_command.drive.speed
           front_treshold = self.MARGIN + (speed**2) / (2 * self.MAX_DECELERATION)
           self.get_logger().debug(
               f"Evaluating safety: speed={speed:.2f}, front_threshold={front_treshold:.2f}"
           )
           angle_min = self.scan_data.angle_min
           angle_max = self.scan_data.angle_max
           ranges = np.array(self.scan_data.ranges)
           angles = np.linspace(angle_min, angle_max, num=ranges.shape[0])
           points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T


           # Check cone from front corners
           for corner_index, corner in enumerate(front_corners):
               for point in points:
                   if (
                       np.linalg.norm(point - corner) < front_treshold
                       and abs(
                           np.arctan2(point[1] - corner[1], point[0] - corner[0])
                           * 180
                           / np.pi
                       )
                       < self.CONE_ANGLE
                   ):
                       corner_stop_flags[corner_index] = True
                       self.stop = True
                       self.get_logger().warn(
                           "Frontal object detected! Stopping the robot."
                       )
                       break
           # Check rectangle between the corners as well
           for point in points:
               if (
                   0 < point[0] < self.LIDAR_OFFSET + front_treshold
                   and abs(point[1]) < self.CAR_WIDTH / 2
               ):
                   rectangle_stop = True
                   self.stop = True
                   self.get_logger().warn(
                       "Frontal object detected! Stopping the robot."
                   )
                   break
           if rectangle_stop:
               corner_stop_flags = [True for _ in front_corners]
       self.publish_front_corner_scan(front_corners)
       self.publish_vehicle_markers(front_corners, corner_stop_flags)


       if self.stop:
           safe_command = AckermannDriveStamped()
           safe_command.header.stamp = self.get_clock().now().to_msg()
           safe_command.drive.speed = 0.0
           safe_command.drive.steering_angle = 0.0
           self.drive_publisher.publish(safe_command)


   def publish_front_corner_scan(self, front_corners):
       if self.scan_data is None:
           return
       source_scan = self.scan_data
       num_ranges = len(source_scan.ranges)
       if num_ranges == 0:
           return
       corner_ranges = [float("inf")] * num_ranges
       angle_increment = source_scan.angle_increment
       if angle_increment <= 0.0:
           return
       for corner in front_corners:
           corner_angle = float(np.arctan2(corner[1], corner[0]))
           if corner_angle < source_scan.angle_min or corner_angle > source_scan.angle_max:
               continue
           index = int(round((corner_angle - source_scan.angle_min) / angle_increment))
           if index < 0 or index >= num_ranges:
               continue
           corner_distance = float(np.hypot(corner[0], corner[1]))
           corner_ranges[index] = min(corner_ranges[index], corner_distance)
       scan_msg = LaserScan()
       scan_msg.header = source_scan.header
       scan_msg.angle_min = source_scan.angle_min
       scan_msg.angle_max = source_scan.angle_max
       scan_msg.angle_increment = source_scan.angle_increment
       scan_msg.time_increment = source_scan.time_increment
       scan_msg.scan_time = source_scan.scan_time
       scan_msg.range_min = source_scan.range_min
       scan_msg.range_max = source_scan.range_max
       scan_msg.ranges = corner_ranges
       self.safety_lidar_publisher.publish(scan_msg)


   def publish_vehicle_markers(self, front_corners, corner_stop_flags):
       if self.scan_data is None:
           return
       timestamp = self.get_clock().now().to_msg()


       bbox_marker = Marker()
       bbox_marker.header.stamp = timestamp
       bbox_marker.header.frame_id = self.scan_data.header.frame_id
       bbox_marker.ns = "safety_controller"
       bbox_marker.id = 0
       bbox_marker.type = Marker.LINE_STRIP
       bbox_marker.action = Marker.ADD
       bbox_marker.scale.x = 0.03
       bbox_marker.color.a = 1.0
       bbox_marker.color.r = 0.0
       bbox_marker.color.g = 1.0
       bbox_marker.color.b = 0.0
       bbox_marker.points = [
           Point(x=0.0, y=-self.CAR_WIDTH / 2, z=0.0),
           Point(x=self.LIDAR_OFFSET, y=-self.CAR_WIDTH / 2, z=0.0),
           Point(x=self.LIDAR_OFFSET, y=self.CAR_WIDTH / 2, z=0.0),
           Point(x=0.0, y=self.CAR_WIDTH / 2, z=0.0),
           Point(x=0.0, y=-self.CAR_WIDTH / 2, z=0.0),
       ]
       self.bbox_publisher.publish(bbox_marker)


       corner_marker = Marker()
       corner_marker.header.stamp = timestamp
       corner_marker.header.frame_id = self.scan_data.header.frame_id
       corner_marker.ns = "safety_controller"
       corner_marker.id = 1
       corner_marker.type = Marker.POINTS
       corner_marker.action = Marker.ADD
       corner_marker.scale.x = 0.1
       corner_marker.scale.y = 0.1
       corner_marker.points = [
           Point(x=corner[0], y=corner[1], z=0.0) for corner in front_corners
       ]
       corner_marker.colors = [
           ColorRGBA(
               r=1.0 if should_stop else 0.0,
               g=0.0 if should_stop else 1.0,
               b=0.0,
               a=1.0,
           )
           for should_stop in corner_stop_flags
       ]
       self.bbox_publisher.publish(corner_marker)




def main():
   rclpy.init()
   safety_controller = SafetyController()
   rclpy.spin(safety_controller)
   safety_controller.destroy_node()
   rclpy.shutdown()




if __name__ == "__main__":
   main()



