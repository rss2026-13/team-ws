#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point




class Path(Node):


   def __init__(self):
       super().__init__("path")
       # Declare parameters to make them available for use
       # DO NOT MODIFY THIS!
       # Odometry -> trajectory visualization
       self.declare_parameter("odom_topic", "/odom")
       self.declare_parameter("trajectory_topic", "/trajectory")
       self.declare_parameter("trajectory_frame_id", "odom")
       self.declare_parameter("trajectory_history", 200)  # max stored points
       self.declare_parameter("trajectory_min_point_dist", 0.05)  # meters
       self.declare_parameter("trajectory_line_width", 0.04)


       # Fetch odometry-related constants
       self.ODOM_TOPIC = (
           self.get_parameter("odom_topic").get_parameter_value().string_value
       )
       self.TRAJECTORY_TOPIC = (
           self.get_parameter("trajectory_topic").get_parameter_value().string_value
       )
       self.TRAJECTORY_FRAME_ID = (
           self.get_parameter("trajectory_frame_id").get_parameter_value().string_value
       )
       self.TRAJECTORY_HISTORY = (
           self.get_parameter("trajectory_history").get_parameter_value().integer_value
       )
       self.TRAJECTORY_MIN_POINT_DIST = (
           self.get_parameter("trajectory_min_point_dist").get_parameter_value().double_value
       )
       self.TRAJECTORY_LINE_WIDTH = (
           self.get_parameter("trajectory_line_width").get_parameter_value().double_value
       )

       # Publish odometry trajectory as a LINE_LIST of past points
       self.trajectory_pub = self.create_publisher(Marker, self.TRAJECTORY_TOPIC, 10)
       self.odom_sub = self.create_subscription(
           Odometry, self.ODOM_TOPIC, self.odom_callback, 10
       )
       self.trajectory_points = []  # list[(x, y)]
       self._trajectory_marker_ns = "odom_trajectory"
       self._trajectory_marker_id = 0

   def odom_callback(self, msg):
       pos = msg.pose.pose.position
       x = float(pos.x)
       y = float(pos.y)

       # Append new point only if we've moved enough to avoid spamming RViz.
       if not self.trajectory_points:
           self.trajectory_points.append((x, y))
       else:
           last_x, last_y = self.trajectory_points[-1]
           dx = x - last_x
           dy = y - last_y
           if (dx * dx + dy * dy) >= (self.TRAJECTORY_MIN_POINT_DIST ** 2):
               self.trajectory_points.append((x, y))

       # Keep only the last N points.
       if len(self.trajectory_points) > self.TRAJECTORY_HISTORY:
           self.trajectory_points = self.trajectory_points[-self.TRAJECTORY_HISTORY:]

       # Need at least 2 points to draw a line segment.
       if len(self.trajectory_points) < 2:
           return

       marker = Marker()
       marker.header.stamp = self.get_clock().now().to_msg()
       marker.header.frame_id = (
           msg.header.frame_id if msg.header.frame_id else self.TRAJECTORY_FRAME_ID
       )
       # RViz expects a valid quaternion. `Marker()` initializes orientation.w to 0,
       # which is not a valid identity rotation.
       marker.pose.orientation.w = 1.0
       marker.ns = self._trajectory_marker_ns
       marker.id = self._trajectory_marker_id
       marker.type = Marker.LINE_LIST
       marker.action = Marker.ADD

       marker.scale.x = float(self.TRAJECTORY_LINE_WIDTH)  # line width
       marker.color.r = 0.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.color.a = 1.0

       marker.points = []
       for i in range(1, len(self.trajectory_points)):
           x0, y0 = self.trajectory_points[i - 1]
           x1, y1 = self.trajectory_points[i]

           p0 = Point()
           p0.x = x0
           p0.y = y0
           p0.z = 0.0

           p1 = Point()
           p1.x = x1
           p1.y = y1
           p1.z = 0.0

           marker.points.append(p0)
           marker.points.append(p1)

       self.trajectory_pub.publish(marker)
  
   def parameters_callback(self, params):
       return None




def main():
   rclpy.init()
   node = Path()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()