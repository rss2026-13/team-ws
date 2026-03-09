#!/usr/bin/env python3
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")
        # /vesc/low_level/ackermann_cmd
        # /vesc/low_level/input/safety
        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("output_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("margin", 0.2)
        self.declare_parameter("max_deceleration", 2.0)
        self.declare_parameter("car_width", 0.25)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("lidar_offset", 0.12)  # Distance from lidar to front bumper
        self.declare_parameter("visualize", True)

        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.OUTPUT_TOPIC = self.get_parameter("output_topic").get_parameter_value().string_value
        self.SCAN_TOPIC = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.MARGIN = self.get_parameter("margin").get_parameter_value().double_value
        self.MAX_DECELERATION = self.get_parameter("max_deceleration").get_parameter_value().double_value
        self.CAR_WIDTH = self.get_parameter("car_width").get_parameter_value().double_value
        self.WHEELBASE = self.get_parameter("wheelbase").get_parameter_value().double_value
        self.LIDAR_OFFSET = self.get_parameter("lidar_offset").get_parameter_value().double_value
        self.VISUALIZE = self.get_parameter("visualize").get_parameter_value().bool_value

        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.OUTPUT_TOPIC, 10
        )
        self.distance_pub = self.create_publisher(Float32, "/sc_wall_dist", 10)

        self.marker_pub = self.create_publisher(Marker, "/safety_marker", 1)
        self.is_collision = False
        self.scan_data = None
        self.drive_command = None
        self.scan_cos_angles = None
        self.scan_sin_angles = None
        self.collision_counter = 0 # Needed to account for salt and pepper LiDAR noise
        self.DETECTION_THRESHOLD = 3 # Number of frames scan must remain in collision zone for stop to occur

    def drive_callback(self, msg):
        self.drive_command = msg
        self.get_logger().debug("Received new drive command")
        self.evaluate_safety()

    def scan_callback(self, msg):
        if (self.scan_cos_angles is None) or (self.scan_sin_angles is None): # Lazy initialization
            angles = np.linspace(msg.angle_min, msg.angle_max, num=np.array(msg.ranges).shape[0])
            self.scan_cos_angles = np.cos(angles)
            self.scan_sin_angles = np.sin(angles)
        
        self.scan_data = msg
        self.get_logger().debug("Received new scan data")
        self.evaluate_safety()

    def evaluate_safety(self):
        if self.scan_data is None or self.drive_command is None:
            return

        velocity = self.drive_command.drive.speed
        if velocity < 0.001:
            return

        # Kinematic Threshold
        front_threshold = self.MARGIN + (velocity ** 2) / (2 * self.MAX_DECELERATION)
        delta = self.drive_command.drive.steering_angle
        if self.VISUALIZE:
            self.publish_safety_marker(delta, front_threshold)

        # Get Cartesian points
        ranges = np.array(self.scan_data.ranges)
        px = ranges * self.scan_cos_angles
        py = ranges * self.scan_sin_angles

        # Publish distance from front bumper
        dist_x = px - self.LIDAR_OFFSET
        mask = (np.abs(py) < self.CAR_WIDTH/2) & (dist_x > 0)
        if np.any(mask):
            distance = Float32()
            distance.data = float(np.median(dist_x[mask])) 
            self.distance_pub.publish(distance)

        # Adjust for LIDAR offset (Move points to car's front bumper frame)
        # We check if points are within 'front_threshold' OF THE BUMPER
        collision_zone_start = self.LIDAR_OFFSET
        collision_zone_end = self.LIDAR_OFFSET + front_threshold

        self.is_collision = False
        
        if abs(delta) < 0.01: # Straight Path
            # Points must be ahead of bumper AND within threshold
            in_path = (px > collision_zone_start) & \
                    (px < collision_zone_end) & \
                    (np.abs(py) < self.CAR_WIDTH / 2)
            self.is_collision = np.any(in_path)
        else: # Curved Path (Bicycle Model)
            R = self.WHEELBASE / np.tan(delta) # Radius of car rotation, max and min account for front and back corner
            R_max = np.sqrt((self.WHEELBASE + self.LIDAR_OFFSET)**2 + (abs(R) + self.CAR_WIDTH/2)**2)
            R_min = abs(R) - self.CAR_WIDTH/2

            # Points in car center of rotation (cor) frame
            px_cor = px + self.WHEELBASE
            py_cor = py - R
            pr = np.sqrt((px_cor)**2 + (py_cor)**2)
            pangle = np.mod(np.arctan2(px_cor, py_cor * -np.sign(R)), 2 * np.pi)
            
            # Angle that bumper is at in point's cor frame
            bumper_angle = np.arcsin(np.clip((self.WHEELBASE + self.LIDAR_OFFSET) / pr, -1.0, 1.0))
            p_bumper_ahead_dist = (pangle - bumper_angle) * pr 
            
            # Obstacle must be within the car's curved path
            # Obstacle must be past the bumper but within stopping distance
            in_path = (pr > R_min) & (pr < R_max) & \
                      (p_bumper_ahead_dist > 0) & (p_bumper_ahead_dist < front_threshold)
            
            self.is_collision = np.any(in_path)
        
        # Ignore salt and pepper LiDAR noise
        if self.is_collision:
            self.collision_counter += 1
        else:
            self.collision_counter = 0

        if self.collision_counter > self.DETECTION_THRESHOLD:
            safe_command = AckermannDriveStamped()
            safe_command.header.stamp = self.get_clock().now().to_msg()
            safe_command.drive.speed = 0.0
            safe_command.drive.steering_angle = 0.0
            self.drive_publisher.publish(safe_command)
            self.get_logger().warn(
                "Frontal object detected! Stopping the robot."
            )

    def create_point(self, x, y):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p

    def publish_safety_marker(self, delta, stop_dist):
        marker = Marker()
        marker.header.frame_id = "base_link" # Set to rear axle frame
        marker.ns = "safety_zone"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Line thickness
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6  # Semi-transparent red
        
        num_steps = 20
        
        if abs(delta) < 0.01:
            # --- Straight Case: Two Parallel Lines ---
            y_inner = -self.CAR_WIDTH / 2
            y_outer = self.CAR_WIDTH / 2
            x_start = self.LIDAR_OFFSET 
            x_end = x_start + stop_dist
            
            # Left Boundary
            marker.points.append(self.create_point(x_start, y_outer))
            marker.points.append(self.create_point(x_end, y_outer))
            # Right Boundary
            marker.points.append(self.create_point(x_start, y_inner))
            marker.points.append(self.create_point(x_end, y_inner))
            # Front Bumper Crossbar
            marker.points.append(self.create_point(x_start, y_inner))
            marker.points.append(self.create_point(x_start, y_outer))
            
        else:
            # --- Curved Case: Swept Path (Bicycle Model) ---
            R = self.WHEELBASE / np.tan(delta)
            R_min = abs(R) - self.CAR_WIDTH / 2
            R_max = np.sqrt((self.WHEELBASE + self.LIDAR_OFFSET)**2 + (abs(R) + self.CAR_WIDTH / 2)**2)
            
            # Helper to calculate points along an arc for a specific radius
            def get_arc_point(radius, current_arc_dist, theta_start):
                # Calculate the angle where the front bumper (x=L) starts for this radius
                theta = theta_start + (current_arc_dist / radius)
                
                x = radius * np.sin(theta)
                # Center is at (0, R). Handle Left vs Right turn Y-offsets.
                if R > 0: # Left Turn
                    y = R - radius * np.cos(theta)
                else:     # Right Turn
                    y = R + radius * np.cos(theta)
                return self.create_point(x, y)


            theta_start_min = np.arcsin(np.clip((self.WHEELBASE + self.LIDAR_OFFSET) / R_min, -1.0, 1.0))
            theta_start_max = np.arcsin(np.clip((self.WHEELBASE + self.LIDAR_OFFSET) / R_max, -1.0, 1.0))
            # Generate segments for both inner and outer boundaries
            for i in range(num_steps):
                d1 = (i / num_steps) * stop_dist
                d2 = ((i + 1) / num_steps) * stop_dist
                
                # Outer Swept Path Segment (The wide swing)
                marker.points.append(get_arc_point(R_max, d1, theta_start_max))
                marker.points.append(get_arc_point(R_max, d2, theta_start_max))
                
                # Inner Swept Path Segment (The tight turn)
                marker.points.append(get_arc_point(R_min, d1, theta_start_min))
                marker.points.append(get_arc_point(R_min, d2, theta_start_min))
            
            # Draw the front bumper "line" to show where the zone starts
            marker.points.append(get_arc_point(R_max, 0.0, theta_start_max))
            marker.points.append(get_arc_point(R_min, 0.0, theta_start_min))

        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
