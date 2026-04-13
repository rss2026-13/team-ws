import rclpy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from .utils import LineTrajectory
import numpy as np


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 0  # FILL IN #
        self.base_lookahead = 0.75
        self.k_curv = 1.0
        self.speed = 4.0  # FILL IN #
        self.wheelbase_length = 0.325  # FILL IN #

        self.starting_points = 0
        self.ending_points  = 0 
        self.segments = 0
        self.segments_mag_sq = 0
        self.inv_segments_mag_sq = 0
        self.normalized_segments = 0

        self.initialized_traj = False
        self.trajectory = LineTrajectory(self, "/followed_trajectory")

        self.pose_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
 

    def pose_callback(self, odometry_msg):
        """
        Need to add evaluation for:
        1. Cross track Error (distance_to_segments[min_dist_idx])
        2. Heading Error (car_theta - closest segment angle)
        3. Lookahead distance over time / Curvature over time
        4. Steering angle over time

        Need to add visualization for:
        1. lookahead point
        2. steering angle
        3. closest point on segment to car
        """
        if not self.initialized_traj:
            return

        car_position = np.array([odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y])
        car_theta = 2.0 * np.arctan2(odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w)

        # Compute index of line segment closest to car
        car_to_starting = self.starting_points - car_position
        t = -np.einsum("ij,ij,i->i", self.segments, car_to_starting, self.inv_segments_mag_sq)
        t = np.clip(t, 0.0, 1.0)
        projections = self.starting_points + t[:, np.newaxis] * self.segments
        distance_to_segments = np.linalg.norm(projections - car_position, axis=1)
        min_dist_idx = np.argmin(distance_to_segments)

        # Compute the lookahead distance
        self.calculate_lookahead_distance(min_dist_idx)

        # Compute the lookahead point
        lookahead_point = self.find_lookahead_point(car_to_starting, min_dist_idx, t[min_dist_idx])

        # Give pure pursuit drive command
        self.pub_pure_pursuit_drive_msg(lookahead_point, car_position, car_theta)


    def calculate_lookahead_distance(self, start_idx, n_segments=3, decay = 0.5):
        end_idx = min(start_idx + n_segments, len(self.segments) - 1)

        cos_angles = np.sum(self.normalized_segments[start_idx:end_idx] * self.normalized_segments[start_idx + 1 : end_idx + 1], axis=1)
        angles = np.arccos(np.clip(cos_angles, -1.0, 1.0))

        weights = decay ** np.arange(len(angles))
        total_weight = np.sum(weights)
        weighted_curvature = np.sum(angles * weights) / total_weight if total_weight > 0 else 0.0

        self.lookahead = self.base_lookahead / (1 + self.k_curv * weighted_curvature)
    

    def find_lookahead_point(self, car_to_starting, min_dist_idx, t_on_closest):
        t_continuous = min_dist_idx + t_on_closest

        if self.trajectory.distance_to_end(t_continuous) < self.lookahead:
            return self.ending_points[-1]
        
        for i in range(min_dist_idx, len(self.starting_points)):
            a = self.segments_mag_sq[i]
            b = 2 * self.segments[i] @ car_to_starting[i]
            c = (car_to_starting[i] @ car_to_starting[i]) - self.lookahead**2

            discriminant = b**2 - (4 * a * c)

            if discriminant < 0:
                continue

            discriminant_sqrt = np.sqrt(discriminant)
            t_plus = (-b + discriminant_sqrt) / (2 * a)

            t_min = t_on_closest if i == min_dist_idx else 0.0

            if t_min <= t_plus <= 1.0:
                return self.starting_points[i] + (t_plus * self.segments[i])
        
        return self.starting_points[min_dist_idx] + (t_on_closest * self.segments[min_dist_idx])
    

    def pub_pure_pursuit_drive_msg(self, lookahead_point, car_position, car_theta):
        dx = lookahead_point[0] - car_position[0]
        dy = lookahead_point[1] - car_position[1]

        car_lpx = np.cos(car_theta)*dx + np.sin(car_theta)*dy
        car_lpy = -np.sin(car_theta)*dx + np.cos(car_theta)*dy

        ref_angle = np.arctan2(car_lpy, car_lpx)
        steering_angle = np.arctan2(2 * self.wheelbase_length * np.sin(ref_angle), self.lookahead)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.speed
        self.drive_pub.publish(drive_msg)


    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.starting_points = np.array(self.trajectory.points[:-1]) # (N, 2) list of tuples
        self.ending_points = np.array(self.trajectory.points[1:]) # (N, 2) list of tuples
        self.segments = self.ending_points - self.starting_points
        self.segments_mag_sq = np.sum(self.segments**2, axis=1)
        self.inv_segments_mag_sq = 1.0 / np.where(self.segments_mag_sq > 0, self.segments_mag_sq, 1.0)
        self.normalized_segments = self.segments / np.sqrt(self.segments_mag_sq[:, np.newaxis])

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()