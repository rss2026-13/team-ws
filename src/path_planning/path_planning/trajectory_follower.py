import rclpy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, String, Bool
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

        self.lookahead = 0

        self.declare_parameter("base_lookahead", 0.75)
        self.base_lookahead = self.get_parameter("base_lookahead").get_parameter_value().double_value
        
        self.declare_parameter("k_curv", 1.0)
        self.k_curv = self.get_parameter("k_curv").get_parameter_value().double_value

        self.declare_parameter("velocity", 0.5)
        self.velocity = self.get_parameter("velocity").get_parameter_value().double_value

        self.wheelbase_length = 0.325

        self.declare_parameter("goal_threshold", 0.5)
        self.goal_threshold = self.get_parameter("goal_threshold").get_parameter_value().double_value

        self.declare_parameter("n_segments_to_weight", 3)
        self.n_segments_to_weight = self.get_parameter("n_segments_to_weight").get_parameter_value().integer_value

        self.declare_parameter("weight_decay", 0.5)
        self.weight_decay = self.get_parameter("weight_decay").get_parameter_value().double_value

        self.starting_points = 0
        self.ending_points  = 0 
        self.segments = 0
        self.segments_mag_sq = 0
        self.inv_segments_mag_sq = 0
        self.normalized_segments = 0

        self.initialized_traj = False
        self.trajectory = LineTrajectory(self, "/followed_trajectory")
        self.robot_state = "MCL_INITIALIZATION"
        self.goal_already_reached = False

        self.pose_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.robot_state_sub = self.create_subscription(
            String,
            "/robot_state",
            self.robot_state_cb,
            10
        )
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        self.cte_pub = self.create_publisher(
            Float32,
            "/trajectory/cte",
            10 
        )
        self.heading_error_pub = self.create_publisher(
            Float32,
            "/trajectory/heading_error",
            10 
        )
        self.lookahead_pub = self.create_publisher(
            Float32,
            "/trajectory/lookahead",
            10 
        )
        self.steering_angle_pub = self.create_publisher(
            Float32,
            "/trajectory/steering_angle",
            10 
        )
        self.goal_reached_pub = self.create_publisher(
            Bool,
            "/goal_reached",
            10
        )
 
    def robot_state_cb(self, msg: String):
        new_state = msg.data
        # If we just transitioned INTO a Navigating state, 
        # we must allow goal_reached to be published again.
        if "NAVIGATING" in new_state and self.robot_state != new_state:
            self.goal_already_reached = False
            self.initialized_traj = False  # blocks pose_callback until new trajectory arrives
            self.goal_reached_pub.publish(Bool(data = False))
            # self.get_logger().info(f"Navigating to new goal: Reseting goal_reached latch.")
        
        self.robot_state = new_state

    def pose_callback(self, odometry_msg):
        """
        Need to add visualization for:
        1. lookahead point
        2. steering angle
        3. closest point on segment to car
        """
        if not self.initialized_traj or ("NAVIGATING" not in self.robot_state):
            return

        car_position = np.array([odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y])
        car_theta = 2.0 * np.arctan2(odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w)

        if len(self.trajectory.points) == 1:
            lookahead_point = self.trajectory.points[0]
            self.lookahead = np.linalg.norm(lookahead_point -  car_position)
            if not self.goal_already_reached and (self.lookahead < self.goal_threshold):
                self.pub_pure_pursuit_drive_msg(lookahead_point, 0.0, car_position, car_theta)
                self.goal_reached_pub.publish(Bool(data = True))
                self.goal_already_reached = True
            else:
                self.pub_pure_pursuit_drive_msg(lookahead_point, self.velocity, car_position, car_theta)
            return

        # Compute index of line segment closest to car
        car_to_starting = self.starting_points - car_position
        t = -np.einsum("ij,ij,i->i", self.segments, car_to_starting, self.inv_segments_mag_sq)
        t = np.clip(t, 0.0, 1.0)
        projections = self.starting_points + t[:, np.newaxis] * self.segments
        car_to_segment = projections - car_position
        distance_to_segments = np.linalg.norm(car_to_segment, axis=1)
        min_dist_idx = np.argmin(distance_to_segments)

        # Publish cross track error
        cte_float = Float32()
        cte_float.data = distance_to_segments[min_dist_idx]
        self.cte_pub.publish(cte_float)

        # Publish heading error
        segment_angle = np.arctan2(self.segments[min_dist_idx][1], self.segments[min_dist_idx][0])
        heading_error = car_theta - segment_angle
        # Wrap to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        heading_error_float = Float32()
        heading_error_float.data = heading_error
        self.heading_error_pub.publish(heading_error_float)

        # Compute the lookahead distance
        self.compute_lookahead_distance(min_dist_idx, self.n_segments_to_weight, self.weight_decay)

        # Publish lookahead distance
        lookahead_float = Float32()
        lookahead_float.data = self.lookahead
        self.lookahead_pub.publish(lookahead_float)

        # Compute the lookahead point
        lookahead_point = self.find_lookahead_point(car_to_starting, min_dist_idx, t[min_dist_idx])

        # Give pure pursuit drive command
        if not self.goal_already_reached and (np.allclose(lookahead_point, self.ending_points[-1])) and (np.linalg.norm(self.ending_points[-1] - car_position) < self.goal_threshold):
            self.pub_pure_pursuit_drive_msg(lookahead_point, 0.0, car_position, car_theta)
            self.goal_reached_pub.publish(Bool(data = True))
            self.goal_already_reached = True
        else:
            self.pub_pure_pursuit_drive_msg(lookahead_point, self.velocity, car_position, car_theta)


    def compute_lookahead_distance(self, start_idx, n_segments, decay):
        end_idx = min(start_idx + n_segments, len(self.segments) - 1)

        if end_idx <= start_idx:
            self.lookahead = self.base_lookahead
            return

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
    

    def pub_pure_pursuit_drive_msg(self, lookahead_point, speed, car_position, car_theta):
        # self.get_logger().info("Trajectory follower pubing drive")
        dx = lookahead_point[0] - car_position[0]
        dy = lookahead_point[1] - car_position[1]

        car_lpx = np.cos(car_theta)*dx + np.sin(car_theta)*dy
        car_lpy = -np.sin(car_theta)*dx + np.cos(car_theta)*dy

        ref_angle = np.arctan2(car_lpy, car_lpx)

        lookahead = self.lookahead if self.lookahead > 0 else self.base_lookahead
        steering_angle = np.arctan2(2 * self.wheelbase_length * np.sin(ref_angle), lookahead)

        # Publish steering angle
        steering_angle_float = Float32()
        steering_angle_float.data = steering_angle
        self.steering_angle_pub.publish(steering_angle_float)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


    def trajectory_callback(self, msg):
        #self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty trajectory.")
            return

        # 1. Convert all poses to a (N, 2) NumPy array immediately
        new_points = np.array([[p.position.x, p.position.y] for p in msg.poses])

        if len(new_points) > 1:
            # 2. Vectorized check: Find where the difference between consecutive points is non-zero
            # np.diff computes out[i] = a[i+1] - a[i]
            diffs = np.diff(new_points, axis=0)
            
            # A point is "kept" if the distance to the next point is greater than a tiny epsilon
            # We always keep the first point, then check the diffs
            mask = np.linalg.norm(diffs, axis=1) > 1e-5
            
            # Combine: always keep the first point + any point that isn't a duplicate of the previous
            # We prepend 'True' because np.diff returns N-1 elements
            keep_mask = np.concatenate(([True], mask))
            filtered_points = new_points[keep_mask]
        else:
            filtered_points = new_points

        # 3. Update the trajectory object with the cleaned data
        self.trajectory.clear()
        self.trajectory.points = [tuple(p) for p in filtered_points]
        self.trajectory.update_distances() 
        self.trajectory.mark_dirty()

        self.trajectory.publish_viz(duration=0.0)

        if len(self.trajectory.points) <= 1:
            return

        self.starting_points = np.array(self.trajectory.points[:-1]) # (N, 2) list of tuples
        self.ending_points = np.array(self.trajectory.points[1:]) # (N, 2) list of tuples

        self.segments = self.ending_points - self.starting_points
        self.segments_mag_sq = np.sum(self.segments**2, axis=1)
        self.inv_segments_mag_sq = 1.0 / np.where(self.segments_mag_sq > 0, self.segments_mag_sq, 1.0)
        self.normalized_segments = self.segments * np.sqrt(self.inv_segments_mag_sq[:, np.newaxis])

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
