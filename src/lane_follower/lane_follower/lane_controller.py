import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped


class SimulationController(Node):
    """
    Lane-following controller that mirrors the structure of 
    wall-follower PD controller.

    It extracts two signals from the lane lines, just as the wall follower
    used both distance and angle to the wall:

      1. lateral_error  – how far the car is from the lane centre (pixels)
      2. heading_error  – angle between the car's heading and the lane direction (radians)

    Steering = Kp * lateral_error + Kd * heading_error
    """

    def __init__(self):
        super().__init__('sim_controller')

        self.subscription = self.create_subscription(
            Float32MultiArray, '/lane_lines_transformed', self.listener_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/high_level/input/nav_1', 10)

        # --- Tuning parameters ---
        self.target_v = 1.0         # m/s

        # Kp acts on lateral error in meters
        # Kd damps oscillations based on rate of change of lateral error
        self.Kp = 0.3
        self.Kd = 0.0
        self.clockwise = True

        self.lane_width = 0.85  # meters

        # Maximum steering angle your car supports (radians)
        self.max_steer = 0.4

        self.prev_error = 0.0

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def line_angle(x1, y1, x2, y2):
        """
        Angle of a line segment relative to the car's forward (x) axis.
        Ground-plane coords: x=forward, y=left.
        A perfectly straight lane line has dy≈0, so angle≈0.
        Returns angle in radians in (-pi/2, pi/2).
        """
        dx = x2 - x1
        dy = y2 - y1
        if abs(dx) < 1e-6:
            return 0.0
        return np.arctan2(dy, dx)   # angle from forward axis

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def listener_callback(self, msg):
        lines = msg.data
        if len(lines) < 8:
            return

        # Ground-plane coords: x=forward (meters), y=left (meters)
        lx1, ly1, lx2, ly2 = lines[0:4]
        rx1, ry1, rx2, ry2 = lines[4:8]

        left_valid  = (lx1 != -1.0)
        right_valid = (rx1 != -1.0)

        if not left_valid and not right_valid:
            return

        # ---- 1. Guess missing line using lane width, then compute lane centre ----
        if left_valid and not right_valid:
            rx1, ry1 = lx1, ly1 - self.lane_width
            rx2, ry2 = lx2, ly2 - self.lane_width
        elif right_valid and not left_valid:
            lx1, ly1 = rx1, ry1 + self.lane_width
            lx2, ly2 = rx2, ry2 + self.lane_width

        lane_center_y = (ly1 + ry1) / 2.0
        lateral_error = -lane_center_y  # positive = car is right of centre
        d_error = (lateral_error - self.prev_error) / 1.0
        self.prev_error = lateral_error

        # ---- 2. Heading error (lane angle relative to car forward axis) ----
        angles = [
            self.line_angle(lx1, ly1, lx2, ly2),
            self.line_angle(rx1, ry1, rx2, ry2),
        ]
        heading_error = float(np.mean(angles))

        # ---- 3. PD steering ----
        sign = lateral_error / abs(lateral_error) if lateral_error else 0
        steering_angle = self.Kp * abs(lateral_error) ** 1.1 * sign + self.Kd * d_error

        # Clamp to physical limits
        steering_angle = float(np.clip(steering_angle, -self.max_steer, self.max_steer))

        self._publish(self.target_v, steering_angle)

        self.get_logger().debug(
            f'lat_err={lateral_error:.3f}  head_err={heading_error:.3f}  '
            f'steer={steering_angle:.3f}')

    # ------------------------------------------------------------------

    def _publish(self, speed, steer):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
