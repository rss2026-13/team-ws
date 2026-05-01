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
            Float32MultiArray, '/lane_lines', self.listener_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/high_level/input/nav_1', 10)

        # --- Tuning parameters ---
        self.target_v = 2.5         # m/s

        # Kp acts on normalised lateral error  (pixels / img_half_width)
        # Kd acts on heading error in radians, dampens oscillations on curves
        # will likely need to tune for such high speeds
        self.Kp = 0.04
        self.Kd = 0.00
        self.clockwise = True

        self.img_w = 640
        self.img_center = self.img_w / 2.0   # 320 px

        # Maximum steering angle your car supports (radians)
        self.max_steer = 0.4

        self.prev_error=0.0

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def line_angle(x1, y1, x2, y2):
        """
        Angle of a line segment relative to the vertical image axis.
        Image coordinates: y increases downward, so a straight-ahead lane
        line is vertical (angle = 0).  A left-curving lane tilts the top
        of the line to the left (negative u), giving a positive angle here.
        Returns angle in radians in (-pi/2, pi/2).
        """
        dx = x2 - x1   # positive = top of line is to the right
        dy = y1 - y2   # positive because image y increases downward
        if abs(dy) < 1e-6:
            return 0.0
        return np.arctan2(dx, dy)   # angle from vertical

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def listener_callback(self, msg):
        lines = msg.data
        if len(lines) < 8:
            return

        lx1, ly1, lx2, ly2 = lines[0:4]
        rx1, ry1, rx2, ry2 = lines[4:8]

        left_valid  = (lx1 != -1.0)
        right_valid = (rx1 != -1.0)

        if not left_valid and not right_valid:
            #self._publish(0.0, 0.0)   # no lanes visible – stop
            return

        # ---- 1. Lateral error (distance of lane centre from image centre) ----
        if left_valid and right_valid:
            # Use the near (bottom) points for lateral position
            lane_center_px = (lx1 + rx1) / 2.0
        elif left_valid:
            # Estimate lane centre assuming standard half-width (~150 px)
            lane_center_px = lx1 + 700.0
        else:
            lane_center_px = rx1 - 640.0

        # Normalise: -1 (far left) … +1 (far right)
        lateral_error = (self.img_center - lane_center_px) / self.img_center
        d_error=(lateral_error-self.prev_error)/1.0
        self.prev_error=lateral_error
        
        # ---- 2. Heading error (lane angle relative to image vertical) ----
        angles = []
        if left_valid:
            angles.append(self.line_angle(lx1, ly1, lx2, ly2))
        if right_valid:
            angles.append(self.line_angle(rx1, ry1, rx2, ry2))
        heading_error = float(np.mean(angles))

        # ---- 3. PD steering (mirrors wall-follower structure) ----
        #   Kp term corrects lateral position (like dist_error in wall follower)
        #   Kd term corrects heading           (like wall_angle in wall follower)
        sign = lateral_error/abs(lateral_error) if lateral_error else 0
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
