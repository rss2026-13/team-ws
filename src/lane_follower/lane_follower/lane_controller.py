import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class SimulationController(Node):
    """
    Pure-pursuit lane controller.

    The lane centerline is the segment connecting the midpoints of the near
    and far lane-line endpoints (in base_link ground-plane coords: x=forward,
    y=left, metres).  A lookahead point at distance `lookahead` is found on
    that segment; steering is computed via the pure-pursuit formula:

        δ = arctan(2 * L * sin(α) / ld)

    where α is the angle to the lookahead point and L is the wheelbase.
    """

    def __init__(self):
        super().__init__('sim_controller')

        self.subscription = self.create_subscription(
            Float32MultiArray, '/lane_lines_transformed', self.listener_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/high_level/input/nav_1', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/lane_viz', 10)

        self.target_v = 3.0       # m/s
        self.lookahead = 25      # metres — primary tuning knob
        self.wheelbase = 0.325    # metres (MIT RACECAR)
        self.lane_width = 0.9    # metres
        self.max_steer = 0.3      # radians

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
            return

        # Fill in missing line by offsetting the visible one
        if left_valid and not right_valid:
            rx1, ry1 = lx1, ly1 - self.lane_width
            rx2, ry2 = lx2, ly2 - self.lane_width
        elif right_valid and not left_valid:
            lx1, ly1 = rx1, ry1 + self.lane_width
            lx2, ly2 = rx2, ry2 + self.lane_width

        # Lane centerline: near midpoint C1, far midpoint C2
        cx1, cy1 = (lx1 + rx1) / 2.0, (ly1 + ry1) / 2.0
        cx2, cy2 = (lx2 + rx2) / 2.0, (ly2 + ry2) / 2.0

        gx, gy = self._lookahead_point(cx1, cy1, cx2, cy2)

        # Pure pursuit: α is the angle from the car's heading (x-axis) to the goal
        alpha = np.arctan2(gy, gx)
        ld = np.hypot(gx, gy)
        steering_angle = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), ld)
        steering_angle = float(np.clip(steering_angle, -self.max_steer, self.max_steer))

        self._publish(self.target_v, steering_angle)
        self._publish_viz(lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2,
                          cx1, cy1, cx2, cy2, gx, gy)

        self.get_logger().debug(
            f'goal=({gx:.2f},{gy:.2f})  alpha={np.degrees(alpha):.1f}°  '
            f'steer={np.degrees(steering_angle):.1f}°')

    # ------------------------------------------------------------------

    def _lookahead_point(self, cx1, cy1, cx2, cy2):
        """
        Find the point on segment C1→C2 at distance `self.lookahead` from
        the origin (the car).  Falls back to the far endpoint if the
        lookahead circle doesn't intersect the segment.
        """
        ld = self.lookahead
        dx, dy = cx2 - cx1, cy2 - cy1

        # Quadratic: |C1 + t*(C2-C1)|² = ld²
        a = dx*dx + dy*dy
        b = 2.0 * (cx1*dx + cy1*dy)
        c = cx1*cx1 + cy1*cy1 - ld*ld

        if abs(a) < 1e-9:
            return cx1, cy1

        discriminant = b*b - 4*a*c
        if discriminant < 0:
            # Lookahead circle misses the segment entirely — use far end
            return cx2, cy2

        t1 = (-b + np.sqrt(discriminant)) / (2*a)
        t2 = (-b - np.sqrt(discriminant)) / (2*a)

        # Pick the smallest t ≥ 0 that lies on [0, 1]; prefer the far solution
        candidates = [t for t in (t1, t2) if t >= 0]
        if not candidates:
            return cx2, cy2

        t = min(candidates)
        t = float(np.clip(t, 0.0, 1.0))
        return cx1 + t*dx, cy1 + t*dy

    # ------------------------------------------------------------------

    def _publish_viz(self, lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2,
                     cx1, cy1, cx2, cy2, gx, gy):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        def line_marker(mid, x1, y1, x2, y2, r, g, b):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = now
            m.ns = 'lane_lines'
            m.id = mid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            m.pose.orientation.w = 1.0
            m.points = [
                Point(x=float(x1), y=float(y1), z=0.0),
                Point(x=float(x2), y=float(y2), z=0.0),
            ]
            return m

        def sphere_marker(mid, x, y, r, g, b, size=0.12):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = now
            m.ns = 'lane_lines'
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = size
            m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            m.pose.orientation.w = 1.0
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.0
            return m

        markers.markers.append(line_marker(0, lx1, ly1, lx2, ly2, 1.0, 0.0, 0.0))  # left: red
        markers.markers.append(line_marker(1, rx1, ry1, rx2, ry2, 0.0, 1.0, 0.0))  # right: green

        # Centerline
        markers.markers.append(line_marker(2, cx1, cy1, cx2, cy2, 1.0, 1.0, 0.0))

        # Lookahead point: cyan sphere
        markers.markers.append(sphere_marker(3, gx, gy, 0.0, 1.0, 1.0, size=0.15))

        # Lookahead circle (approximated as a LINE_STRIP ring)
        ring = Marker()
        ring.header.frame_id = 'base_link'
        ring.header.stamp = now
        ring.ns = 'lane_lines'
        ring.id = 4
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.02
        ring.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)
        ring.pose.orientation.w = 1.0
        thetas = np.linspace(0, 2*np.pi, 36)
        for th in thetas:
            ring.points.append(Point(
                x=float(self.lookahead * np.cos(th)),
                y=float(self.lookahead * np.sin(th)),
                z=0.0))
        markers.markers.append(ring)

        self.viz_pub.publish(markers)

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
