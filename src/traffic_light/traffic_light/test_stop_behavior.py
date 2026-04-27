#!/usr/bin/env python3
"""
test_stop_behavior.py

Self-contained ROS2 test for traffic light stop behavior.

This node drives the car directly by publishing to /drive, simulating
what the traffic light node would output. It does NOT require the
traffic light node to be running.

When red=True:  drives forward, then stops when within stop_distance of the light
When red=False: drives forward continuously

Usage:
    # Test red light → car should stop within 1m of light
    ros2 run <your_pkg> test_stop_behavior --red true --tl-x 5.0 --tl-y 0.0

    # Test green light → car should keep moving
    ros2 run <your_pkg> test_stop_behavior --red false --tl-x 5.0 --tl-y 0.0

    # Custom tolerance and speed
    ros2 run <your_pkg> test_stop_behavior --red true --tl-x 5.0 --tolerance 1.0 --speed 1.5
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class StopBehaviorTester(Node):
    def __init__(self, red_light_on: bool, tl_x: float, tl_y: float,
                 stop_tolerance: float, test_duration: float, drive_speed: float):
        super().__init__("stop_behavior_tester")

        self.red_light_on = red_light_on
        self.tl_x = tl_x
        self.tl_y = tl_y
        self.stop_tolerance = stop_tolerance
        self.test_duration = test_duration
        self.drive_speed = drive_speed

        self.car_x = 0.0
        self.car_y = 0.0
        self.received_odom = False
        self.drive_cmds_sent = []
        self.start_time = time.time()
        self.test_done = False

        # Publish directly to /drive so the simulator sees it immediately
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # 20 Hz drive loop
        self.cmd_timer = self.create_timer(0.05, self.drive_loop)
        # End test after duration
        self.eval_timer = self.create_timer(test_duration, self.evaluate_test)

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  Stop Behavior Test Starting\n"
            f"  Red light:       {self.red_light_on}\n"
            f"  Light position:  ({tl_x}, {tl_y})\n"
            f"  Stop tolerance:  {stop_tolerance} m\n"
            f"  Drive speed:     {drive_speed} m/s\n"
            f"  Test duration:   {test_duration} s\n"
            f"{'='*60}"
        )

    def odom_callback(self, msg: Odometry):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.received_odom = True

    def drive_loop(self):
        """
        Core logic: mirrors exactly what traffic_light_node would output.
        If red and within stop_distance → send speed=0, else send full speed.
        """
        if self.test_done:
            return

        distance = self._distance_to_light()
        should_stop = self.red_light_on and distance <= self.stop_tolerance

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = 0.0
        cmd.drive.speed = 0.0 if should_stop else self.drive_speed

        self.drive_pub.publish(cmd)
        self.drive_cmds_sent.append((time.time() - self.start_time, cmd.drive.speed, distance))

        # Live status log every ~1 second (every 20 ticks at 20 Hz)
        if len(self.drive_cmds_sent) % 20 == 0:
            status = "STOPPED" if should_stop else f"DRIVING @ {self.drive_speed} m/s"
            self.get_logger().info(
                f"  t={time.time()-self.start_time:.1f}s | "
                f"pos=({self.car_x:.2f}, {self.car_y:.2f}) | "
                f"dist_to_light={distance:.2f}m | {status}"
            )

    def evaluate_test(self):
        """Called once after test_duration. Prints PASS / FAIL and shuts down."""
        self.eval_timer.cancel()
        self.cmd_timer.cancel()
        self.test_done = True

        # Send a final stop command so the car doesn't keep rolling
        stop = AckermannDriveStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.drive.speed = 0.0
        self.drive_pub.publish(stop)

        distance = self._distance_to_light()
        final_speeds = [spd for _, spd, _ in self.drive_cmds_sent[-20:]]
        avg_final_speed = sum(final_speeds) / len(final_speeds) if final_speeds else 0.0

        print(f"\n{'='*60}")
        print(f"  TEST RESULTS")
        print(f"{'='*60}")
        print(f"  Red light:           {self.red_light_on}")
        print(f"  Final position:      ({self.car_x:.3f}, {self.car_y:.3f})")
        print(f"  Distance to light:   {distance:.3f} m")
        print(f"  Avg final speed:     {avg_final_speed:.3f} m/s")
        print(f"  Stop tolerance:      {self.stop_tolerance} m")

        if self.red_light_on:
            stopped = avg_final_speed < 0.05
            within_range = distance <= self.stop_tolerance
            passed = stopped and within_range
            print(f"\n  Expected: STOP within {self.stop_tolerance} m of light")
            print(f"  Car stopped:         {'✓ YES' if stopped else '✗ NO  (still moving!)'}")
            print(f"  Within tolerance:    {'✓ YES' if within_range else f'✗ NO  ({distance:.2f}m > {self.stop_tolerance}m)'}")
        else:
            still_moving = avg_final_speed > 0.05
            passed = still_moving
            print(f"\n  Expected: KEEP DRIVING (no red light)")
            print(f"  Still moving:        {'✓ YES' if still_moving else '✗ NO  (car stopped unexpectedly!)'}")

        result = "✅ PASS" if passed else "❌ FAIL"
        print(f"\n  RESULT: {result}")
        print(f"{'='*60}\n")

        time.sleep(0.5)
        rclpy.shutdown()

    def _distance_to_light(self) -> float:
        dx = self.car_x - self.tl_x
        dy = self.car_y - self.tl_y
        return math.sqrt(dx * dx + dy * dy)


def main():
    parser = argparse.ArgumentParser(
        description="Test racecar stop behavior for a traffic light."
    )
    parser.add_argument(
        "--red", type=lambda v: v.lower() in ("true", "1", "yes"),
        default=True,
        help="Simulated red light state: true or false (default: true)"
    )
    parser.add_argument("--tl-x", type=float, default=5.0,
                        help="Traffic light world x (default: 5.0)")
    parser.add_argument("--tl-y", type=float, default=0.0,
                        help="Traffic light world y (default: 0.0)")
    parser.add_argument("--tolerance", type=float, default=1.0,
                        help="Stop distance tolerance in metres (default: 1.0)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Test duration in seconds (default: 10.0)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Drive speed in m/s (default: 1.0)")

    args, _ = parser.parse_known_args()

    rclpy.init()
    tester = StopBehaviorTester(
        red_light_on=args.red,
        tl_x=args.tl_x,
        tl_y=args.tl_y,
        stop_tolerance=args.tolerance,
        test_duration=args.duration,
        drive_speed=args.speed,
    )
    rclpy.spin(tester)


if __name__ == "__main__":
    main()
