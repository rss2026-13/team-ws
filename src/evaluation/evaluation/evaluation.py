# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")
        self.max_buffer_size = 1000  # limit history for rolling stats
        self.past_angles = []
        self.past_distances = []
        self.publisher_ = self.create_publisher(Float32, "random_float_log", 10)
        self.distance_subscription = self.create_subscription(
            Float32, "distance", self.distance_callback, 10
        )
        self.angle_subscription = self.create_subscription(
            Float32, "angle", self.angle_callback, 10
        )
        self.test_end_subscription = self.create_subscription(
            Int32, "test_end", self.test_end_callback, 10
        )
        # Distance metrics (for graphs / oscillations analysis)
        self.average_distance = self.create_publisher(Float32, "average_distance", 10)
        self.squared_distance = self.create_publisher(Float32, "squared_distance", 10)
        self.std_distance = self.create_publisher(Float32, "std_distance", 10)
        self.ste_distance = self.create_publisher(Float32, "ste_distance", 10)
        # Angle relative to wall metrics
        self.average_angle = self.create_publisher(Float32, "average_angle", 10)
        self.squared_angle = self.create_publisher(Float32, "squared_angle", 10)
        self.last_average_distance = None
        self.last_squared_distance = None
        self.last_std_distance = None
        self.last_ste_distance = None
        self.last_average_angle = None
        self.last_squared_angle = None

    def distance_callback(self, msg):
        self.past_distances.append(float(msg.data))
        if len(self.past_distances) > self.max_buffer_size:
            self.past_distances.pop(0)

        arr = np.array(self.past_distances)
        n = len(arr)
        if n == 0:
            return

        # Average distance
        avg_msg = Float32()
        avg_msg.data = float(np.mean(arr))
        self.last_average_distance = avg_msg.data
        self.average_distance.publish(avg_msg)

        # Squared distance (mean of squared distances)
        sq_msg = Float32()
        sq_msg.data = float(np.mean(arr**2))
        self.last_squared_distance = sq_msg.data
        self.squared_distance.publish(sq_msg)

        # Standard deviation of distance (spread; useful for oscillations)
        std_msg = Float32()
        std_msg.data = float(np.std(arr)) if n > 1 else 0.0
        self.last_std_distance = std_msg.data
        self.std_distance.publish(std_msg)

        # Standard error of distance (std / sqrt(n))
        ste_msg = Float32()
        ste_msg.data = float(np.std(arr) / math.sqrt(n)) if n > 1 else 0.0
        self.last_ste_distance = ste_msg.data
        self.ste_distance.publish(ste_msg)

    def angle_callback(self, msg):
        self.past_angles.append(float(msg.data))
        if len(self.past_angles) > self.max_buffer_size:
            self.past_angles.pop(0)

        arr = np.array(self.past_angles)
        n = len(arr)
        if n == 0:
            return

        # Angle relative to wall average
        avg_msg = Float32()
        avg_msg.data = float(np.mean(arr))
        self.last_average_angle = avg_msg.data
        self.average_angle.publish(avg_msg)

        # Angle relative to wall squared (mean of squared angles)
        sq_msg = Float32()
        sq_msg.data = float(np.mean(arr**2))
        self.last_squared_angle = sq_msg.data
        self.squared_angle.publish(sq_msg)

    def test_end_callback(self, msg):
        # Reset history when test ends
        self.past_angles.clear()
        self.past_distances.clear()
        if msg.data == 1:
            return
        self.get_logger().info("Test ended, cleared history of angles and distances.")
        self.get_logger().info(
            f"{self.last_average_distance:.3f}, "
            f"{self.last_squared_distance:.3f}, "
            f"{self.last_std_distance:.3f}, "
            f"{self.last_ste_distance:.3f}, "
            f"{self.last_average_angle:.3f}, "
            f"{self.last_squared_angle:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = SimpleSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
