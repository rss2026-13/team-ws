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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
import math
import numpy as np


class Evaluation(Node):

    def __init__(self):
        super().__init__('evaluation')
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('desired_angle', 0.0)
        self.desired_distance = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.desired_angle = self.get_parameter('desired_angle').get_parameter_value().double_value

        self.max_buffer_size = 1000  # limit history for rolling stats
        self.past_angles = []
        self.past_distances = []
        self.distance_subscription = self.create_subscription(
            Float32,
            'distance',
            self.distance_callback,
            10)
        self.angle_subscription = self.create_subscription(
            Float32,
            'angle',
            self.angle_callback,
            10)

        # Error metrics: |actual - desired|, then average or average of squared
        self.average_distance = self.create_publisher(Float32, 'average_distance', 10)
        self.squared_distance = self.create_publisher(Float32, 'squared_distance', 10)
        self.std_distance = self.create_publisher(Float32, 'std_distance', 10)
        self.ste_distance = self.create_publisher(Float32, 'ste_distance', 10)
        self.average_angle = self.create_publisher(Float32, 'average_angle', 10)
        self.std_angle = self.create_publisher(Float32, 'std_angle', 10)

    def distance_callback(self, msg):
        self.past_distances.append(float(msg.data))
        if len(self.past_distances) > self.max_buffer_size:
            self.past_distances.pop(0)

        arr = np.array(self.past_distances)
        n = len(arr)
        if n == 0:
            return

        # Error from desired distance: use absolute value, then average or average of squared
        err = np.abs(arr - self.desired_distance)
        err_sq = (arr - self.desired_distance) ** 2

        avg_msg = Float32()
        avg_msg.data = float(np.mean(err))
        self.average_distance.publish(avg_msg)

        sq_msg = Float32()
        sq_msg.data = float(np.mean(err_sq))
        self.squared_distance.publish(sq_msg)

        std_msg = Float32()
        std_msg.data = float(np.std(arr - self.desired_distance)) if n > 1 else 0.0
        self.std_distance.publish(std_msg)

        ste_msg = Float32()
        ste_msg.data = float(np.std(arr - self.desired_distance) / math.sqrt(n)) if n > 1 else 0.0
        self.ste_distance.publish(ste_msg)

    def angle_callback(self, msg):
        self.past_angles.append(float(msg.data))
        if len(self.past_angles) > self.max_buffer_size:
            self.past_angles.pop(0)

        arr = np.array(self.past_angles)
        n = len(arr)
        if n == 0:
            return

        # Error from desired angle (0 = parallel): average |error| and std of error
        err = np.abs(arr - self.desired_angle)
        avg_msg = Float32()
        avg_msg.data = float(np.mean(err))
        self.average_angle.publish(avg_msg)
        std_msg = Float32()
        std_msg.data = float(np.std(arr - self.desired_angle)) if n > 1 else 0.0
        self.std_angle.publish(std_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Evaluation()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
