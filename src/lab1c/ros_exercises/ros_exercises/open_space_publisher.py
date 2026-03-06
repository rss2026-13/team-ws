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
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace


import random
import math


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.publisher_ = self.create_publisher(OpenSpace, 'open_space', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        distance_msg = max(msg.ranges)
        angle_msg = msg.ranges.index(distance_msg) * msg.angle_increment + msg.angle_min
        
        new_msg = OpenSpace()
        new_msg.angle = angle_msg
        new_msg.distance = distance_msg
        self.publisher_.publish(new_msg)
        self.get_logger().info('Custom Data Type: "%s"' % new_msg.angle)
        self.get_logger().info('Custom Data Type: "%s"' % new_msg.distance)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = OpenSpacePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
