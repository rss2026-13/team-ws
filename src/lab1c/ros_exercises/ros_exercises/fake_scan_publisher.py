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

import random
import math


class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('topic', 'fake_scan')
        self.declare_parameter('angle_min', -2.0/3.0 * math.pi)
        self.declare_parameter('angle_max',  2.0/3.0 * math.pi)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_increment', math.pi/300.0)

        self.publish_rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('topic').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_increment = self.get_parameter('angle_increment').value

        self.publisher_ = self.create_publisher(LaserScan, topic, 10)
        self.publisher1 = self.create_publisher(Float32, 'range_test', 10)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.scan_time = 1.0 / self.publish_rate
        count = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for _ in range(count)]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.header.stamp)
        

        length_msg = Float32()
        length_msg.data = float(len(msg.ranges))
        self.publisher1.publish(length_msg)
        self.get_logger().info('Publishing: "%s"' % length_msg.data)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = FakeScanPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
