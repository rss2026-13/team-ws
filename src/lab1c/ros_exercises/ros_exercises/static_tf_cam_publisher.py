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
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException, StaticTransformBroadcaster


class StaticCamPublisher(Node):

    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')

        #pre-computed transforms
        self.LTransform = np.array([[1, 0, 0, 0], 
                               [0, 1, 0, 0.05], 
                               [0, 0, 1, 0], 
                               [0, 0, 0, 1]])
        self.RTransform = np.array([[1, 0, 0, 0], 
                               [0, 1, 0, -0.05], 
                               [0, 0, 1, 0], 
                               [0, 0, 0, 1]])

        #broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        #left
        left = TransformStamped()
        left.header.stamp = self.get_clock().now().to_msg()
        left.header.frame_id = 'base_link'
        left.child_frame_id = 'left_cam'

        left.transform.translation.x = 0.0
        left.transform.translation.y = 0.05
        left.transform.translation.z = 0.0

        left.transform.rotation.x = 0.0
        left.transform.rotation.y = 0.0
        left.transform.rotation.z = 0.0
        left.transform.rotation.w = 1.0

        #right
        right = TransformStamped()
        right.header.stamp = self.get_clock().now().to_msg()
        right.header.frame_id = 'left_cam'
        right.child_frame_id = 'right_cam'

        right.transform.translation.x = 0.0
        right.transform.translation.y = -0.10
        right.transform.translation.z = 0.0

        right.transform.rotation.x = 0.0
        right.transform.rotation.y = 0.0
        right.transform.rotation.z = 0.0
        right.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([left, right])
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StaticCamPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
