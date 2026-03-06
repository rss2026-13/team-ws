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
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException


class DynamicCamPublisher(Node):

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
        
        #listener
        self.target_frame = 'odom'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #broadcaster
        self.tf_broadcasterLeft = TransformBroadcaster(self)
        self.tf_broadcasterRight = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)


    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time()
            )

            baseLinkT = self.toMatrix(t)
            # with respect to base (first go up to base, then go up to odom)
            leftCamT = baseLinkT @ self.LTransform
            # with respect to left camera (convert to base, then undo L to get to L)
            rightCamT = np.linalg.inv(self.LTransform) @ self.RTransform
            
            self.tf_broadcasterLeft.sendTransform(self.toTransformStamped(t, leftCamT, 'odom', 'left_cam'))
            self.tf_broadcasterRight.sendTransform(self.toTransformStamped(t, rightCamT, 'left_cam', 'right_cam'))
            self.get_logger().info(f'Broadcasting Dynamic!')
        except TransformException as ex:
            self.get_logger().info(f'Could not get coordinates')
        


    def toMatrix(self, t):
        quat = np.array([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ])
        rotMatrix = R.from_quat(quat).as_matrix()

        pos = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])

        result = np.eye(4)
        result[:3, :3] = rotMatrix
        result[:3, 3] = pos
        return result

        

    def toTransformStamped(self, t, arr, parent, name):
        msg = TransformStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = name

        msg.transform.translation.x = arr[0, 3]
        msg.transform.translation.y = arr[1, 3]
        msg.transform.translation.z = arr[2, 3]

        quat = R.from_matrix(arr[:3, :3]).as_quat()
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]

        return msg
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DynamicCamPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
