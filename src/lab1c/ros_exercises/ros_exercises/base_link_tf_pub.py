import rclpy
from rclpy.node import Node

import numpy as np

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

from scipy.spatial.transform import Rotation as R

class BaseLinkTwoPublisher(Node):
    def __init__(self):
        super().__init__('base_link_tf_pub')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.T_left_base=np.array([[1,0,0,0],
                              [0,1,0,-0.05],
                              [0,0,1,0],
                              [0,0,0,1]])

        self.timer = self.create_timer(1/20.0, self.publish_base_link_2)

    def publish_base_link_2(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'left_cam', now)
            
            T_odom_left = np.eye(4)
            q = trans.transform.rotation
            T_odom_left[0:3, 0:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T_odom_left[0:3, 3] = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]

            T_odom_baselink2 =T_odom_left@self.T_left_base

            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id ='odom'
            msg.child_frame_id='base_link_2'
            
            msg.transform.translation.x=float(T_odom_baselink2[0,3])
            msg.transform.translation.y =float(T_odom_baselink2[1,3])
            msg.transform.translation.z=float(T_odom_baselink2[2,3])
            
            quat = R.from_matrix(T_odom_baselink2[0:3, 0:3]).as_quat()
            msg.transform.rotation.x = quat[0]
            msg.transform.rotation.y = quat[1]
            msg.transform.rotation.z = quat[2]
            msg.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(msg)

        except TransformException:
            return

def main():
    rclpy.init()
    node = BaseLinkTwoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()