import rclpy
from rclpy.node import Node
import numpy as np

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

from scipy.spatial.transform import Rotation as R

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')
        
        # Initialize the transform broadcaster and listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.T_base_left=np.array([[1,0,0,0],
                              [0,1,0,0.05],
                              [0,0,1,0],
                              [0,0,0,1]])

        self.T_base_right=np.array([[1,0,0,0],
                              [0,1,0,-0.05],
                              [0,0,1,0],
                              [0,0,0,1]])

        self.T_left_right=np.linalg.inv(self.T_base_left)@self.T_base_right
        
        # timer running at 20 Hz
        self.timer = self.create_timer(1/20.0, self.publish_transforms)

    def matrix_to_transform(self, matrix, frame_id, child_frame_id):
        """Helper to convert a 4x4 matrix back to a TransformStamped message."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        
        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])
        
        quat = R.from_matrix(matrix[0:3, 0:3]).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    def get_matrix(self, x, y, z, quat=[0, 0, 0, 1]):
            """Helper to create a 4x4 matrix from translation and quaternion."""
            T = np.eye(4)
            T[0:3, 0:3] = R.from_quat(quat).as_matrix()
            T[0:3, 3] = [x, y, z]
            return T
    
    def publish_transforms(self): #this is the function that will be ran repeatedly according to the timer
        try:
            #1) Get current odom to base_link transform
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            
            #2) Convert to 4x4 matrix
            q = trans.transform.rotation
            T_odom_base = self.get_matrix(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                [q.x, q.y, q.z, q.w]
            )

            # 3) Compute odom to left_cam transform
            T_odom_left= T_odom_base@self.T_base_left
            
            # 4)Convert and broadcast
            # Left cam is w.r.t odom
            left_msg = self.matrix_to_transform(T_odom_left, 'odom', 'left_cam')
            # Right cam is w.r.t left_cam
            right_msg = self.matrix_to_transform(self.T_left_right, 'left_cam', 'right_cam')
            
            self.tf_broadcaster.sendTransform([left_msg, right_msg])

        except TransformException as ex:
            return

def main():
    rclpy.init()
    node = DynamicTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()