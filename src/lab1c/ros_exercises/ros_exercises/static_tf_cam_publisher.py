import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_cam_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        left_cam =self.create_tf_msg('base_link','left_cam',0,0.05,0)
        right_cam = self.create_tf_msg('left_cam','right_cam', 0,-0.10, 0)
        
        static_transforms=[left_cam,right_cam]

        self.broadcaster.sendTransform(static_transforms)

    def create_tf_msg(self, frame_id, child_id, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_id
        
        # Translation
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        
        #identity quaternion(no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        return t

def main():
    rclpy.init()
    node = StaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()