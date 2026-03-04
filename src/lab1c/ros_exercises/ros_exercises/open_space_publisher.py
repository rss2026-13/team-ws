import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
#from std_msgs.msg import Float32
import numpy as np
from custom_msgs.msg import OpenSpace

class OpenSpacePublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')
        
        self.declare_parameter('subscriber_topic','fake_scan')
        self.declare_parameter('publisher_topic','open_space')
        
        self.subscriber_topic=self.get_parameter('subscriber_topic').get_parameter_value().string_value
        self.publisher_topic=self.get_parameter('publisher_topic').get_parameter_value().string_value

        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     'fake_scan',
        #     self.listener_callback,
        #     10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.subscriber_topic,
            self.listener_callback,
            10)
        
        # self.dist_pub = self.create_publisher(Float32, 'open_space/distance', 10)
        # self.angle_pub = self.create_publisher(Float32, 'open_space/angle', 10)
        # self.publisher_=self.create_publisher(OpenSpace, 'open_space',10)
        self.publisher_=self.create_publisher(OpenSpace, self.publisher_topic,10)


    def listener_callback(self, msg):

        ranges = np.array(msg.ranges)
        max_idx = np.argmax(ranges)
        max_range = ranges[max_idx]
        
        target_angle = msg.angle_min + (max_idx * msg.angle_increment)
        
       # dist_msg = Float32()
       # dist_msg.data = float(max_range)
       # self.dist_pub.publish(dist_msg)
        
       # angle_msg = Float32()
       # angle_msg.data = float(target_angle)
       # self.angle_pub.publish(angle_msg)
        
        msg=OpenSpace()
        msg.angle=float(target_angle)
        msg.distance=float(max_range)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
