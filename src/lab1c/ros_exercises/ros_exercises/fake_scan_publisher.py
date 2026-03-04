import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import math

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('fake_scan_topic','fake_scan')
        self.declare_parameter('angle_min', -2/3*math.pi)
        self.declare_parameter('angle_max', 2/3*math.pi)
        self.declare_parameter('angle_increment', 1/300*math.pi)
        
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max',10.0)
        
        self.publish_rate=self.get_parameter('publish_rate').get_parameter_value().double_value
        self.topic_name=self.get_parameter('fake_scan_topic').get_parameter_value().string_value
        self.angle_min=self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max=self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_increment=self.get_parameter('angle_increment').get_parameter_value().double_value
        
        # self.scan_pub = self.create_publisher(LaserScan, 'fake_scan', 10)
        self.scan_pub = self.create_publisher(LaserScan, self.topic_name, 10)
        self.range_pub = self.create_publisher(Float32, 'range_test', 10)
        

        # self.angle_min=-2/3*math.pi
        # self.angle_max=2/3*math.pi
        # self.angle_increment= 1/300*math.pi
        
        self.num_elements =int((self.angle_max-self.angle_min)/self.angle_increment)+1
        
        # self.timer =self.create_timer(1/20.0, self.timer_callback)
        self.timer =self.create_timer(1/self.publish_rate, self.timer_callback)

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        # scan.range_min = 1.0
        # scan.range_max = 10.0
        scan.range_min=self.get_parameter('range_min').get_parameter_value().double_value
        scan.range_max=self.get_parameter('range_max').get_parameter_value().double_value
        
        # scan.ranges = np.random.uniform(1.0, 10.0, self.num_elements).tolist()
        scan.ranges = np.random.uniform(scan.range_min, scan.range_max, self.num_elements).tolist()
        
        self.scan_pub.publish(scan)
        length_msg = Float32()
        length_msg.data = float(len(scan.ranges))
        self.range_pub.publish(length_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
