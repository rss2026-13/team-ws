import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import random

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', 10)
        timer_period = 1/20  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32()
        num=random.uniform(0,10)
        msg.data = num
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
