import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import math


class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        new_msg=Float32()
        new_msg.data=math.log(msg.data)
        self.publisher_.publish(new_msg)
        self.get_logger().info('Natural log of that random number: "%s"' % new_msg.data)


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
