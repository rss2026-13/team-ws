import rclpy
from rclpy.node import Node


from std_msgs.msg import String, Float32
import math
import numpy as np




class SimpleSubscriber(Node):


   def __init__(self):
       super().__init__('simple_subscriber')
       self.max_buffer_size = 1000  # limit history for rolling stats
       self.past_angles = []
       self.past_distances = []
       self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)
       self.distance_subscription = self.create_subscription(
           Float32,
           'distance',
           self.distance_callback,
           10)
       self.angle_subscription = self.create_subscription(
           Float32,
           'angle',
           self.angle_callback,
           10)


       # Distance metrics (for graphs / oscillations analysis)
       self.average_distance = self.create_publisher(Float32, 'average_distance', 10)
       self.squared_distance = self.create_publisher(Float32, 'squared_distance', 10)
       self.std_distance = self.create_publisher(Float32, 'std_distance', 10)
       self.ste_distance = self.create_publisher(Float32, 'ste_distance', 10)
       # Angle relative to wall metrics
       self.average_angle = self.create_publisher(Float32, 'average_angle', 10)
       self.squared_angle = self.create_publisher(Float32, 'squared_angle', 10)


   def distance_callback(self, msg):
       self.past_distances.append(float(msg.data))
       if len(self.past_distances) > self.max_buffer_size:
           self.past_distances.pop(0)


       arr = np.array(self.past_distances)
       n = len(arr)
       if n == 0:
           return


       # Average distance
       avg_msg = Float32()
       avg_msg.data = float(np.mean(arr))
       self.average_distance.publish(avg_msg)


       # Squared distance (mean of squared distances)
       sq_msg = Float32()
       sq_msg.data = float(np.mean(arr ** 2))
       self.squared_distance.publish(sq_msg)


       # Standard deviation of distance (spread; useful for oscillations)
       std_msg = Float32()
       std_msg.data = float(np.std(arr)) if n > 1 else 0.0
       self.std_distance.publish(std_msg)


       # Standard error of distance (std / sqrt(n))
       ste_msg = Float32()
       ste_msg.data = float(np.std(arr) / math.sqrt(n)) if n > 1 else 0.0
       self.ste_distance.publish(ste_msg)


   def angle_callback(self, msg):
       self.past_angles.append(float(msg.data))
       if len(self.past_angles) > self.max_buffer_size:
           self.past_angles.pop(0)


       arr = np.array(self.past_angles)
       n = len(arr)
       if n == 0:
           return


       # Angle relative to wall average
       avg_msg = Float32()
       avg_msg.data = float(np.mean(arr))
       self.average_angle.publish(avg_msg)


       # Angle relative to wall squared (mean of squared angles)
       sq_msg = Float32()
       sq_msg.data = float(np.mean(arr ** 2))
       self.squared_angle.publish(sq_msg)




def main(args=None):
   rclpy.init(args=args)


   minimal_subscriber = SimpleSubscriber()


   rclpy.spin(minimal_subscriber)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   minimal_subscriber.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()
