import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LaneOracle(Node):
    def __init__(self):
        super().__init__('lane_oracle')
        
        # Subscribes to Odom to know where the robot is
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishes the same format as your real lane follower
        self.lane_pub = self.create_publisher(Float32MultiArray, '/lane_lines', 10)
        
        # Publishes Markers for RViz visualization
        self.marker_pub = self.create_publisher(Marker, '/lane_markers', 10)
        
        self.lane_width = 0.6  # meters

    def odom_callback(self, msg):
        # In a real "oracle," you'd use the robot's X,Y to find the closest 
        # point on a pre-defined map path. For now, let's assume a straight line.
        
        # Logic: Assume lane is at Y=0, and robot is moving along X.
        # Robot's Y position tells us how far off-center we are.
        robot_y = msg.pose.pose.position.y
        
        # Project these into "Image Space" to mimic your existing node
        # Image center = 320. We'll map -0.3m to 0 and +0.3m to 640.
        img_center = 320
        pixels_per_meter = 500 
        
        # Calculate line positions in "pixel" space
        left_x = img_center - (self.lane_width/2 + robot_y) * pixels_per_meter
        right_x = img_center + (self.lane_width/2 - robot_y) * pixels_per_meter
        
        # 1. Publish the /lane_lines topic
        line_msg = Float32MultiArray()
        # [lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2]
        # We'll just provide vertical lines in the image for simplicity
        line_msg.data = [left_x, 300.0, left_x, 100.0, right_x, 300.0, right_x, 100.0]
        self.lane_pub.publish(line_msg)
        
        # 2. Publish RViz Markers
        self.publish_markers()

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05 # line width
        marker.color.a = 1.0
        marker.color.r = 1.0 # Red lines
        
        # Create two parallel lines 10 meters long in the map
        for offset in [-self.lane_width/2, self.lane_width/2]:
            p1 = Point(x=0.0, y=offset, z=0.0)
            p2 = Point(x=10.0, y=offset, z=0.0)
            marker.points.append(p1)
            marker.points.append(p2)
            
        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    rclpy.spin(LaneOracle())
    rclpy.shutdown()

if __name__ == '__main__':
    main()                                              
