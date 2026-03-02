import numpy as np
import rclpy
from geometry_msgs.msg import Point, Transform, TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker


def quaternion_to_array(quaternion):
    return np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])


def translation_to_array(translation):
    return np.array([translation.x, translation.y, translation.z])


def transform_to_matrix(transform):
    translation = translation_to_array(transform.translation)
    quaternion = quaternion_to_array(transform.rotation)
    output = np.zeros((4, 4))
    rot_matrix = R.from_quat(quaternion).as_matrix()
    output[0:3, 0:3] = rot_matrix
    output[0:3, 3] = translation
    output[3, 3] = 1
    return output


def matrix_to_transform(matrix):
    transform = Transform()
    transform.translation.x = matrix[0][3]
    transform.translation.y = matrix[1][3]
    transform.translation.z = matrix[2][3]
    quaternion = R.from_matrix(matrix[0:3, 0:3]).as_quat()
    transform.rotation.x = quaternion[0]
    transform.rotation.y = quaternion[1]
    transform.rotation.z = quaternion[2]
    transform.rotation.w = quaternion[3]
    return transform


class VisualizationTools:
    def __init__(self, publisher, frame, tf_buffer):
        self.publisher = publisher
        self.frame = frame
        self.tf_buffer = tf_buffer

    def plot_line(self, points, frame=None, color=(1.0, 0.0, 0.0)):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_LIST
        if frame is None:
            frame = self.frame
        line_strip.header.frame_id = frame

        # Set the size and color
        line_strip.scale.x = 0.1
        line_strip.scale.y = 0.1
        line_strip.color.a = 1.0
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.g = color[2]

        # Fill the line with the desired values
        for xi, yi in points:
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        self.publisher.publish(line_strip)

    def plot_walls(self, walls, timestamp, color=(1.0, 0.0, 0.0)):
        points = []
        for wall in walls:
            points.append(wall[0])
            points.append(wall[1])
        points = np.array(points)
        target_frame = "map"
        try:
            t = self.tf_buffer.lookup_transform(target_frame, self.frame, timestamp)

        except TransformException as ex:
            print(f"Could not transform {target_frame} to {self.frame}: {ex}")
            self.plot_line(points, self.frame, color=color)
            return

        mat = transform_to_matrix(t.transform)
        ones = np.ones((points.shape[0], 1))
        zeros = np.zeros((points.shape[0], 1))
        homogeneous = np.hstack([points, zeros, ones])
        transformed = (mat @ homogeneous.T).T
        transformed_points = transformed[:, 0:2].tolist()
        transformed_points = [(p[0], p[1]) for p in transformed_points]

        self.plot_line(transformed_points, target_frame, color=color)
