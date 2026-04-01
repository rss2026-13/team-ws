import sys

import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import OccupancyGrid
from scan_simulator_2d import PyScanSimulator2D

# Try to change to just `from scan_simulator_2d import PyScanSimulator2D`
# if any error re: scan_simulator_2d occurs
from scipy.spatial.transform import Rotation as R

np.set_printoptions(threshold=sys.maxsize)


class SensorModel:
    def __init__(self, node):
        node.declare_parameter("map_topic", "default")
        node.declare_parameter("num_beams_per_particle", 1)
        node.declare_parameter("scan_theta_discretization", 1.0)
        node.declare_parameter("scan_field_of_view", 1.0)
        node.declare_parameter("lidar_scale_to_map_scale", 1.0)

        self.map_topic = (
            node.get_parameter("map_topic").get_parameter_value().string_value
        )
        self.num_beams_per_particle = (
            node.get_parameter("num_beams_per_particle")
            .get_parameter_value()
            .integer_value
        )
        self.scan_theta_discretization = (
            node.get_parameter("scan_theta_discretization")
            .get_parameter_value()
            .double_value
        )
        self.scan_field_of_view = (
            node.get_parameter("scan_field_of_view").get_parameter_value().double_value
        )
        self.lidar_scale_to_map_scale = (
            node.get_parameter("lidar_scale_to_map_scale")
            .get_parameter_value()
            .double_value
        )

        ####################################
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        self.softening_factor = 10.0
        self.DEBUG = False
        ####################################

        node.get_logger().info("%s" % self.map_topic)
        node.get_logger().info("%s" % self.num_beams_per_particle)
        node.get_logger().info("%s" % self.scan_theta_discretization)
        node.get_logger().info("%s" % self.scan_field_of_view)

        # Precompute the sensor model table
        self.sensor_model_table = np.empty((self.table_width, self.table_width))
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization,
        )

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_subscriber = node.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1
        )

        self.logger = node.get_logger()

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        d, z = np.meshgrid(
            np.linspace(0, self.table_width - 1, self.table_width),
            np.linspace(0, self.table_width - 1, self.table_width),
        )
        p_hit = np.exp(-0.5 * ((z - d) / self.sigma_hit) ** 2)
        p_hit /= np.sum(p_hit, axis=0)
        p_short = np.zeros_like(p_hit)
        mask = (z <= d) & (d > 0)
        p_short[mask] = 2 / d[mask] * (1 - z[mask] / d[mask])
        # p_short /= np.sum(p_short, axis=0)
        p_max = np.zeros_like(p_hit)
        p_max[z == self.table_width - 1] = 1
        p_rand = np.ones_like(p_hit) / self.table_width
        self.sensor_model_table = (
            self.alpha_hit * p_hit
            + self.alpha_short * p_short
            + self.alpha_max * p_max
            + self.alpha_rand * p_rand
        )
        self.sensor_model_table /= np.sum(self.sensor_model_table, axis=0)
        self.sensor_model_table = self.sensor_model_table ** (1 / self.softening_factor)

        if self.DEBUG:
            plt.imshow(self.sensor_model_table, origin="lower")
            plt.colorbar()
            plt.title("Precomputed Sensor Model Table")
            plt.xlabel("Measured Range (z)")
            plt.ylabel("Expected Range (d)")
            plt.show()

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. THIS IS Z_K. Each range in Z_K is Z_K^i

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        scans = self.scan_sim.scan(particles)
        scan_indices = np.clip(
            np.round(scans / (self.resolution * self.lidar_scale_to_map_scale)).astype(
                int
            ),
            0,
            self.table_width - 1,
        )
        obs_indices = np.clip(
            np.round(
                observation / (self.resolution * self.lidar_scale_to_map_scale)
            ).astype(int),
            0,
            self.table_width - 1,
        )
        probabilities = self.sensor_model_table[obs_indices, scan_indices]
        probabilities = np.prod(probabilities, axis=1)
        return probabilities

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map_data = np.array(map_msg.data, np.double)
        self.map = np.zeros_like(self.map_data, dtype=np.double)

        unique = np.unique(self.map_data)
        self.logger.info(f"Unique values in the map data: {unique}")
        self.map[self.map_data != 0] = 1.0

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation

        quat = [origin_o.x, origin_o.y, origin_o.z, origin_o.w]
        yaw = R.from_quat(quat).as_euler("xyz")[2]

        self.origin = (origin_p.x, origin_p.y, yaw)
        self.map_info = map_msg.info
        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            self.origin,
            0.5,
        )

        self.map_set = True
        print("Map initialized")
