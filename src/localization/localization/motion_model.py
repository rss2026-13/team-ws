import numpy as np


class MotionModel:
    def __init__(self, node):
        node.declare_parameter("deterministic", False)
        node.declare_parameter("motion_model.sigma_x", 0.2)
        node.declare_parameter("motion_model.sigma_y", 0.2)
        node.declare_parameter("motion_model.sigma_theta", 0.002)

        self.deterministic = (
            node.get_parameter("deterministic").get_parameter_value().bool_value
        )
        self.sigma_x = (
            node.get_parameter("motion_model.sigma_x")
            .get_parameter_value()
            .double_value
        )
        self.sigma_y = (
            node.get_parameter("motion_model.sigma_y")
            .get_parameter_value()
            .double_value
        )
        self.sigma_theta = (
            node.get_parameter("motion_model.sigma_theta")
            .get_parameter_value()
            .double_value
        )
        self.logger = node.get_logger()

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        x, y, theta = particles[:, 0], particles[:, 1], particles[:, 2]
        dx, dy, dtheta = odometry
        if self.deterministic:
            return np.stack(
                (
                    x + dx * np.cos(theta) - dy * np.sin(theta),
                    y + dx * np.sin(theta) + dy * np.cos(theta),
                    theta + dtheta,
                ),
                axis=1,
            )
        else:
            dx_noisy = dx + np.random.normal(0, self.sigma_x, size=x.shape)
            dy_noisy = dy + np.random.normal(0, self.sigma_y, size=y.shape)
            theta_noise = np.random.normal(0, self.sigma_theta, size=theta.shape)
            return np.stack(
                (
                    x
                    + dx_noisy * np.cos(theta + theta_noise)
                    - dy_noisy * np.sin(theta + theta_noise),
                    y
                    + dx_noisy * np.sin(theta + theta_noise)
                    + dy_noisy * np.cos(theta + theta_noise),
                    theta + dtheta + theta_noise,
                ),
                axis=1,
            )
