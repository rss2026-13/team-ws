import numpy as np


class MotionModel:

    def __init__(self, node):
        # Standard deviations on body-frame odometry (independent per axis)
        node.declare_parameter("motion_model.sigma_odom_x", 0.05)
        node.declare_parameter("motion_model.sigma_odom_y", 0.05)
        node.declare_parameter("motion_model.sigma_odom_theta", 0.05)

        self.sigma_odom_x = (
            node.get_parameter("motion_model.sigma_odom_x")
            .get_parameter_value()
            .double_value
        )
        self.sigma_odom_y = (
            node.get_parameter("motion_model.sigma_odom_y")
            .get_parameter_value()
            .double_value
        )
        self.sigma_odom_theta = (
            node.get_parameter("motion_model.sigma_odom_theta")
            .get_parameter_value()
            .double_value
        )

        # Set True to disable noise (used by unit tests)
        self.deterministic = False

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
        p = np.asarray(particles, dtype=np.float64)
        if p.ndim != 2 or p.shape[1] != 3:
            raise ValueError("particles must be Nx3")
        n = p.shape[0]
        odom = np.asarray(odometry, dtype=np.float64).reshape(3)
        
        dx = float(odom[0])
        dy = float(odom[1])
        dtheta = float(odom[2])

        if not self.deterministic:
            dx = dx + np.random.randn(n) * self.sigma_odom_x
            dy = dy + np.random.randn(n) * self.sigma_odom_y
            dtheta = dtheta + np.random.randn(n) * self.sigma_odom_theta

        x = p[:, 0]
        y = p[:, 1]
        theta = p[:, 2]
        c = np.cos(theta)
        s = np.sin(theta)

        x_new = x + c * dx - s * dy
        y_new = y + s * dx + c * dy
        theta_new = theta + dtheta

        return np.column_stack([x_new, y_new, theta_new])
