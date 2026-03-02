#!/usr/bin/env python3
import multiprocessing
from collections import deque

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


def closest_point(wall):
    p1, p2 = wall
    line_vec = p2 - p1
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return np.linalg.norm(p1)
    line_vec /= line_len
    point_vec = -p1
    proj = point_vec @ line_vec
    if proj < 0:
        closest_point = p1
    elif proj > line_len:
        closest_point = p2
    else:
        closest_point = proj * line_vec + p1
    return closest_point


def point_dir(wall, dir, margin=0.3):
    p1, p2 = wall
    line_vec = p2 - p1
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return None
    line_vec /= line_len
    dir_vec = np.array(dir)
    dir_vec /= np.linalg.norm(dir_vec)
    det = line_vec[0] * dir_vec[1] - line_vec[1] * dir_vec[0]
    if det == 0:
        return None
    t = (dir_vec[0] * (p1[1] - 0) - dir_vec[1] * (p1[0] - 0)) / det
    u = (line_vec[0] * (p1[1] - 0) - line_vec[1] * (p1[0] - 0)) / det
    if t < -margin or t > line_len + margin or u < 0:
        return None
    return p1 + t * line_vec


def _visualizer_process_main(data_queue, max_points):
    """
    Entry point for the visualizer subprocess.
    Runs Qt/pyqtgraph on this process's main thread (required by Qt on Linux).
    Reads data points from the multiprocessing Queue and updates plots.
    """
    import signal

    signal.signal(signal.SIGINT, signal.SIG_IGN)

    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except ImportError as e:
        print(f"[PIDVisualizer] pyqtgraph not available: {e}")
        print("[PIDVisualizer] Install with: pip install pyqtgraph PyQt5 PyOpenGL")
        return

    # Disable OpenGL — many environments (Docker, software renderers) lack
    # proper desktop OpenGL support and fail at paint time with
    # "failed to obtain functions for OpenGL Desktop".
    # pyqtgraph's QPainter path is already far faster than matplotlib.
    pg.setConfigOptions(useOpenGL=False, antialias=True)

    # Resolve Qt.DashLine for both PyQt5 and PyQt6
    try:
        _DashLine = QtCore.Qt.PenStyle.DashLine
    except AttributeError:
        _DashLine = QtCore.Qt.DashLine

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])

    # Data buffers local to this process
    times = deque(maxlen=max_points)
    side_errors = deque(maxlen=max_points)
    front_errors = deque(maxlen=max_points)
    combined_errors = deque(maxlen=max_points)
    p_terms = deque(maxlen=max_points)
    i_terms = deque(maxlen=max_points)
    d_terms = deque(maxlen=max_points)
    control_outputs = deque(maxlen=max_points)
    steering_angles = deque(maxlen=max_points)

    # Build the window
    win = pg.GraphicsLayoutWidget(title="PID Controller Visualization")
    win.resize(1000, 750)
    win.setBackground("#f8f8f8")

    # --- Subplot 1: Error Signals ---
    p0 = win.addPlot(row=0, col=0, title="Error Signals")
    p0.setLabel("left", "Error", units="m")
    p0.showGrid(x=True, y=True, alpha=0.3)
    p0.addLegend(offset=(60, 10))
    p0.addItem(
        pg.InfiniteLine(
            pos=0, angle=0, pen=pg.mkPen("gray", width=0.5, style=_DashLine)
        )
    )
    c_side = p0.plot(pen=pg.mkPen(color="b", width=1.5), name="Side Error")
    c_front = p0.plot(pen=pg.mkPen(color="r", width=1.5), name="Front Error")
    c_combined = p0.plot(pen=pg.mkPen(color="k", width=2.0), name="Combined Error")

    # --- Subplot 2: PID Components ---
    p1 = win.addPlot(row=1, col=0, title="PID Components")
    p1.setLabel("left", "Magnitude")
    p1.showGrid(x=True, y=True, alpha=0.3)
    p1.addLegend(offset=(60, 10))
    p1.addItem(
        pg.InfiniteLine(
            pos=0, angle=0, pen=pg.mkPen("gray", width=0.5, style=_DashLine)
        )
    )
    c_p = p1.plot(pen=pg.mkPen(color="g", width=1.5), name="P term")
    c_i = p1.plot(pen=pg.mkPen(color="m", width=1.5), name="I term")
    c_d = p1.plot(pen=pg.mkPen(color="c", width=1.5), name="D term")

    # --- Subplot 3: Control Output ---
    p2 = win.addPlot(row=2, col=0, title="Control Output")
    p2.setLabel("left", "Angle", units="rad")
    p2.setLabel("bottom", "Time", units="s")
    p2.showGrid(x=True, y=True, alpha=0.3)
    p2.addLegend(offset=(60, 10))
    p2.addItem(
        pg.InfiniteLine(
            pos=0, angle=0, pen=pg.mkPen("gray", width=0.5, style=_DashLine)
        )
    )
    c_control = p2.plot(pen=pg.mkPen(color="b", width=1.5), name="PID Output")
    c_steering = p2.plot(pen=pg.mkPen(color="r", width=2.0), name="Steering Angle")

    # Link X axes
    p1.setXLink(p0)
    p2.setXLink(p0)

    curves = {
        "side_error": c_side,
        "front_error": c_front,
        "combined_error": c_combined,
        "p_term": c_p,
        "i_term": c_i,
        "d_term": c_d,
        "control_output": c_control,
        "steering_angle": c_steering,
    }

    def poll_and_redraw():
        """Drain the queue and update curves. Called by QTimer on the main thread."""
        changed = False
        # Drain all available data points from the queue (non-blocking)
        while True:
            try:
                point = data_queue.get_nowait()
            except Exception:
                break
            times.append(point["t"])
            side_errors.append(point["side_error"])
            front_errors.append(point["front_error"])
            combined_errors.append(point["combined_error"])
            p_terms.append(point["p_term"])
            i_terms.append(point["i_term"])
            d_terms.append(point["d_term"])
            control_outputs.append(point["control_output"])
            steering_angles.append(point["steering_angle"])
            changed = True

        if not changed or len(times) < 2:
            return

        t = np.array(times)
        curves["side_error"].setData(t, np.array(side_errors))
        curves["front_error"].setData(t, np.array(front_errors))
        curves["combined_error"].setData(t, np.array(combined_errors))
        curves["p_term"].setData(t, np.array(p_terms))
        curves["i_term"].setData(t, np.array(i_terms))
        curves["d_term"].setData(t, np.array(d_terms))
        curves["control_output"].setData(t, np.array(control_outputs))
        curves["steering_angle"].setData(t, np.array(steering_angles))

        x_min, x_max = t[0], t[-1]
        margin = max(0.5, (x_max - x_min) * 0.02)
        p0.setXRange(x_min - margin, x_max + margin, padding=0)

    timer = QtCore.QTimer()
    timer.timeout.connect(poll_and_redraw)
    timer.start(33)  # ~30 FPS

    win.show()

    # Run the Qt event loop (blocks until the window is closed)
    app.exec_() if hasattr(app, "exec_") else app.exec()


class PIDVisualizer:
    """GPU-accelerated real-time PID visualization using pyqtgraph + OpenGL in a separate process."""

    def __init__(self, max_points=500, update_interval=5):
        """
        Args:
            max_points: Maximum number of data points to display (sliding window).
            update_interval: Not used in pyqtgraph version (kept for API compat).
        """
        self.max_points = max_points
        self._t0 = None
        self._process = None
        self._queue = None
        self._started = False

    def _ensure_started(self):
        """Spawn the visualizer subprocess on first use."""
        if self._started:
            return True
        if self._process is not None:
            # Already attempted, don't retry
            return False
        try:
            ctx = multiprocessing.get_context("spawn")
            self._queue = ctx.Queue(maxsize=2000)
            self._process = ctx.Process(
                target=_visualizer_process_main,
                args=(self._queue, self.max_points),
                daemon=True,
            )
            self._process.start()
            self._started = True
            return True
        except Exception as e:
            print(f"[PIDVisualizer] Could not start visualizer process: {e}")
            self._process = None
            return False

    def record(
        self,
        timestamp_ns,
        side_error,
        front_error,
        combined_error,
        p_term,
        i_term,
        d_term,
        control_output,
        steering_angle,
    ):
        """Record a new data point. Safe to call from any thread."""
        if not self._ensure_started():
            return

        if self._t0 is None:
            self._t0 = timestamp_ns

        t = (timestamp_ns - self._t0) / 1e9

        point = {
            "t": t,
            "side_error": side_error,
            "front_error": front_error,
            "combined_error": combined_error,
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
            "control_output": control_output,
            "steering_angle": steering_angle,
        }

        try:
            self._queue.put_nowait(point)
        except Exception:
            # Queue full — drop the point rather than blocking the ROS callback
            pass


class PIDController:
    def __init__(self, kp, ki, kd, max_i, max_d):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_i = max_i
        self.max_d = max_d

        self.previous_error = 0
        self.integral = 0

        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0

    def update(self, setpoint, pv, dt):
        error = setpoint - pv
        self.integral *= 0.9
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_i), -self.max_i)
        derivative = (error - self.previous_error) / dt
        derivative = max(min(derivative, self.max_d), -self.max_d)

        self.last_p_term = self.kp * error
        self.last_i_term = self.ki * self.integral
        self.last_d_term = self.kd * derivative

        control = self.last_p_term + self.last_i_term + self.last_d_term
        self.previous_error = error
        return control


class DriveController:
    def __init__(
        self,
        kp,
        ki,
        kd,
        max_i,
        max_d,
        side,
        side_spread,
        side_samples,
        front_spread,
        front_samples,
        velocity,
        desired_distance,
        drive_publisher,
        front_treshold,
        front_error_ratio,
        side_error_publisher=None,
        front_error_publisher=None,
        enable_visualization=True,
    ):
        self.pid_controller = PIDController(kp, ki, kd, max_i, max_d)
        self.side = side
        self.side_spread = side_spread
        self.side_samples = side_samples
        self.front_spread = front_spread
        self.front_samples = front_samples
        self.velocity = velocity
        self.desired_distance = desired_distance
        self.drive_publisher = drive_publisher
        self.previous_error = 0
        self.integral = 0
        self.last_time = None
        self.front_treshold = front_treshold
        self.front_error_ratio = front_error_ratio

        self.prev_front_error = 0
        self.prev_closest_dist = 0
        # Visualization
        self.enable_visualization = enable_visualization
        if self.enable_visualization:
            self.visualizer = PIDVisualizer(max_points=100, update_interval=1)
        else:
            self.visualizer = None

        self.side_error_publisher = side_error_publisher
        self.front_error_publisher = front_error_publisher

    def update(self, walls, laser_scan):
        if len(walls) == 0:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.velocity
            self.drive_publisher.publish(drive_msg)
            return

        closest_dist = np.inf
        angles = np.linspace(
            -self.side_spread + self.side * np.pi / 2,
            self.side_spread + self.side * np.pi / 2,
            self.side_samples,
        )
        for angle in angles:
            for wall in walls:
                dist = point_dir(wall, (np.cos(angle), np.sin(angle)), 0.1)
                if dist is not None:
                    closest_dist = min(closest_dist, np.linalg.norm(dist))
        if closest_dist == np.inf:
            closest_dist = self.prev_closest_dist
        self.prev_closest_dist = closest_dist
        side_error = closest_dist - self.desired_distance
        forward_dist = np.inf
        angles = np.linspace(
            -self.front_spread,
            self.front_spread,
            self.front_samples,
        )
        for angle in angles:
            for wall in walls:
                dist = point_dir(wall, (np.cos(angle), np.sin(angle)), 0.3)
                if dist is not None:
                    forward_dist = min(
                        forward_dist / np.cos(angle * 3), np.linalg.norm(dist)
                    )
        if forward_dist == np.inf:
            front_error = self.prev_front_error
        else:
            front_error = (
                max(0, self.front_treshold - forward_dist) / self.front_treshold
            ) ** 2
        self.prev_front_error = front_error
        msg = Float64()
        msg.data = side_error
        if self.side_error_publisher is not None:
            self.side_error_publisher.publish(msg)
        msg.data = -front_error * self.front_error_ratio
        if self.front_error_publisher is not None:
            self.front_error_publisher.publish(msg)

        error = side_error - self.front_error_ratio * front_error
        current_time = rclpy.clock.Clock().now()
        if self.last_time is None:
            dt = 0.05
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        control = self.pid_controller.update(0, error, dt)
        steering_angle = control * (-self.side)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.velocity
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

        # Feed data to the visualizer
        if self.visualizer is not None:
            self.visualizer.record(
                timestamp_ns=current_time.nanoseconds,
                side_error=side_error,
                front_error=-self.front_error_ratio * front_error,
                combined_error=error,
                p_term=self.pid_controller.last_p_term,
                i_term=self.pid_controller.last_i_term,
                d_term=self.pid_controller.last_d_term,
                control_output=control,
                steering_angle=steering_angle,
            )
