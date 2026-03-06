#!/usr/bin/env python3


class PIDController:
    def __init__(self, kp, ki, kd, max_i, max_d, decay):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_i = max_i
        self.max_d = max_d
        self.decay = decay

        self.previous_error = 0
        self.integral = 0

        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0

    def update(self, setpoint, pv, dt):
        error = setpoint - pv
        self.integral *= self.decay
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
