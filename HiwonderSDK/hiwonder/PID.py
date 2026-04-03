"""Simple PID controller.

This class is a near‑verbatim copy of the PID controller from
HiwonderSDK.  It maintains internal state (proportional, integral
and derivative terms) and provides an update method that returns
the computed output given a new measurement.  These values may
need tuning for your specific application.
"""


class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, integral_limit=100.0, dead_zone=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_limit = integral_limit
        self.dead_zone = dead_zone

    def reset(self):
        """Reset the PID controller state."""
        self.integral = 0.0
        self.previous_error = 0.0

    # Backwards compatibility alias
    def clear(self):
        """Alias for `reset`.  Provided for compatibility with the original SDK."""
        self.reset()

    def update(self, error):
        """Compute the PID output for the given error.

        Args:
            error (float): The difference between the desired setpoint and
                the measured value.

        Returns:
            float: The PID control output.
        """
        # Apply a dead zone around zero to reduce noise.
        if abs(error) < self.dead_zone:
            error = 0.0

        # Proportional term
        P_term = self.Kp * error

        # Integral term with anti‑windup
        self.integral += error
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        I_term = self.Ki * self.integral

        # Derivative term
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error

        return P_term + I_term + D_term