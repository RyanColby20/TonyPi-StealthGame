"""Stub of the ros_robot_controller_sdk module.

The real `ros_robot_controller_sdk` module communicates with a
HiWonder servo controller via a serial port.  This stub defines
a `Board` class with methods referenced by the TonyPi example
scripts.  Each method logs its invocation instead of performing
hardware actions.
"""

import logging

logging.basicConfig(level=logging.INFO)


class Board:
    """Stub for the HiWonder robot controller board."""

    def __init__(self):
        self._name = 'StubBoard'

    def set_pwm_servo_pulse(self, channel: int, pulse: int, interval: int = 20):
        """Simulate sending a PWM pulse to a servo."""
        logging.info("[ros_robot_controller_sdk] Servo %d pulse=%d interval=%d", channel, pulse, interval)

    def set_digital_output(self, channel: int, value: int):
        logging.info("[ros_robot_controller_sdk] Digital output %d -> %d", channel, value)

    def set_motor(self, index: int, speed: int):
        logging.info("[ros_robot_controller_sdk] Motor %d -> speed %d", index, speed)

    def get_battery_voltage(self):
        """Return a dummy battery voltage."""
        return 7.4