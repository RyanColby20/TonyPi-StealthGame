"""Servo and motor controller stub.

This module provides stubbed functions for controlling servos,
motors and digital outputs on the TonyPi robot.  In the original
HiWonder SDK these functions communicate with a dedicated micro
controller over a serial connection.  Here we simply log the
commands to aid in development on non‑robot systems.
"""

import logging

logging.basicConfig(level=logging.INFO)


def set_pwm_servo_position(channel: int, angle: int, interval: int = 20):
    """Set the position of a servo on the specified channel.

    Args:
        channel (int): The servo channel (1‑N).
        angle (int): The desired angle in degrees.
        interval (int): Transition time in milliseconds.
    """
    logging.info("[Controller] Servo %d -> %d degrees (interval=%d ms)", channel, angle, interval)


def set_digital_output(channel: int, value: int):
    """Set a digital output to 0 or 1."""
    logging.info("[Controller] Digital output %d -> %d", channel, value)


def set_motor(index: int, speed: int):
    """Set the speed of a motor.

    Args:
        index (int): Motor index (1‑N).
        speed (int): Speed value, typically between -100 and 100.
    """
    logging.info("[Controller] Motor %d -> speed %d", index, speed)


def set_pwm_servo_release(channel: int):
    """Release the servo so it no longer holds position."""
    logging.info("[Controller] Servo %d released", channel)


# Provide a class interface for compatibility with the original
# HiWonder SDK.  Scripts typically instantiate `Controller(board)`
# and call methods on the returned object.  Each method simply
# forwards to the corresponding module‑level stub above.
class Controller:
    def __init__(self, board=None):
        # The board parameter is accepted for API compatibility but
        # is unused in this stub implementation.
        self.board = board

    def set_pwm_servo_pulse(self, channel: int, angle: int, interval: int = 20):
        set_pwm_servo_position(channel, angle, interval)

    def set_digital_output(self, channel: int, value: int):
        set_digital_output(channel, value)

    def set_motor(self, index: int, speed: int):
        set_motor(index, speed)

    def set_pwm_servo_release(self, channel: int):
        set_pwm_servo_release(channel)