"""Hiwonder hardware abstraction package.

This package provides simplified stubs of the modules from the
original HiWonder TonyPi project.  The goal of these stubs is to
offer the same public interfaces expected by scripts such as
Follow.py and VisualPatrol.py while allowing the code to run on
hardware‑agnostic environments.  Where appropriate, functions log
their activity rather than attempting to control actual motors or
servos.  Users wanting to run the code on a real TonyPi robot
should replace these stubs with the original HiWonder SDK.
"""

__all__ = [
    'PID',
    'Camera',
    'Misc',
    'Controller',
    'ActionGroupControl',
    'common',
    'yaml_handle',
    'ros_robot_controller_sdk',
]