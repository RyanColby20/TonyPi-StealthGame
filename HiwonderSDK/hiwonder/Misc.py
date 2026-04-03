"""Miscellaneous helper functions.

This module contains simple utility functions.  It is copied from
the HiWonder SDK and provides the `map`, `empty_func` and
`set_range` functions used by the line‑following scripts.
"""


def map(x, in_min, in_max, out_min, out_max):
    """Re‑map a number from one range to another.

    Equivalent to the Arduino `map` function.

    Args:
        x (float): The input value to remap.
        in_min (float): Lower bound of the input range.
        in_max (float): Upper bound of the input range.
        out_min (float): Lower bound of the output range.
        out_max (float): Upper bound of the output range.

    Returns:
        float: The value of `x` mapped into the output range.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def empty_func(*args, **kwargs):
    """A no‑op function used as a placeholder callback."""
    return None


def set_range(value, minimum, maximum):
    """Clamp a value to the provided range."""
    return max(min(value, maximum), minimum)