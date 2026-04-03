"""Action group execution stub.

HiWonder robots store complex movement sequences in binary files
called **action groups**.  These `.d6a` files can be played back
by the HiWonder controller to perform coordinated servo motion.
This stub provides a minimal API for loading and running action
groups.  On systems without a servo controller connected, the
functions simply log the requested action name.
"""

import logging
import os

logging.basicConfig(level=logging.INFO)

ACTION_GROUPS_PATH = os.path.join(os.path.dirname(__file__), '..', 'ActionGroups')


def runActionGroup(name: str, times: int = 1):
    """Simulate running an action group.

    Args:
        name (str): Name of the action group without extension.
        times (int): Number of times to repeat the sequence.
    """
    # Determine file name
    possible = os.path.join(ACTION_GROUPS_PATH, f"{name}.d6a")
    if not os.path.exists(possible):
        logging.warning("[ActionGroupControl] Action group '%s' not found at %s", name, possible)
    logging.info("[ActionGroupControl] Running action group '%s' %d time(s)", name, times)


def actionGroupRun(name: str):
    """Compatibility alias for runActionGroup."""
    runActionGroup(name, times=1)