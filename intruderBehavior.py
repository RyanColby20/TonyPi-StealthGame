# intruderBehavior.py

"""
intruderBehavior.py
--------------------
Defines the intruder robot's high-level behavior states.

The intruder is ALWAYS controlled manually via the PlayStation controller.
This module does NOT override or interfere with the controller scripts.

Instead, it tracks the game state:
- idle (waiting for game start)
- evading (game in progress, player moves freely)
- caught (game over)

MQTT messages from the guard trigger state transitions.
"""

import time

# ---------------- INTERNAL STATE ---------------- #

current_state = "idle"   # idle | evading | caught
state_start_time = time.time()


# ---------------- STATE MANAGEMENT ---------------- #

def set_state(new_state):
    """Switch to a new behavior state."""
    global current_state, state_start_time
    current_state = new_state
    state_start_time = time.time()
    print(f"[Intruder] State changed to: {new_state}")


def get_state():
    """Return the current state."""
    return current_state


# ---------------- MAIN LOOP ENTRY ---------------- #

def update():
    """
    Called repeatedly by intruder_main.py.
    This function does NOT control movement — the player does.
    It simply tracks the state and can trigger optional effects.
    """
    if current_state == "idle":
        idle_step()

    elif current_state == "evading":
        evading_step()

    elif current_state == "caught":
        caught_step()

    # Small delay to avoid busy looping
    time.sleep(0.05)


# ---------------- STATE STEP FUNCTIONS ---------------- #

def idle_step():
    """
    Game not started yet.
    The robot is fully player-controlled.
    """
    # Optional: LED color, pose, or idle animation
    pass


def evading_step():
    """
    Game in progress.
    Player is moving freely using the controller.
    """
    # Optional: heartbeat LED, timer, etc.
    pass


def caught_step():
    """
    Game over.
    Player is still in control, but you may want to:
    - freeze movement
    - play an animation
    - flash LEDs
    """
    # Optional: freeze or play animation
    pass


# ---------------- MQTT-TRIGGERED EVENTS ---------------- #

def on_guard_detected():
    """
    Called when the guard publishes 'guard/detected'.
    This means the guard has spotted the intruder.
    The intruder should enter the 'evading' state.
    """
    if current_state != "caught":
        print("[Intruder] Guard spotted me! Switching to evading.")
        set_state("evading")


def on_guard_caught():
    """
    Called when the guard has successfully followed the intruder long enough.
    This should be triggered by the guard via MQTT.
    """
    print("[Intruder] I've been caught! Switching to caught state.")
    set_state("caught")
