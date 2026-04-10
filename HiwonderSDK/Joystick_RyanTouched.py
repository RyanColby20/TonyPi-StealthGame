#!/usr/bin/python3
# coding=utf8

import os
import time
import pygame
import threading
import subprocess

import hiwonder.ActionGroupControl as AGC
import hiwonder.Board as Board

# ============================================================
# TonyPi Joystick Control - Refactored
#
# Main changes:
# 1. Normal controller buttons trigger action groups
# 2. Left stick moves robot in a more traditional way
# 3. Circle button triggers crouch
# 4. Head always resets to 1500 / 1500 when program exits
# ============================================================

# ----------------------------
# GENERAL SETTINGS
# ----------------------------
LOOP_SLEEP = 0.02
LEARN_MODE = False
BUTTON_COOLDOWN = 0.25

# Left stick movement
MOVE_DEADZONE = 0.55
AXIS_COOLDOWN = 0.15

# ----------------------------
# LOOK AROUND SETTINGS
# ----------------------------
RX_AXIS = 2
RY_AXIS = 3

SWAP_RIGHT_STICK_AXES = False
INVERT_PAN = False
INVERT_TILT = False

LOOK_DEADZONE = 0.08
LOOK_SENS_PAN = 1400
LOOK_SENS_TILT = 1200
LOOK_SMOOTH = 0.15
SERVO_MOVE_TIME_MS = 80

# ----------------------------
# HEAD SERVO IDs
# ----------------------------
# Based on your notes:
# servo1 = up/down
# servo2 = left/right
TILT_SERVO_ID = 1
PAN_SERVO_ID = 2

# ----------------------------
# HEAD POSITIONS / LIMITS
# ----------------------------
PAN_CENTER = 1500
TILT_CENTER = 1500

PAN_MIN, PAN_MAX = 500, 2500
TILT_MIN, TILT_MAX = 500, 2500

RECENTER_BUTTON = None
LOOK_ALWAYS_ON = True
LOOK_ENABLE_BUTTON = None

# ----------------------------
# EXIT / RESET SETTINGS
# ----------------------------
RESET_HEAD_BEFORE_EXTERNAL_LAUNCH = True
RESET_HEAD_MOVE_TIME_MS = 400
RESET_HEAD_SETTLE_SECONDS = 0.40

EXIT_HEAD_PAN = 1500
EXIT_HEAD_TILT = 1500
EXIT_HEAD_MOVE_TIME_MS = 400
EXIT_HEAD_SETTLE_SECONDS = 0.35

# ----------------------------
# CONTROLLER BUTTONS
# ----------------------------
# These are common pygame mappings for many PS-style controllers.
# If LEARN_MODE shows different numbers on your setup, just change these.
BTN_SQUARE = 0
BTN_X = 1
BTN_CIRCLE = 2
BTN_TRIANGLE = 3

BTN_L1 = 4
BTN_R1 = 5
BTN_L2 = 6
BTN_R2 = 7

BTN_SELECT = 8
BTN_START = 9
BTN_PS = 10
BTN_L3 = 11
BTN_R3 = 12

# ----------------------------
# ACTION MAPPINGS
# ----------------------------
# Change these to action group names that actually exist on your robot.
ACTION_WAVE = 'wave'
ACTION_BOW = 'bow'
ACTION_TWIST = 'twist'
ACTION_STAND = 'stand'
ACTION_CROUCH = 'crouch'

TURN_LEFT_ACTION = 'turn_left'
TURN_RIGHT_ACTION = 'turn_right'
MOVE_FWD_ACTION = 'go_forward'
MOVE_BACK_ACTION = 'back_fast'

button_to_action = {
    BTN_SQUARE: ACTION_WAVE,
    BTN_X: ACTION_BOW,
    BTN_TRIANGLE: ACTION_TWIST,
    BTN_CIRCLE: ACTION_CROUCH,   # crouch button
}

# ----------------------------
# INTERNAL STATE
# ----------------------------
_stop_all = False

_pan = PAN_CENTER
_tilt = TILT_CENTER
_pan_vel = 0.0
_tilt_vel = 0.0
_last_look_t = time.time()
_last_debug_print = 0.0

_action_lock = threading.Lock()


def joystick_init():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() <= 0:
        return None

    js = pygame.joystick.Joystick(0)
    js.init()

    print("Name of the joystick:", js.get_name())
    print("Axes:", js.get_numaxes(), "Buttons:", js.get_numbuttons(), "Hats:", js.get_numhats())
    return js


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _deadzone(v, dz):
    return 0.0 if abs(v) < dz else v


def send_head_positions(move_time_ms=SERVO_MOVE_TIME_MS):
    Board.setPWMServoPulse(PAN_SERVO_ID, int(_pan), move_time_ms)
    Board.setPWMServoPulse(TILT_SERVO_ID, int(_tilt), move_time_ms)


def look_recenter(move_time_ms=300):
    global _pan, _tilt, _pan_vel, _tilt_vel

    _pan = PAN_CENTER
    _tilt = TILT_CENTER
    _pan_vel = 0.0
    _tilt_vel = 0.0

    send_head_positions(move_time_ms)

    if LEARN_MODE:
        print("[HEAD] recenter -> pan:", _pan, "tilt:", _tilt)


def force_head_reset_on_exit():
    global _pan, _tilt, _pan_vel, _tilt_vel

    try:
        AGC.stopActionGroup()
    except Exception:
        pass

    try:
        _pan = EXIT_HEAD_PAN
        _tilt = EXIT_HEAD_TILT
        _pan_vel = 0.0
        _tilt_vel = 0.0

        Board.setPWMServoPulse(PAN_SERVO_ID, int(_pan), EXIT_HEAD_MOVE_TIME_MS)
        Board.setPWMServoPulse(TILT_SERVO_ID, int(_tilt), EXIT_HEAD_MOVE_TIME_MS)

        time.sleep(EXIT_HEAD_SETTLE_SECONDS)
        print("[EXIT] Head reset to 1500 / 1500")
    except Exception as e:
        print("[EXIT] Head reset error:", e)


def reset_head_before_external_launch():
    try:
        AGC.stopActionGroup()
    except Exception:
        pass

    try:
        look_recenter(RESET_HEAD_MOVE_TIME_MS)
        time.sleep(RESET_HEAD_SETTLE_SECONDS)
    except Exception as e:
        print("Head reset before launch error:", e)


def launch_external(command, use_popen=True):
    if RESET_HEAD_BEFORE_EXTERNAL_LAUNCH:
        reset_head_before_external_launch()

    try:
        if use_popen:
            subprocess.Popen(command, shell=True)
        else:
            os.system(command)
    except Exception as e:
        print("External launch error:", e)


def _get_right_stick(js):
    a = js.get_axis(RX_AXIS)
    b = js.get_axis(RY_AXIS)

    if SWAP_RIGHT_STICK_AXES:
        rx, ry = b, a
    else:
        rx, ry = a, b

    rx = _deadzone(rx, LOOK_DEADZONE)
    ry = _deadzone(ry, LOOK_DEADZONE)

    if INVERT_PAN:
        rx = -rx
    if INVERT_TILT:
        ry = -ry

    return rx, ry


def look_update(js, enabled=True):
    global _pan, _tilt, _pan_vel, _tilt_vel, _last_look_t, _last_debug_print

    if not enabled:
        _last_look_t = time.time()
        return

    now = time.time()
    dt = now - _last_look_t
    _last_look_t = now

    if dt <= 0:
        return

    rx, ry = _get_right_stick(js)

    target_pan_vel = rx * LOOK_SENS_PAN
    target_tilt_vel = (-ry) * LOOK_SENS_TILT

    _pan_vel = (1 - LOOK_SMOOTH) * target_pan_vel + LOOK_SMOOTH * _pan_vel
    _tilt_vel = (1 - LOOK_SMOOTH) * target_tilt_vel + LOOK_SMOOTH * _tilt_vel

    _pan += _pan_vel * dt
    _tilt += _tilt_vel * dt

    _pan = _clamp(int(_pan), PAN_MIN, PAN_MAX)
    _tilt = _clamp(int(_tilt), TILT_MIN, TILT_MAX)

    send_head_positions(SERVO_MOVE_TIME_MS)

    if LEARN_MODE and (now - _last_debug_print) > 0.15:
        _last_debug_print = now
        print(
            "[LOOK]",
            "rx=", round(rx, 3),
            "ry=", round(ry, 3),
            "pan=", int(_pan),
            "tilt=", int(_tilt)
        )


def run_action(action_name: str, times: int = 1):
    if not action_name:
        return

    with _action_lock:
        try:
            AGC.stopActionGroup()
        except Exception:
            pass

        def _worker():
            try:
                AGC.runActionGroup(action_name, times)
            except Exception as e:
                print("AGC error:", e)

        threading.Thread(target=_worker, daemon=True).start()


def stop_actions():
    try:
        AGC.stopActionGroup()
    except Exception:
        pass


def cleanup():
    global _stop_all
    _stop_all = True
    stop_actions()
    force_head_reset_on_exit()
    print("Exited cleanly.")


def main():
    global _stop_all

    js = None
    connected = False

    pressed = set()
    last_button_fire = {}

    move_state = None
    last_axis_fire = 0.0

    prev_buttons = {}
    prev_hat = (0, 0)

    try:
        look_recenter()
    except Exception as e:
        if LEARN_MODE:
            print("Initial recenter error:", e)

    while not _stop_all:
        if os.path.exists("/dev/input/js0"):
            if not connected:
                js = joystick_init()
                connected = js is not None

                if connected:
                    prev_buttons = {i: 0 for i in range(js.get_numbuttons())}
                    if js.get_numhats() > 0:
                        prev_hat = js.get_hat(0)
        else:
            if connected:
                connected = False
                try:
                    js.quit()
                except Exception:
                    pass
                pygame.joystick.quit()
                js = None

        if not connected or js is None:
            time.sleep(0.2)
            continue

        pygame.event.pump()

        # ----------------------------
        # BUTTON HANDLING
        # ----------------------------
        for b in range(js.get_numbuttons()):
            current = js.get_button(b)
            previous = prev_buttons.get(b, 0)

            if current and not previous:
                pressed.add(b)

                if LEARN_MODE:
                    print("BUTTON PRESSED:", b)
                    for a in range(js.get_numaxes()):
                        v = js.get_axis(a)
                        if abs(v) > 0.35:
                            print("  AXIS", a, "=", round(v, 3))

                now = time.time()

                if b in button_to_action:
                    if (now - last_button_fire.get(b, 0)) >= BUTTON_COOLDOWN:
                        last_button_fire[b] = now
                        print("[BUTTON] Running action:", button_to_action[b])
                        run_action(button_to_action[b], times=1)

                if RECENTER_BUTTON is not None and b == RECENTER_BUTTON:
                    look_recenter()

                if b == BTN_START:
                    print("[BUTTON] START pressed -> standing and recentring head")
                    run_action(ACTION_STAND, times=1)
                    look_recenter()

            prev_buttons[b] = current

        # ----------------------------
        # HAT DEBUG
        # ----------------------------
        hat = (0, 0)
        if js.get_numhats() > 0:
            hat = js.get_hat(0)

        if hat != prev_hat and LEARN_MODE:
            print("HAT:", hat)

        prev_hat = hat

        # ----------------------------
        # RIGHT STICK LOOK
        # ----------------------------
        look_enabled = LOOK_ALWAYS_ON
        if not LOOK_ALWAYS_ON and LOOK_ENABLE_BUTTON is not None:
            look_enabled = (LOOK_ENABLE_BUTTON in pressed)

        try:
            look_update(js, enabled=look_enabled)
        except Exception as e:
            if LEARN_MODE:
                print("look_update error:", e)

        # ----------------------------
        # LEFT STICK MOVEMENT
        # Traditional style:
        # up/down = forward/back
        # left/right = turn left/right
        # ----------------------------
        try:
            lx = _deadzone(js.get_axis(0), MOVE_DEADZONE)
            ly = _deadzone(js.get_axis(1), MOVE_DEADZONE)
        except Exception as e:
            if LEARN_MODE:
                print("Left stick read error:", e)
            lx, ly = 0.0, 0.0

        desired = None

        # Prioritize forward/back first
        if ly < 0:
            desired = 'fwd'
        elif ly > 0:
            desired = 'back'
        elif lx < 0:
            desired = 'turn_left'
        elif lx > 0:
            desired = 'turn_right'
        else:
            desired = None

        now = time.time()
        if desired != move_state and (now - last_axis_fire) >= AXIS_COOLDOWN:
            last_axis_fire = now
            move_state = desired

            if move_state == 'fwd':
                if LEARN_MODE:
                    print("Left stick ->", MOVE_FWD_ACTION)
                run_action(MOVE_FWD_ACTION, times=0)

            elif move_state == 'back':
                if LEARN_MODE:
                    print("Left stick ->", MOVE_BACK_ACTION)
                run_action(MOVE_BACK_ACTION, times=0)

            elif move_state == 'turn_left':
                if LEARN_MODE:
                    print("Left stick ->", TURN_LEFT_ACTION)
                run_action(TURN_LEFT_ACTION, times=0)

            elif move_state == 'turn_right':
                if LEARN_MODE:
                    print("Left stick ->", TURN_RIGHT_ACTION)
                run_action(TURN_RIGHT_ACTION, times=0)

            else:
                stop_actions()

        pressed.clear()
        time.sleep(LOOP_SLEEP)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received.")
    except Exception as e:
        print("Fatal error:", e)
    finally:
        cleanup() 