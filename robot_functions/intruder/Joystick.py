#!/usr/bin/python3
# coding=utf8

import os
import time
import pygame
import threading

import hiwonder.ActionGroupControl as AGC
import hiwonder.Board as Board

# ============================================================
# TonyPi Joystick Control - final hybrid version
#
# What this version does:
# 1. Left stick movement uses short step-based AGC actions
# 2. Face buttons use DIRECT head-servo actions for fast response
# 3. Shoulder buttons can still run AGC actions if you want them
# 4. Every button press prints its BUTTON ID in the terminal
#
# Why this version is faster:
# - Direct servo button actions do not wait on AGC action groups
# - Movement uses one-step actions instead of long looped actions
# ============================================================

# ----------------------------
# GENERAL SETTINGS
# ----------------------------
LOOP_SLEEP = 0.01
PRINT_BUTTON_IDS = True
LEARN_MODE = False

BUTTON_COOLDOWN = 0.18

# ----------------------------
# MOVEMENT SETTINGS
# ----------------------------
MOVE_DEADZONE = 0.22
STEP_INTERVAL = 0.02

# Change these if your robot uses different movement action names
MOVE_FWD_ACTION = 'go_forward'
MOVE_BACK_ACTION = 'back_fast'
TURN_LEFT_ACTION = 'turn_left_small_step'
TURN_RIGHT_ACTION = 'turn_right_small_step'

# ----------------------------
# RIGHT STICK LOOK SETTINGS
# ----------------------------
# Set to True only if you really want right stick head control.
# Leaving it False keeps the loop lighter and usually feels better.
ENABLE_RIGHT_STICK_LOOK = False

RX_AXIS = 2
RY_AXIS = 3

SWAP_RIGHT_STICK_AXES = False
INVERT_PAN = False
INVERT_TILT = False

LOOK_DEADZONE = 0.10
LOOK_SENS_PAN = 1400
LOOK_SENS_TILT = 1200
LOOK_SMOOTH = 0.18
LOOK_UPDATE_INTERVAL = 0.03
SERVO_MOVE_TIME_MS = 30

# ----------------------------
# HEAD SERVO SETTINGS
# ----------------------------
# Based on your original file:
# servo1 = up/down
# servo2 = left/right
TILT_SERVO_ID = 1
PAN_SERVO_ID = 2

PAN_CENTER = 1500
TILT_CENTER = 1500

PAN_MIN, PAN_MAX = 500, 2500
TILT_MIN, TILT_MAX = 500, 2500

# Direct button positions
DIRECT_PAN_LEFT = 1850
DIRECT_PAN_RIGHT = 1150
DIRECT_TILT_UP = 1200
DIRECT_TILT_DOWN = 1800
DIRECT_MOVE_TIME_MS = 120

# ----------------------------
# EXIT / RESET SETTINGS
# ----------------------------
EXIT_HEAD_PAN = 1500
EXIT_HEAD_TILT = 1500
EXIT_HEAD_MOVE_TIME_MS = 300
EXIT_HEAD_SETTLE_SECONDS = 0.25

# ----------------------------
# CONTROLLER BUTTON IDS
# ----------------------------
# These came from your original file
BTN_SQUARE = 3 # 3
BTN_X = 0 #0
BTN_CIRCLE = 1 #1
BTN_TRIANGLE = 4 #4

BTN_L1 = 6 #6
BTN_R1 = 7 # 7
BTN_L2 = 8 # 8
BTN_R2 = 9 # 9

BTN_SELECT = 10 # 10
BTN_START = 11 # 11
BTN_MODE = 12 #12
BTN_L3 = 13 #13 (SOMETIMES IF YOU SHAKE THE CONTROLLER IT PRESSES THIS???)
BTN_R3 = 14 # 14
#BTN_SHAKE_CONTROLLER = 13????????

# ----------------------------
# OPTIONAL AGC BUTTON ACTIONS
# ----------------------------
# These may still feel slower than the direct servo buttons.
# Change these names if needed, or set them to None.
ACTION_L1 = 'wave'
ACTION_R1 = 'bow'
ACTION_L2 = None
ACTION_R2 = 'crouch'

AGC_BUTTON_ACTIONS = {
    BTN_L1: ACTION_L1,
    BTN_R1: ACTION_R1,
    BTN_L2: ACTION_L2,
    BTN_R2: ACTION_R2,
}

# ----------------------------
# INTERNAL STATE
# ----------------------------
_stop_all = False
_js = None

_desired_motion = None
_last_movement_step_time = 0.0

_pan = PAN_CENTER
_tilt = TILT_CENTER
_pan_vel = 0.0
_tilt_vel = 0.0
_last_look_t = time.time()

_state_lock = threading.Lock()
_agc_lock = threading.Lock()

# When a button AGC action is running, movement pauses
_button_action_active = threading.Event()


def joystick_init():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() <= 0:
        return None

    js = pygame.joystick.Joystick(0)
    js.init()

    print("Name of the joystick:", js.get_name())
    print("Axes:", js.get_numaxes(), "Buttons:", js.get_numbuttons(), "Hats:", js.get_numhats())

    print("\nCurrent button IDs:")
    print("  Square  =", BTN_SQUARE)
    print("  X       =", BTN_X)
    print("  Circle  =", BTN_CIRCLE)
    print("  Triangle=", BTN_TRIANGLE)
    print("  L1      =", BTN_L1)
    print("  R1      =", BTN_R1)
    print("  L2      =", BTN_L2)
    print("  R2      =", BTN_R2)
    print("  Select  =", BTN_SELECT)
    print("  Start   =", BTN_START)
    print("  PS      =", BTN_MODE)
    print("  L3      =", BTN_L3)
    print("  R3      =", BTN_R3)
    print()

    return js


def joystick_shutdown(js=None):
    try:
        if js is not None:
            js.quit()
    except Exception:
        pass

    try:
        pygame.joystick.quit()
    except Exception:
        pass

    try:
        pygame.quit()
    except Exception:
        pass


def _clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    return v


def set_head(pan=None, tilt=None, move_time_ms=DIRECT_MOVE_TIME_MS):
    global _pan, _tilt

    if pan is not None:
        _pan = _clamp(int(pan), PAN_MIN, PAN_MAX)
        Board.setPWMServoPulse(PAN_SERVO_ID, _pan, move_time_ms)

    if tilt is not None:
        _tilt = _clamp(int(tilt), TILT_MIN, TILT_MAX)
        Board.setPWMServoPulse(TILT_SERVO_ID, _tilt, move_time_ms)


def head_center(move_time_ms=180):
    global _pan, _tilt, _pan_vel, _tilt_vel
    _pan = PAN_CENTER
    _tilt = TILT_CENTER
    _pan_vel = 0.0
    _tilt_vel = 0.0
    Board.setPWMServoPulse(PAN_SERVO_ID, _pan, move_time_ms)
    Board.setPWMServoPulse(TILT_SERVO_ID, _tilt, move_time_ms)


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


def look_update(js):
    global _pan, _tilt, _pan_vel, _tilt_vel, _last_look_t

    if not ENABLE_RIGHT_STICK_LOOK:
        _last_look_t = time.time()
        return

    now = time.time()
    dt = now - _last_look_t

    if dt < LOOK_UPDATE_INTERVAL:
        return

    _last_look_t = now

    rx, ry = _get_right_stick(js)

    target_pan_vel = rx * LOOK_SENS_PAN
    target_tilt_vel = (-ry) * LOOK_SENS_TILT

    _pan_vel = (1.0 - LOOK_SMOOTH) * target_pan_vel + LOOK_SMOOTH * _pan_vel
    _tilt_vel = (1.0 - LOOK_SMOOTH) * target_tilt_vel + LOOK_SMOOTH * _tilt_vel

    if rx == 0.0 and abs(_pan_vel) < 1.0:
        _pan_vel = 0.0
    if ry == 0.0 and abs(_tilt_vel) < 1.0:
        _tilt_vel = 0.0

    _pan += _pan_vel * dt
    _tilt += _tilt_vel * dt

    _pan = _clamp(int(_pan), PAN_MIN, PAN_MAX)
    _tilt = _clamp(int(_tilt), TILT_MIN, TILT_MAX)

    Board.setPWMServoPulse(PAN_SERVO_ID, int(_pan), SERVO_MOVE_TIME_MS)
    Board.setPWMServoPulse(TILT_SERVO_ID, int(_tilt), SERVO_MOVE_TIME_MS)

def stop():
    _stop_all = True

def stop_actions():
    try:
        AGC.stopActionGroup()
    except Exception:
        pass


def set_desired_motion(new_motion):
    global _desired_motion
    with _state_lock:
        _desired_motion = new_motion


def get_desired_motion():
    with _state_lock:
        return _desired_motion


def motion_to_action(motion_name):
    if motion_name == 'fwd':
        return MOVE_FWD_ACTION
    elif motion_name == 'back':
        return MOVE_BACK_ACTION
    elif motion_name == 'turn_left':
        return TURN_LEFT_ACTION
    elif motion_name == 'turn_right':
        return TURN_RIGHT_ACTION
    return None


def movement_worker():
    global _last_movement_step_time

    while not _stop_all:
        if _button_action_active.is_set():
            time.sleep(0.01)
            continue

        motion = get_desired_motion()
        if motion is None:
            time.sleep(0.01)
            continue

        now = time.time()
        if (now - _last_movement_step_time) < STEP_INTERVAL:
            time.sleep(0.005)
            continue

        action_name = motion_to_action(motion)
        if action_name is None:
            time.sleep(0.01)
            continue

        _last_movement_step_time = now

        try:
            with _agc_lock:
                if _button_action_active.is_set():
                    continue
                AGC.runActionGroup(action_name, 1)
        except Exception as e:
            print("Movement action error:", e)
            time.sleep(0.05)


# ----------------------------
# DIRECT BUTTON ACTIONS
# These are the fast ones
# ----------------------------
#
#def direct_button_action(button_id):
#    if button_id == BTN_SQUARE:
#        print("[DIRECT] Square -> pan left")
#        set_head(pan=DIRECT_PAN_LEFT)
#
#    elif button_id == BTN_X:
#        print("[DIRECT] X -> pan right")
#        set_head(pan=DIRECT_PAN_RIGHT)
#
#    elif button_id == BTN_TRIANGLE:
#        print("[DIRECT] Triangle -> tilt up")
#        set_head(tilt=DIRECT_TILT_UP)
#
#    elif button_id == BTN_CIRCLE:
#        print("[DIRECT] Circle -> head center")
#        head_center()

def direct_button_action(button_id):
    if button_id == BTN_SQUARE:
        print("[DIRECT] Square -> Squat")
        _button_action_active.set()
        set_desired_motion(None)

        try:
            with _agc_lock:
                try:
                    AGC.stopActionGroup()
                except Exception:
                    pass

                time.sleep(0.03)
                AGC.runActionGroup('squat', 1)
        except Exception as e:
            print("Square squat error:", e)
        finally:
            _button_action_active.clear()


# ----------------------------
# OPTIONAL AGC BUTTON ACTIONS
# These may still be slower
# ----------------------------
def run_agc_button_action(action_name):
    if not action_name:
        return

    def _worker():
        _button_action_active.set()
        set_desired_motion(None)

        try:
            with _agc_lock:
                try:
                    AGC.stopActionGroup()
                except Exception:
                    pass

                time.sleep(0.03)
                print("[AGC BUTTON] Running:", action_name)
                AGC.runActionGroup(action_name, 1)

        except Exception as e:
            print("Button AGC action error:", e)
        finally:
            _button_action_active.clear()

    threading.Thread(target=_worker, daemon=True).start()


def cleanup():
    global _stop_all, _js

    _stop_all = True
    _button_action_active.set()
    set_desired_motion(None)
    stop_actions()
    force_head_reset_on_exit()
    joystick_shutdown(_js)
    _js = None
    print("Exited cleanly.")


def main():
    global _js

    prev_buttons = {}
    last_button_fire = {}

    head_center(move_time_ms=180)
    threading.Thread(target=movement_worker, daemon=True).start()

    while not _stop_all:
        if os.path.exists("/dev/input/js0"):
            if _js is None:
                try:
                    _js = joystick_init()
                    if _js is not None:
                        prev_buttons = {i: 0 for i in range(_js.get_numbuttons())}
                    else:
                        time.sleep(0.1)
                        continue
                except Exception as e:
                    print("Joystick init error:", e)
                    time.sleep(0.2)
                    continue
        else:
            if _js is not None:
                joystick_shutdown(_js)
                _js = None
            time.sleep(0.2)
            continue

        try:
            pygame.event.pump()
        except Exception as e:
            print("pygame pump error:", e)
            joystick_shutdown(_js)
            _js = None
            time.sleep(0.1)
            continue

        # ----------------------------
        # BUTTON HANDLING
        # ----------------------------
        try:
            for b in range(_js.get_numbuttons()):
                current = _js.get_button(b)
                previous = prev_buttons.get(b, 0)

                if current and not previous:
                    now = time.time()

                    if PRINT_BUTTON_IDS:
                        print("BUTTON PRESSED ID:", b)

                    if LEARN_MODE:
                        for a in range(_js.get_numaxes()):
                            v = _js.get_axis(a)
                            if abs(v) > 0.35:
                                print("  AXIS", a, "=", round(v, 3))

                    if (now - last_button_fire.get(b, 0)) >= BUTTON_COOLDOWN:
                        last_button_fire[b] = now

                        # Fast direct face-button actions
                        if b in (BTN_SQUARE, BTN_X, BTN_TRIANGLE, BTN_CIRCLE):
                            direct_button_action(b)

                        # Optional slower AGC shoulder-button actions
                        elif b in AGC_BUTTON_ACTIONS:
                            run_agc_button_action(AGC_BUTTON_ACTIONS[b])

                        # START = stop movement + center head
                        elif b == BTN_START:
                            print("[BUTTON] Start -> stop + center")
                            set_desired_motion(None)
                            stop_actions()
                            head_center()

                        # SELECT = stop movement only
                        elif b == BTN_SELECT:
                            print("[BUTTON] Select -> stop movement")
                            set_desired_motion(None)
                            stop_actions()
                prev_buttons[b] = current

        except Exception as e:
            print("Button read error:", e)
            joystick_shutdown(_js)
            _js = None
            time.sleep(0.1)
            continue

        # ----------------------------
        # RIGHT STICK LOOK
        # ----------------------------
        try:
            look_update(_js)
        except Exception as e:
            print("look_update error:", e)

        # ----------------------------
        # LEFT STICK MOVEMENT
        # up/down = forward/back
        # left/right = turn left/right
        # ----------------------------
        try:
            lx = _js.get_axis(0)
            ly = _js.get_axis(1)
        except Exception as e:
            print("Left stick read error:", e)
            lx, ly = 0.0, 0.0

        desired = None

        if not _button_action_active.is_set():
            if abs(ly) >= abs(lx):
                if ly <= -MOVE_DEADZONE:
                    desired = 'fwd'
                elif ly >= MOVE_DEADZONE:
                    desired = 'back'
            else:
                if lx <= -MOVE_DEADZONE:
                    desired = 'turn_left'
                elif lx >= MOVE_DEADZONE:
                    desired = 'turn_right'

        set_desired_motion(desired)

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
