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
BTN_CROSS = 0
BTN_CIRCLE = 1
BTN_SQUARE = 3
BTN_TRIANGLE = 4

BTN_L1 = 6
BTN_R1 = 7

BTN_L2 = 8
BTN_R2 = 9

BTN_SELECT = 10
BTN_START = 11
BTN_MODE = 12

BTN_L3 = 13
BTN_R3 = 14

# ----------------------------
# ACTION MAPPINGS
# ----------------------------
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
    BTN_CROSS: ACTION_BOW,
    BTN_TRIANGLE: ACTION_TWIST,
    BTN_CIRCLE: ACTION_CROUCH,   # crouch button
}