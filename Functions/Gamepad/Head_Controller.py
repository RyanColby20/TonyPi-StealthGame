import time
from hiwonder import Board

from .Constants import *

class HeadController:
    """
    Handles smooth pan/tilt, deadzones, clamping, and servo output.
    """

    def __init__(self, robot, learn_mode=False):
        self.robot = robot
        self.learn_mode = learn_mode

        self.pan_vel = 0.0
        self.tilt_vel = 0.0
        self.last_t = time.time()
        
        self.RX_AXIS = RX_AXIS
        self.RY_AXIS = RY_AXIS

        self.PAN_SERVO_ID = PAN_SERVO_ID
        self.TILT_SERVO_ID = TILT_SERVO_ID

        self.PAN_CENTER = PAN_CENTER
        self.TILT_CENTER = TILT_CENTER

        self.PAN_MIN, self.PAN_MAX = PAN_MIN, PAN_MAX
        self.TILT_MIN, self.TILT_MAX = TILT_MIN, TILT_MAX

        self.LOOK_DEADZONE = LOOK_DEADZONE
        self.LOOK_SENS_PAN = LOOK_SENS_PAN
        self.LOOK_SENS_TILT = LOOK_SENS_TILT
        self.LOOK_SMOOTH = LOOK_SMOOTH
        self.SERVO_MOVE_TIME_MS = SERVO_MOVE_TIME_MS

        self.EXIT_MOVE_TIME_MS = EXIT_HEAD_MOVE_TIME_MS
        self.EXIT_SETTLE = EXIT_HEAD_SETTLE_SECONDS

    # ---------------------------------------------------------
    def recenter(self, move_time_ms=300):
        self.pan = self.PAN_CENTER
        self.tilt = self.TILT_CENTER
        self.pan_vel = 0
        self.tilt_vel = 0
        self._send(move_time_ms)

    def force_exit_reset(self):
        self.pan = self.PAN_CENTER
        self.tilt = self.TILT_CENTER
        self._send(self.EXIT_MOVE_TIME_MS)
        time.sleep(self.EXIT_SETTLE)

    # ---------------------------------------------------------
    def update(self, rx, ry):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0:
            return

        # Deadzone
        if abs(rx) < self.LOOK_DEADZONE:
            rx = 0
        if abs(ry) < self.LOOK_DEADZONE:
            ry = 0

        # Convert to velocities
        target_pan_vel = rx * self.LOOK_SENS_PAN
        target_tilt_vel = (-ry) * self.LOOK_SENS_TILT

        # Smooth
        self.pan_vel = (1 - self.LOOK_SMOOTH) * target_pan_vel + self.LOOK_SMOOTH * self.pan_vel
        self.tilt_vel = (1 - self.LOOK_SMOOTH) * target_tilt_vel + self.LOOK_SMOOTH * self.tilt_vel

        # Integrate
        self.pan += self.pan_vel * dt
        self.tilt += self.tilt_vel * dt

        # Clamp
        self.pan = max(self.PAN_MIN, min(self.PAN_MAX, int(self.pan)))
        self.tilt = max(self.TILT_MIN, min(self.TILT_MAX, int(self.tilt)))

        self._send(self.SERVO_MOVE_TIME_MS)

    # ---------------------------------------------------------
    def _send(self, move_time_ms):
        Board.setPWMServoPulse(self.PAN_SERVO_ID, int(self.pan), move_time_ms)
        Board.setPWMServoPulse(self.TILT_SERVO_ID, int(self.tilt), move_time_ms)
