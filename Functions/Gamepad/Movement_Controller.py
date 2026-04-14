import time

from Constants import *

class MovementController:
    """
    Handles left-stick movement:
      - forward
      - back
      - turn left
      - turn right
    """

    MOVE_DEADZONE = 0.55
    AXIS_COOLDOWN = 0.15

    def __init__(self, robot, learn_mode=False):
        self.robot = robot
        self.learn_mode = learn_mode

        self.last_fire = 0
        self.state = None

        # Action names
        self.FWD = "go_forward"
        self.BACK = "back_fast"
        self.TL = "turn_left"
        self.TR = "turn_right"

    # ---------------------------------------------------------
    def update(self, lx, ly):
        # Deadzone
        if abs(lx) < self.MOVE_DEADZONE:
            lx = 0
        if abs(ly) < self.MOVE_DEADZONE:
            ly = 0

        # Determine desired state
        if ly < 0:
            desired = "fwd"
        elif ly > 0:
            desired = "back"
        elif lx < 0:
            desired = "turn_left"
        elif lx > 0:
            desired = "turn_right"
        else:
            desired = None

        now = time.time()
        if desired != self.state and (now - self.last_fire) >= self.AXIS_COOLDOWN:
            self.last_fire = now
            self.state = desired

            if desired == "fwd":
                self.robot.run_action(self.FWD, times=0)
            elif desired == "back":
                self.robot.run_action(self.BACK, times=0)
            elif desired == "turn_left":
                self.robot.run_action(self.TL, times=0)
            elif desired == "turn_right":
                self.robot.run_action(self.TR, times=0)
            else:
                self.robot.stop_actions()
